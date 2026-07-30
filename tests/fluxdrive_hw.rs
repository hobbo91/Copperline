// SPDX-License-Identifier: GPL-3.0-or-later

//! Hardware probes for the flux-level floppy path.
//!
//! These need a flux interface with a drive and a disk in it, so they are
//! ignored by default. There is one drive, so they run one at a time:
//!
//! ```sh
//! cargo test --release --test fluxdrive_hw -- --ignored --nocapture --test-threads=1
//! ```
//!
//! `COPPERLINE_FLUXDRIVE_PORT` names the serial port, or is left unset to pick
//! the only interface attached. `COPPERLINE_FLUXDRIVE_DRIVE` selects the drive
//! on the cable (`a`/`b`, or `0`..`3` for Shugart); it defaults to the position
//! a lone drive on a PC cable ends up in.

#![cfg(feature = "fluxdrive")]

use copperline::fluxdrive::amigados::{self, BYTES_PER_SECTOR, SECTORS_PER_TRACK};
use copperline::fluxdrive::cells::recover_cells;
use copperline::fluxdrive::greaseweazle::{DriveSelect, Greaseweazle};
use copperline::fluxdrive::{FluxSource, Head};

/// A 3.5" spindle turns at 300 rpm, so index pulses fall 200 ms apart. Drives
/// are allowed to be a little off, and how far off this one is matters: the
/// measured period, not the nominal one, is what sets the bit-cell rate.
const NOMINAL_REVOLUTION_SECONDS: f64 = 0.2;

/// Amiga double-density cells are 2 us. MFM never puts two reversals in
/// adjacent cells and never leaves more than three cells without one, so every
/// interval on the disk is 2, 3 or 4 cells: 4, 6 or 8 us.
const CELL_MICROSECONDS: f64 = 2.0;
const EXPECTED_CELL_SPACINGS: [f64; 3] = [2.0, 3.0, 4.0];

fn open_drive() -> Greaseweazle {
    let port = std::env::var("COPPERLINE_FLUXDRIVE_PORT").ok();
    let drive = std::env::var("COPPERLINE_FLUXDRIVE_DRIVE")
        .ok()
        .map(|spec| DriveSelect::parse(&spec).expect("valid drive selector"))
        .unwrap_or_default();
    Greaseweazle::open(port.as_deref(), drive).expect("open the flux interface")
}

/// Read flux off a real disk and check it against what the geometry of an Amiga
/// DD disk demands, without decoding a single MFM cell.
///
/// This is the whole transport and protocol under test: if the sample rate,
/// interval encoding, escape opcodes or index timestamps were wrong, the
/// rotation period and the spacing of flux reversals could not both come out
/// right.
#[test]
#[ignore = "requires a flux interface with a disk in the drive"]
fn flux_from_a_real_disk_has_amiga_dd_geometry() {
    let mut drive = open_drive();
    println!("{}", drive.describe());

    let cylinder: u8 = std::env::var("COPPERLINE_FLUXDRIVE_CYLINDER")
        .ok()
        .and_then(|c| c.parse().ok())
        .unwrap_or(0);

    drive.motor(true).expect("spin the disk up");
    drive.seek(cylinder).expect("seek");
    drive
        .select_head(Head::Lower)
        .expect("select the lower head");

    let capture = drive.read_flux(3).expect("capture flux");
    let ticks_per_sec = f64::from(capture.ticks_per_sec);
    println!(
        "captured {} reversals, {} index pulses, {:.1} ms of flux",
        capture.intervals.len(),
        capture.index_ticks.len(),
        capture.total_ticks() as f64 / ticks_per_sec * 1.0e3,
    );

    // The disk's true speed. A drive well off 300 rpm shifts every cell in the
    // track, which is why this is measured rather than assumed.
    let periods = capture.revolution_seconds();
    assert!(
        capture.revolutions() >= 3,
        "asked for 3 whole revolutions, got {}",
        capture.revolutions()
    );
    for (rev, period) in periods.iter().enumerate() {
        let rpm = 60.0 / period;
        let error_percent = (period / NOMINAL_REVOLUTION_SECONDS - 1.0) * 100.0;
        println!(
            "  revolution {rev}: {:.3} ms, {rpm:.1} rpm ({error_percent:+.2}%)",
            period * 1.0e3
        );
        assert!(
            (period - NOMINAL_REVOLUTION_SECONDS).abs() < NOMINAL_REVOLUTION_SECONDS * 0.1,
            "revolution {rev} lasted {period} s, nowhere near a 300 rpm spindle: \
             the tick rate or the index timestamps are being read wrongly"
        );
    }

    // Every reversal on an MFM track is 2, 3 or 4 bit cells after the last.
    // Bucket the intervals of one whole revolution by cell count and check the
    // population sits where the encoding says it must.
    let rev = capture.revolution(0).expect("first whole revolution");
    let counted: u64 = rev.intervals.iter().map(|&t| u64::from(t)).sum();
    assert_eq!(
        counted + rev.trailing_ticks,
        rev.ticks,
        "a revolution's flux must account for exactly its rotation period"
    );

    // Cell time from this revolution's own measured length, not from nominal.
    let cell_ticks = CELL_MICROSECONDS * 1.0e-6 * ticks_per_sec;
    let mut buckets = [0usize; 8];
    let mut outliers = 0usize;
    let mut sum_by_bucket = [0.0f64; 8];
    for &interval in &rev.intervals {
        let cells = f64::from(interval) / cell_ticks;
        let nearest = cells.round();
        if !(2.0..=4.0).contains(&nearest) || (cells - nearest).abs() > 0.4 {
            outliers += 1;
            continue;
        }
        let slot = nearest as usize;
        buckets[slot] += 1;
        sum_by_bucket[slot] += cells;
    }

    let total: usize = buckets.iter().sum();
    println!("  {total} reversals in one revolution, {outliers} off-grid:");
    for spacing in EXPECTED_CELL_SPACINGS {
        let slot = spacing as usize;
        let count = buckets[slot];
        let mean = if count > 0 {
            sum_by_bucket[slot] / count as f64
        } else {
            0.0
        };
        println!(
            "    {spacing:.0} cells ({:.1} us): {count:6} reversals, mean {mean:.3} cells",
            spacing * CELL_MICROSECONDS
        );
        assert!(
            count > 0,
            "no reversals {spacing} cells apart: MFM cannot produce such a track"
        );
        assert!(
            (mean - spacing).abs() < 0.15,
            "reversals near {spacing} cells average {mean:.3} cells: the recovered \
             cell rate is off"
        );
    }

    // A little off-grid population is ordinary: speed variation, the write
    // splice, and the odd weak bit. A lot of it means the timebase is wrong.
    let off_grid = outliers as f64 / (total + outliers) as f64;
    println!("  off-grid: {:.2}%", off_grid * 100.0);
    assert!(
        off_grid < 0.02,
        "{:.1}% of reversals do not sit on the MFM cell grid: the flux timebase \
         is not being interpreted correctly",
        off_grid * 100.0
    );

    // An Amiga DD track holds 11 sectors of 512 bytes plus headers and gap,
    // which is about 100_000 cells. Reversals are fewer than cells, since only
    // some cells carry one.
    let cells = rev.ticks as f64 / cell_ticks;
    println!("  {cells:.0} cells in the revolution");
    assert!(
        (90_000.0..115_000.0).contains(&cells),
        "{cells:.0} cells is not a double-density Amiga track"
    );

    drive.motor(false).expect("spin the disk down");
}

/// The track number an AmigaDOS sector header carries: two per cylinder.
fn track_number(cylinder: u8, head: Head) -> u8 {
    cylinder * 2 + head.index()
}

/// Read one track off the disk and recover its sectors, revolution by
/// revolution.
fn read_track(drive: &mut Greaseweazle, cylinder: u8, head: Head, revolutions: u8) -> Vec<Scanned> {
    drive.seek(cylinder).expect("seek");
    drive.select_head(head).expect("select head");
    let capture = drive.read_flux(revolutions).expect("capture flux");
    (0..capture.revolutions())
        .map(|rev| {
            let revolution = capture.revolution(rev).expect("whole revolution");
            let recovered =
                recover_cells(&revolution, capture.ticks_per_sec).expect("recover cells");
            let scan = amigados::scan_track(
                &recovered.words,
                recovered.bit_len as usize,
                Some(track_number(cylinder, head)),
            );
            Scanned {
                mean_cell_ns: recovered.mean_cell_ns(),
                bit_len: recovered.bit_len,
                scan,
            }
        })
        .collect()
}

struct Scanned {
    mean_cell_ns: f64,
    bit_len: u32,
    scan: amigados::TrackScan,
}

/// Phase 0: raw flux off a physical disk, through Copperline's own data
/// separator, to eleven checksummed AmigaDOS sectors.
///
/// Every revolution is decoded on its own. That matters beyond redundancy: the
/// guest's trackdisk driver recovers a marginal sector by re-reading the track,
/// which only helps if a re-read sees freshly measured flux. A revolution that
/// decodes independently is what makes that possible.
#[test]
#[ignore = "requires a flux interface with an AmigaDOS disk in the drive"]
fn flux_decodes_to_complete_amigados_tracks() {
    let mut drive = open_drive();
    println!("{}", drive.describe());
    drive.motor(true).expect("spin the disk up");

    // A spread across the disk: the outermost cylinder, one in the middle where
    // the guest keeps the root block, and one near the inside where cells are
    // physically shortest and reads are hardest.
    let probes = [(0u8, Head::Lower), (40, Head::Upper), (79, Head::Lower)];
    let mut revolutions_seen = 0;

    for (cylinder, head) in probes {
        let track = track_number(cylinder, head);
        println!("cylinder {cylinder}, head {}, track {track}:", head.index());
        let scans = read_track(&mut drive, cylinder, head, 3);
        assert!(
            scans.len() >= 3,
            "expected 3 revolutions, got {}",
            scans.len()
        );

        for (rev, scanned) in scans.iter().enumerate() {
            println!(
                "  revolution {rev}: {}, {} cells, mean {:.1} ns/cell",
                scanned.scan.summary(),
                scanned.bit_len,
                scanned.mean_cell_ns,
            );
            assert!(
                scanned.scan.is_complete(),
                "cylinder {cylinder} head {} revolution {rev}: {}",
                head.index(),
                scanned.scan.summary()
            );
            for sector in &scanned.scan.sectors {
                assert_eq!(sector.track, track, "sector header names the wrong track");
            }
            revolutions_seen += 1;
        }
    }

    println!("{revolutions_seen} revolutions, every one complete");
    drive.motor(false).expect("spin the disk down");
}

/// The strongest check available: compare what came off the physical disk with
/// an image of that same disk, byte for byte.
///
/// Checksums only prove a sector is self-consistent. This proves the whole path
/// -- flux timing, cell recovery, the odd/even split, sector ordering -- puts
/// back exactly the bytes that are on the disk.
///
/// Set `COPPERLINE_FLUXDRIVE_ADF` to an image **of the disk in the drive**, not
/// merely to the same title. A pressed or well-used disk is very often not a
/// byte-exact copy of any image of it going around, so comparing against the
/// wrong dump reports differences that are really the disk's own history. The
/// straightforward way to get a correct one is to dump the disk itself first,
/// with a tool that decodes independently of this code.
#[test]
#[ignore = "requires a flux interface plus an image of the disk in the drive"]
fn decoded_sectors_match_an_image_of_the_same_disk() {
    let Ok(adf_path) = std::env::var("COPPERLINE_FLUXDRIVE_ADF") else {
        eprintln!("skipped: set COPPERLINE_FLUXDRIVE_ADF to an image of the disk in the drive");
        return;
    };
    let adf = std::fs::read(&adf_path).expect("read the reference image");
    let expected_len = 160 * SECTORS_PER_TRACK * BYTES_PER_SECTOR;
    assert_eq!(
        adf.len(),
        expected_len,
        "{adf_path} is not a standard 880K double-density image"
    );

    let mut drive = open_drive();
    println!("{}", drive.describe());
    println!("comparing against {adf_path}");
    drive.motor(true).expect("spin the disk up");

    // Across the disk, including cylinder 58, where this disk's oxide is weakest
    // and a revolution often gives up a sector.
    let probes = [
        (0u8, Head::Lower),
        (0, Head::Upper),
        (40, Head::Lower),
        (58, Head::Lower),
        (58, Head::Upper),
        (79, Head::Upper),
    ];
    let mut compared = 0usize;

    for (cylinder, head) in probes {
        let track = track_number(cylinder, head);
        let scans = read_track(&mut drive, cylinder, head, 3);

        // Take the first revolution that read whole. Marginal oxide gives up a
        // sector on one pass and not the next, and a real Amiga recovers by
        // re-reading until the software checksum passes, so a track is only
        // genuinely bad if no revolution of it is complete.
        let complete = scans.iter().position(|s| s.scan.is_complete());
        let Some(rev) = complete else {
            for (rev, scanned) in scans.iter().enumerate() {
                eprintln!(
                    "  track {track} revolution {rev}: {}",
                    scanned.scan.summary()
                );
            }
            panic!(
                "track {track} did not read whole in any of {} revolutions",
                scans.len()
            );
        };
        if rev != 0 {
            println!(
                "  track {track}: revolution 0 read {}, recovered on revolution {rev}",
                scans[0].scan.summary()
            );
        }
        let sectors = scans[rev]
            .scan
            .assemble()
            .expect("complete track assembles");

        for (sector, data) in sectors.iter().enumerate() {
            let offset = (usize::from(track) * SECTORS_PER_TRACK + sector) * BYTES_PER_SECTOR;
            let reference = &adf[offset..offset + BYTES_PER_SECTOR];
            let first_difference = data.iter().zip(reference).position(|(a, b)| a != b);
            assert!(
                first_difference.is_none(),
                "track {track} sector {sector} differs from the image at byte {}: \
                 read {:#04x}, image {:#04x}",
                first_difference.unwrap(),
                data[first_difference.unwrap()],
                reference[first_difference.unwrap()],
            );
            compared += 1;
        }
        println!("  track {track}: all {SECTORS_PER_TRACK} sectors match the image");
    }

    println!("{compared} sectors identical to the reference image");
    drive.motor(false).expect("spin the disk down");
}

/// Watch the drive's status lines while a disk is taken out and put back.
///
/// Pin 34 is `/DSKCHG` on some 3.5" drives and `/RDY` on others -- it is
/// jumper-selectable on many -- and the two mean almost opposite things. Which
/// one this drive fits decides whether an empty slot can be sensed at all, so it
/// is worth establishing rather than assuming.
///
/// Run it, then take the disk out and put it back while it watches.
#[test]
#[ignore = "requires someone to eject and re-insert a disk while it runs"]
fn watch_the_drive_status_lines() {
    let mut drive = open_drive();
    println!("{}", drive.describe());
    println!("watching for 40s -- eject the disk, wait, then put it back\n");

    let start = std::time::Instant::now();
    let mut last = String::new();
    while start.elapsed() < std::time::Duration::from_secs(40) {
        let status = drive.status().expect("read the drive status");
        let line = format!(
            "disk_present={:?} write_protected={:?} cylinder={:?} motor={}",
            status.disk_present, status.write_protected, status.cylinder, status.motor_on,
        );
        if line != last {
            println!("[{:5.1}s] {line}", start.elapsed().as_secs_f64());
            last = line;
        }
        // Slow: this only watches, and hammering the interface teaches nothing.
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    println!("\ndone");
}

/// Print raw drive-line levels, to compare against another tool's reading.
#[test]
#[ignore = "requires a flux interface"]
fn print_raw_pin_levels() {
    let mut drive = open_drive();
    println!("{}", drive.describe());
    for pin in [2u8, 26, 28, 34] {
        let level = drive.pin_level(pin).expect("read the pin");
        let name = match pin {
            2 => "/DENSEL",
            26 => "/TRK0",
            28 => "/WRPROT",
            34 => "/DSKCHG",
            _ => "?",
        };
        println!(
            "  pin {pin:2} {name:8}: {}",
            match level {
                Some(true) => "high",
                Some(false) => "low",
                None => "unsupported",
            }
        );
    }
}

/// Does a spun-up-from-cold read need to wait for the spindle?
///
/// A probe for a disk turns the motor on and reads. If the platter is not yet at
/// speed no index pulse comes round, and the drive reports itself empty when it
/// is not -- so how long the wait has to be decides whether a disk put in while
/// the machine runs is ever noticed.
#[test]
#[ignore = "requires a flux interface with a disk in the drive"]
fn a_cold_spin_up_needs_time_before_reading() {
    let mut drive = open_drive();
    println!("{}", drive.describe());

    for wait_ms in [0u64, 250, 500, 750, 1000] {
        drive.motor(false).expect("stop the motor");
        std::thread::sleep(std::time::Duration::from_millis(1500));
        drive.motor(true).expect("start the motor");
        if wait_ms > 0 {
            std::thread::sleep(std::time::Duration::from_millis(wait_ms));
        }
        let outcome = drive.read_flux(1);
        println!(
            "  waited {wait_ms:4} ms after motor-on: {}",
            match &outcome {
                Ok(c) => format!("read {} revolutions", c.revolutions()),
                Err(e) => format!("FAILED -- {e}"),
            }
        );
    }
    drive.motor(false).expect("stop the motor");
}

/// Where the time in a track read actually goes.
///
/// A revolution is 200 ms and nothing can make it shorter, so anything else is
/// worth knowing about: the seek, the wait for an index, moving the flux over
/// USB, and recovering cells from it.
#[test]
#[ignore = "requires a flux interface with a disk in the drive"]
fn time_the_parts_of_a_track_read() {
    use std::time::Instant;
    let mut drive = open_drive();
    println!("{}", drive.describe());
    drive.motor(true).expect("spin up");
    // Settle the spindle before timing anything.
    std::thread::sleep(std::time::Duration::from_millis(1000));

    for revolutions in [1u8, 2, 3] {
        let mut seek_ms = 0.0;
        let mut read_ms = 0.0;
        let mut decode_ms = 0.0;
        let passes = 8;
        for pass in 0..passes {
            // Alternate cylinders so each pass pays a real one-track seek.
            let cylinder = 20 + (pass % 2) as u8;
            let t = Instant::now();
            drive.seek(cylinder).expect("seek");
            drive.select_head(Head::Lower).expect("head");
            seek_ms += t.elapsed().as_secs_f64() * 1e3;

            let t = Instant::now();
            let capture = drive.read_flux(revolutions).expect("read");
            read_ms += t.elapsed().as_secs_f64() * 1e3;

            let t = Instant::now();
            for rev in 0..capture.revolutions() {
                let r = capture.revolution(rev).expect("revolution");
                recover_cells(&r, capture.ticks_per_sec).expect("cells");
            }
            decode_ms += t.elapsed().as_secs_f64() * 1e3;
        }
        let n = f64::from(passes);
        println!(
            "  {revolutions} rev: seek {:.0} ms + read {:.0} ms + decode {:.0} ms = {:.0} ms \
             (a revolution is 200 ms, so {:.0} ms is not the disk)",
            seek_ms / n,
            read_ms / n,
            decode_ms / n,
            (seek_ms + read_ms + decode_ms) / n,
            (seek_ms + read_ms + decode_ms) / n - 200.0 * f64::from(revolutions),
        );
    }
    drive.motor(false).expect("spin down");
}

/// Does a revolution have to begin at the index hole?
///
/// Waiting for one costs a whole rotation on every read -- a read ends at an
/// index, so the next begins just past it and waits all the way round again.
/// That wait is only worth paying if a ring cut anywhere else is worse.
///
/// It should not be: a ring exactly one rotation long closes on the same
/// physical spot it opened, so the sectors run continuously across the join. The
/// two ends are merely read one revolution apart in time, so only bit-level
/// jitter differs there.
#[test]
#[ignore = "requires a flux interface with an AmigaDOS disk in the drive"]
fn a_revolution_need_not_begin_at_the_index() {
    let mut drive = open_drive();
    println!("{}", drive.describe());
    drive.motor(true).expect("spin up");

    for (cylinder, head) in [(0u8, Head::Lower), (40, Head::Upper), (79, Head::Lower)] {
        let track = track_number(cylinder, head);
        drive.seek(cylinder).expect("seek");
        drive.select_head(head).expect("head");
        let capture = drive.read_flux(3).expect("capture");

        // The rotation period, measured index to index.
        let aligned = capture.revolution(0).expect("a whole revolution");
        let period = aligned.ticks;

        // The index-aligned reading, for comparison.
        let cells = recover_cells(&aligned, capture.ticks_per_sec).expect("cells");
        let scan = amigados::scan_track(&cells.words, cells.bit_len as usize, Some(track));
        println!("track {track}: index-aligned {}", scan.summary());

        // ...and rings cut at arbitrary points instead, each exactly one
        // rotation long, as a read that never waited for an index would give.
        for fraction in [0.17, 0.41, 0.63, 0.88] {
            let start = aligned.ticks / 2 + (period as f64 * fraction) as u64;
            let end = start + period;
            let mut intervals = Vec::new();
            let mut at = 0u64;
            let mut last = start;
            for &interval in &capture.intervals {
                let previous = at;
                at += u64::from(interval);
                if at <= start {
                    continue;
                }
                if at > end {
                    break;
                }
                intervals.push((at - previous.max(start)) as u32);
                last = at;
            }
            let cut = copperline::fluxdrive::Revolution {
                intervals,
                ticks: period,
                trailing_ticks: end.saturating_sub(last),
            };
            let cells = recover_cells(&cut, capture.ticks_per_sec).expect("cells");
            let scan = amigados::scan_track(&cells.words, cells.bit_len as usize, Some(track));
            println!("           cut at {fraction:.2}: {}", scan.summary());
        }
    }
    drive.motor(false).expect("spin down");
}
