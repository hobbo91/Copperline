// SPDX-License-Identifier: GPL-3.0-or-later

//! Hardware probes for the flux-level floppy path.
//!
//! These need a flux interface with a drive and a disk in it, so they are
//! ignored by default:
//!
//! ```sh
//! cargo test --release --test fluxdrive_hw -- --ignored --nocapture
//! ```
//!
//! `COPPERLINE_FLUXDRIVE_PORT` names the serial port, or is left unset to pick
//! the only interface attached. `COPPERLINE_FLUXDRIVE_DRIVE` selects the drive
//! on the cable (`a`/`b`, or `0`..`3` for Shugart); it defaults to the position
//! a lone drive on a PC cable ends up in.

#![cfg(feature = "fluxdrive")]

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
