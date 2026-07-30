// SPDX-License-Identifier: GPL-3.0-or-later

//! The flux stream codec and revolution carving, exercised without a drive.
//!
//! Everything between the wire and recovered cells is pure arithmetic, so it is
//! testable on its own. That is the point of taking flux rather than decoded
//! words: what used to need a disk in a drive is now a unit test.

use super::greaseweazle::{decode_flux_stream, BusType, DriveSelect};
use super::FluxCapture;

/// Ticks per second of a V4-class flux timer. Any rate does, since nothing in
/// the decoder assumes one; this just keeps the numbers recognisable.
const TICKS_PER_SEC: u32 = 72_000_000;

/// Build a stream the way a board does, so the decoder is checked against the
/// documented encoding rather than against itself.
#[derive(Default)]
struct StreamBuilder {
    bytes: Vec<u8>,
}

impl StreamBuilder {
    /// A flux reversal `ticks` after the previous one.
    fn flux(mut self, ticks: u32) -> Self {
        assert!(ticks > 0, "a reversal cannot be zero ticks after the last");
        if ticks < 250 {
            self.bytes.push(ticks as u8);
        } else {
            let high = (ticks - 250) / 255;
            assert!(high < 5, "use space() for intervals this long");
            self.bytes.push(250 + high as u8);
            self.bytes.push(1 + ((ticks - 250) % 255) as u8);
        }
        self
    }

    fn opcode(mut self, opcode: u8, value: u32) -> Self {
        self.bytes.push(0xFF);
        self.bytes.push(opcode);
        for shift in [0u32, 6, 13, 20] {
            let byte = if shift == 0 {
                1 | ((value << 1) & 0xFF) as u8
            } else {
                1 | ((value >> shift) & 0xFF) as u8
            };
            self.bytes.push(byte);
        }
        self
    }

    fn index(self, ticks_after_last_flux: u32) -> Self {
        self.opcode(1, ticks_after_last_flux)
    }

    fn space(self, ticks: u32) -> Self {
        self.opcode(2, ticks)
    }

    fn finish(mut self) -> Vec<u8> {
        self.bytes.push(0);
        self.bytes
    }
}

#[test]
fn single_byte_intervals_decode_to_themselves() {
    let stream = StreamBuilder::default().flux(4).flux(8).flux(249).finish();
    let capture = decode_flux_stream(&stream, TICKS_PER_SEC).expect("decodes");
    assert_eq!(capture.intervals, vec![4, 8, 249]);
    assert_eq!(capture.ticks_per_sec, TICKS_PER_SEC);
    assert!(capture.index_ticks.is_empty());
}

#[test]
fn two_byte_intervals_round_trip() {
    // Either side of the single-byte ceiling, and across a high-byte step.
    for ticks in [250u32, 251, 300, 504, 505, 600, 1000, 1523] {
        let stream = StreamBuilder::default().flux(ticks).finish();
        let capture = decode_flux_stream(&stream, TICKS_PER_SEC).expect("decodes");
        assert_eq!(capture.intervals, vec![ticks], "interval of {ticks} ticks");
    }
}

#[test]
fn space_carries_ticks_into_the_next_reversal() {
    // A long gap is expressed as space plus the reversal that ends it; the
    // decoded interval is the whole span, because that is what the head saw.
    let stream = StreamBuilder::default().space(100_000).flux(40).finish();
    let capture = decode_flux_stream(&stream, TICKS_PER_SEC).expect("decodes");
    assert_eq!(capture.intervals, vec![100_040]);
}

#[test]
fn index_is_timestamped_without_breaking_the_interval_it_falls_in() {
    // The head keeps reading across the index hole, so a reversal spanning it
    // stays one interval and the pulse is recorded as a position in time.
    let stream = StreamBuilder::default()
        .flux(100)
        .index(5)
        .flux(200)
        .finish();
    let capture = decode_flux_stream(&stream, TICKS_PER_SEC).expect("decodes");
    assert_eq!(capture.intervals, vec![100, 200]);
    assert_eq!(capture.index_ticks, vec![105]);
    assert_eq!(capture.total_ticks(), 300);
}

#[test]
fn index_offset_counts_from_accumulated_space_too() {
    let stream = StreamBuilder::default()
        .flux(100)
        .space(1_000)
        .index(7)
        .flux(50)
        .finish();
    let capture = decode_flux_stream(&stream, TICKS_PER_SEC).expect("decodes");
    // The pulse is 1_007 ticks past the reversal at 100.
    assert_eq!(capture.index_ticks, vec![1_107]);
    // ...and the interval still spans it, whole.
    assert_eq!(capture.intervals, vec![100, 1_050]);
}

#[test]
fn twenty_eight_bit_values_survive_the_full_range() {
    for value in [0u32, 1, 127, 128, 16_383, 16_384, 1 << 20, (1 << 28) - 1] {
        let stream = StreamBuilder::default().space(value).flux(10).finish();
        let capture = decode_flux_stream(&stream, TICKS_PER_SEC).expect("decodes");
        assert_eq!(
            capture.intervals,
            vec![value + 10],
            "28-bit value {value} round-trips"
        );
    }
}

#[test]
fn revolutions_are_carved_at_the_index_and_conserve_time() {
    let capture = FluxCapture {
        ticks_per_sec: TICKS_PER_SEC,
        intervals: vec![100, 100, 100, 100],
        index_ticks: vec![150, 350],
    };
    assert_eq!(capture.revolutions(), 1);
    let rev = capture.revolution(0).expect("one whole revolution");
    // The interval straddling the opening index contributes only its tail.
    assert_eq!(rev.intervals, vec![50, 100]);
    assert_eq!(rev.ticks, 200);
    assert_eq!(rev.trailing_ticks, 50);
    // A revolution's parts must add up to the rotation period, or the
    // recovered cell rate would drift against the disk.
    let counted: u64 = rev.intervals.iter().map(|&t| u64::from(t)).sum();
    assert_eq!(counted + rev.trailing_ticks, rev.ticks);
    assert!(
        capture.revolution(1).is_none(),
        "no second whole revolution"
    );
}

#[test]
fn a_capture_yields_one_fewer_revolution_than_index_pulses() {
    // The head starts wherever the disk has reached, so the flux before the
    // first pulse is a partial track and is not offered as a revolution.
    let capture = FluxCapture {
        ticks_per_sec: TICKS_PER_SEC,
        intervals: vec![10; 100],
        index_ticks: vec![100, 400, 700, 1000],
    };
    assert_eq!(capture.revolutions(), 3);
    for rev in 0..3 {
        let carved = capture.revolution(rev).expect("whole revolution");
        assert_eq!(carved.ticks, 300);
        let counted: u64 = carved.intervals.iter().map(|&t| u64::from(t)).sum();
        assert_eq!(counted + carved.trailing_ticks, carved.ticks);
    }
    assert!(capture.revolution(3).is_none());
}

#[test]
fn revolution_seconds_report_the_measured_rotation_period() {
    // A drive turning at exactly 300 rpm puts 200 ms between index pulses.
    let nominal = u64::from(TICKS_PER_SEC) / 5;
    let capture = FluxCapture {
        ticks_per_sec: TICKS_PER_SEC,
        intervals: Vec::new(),
        index_ticks: vec![0, nominal, 2 * nominal],
    };
    let periods = capture.revolution_seconds();
    assert_eq!(periods.len(), 2);
    for period in periods {
        assert!(
            (period - 0.2).abs() < 1.0e-9,
            "{period} s is not the nominal 200 ms"
        );
    }
}

#[test]
fn a_stream_must_end_with_its_terminator() {
    assert!(decode_flux_stream(&[], TICKS_PER_SEC).is_err(), "empty");
    assert!(
        decode_flux_stream(&[10, 20], TICKS_PER_SEC).is_err(),
        "unterminated"
    );
}

#[test]
fn truncated_escapes_are_rejected_rather_than_guessed() {
    // A stream cut off mid-opcode carries no usable timing; decoding it as far
    // as it goes would silently shorten the track.
    assert!(
        decode_flux_stream(&[0xFF, 0], TICKS_PER_SEC).is_err(),
        "no opcode"
    );
    assert!(
        decode_flux_stream(&[0xFF, 1, 1, 1, 0], TICKS_PER_SEC).is_err(),
        "short 28-bit value"
    );
    assert!(
        decode_flux_stream(&[250, 0], TICKS_PER_SEC).is_err(),
        "two-byte interval missing its low byte"
    );
    assert!(
        decode_flux_stream(&[0xFF, 99, 1, 1, 1, 1, 0], TICKS_PER_SEC).is_err(),
        "unknown opcode"
    );
}

#[test]
fn drive_select_spells_positions_the_way_the_cable_does() {
    let a = DriveSelect::parse("a").expect("PC drive A");
    assert_eq!(a.bus, BusType::IbmPc);
    assert_eq!(a.unit, 0);
    let b = DriveSelect::parse("B").expect("case is not significant");
    assert_eq!(b.bus, BusType::IbmPc);
    assert_eq!(b.unit, 1);
    for unit in 0..4u8 {
        let shugart = DriveSelect::parse(&unit.to_string()).expect("Shugart unit");
        assert_eq!(shugart.bus, BusType::Shugart);
        assert_eq!(shugart.unit, unit);
    }
    assert!(DriveSelect::parse("c").is_err());
    assert!(DriveSelect::parse("4").is_err());
    assert!(DriveSelect::parse("").is_err());
}
