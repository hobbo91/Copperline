// SPDX-License-Identifier: GPL-3.0-or-later

//! The sector decoder, checked against an independently written encoder.
//!
//! Building a track the way a drive writes one and reading it back proves the
//! field offsets, the odd/even split and both checksums, without a disk.

use super::*;

/// Split a field's data bytes into the odd-bit half followed by the even-bit
/// half, which is how every AmigaDOS field is stored.
///
/// Clock bits are left clear: they keep a real stream self-clocking, but carry
/// no data and are masked out of both the decode and the checksums, so their
/// value cannot affect what is being tested here.
fn encode_block(data: &[u8]) -> Vec<u8> {
    let mut odd = Vec::with_capacity(data.len());
    let mut even = Vec::with_capacity(data.len());
    for &byte in data {
        let mut o = 0u8;
        let mut e = 0u8;
        for shift in [0u8, 2, 4, 6] {
            if byte & (1 << (shift + 1)) != 0 {
                o |= 1 << shift;
            }
            if byte & (1 << shift) != 0 {
                e |= 1 << shift;
            }
        }
        odd.push(o);
        even.push(e);
    }
    odd.extend(even);
    odd
}

/// XOR of a stored field's longs, masked to the data bits: what AmigaDOS
/// checksums.
fn stored_checksum(stored: &[u8]) -> u32 {
    assert!(stored.len().is_multiple_of(4));
    let mut sum = 0u32;
    for long in stored.chunks_exact(4) {
        sum ^= u32::from_be_bytes(long.try_into().unwrap());
    }
    sum & MFM_DATA_MASK
}

/// Deterministic but varied sector contents, so a mis-split of the odd and even
/// halves cannot pass by symmetry.
fn sector_payload(sector: u8) -> [u8; BYTES_PER_SECTOR] {
    let mut data = [0u8; BYTES_PER_SECTOR];
    for (i, byte) in data.iter_mut().enumerate() {
        *byte = (i as u8) ^ sector.wrapping_mul(0x3B) ^ 0x5A;
    }
    data
}

/// One sector as it lies on the disk: two sync words then the five fields.
fn encode_sector(track: u8, sector: u8, data: &[u8; BYTES_PER_SECTOR]) -> Vec<u8> {
    let togap = SECTORS_PER_TRACK as u8 - sector;
    let info = encode_block(&[AMIGADOS_FORMAT, track, sector, togap]);
    let label = encode_block(&[0u8; LABEL_BYTES]);

    let mut header = Vec::new();
    header.extend_from_slice(&info);
    header.extend_from_slice(&label);
    let header_checksum = encode_block(&stored_checksum(&header).to_be_bytes());

    let stored_data = encode_block(data);
    let data_checksum = encode_block(&stored_checksum(&stored_data).to_be_bytes());

    let mut out = Vec::with_capacity(4 + SECTOR_DISK_BYTES);
    out.extend_from_slice(&SYNC_WORD.to_be_bytes());
    out.extend_from_slice(&SYNC_WORD.to_be_bytes());
    out.extend_from_slice(&header);
    out.extend_from_slice(&header_checksum);
    out.extend_from_slice(&data_checksum);
    out.extend_from_slice(&stored_data);
    assert_eq!(out.len(), 4 + SECTOR_DISK_BYTES);
    out
}

/// A whole track: eleven sectors then the gap the drive leaves before the
/// write splice. Gap bytes are MFM idle, which holds no data and no sync.
fn encode_track(track: u8) -> Vec<u8> {
    let mut out = Vec::new();
    for sector in 0..SECTORS_PER_TRACK as u8 {
        out.extend_from_slice(&encode_sector(track, sector, &sector_payload(sector)));
    }
    out.resize(out.len() + 700, 0xAA);
    out
}

fn to_words(bytes: &[u8]) -> Vec<u16> {
    bytes
        .chunks(2)
        .map(|c| u16::from_be_bytes([c[0], *c.get(1).unwrap_or(&0)]))
        .collect()
}

/// Rotate a bit stream, putting the index somewhere else on the track. A real
/// revolution starts wherever the index hole is, which is nowhere in particular
/// relative to the sectors.
fn rotate_bits(bytes: &[u8], by: usize) -> (Vec<u16>, usize) {
    let bit_len = bytes.len() * 8;
    let bit = |i: usize| {
        let i = i % bit_len;
        bytes[i / 8] & (1 << (7 - (i % 8))) != 0
    };
    let mut words = vec![0u16; bit_len.div_ceil(16)];
    for i in 0..bit_len {
        if bit(i + by) {
            words[i / 16] |= 1 << (15 - (i % 16));
        }
    }
    (words, bit_len)
}

#[test]
fn a_whole_track_decodes_to_eleven_intact_sectors() {
    let track_number = 42u8;
    let bytes = encode_track(track_number);
    let words = to_words(&bytes);
    let scan = scan_track(&words, bytes.len() * 8, Some(track_number));

    assert_eq!(scan.intact(), SECTORS_PER_TRACK, "{}", scan.summary());
    assert!(scan.is_complete(), "{}", scan.summary());
    assert!(scan.damaged_spans.is_empty());
    assert_eq!(scan.sync_marks, SECTORS_PER_TRACK);

    for sector in &scan.sectors {
        assert_eq!(sector.track, track_number);
        assert_eq!(sector.togap, SECTORS_PER_TRACK as u8 - sector.sector);
        assert_eq!(
            sector.data,
            sector_payload(sector.sector),
            "sector {} data",
            sector.sector
        );
        assert_eq!(sector.label, [0u8; LABEL_BYTES]);
    }

    let assembled = scan.assemble().expect("complete track assembles");
    assert_eq!(assembled.len(), SECTORS_PER_TRACK);
    for (sector, data) in assembled.iter().enumerate() {
        assert_eq!(*data, sector_payload(sector as u8));
    }
}

#[test]
fn sectors_straddling_the_index_still_decode() {
    // The head reads a ring. Wherever the index falls, every sector must come
    // back -- including the one cut in half by it.
    let track_number = 8u8;
    let bytes = encode_track(track_number);
    for by in [1usize, 7, 1_000, 40_001, 95_000] {
        let (words, bit_len) = rotate_bits(&bytes, by);
        let scan = scan_track(&words, bit_len, Some(track_number));
        assert_eq!(
            scan.intact(),
            SECTORS_PER_TRACK,
            "rotated by {by} bits: {}",
            scan.summary()
        );
        assert!(scan.is_complete(), "rotated by {by} bits");
    }
}

#[test]
fn a_bad_data_byte_fails_only_its_own_sector() {
    // What marginal oxide looks like: the header reads, the payload does not
    // checksum. The rest of the track is unaffected, and a re-read of the same
    // track may well succeed, which is why nothing here retries.
    let track_number = 16u8;
    let mut bytes = encode_track(track_number);
    // Flip a bit inside sector 4's stored data.
    let sector_start = 4 * (4 + SECTOR_DISK_BYTES);
    bytes[sector_start + 4 + OFF_DATA + 100] ^= 0x40;

    let words = to_words(&bytes);
    let scan = scan_track(&words, bytes.len() * 8, Some(track_number));

    assert_eq!(scan.intact(), SECTORS_PER_TRACK - 1, "{}", scan.summary());
    assert!(!scan.is_complete());
    assert!(
        scan.assemble().is_none(),
        "an incomplete track assembles to nothing"
    );
    assert_eq!(scan.sync_marks, SECTORS_PER_TRACK, "the header still reads");
    assert_eq!(scan.damaged_spans.len(), 1);

    let broken: Vec<_> = scan.sectors.iter().filter(|s| !s.intact()).collect();
    assert_eq!(broken.len(), 1);
    assert_eq!(broken[0].sector, 4);
    assert!(broken[0].header_checksum_ok, "the header is undamaged");
    assert!(!broken[0].data_checksum_ok);
}

#[test]
fn a_header_naming_another_track_is_not_a_sector() {
    // Either the sync matched inside data, or the head is not where it is
    // believed to be. Neither is a sector on this track.
    let bytes = encode_track(42);
    let words = to_words(&bytes);

    let scan = scan_track(&words, bytes.len() * 8, Some(43));
    assert_eq!(scan.intact(), 0);
    assert_eq!(
        scan.sync_marks, SECTORS_PER_TRACK,
        "the marks are still there"
    );

    // Told to accept any track, the same stream reads normally.
    let scan = scan_track(&words, bytes.len() * 8, None);
    assert_eq!(scan.intact(), SECTORS_PER_TRACK);
    assert!(scan.sectors.iter().all(|s| s.track == 42));
}

#[test]
fn a_corrupt_header_checksum_is_reported_not_hidden() {
    let track_number = 5u8;
    let mut bytes = encode_track(track_number);
    // Damage the stored header checksum of sector 0, leaving the info field --
    // and so the structural checks -- intact.
    let sector_start = 4 + OFF_HEADER_CHECKSUM;
    bytes[sector_start] ^= 0x10;

    let words = to_words(&bytes);
    let scan = scan_track(&words, bytes.len() * 8, Some(track_number));

    let sector0: Vec<_> = scan.sectors.iter().filter(|s| s.sector == 0).collect();
    assert_eq!(sector0.len(), 1, "the sector is still found");
    assert!(!sector0[0].header_checksum_ok);
    assert!(!sector0[0].intact());
    assert_eq!(scan.intact(), SECTORS_PER_TRACK - 1);
}

#[test]
fn a_stream_too_short_to_hold_a_sector_yields_nothing() {
    let scan = scan_track(&[0x4489, 0x4489], 32, None);
    assert_eq!(scan.sectors.len(), 0);
    assert_eq!(scan.intact(), 0);
    assert!(!scan.is_complete());
}

#[test]
fn odd_and_even_halves_are_not_interchangeable() {
    // Swapping the halves must not still decode: it would mean the split is
    // being read symmetrically, and every recovered byte would be wrong in a
    // way checksums might not catch.
    let data: [u8; 4] = [0x12, 0x34, 0x56, 0x78];
    let stored = encode_block(&data);
    let (odd, even) = stored.split_at(4);

    let straight = to_words(&stored);
    let track = MfmTrack::new(&straight, stored.len() * 8);
    assert_eq!(track.decode_long(0), u32::from_be_bytes(data));

    let mut swapped = Vec::new();
    swapped.extend_from_slice(even);
    swapped.extend_from_slice(odd);
    let swapped_words = to_words(&swapped);
    let track = MfmTrack::new(&swapped_words, swapped.len() * 8);
    assert_ne!(track.decode_long(0), u32::from_be_bytes(data));
}
