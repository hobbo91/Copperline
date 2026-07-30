// SPDX-License-Identifier: GPL-3.0-or-later

//! AmigaDOS sector decoding, for checking recovered cells.
//!
//! Paula decodes nothing: it DMAs raw MFM words into chip RAM and the guest's
//! trackdisk driver picks sectors out of them in software. This does the same
//! job, for one purpose only -- telling whether a track came off the disk
//! intact. It is a measuring instrument, not part of the read path: turning
//! recovered cells into "11 of 11 sectors, checksums good" takes under a
//! millisecond, where judging the same thing by booting a guest takes a minute
//! and a half.
//!
//! A sector on disk is two `0x4489` sync words followed by five fields, each
//! MFM-encoded with its data bits split into an odd-bit half and an even-bit
//! half stored one after the other:
//!
//! | field           | data bytes | on disk |
//! |-----------------|-----------:|--------:|
//! | info            |          4 |       8 |
//! | label           |         16 |      32 |
//! | header checksum |          4 |       8 |
//! | data checksum   |          4 |       8 |
//! | data            |        512 |    1024 |
//!
//! `info` carries the format byte (`0xFF`), the track (cylinder times two, plus
//! the head), the sector number, and how many sectors remain before the gap.
//! Because the gap is the only fixed point on the track, sector 0 is not where
//! the index is -- a track is read from wherever the head happens to be and
//! reassembled from the sector numbers.

/// MFM interleaves data bits with clock bits; the data bits are the odd ones.
const MFM_DATA_MASK: u32 = 0x5555_5555;
const MFM_DATA_MASK_BYTE: u8 = 0x55;

/// The sync word every AmigaDOS sector is introduced by, twice. Its bit pattern
/// cannot occur in encoded data, which is what makes it findable in a stream the
/// reader has no byte alignment for.
pub const SYNC_WORD: u16 = 0x4489;

pub const SECTORS_PER_TRACK: usize = 11;
pub const BYTES_PER_SECTOR: usize = 512;
const LABEL_BYTES: usize = 16;
/// What a sector occupies on disk after its sync words.
const SECTOR_DISK_BYTES: usize = 8 + 32 + 8 + 8 + 1024;
/// Field offsets after the sync words, in disk bytes.
const OFF_INFO: usize = 0;
const OFF_LABEL: usize = 8;
const OFF_HEADER_CHECKSUM: usize = 40;
const OFF_DATA_CHECKSUM: usize = 48;
const OFF_DATA: usize = 56;
/// The header checksum covers info and label as stored: ten longs.
const HEADER_CHECKSUM_LONGS: usize = 10;
/// The data checksum covers the stored data area: 256 longs.
const DATA_CHECKSUM_LONGS: usize = 256;
/// Format byte of a standard AmigaDOS sector.
const AMIGADOS_FORMAT: u8 = 0xFF;

/// A packed MFM bit stream that wraps at the index, addressed by bit.
///
/// The head reads a ring: a sector may straddle the index, so every read wraps
/// rather than stopping at the end of the buffer.
#[derive(Clone, Copy)]
pub struct MfmTrack<'a> {
    words: &'a [u16],
    bit_len: usize,
}

impl<'a> MfmTrack<'a> {
    pub fn new(words: &'a [u16], bit_len: usize) -> Self {
        Self {
            words,
            bit_len: bit_len.min(words.len() * 16),
        }
    }

    pub fn bit_len(&self) -> usize {
        self.bit_len
    }

    fn bit(&self, bit: usize) -> bool {
        if self.bit_len == 0 {
            return false;
        }
        let bit = bit % self.bit_len;
        self.words[bit / 16] & (1 << (15 - (bit % 16))) != 0
    }

    fn word_at(&self, bit: usize) -> u16 {
        let mut value = 0u16;
        for offset in 0..16 {
            value = (value << 1) | u16::from(self.bit(bit + offset));
        }
        value
    }

    fn byte_at(&self, bit: usize) -> u8 {
        let mut value = 0u8;
        for offset in 0..8 {
            value = (value << 1) | u8::from(self.bit(bit + offset));
        }
        value
    }

    fn long_at(&self, bit: usize) -> u32 {
        let mut value = 0u32;
        for offset in 0..32 {
            value = (value << 1) | u32::from(self.bit(bit + offset));
        }
        value
    }

    /// Recover a field's data bytes from its two stored halves.
    fn decode_block(&self, bit: usize, len: usize, out: &mut [u8]) {
        for (i, byte) in out.iter_mut().enumerate().take(len) {
            let odd = self.byte_at(bit + i * 8);
            let even = self.byte_at(bit + (len + i) * 8);
            *byte = ((odd & MFM_DATA_MASK_BYTE) << 1) | (even & MFM_DATA_MASK_BYTE);
        }
    }

    fn decode_long(&self, bit: usize) -> u32 {
        let odd = self.long_at(bit);
        let even = self.long_at(bit + 32);
        ((odd & MFM_DATA_MASK) << 1) | (even & MFM_DATA_MASK)
    }

    /// XOR of the stored longs, masked to the data bits: the checksum AmigaDOS
    /// computes over a field as it lies on the disk.
    fn stored_checksum(&self, bit: usize, longs: usize) -> u32 {
        let mut sum = 0u32;
        for i in 0..longs {
            sum ^= self.long_at(bit + i * 32);
        }
        sum & MFM_DATA_MASK
    }
}

/// One sector recovered from the stream.
#[derive(Clone)]
pub struct Sector {
    /// Track as the header states it: cylinder times two, plus the head.
    pub track: u8,
    pub sector: u8,
    /// Sectors remaining before the track gap.
    pub togap: u8,
    pub label: [u8; LABEL_BYTES],
    pub data: [u8; BYTES_PER_SECTOR],
    pub header_checksum_ok: bool,
    pub data_checksum_ok: bool,
    /// Where the sector's fields begin, as a bit offset into the revolution.
    pub start_bit: usize,
}

impl Sector {
    pub fn intact(&self) -> bool {
        self.header_checksum_ok && self.data_checksum_ok
    }
}

/// What a scan of one revolution found.
pub struct TrackScan {
    pub sectors: Vec<Sector>,
    /// Sync marks seen, whether or not a usable sector followed. A track with
    /// eleven marks but ten good sectors is ordinary marginal media; one with
    /// no marks at all points at the timebase or the head.
    pub sync_marks: usize,
    /// Bit spans of sectors whose header read but whose data failed its
    /// checksum -- where the disk is actually weak.
    pub damaged_spans: Vec<(usize, usize)>,
}

impl TrackScan {
    /// Sectors that passed both checksums.
    pub fn intact(&self) -> usize {
        self.sectors.iter().filter(|s| s.intact()).count()
    }

    /// Whether every sector of a standard AmigaDOS track is present and intact,
    /// each exactly once.
    pub fn is_complete(&self) -> bool {
        let mut seen = [false; SECTORS_PER_TRACK];
        for sector in self.sectors.iter().filter(|s| s.intact()) {
            let Some(slot) = seen.get_mut(usize::from(sector.sector)) else {
                continue;
            };
            if *slot {
                return false;
            }
            *slot = true;
        }
        seen.iter().all(|&s| s)
    }

    /// The track's 512-byte sectors in sector order, if the track is complete.
    pub fn assemble(&self) -> Option<Vec<[u8; BYTES_PER_SECTOR]>> {
        if !self.is_complete() {
            return None;
        }
        let mut out = vec![[0u8; BYTES_PER_SECTOR]; SECTORS_PER_TRACK];
        for sector in self.sectors.iter().filter(|s| s.intact()) {
            out[usize::from(sector.sector)] = sector.data;
        }
        Some(out)
    }

    pub fn summary(&self) -> String {
        let mut summary = format!(
            "{}/{} sectors, {} sync marks",
            self.intact(),
            SECTORS_PER_TRACK,
            self.sync_marks
        );
        for (start, end) in &self.damaged_spans {
            summary.push_str(&format!(" [bad {start}..{end}]"));
        }
        summary
    }
}

/// Find and decode every AmigaDOS sector in one revolution of MFM cells.
///
/// `expect_track` is the track the head is over, so a header naming a different
/// one can be rejected: it means the sync matched inside data, or the head is
/// not where it is believed to be. Pass `None` to accept whatever the headers
/// say.
///
/// Sectors are found by their sync words rather than by counting from the index,
/// because nothing guarantees where a revolution starts relative to the gap.
pub fn scan_track(words: &[u16], bit_len: usize, expect_track: Option<u8>) -> TrackScan {
    let track = MfmTrack::new(words, bit_len);
    let mut scan = TrackScan {
        sectors: Vec::new(),
        sync_marks: 0,
        damaged_spans: Vec::new(),
    };
    if track.bit_len() < SECTOR_DISK_BYTES * 8 {
        return scan;
    }

    // Field positions already decoded, so the sector straddling the index is
    // not counted twice. A scan starts wherever the index fell, which can be
    // between a sector's two sync words: the second mark alone is enough to
    // find the sector, and the first is then met again on coming round to it.
    let mut decoded_at: Vec<usize> = Vec::new();

    let mut bit = 0usize;
    while bit < track.bit_len() {
        if track.word_at(bit) != SYNC_WORD {
            bit += 1;
            continue;
        }
        // A sector is introduced by two sync words, and the gap ahead of the
        // first sector written can hold more. The fields start after the last.
        let mut fields = bit;
        while track.word_at(fields) == SYNC_WORD {
            fields += 16;
        }
        let position = fields % track.bit_len();
        // Whether found or not, there is no sector inside a sector, so resume
        // past where this one would end.
        let resume = fields + SECTOR_DISK_BYTES * 8;

        if decoded_at.contains(&position) {
            bit = resume;
            continue;
        }
        scan.sync_marks += 1;

        if let Some(sector) = decode_sector(&track, fields, expect_track) {
            if !sector.data_checksum_ok {
                scan.damaged_spans
                    .push((position, position + SECTOR_DISK_BYTES * 8));
            }
            scan.sectors.push(sector);
            decoded_at.push(position);
            bit = resume;
        } else {
            // Not a sector header: the pattern matched inside data. Step past
            // the marks and keep looking.
            bit = fields;
        }
    }
    scan
}

/// Decode the sector whose fields begin at `bit`, or reject it.
///
/// The header is checked structurally before its checksum is trusted, so a
/// stray sync match inside data is discarded rather than reported as a damaged
/// sector.
fn decode_sector(track: &MfmTrack, bit: usize, expect_track: Option<u8>) -> Option<Sector> {
    let info = track.decode_long(bit + OFF_INFO * 8);
    let [format, track_number, sector, togap] = info.to_be_bytes();

    if format != AMIGADOS_FORMAT
        || usize::from(sector) >= SECTORS_PER_TRACK
        || !(1..=SECTORS_PER_TRACK as u8).contains(&togap)
    {
        return None;
    }
    if let Some(expected) = expect_track {
        if track_number != expected {
            return None;
        }
    }

    let header_checksum = track.decode_long(bit + OFF_HEADER_CHECKSUM * 8);
    let header_computed = track.stored_checksum(bit + OFF_INFO * 8, HEADER_CHECKSUM_LONGS);

    let data_checksum = track.decode_long(bit + OFF_DATA_CHECKSUM * 8);
    let data_computed = track.stored_checksum(bit + OFF_DATA * 8, DATA_CHECKSUM_LONGS);

    let mut label = [0u8; LABEL_BYTES];
    track.decode_block(bit + OFF_LABEL * 8, LABEL_BYTES, &mut label);
    let mut data = [0u8; BYTES_PER_SECTOR];
    track.decode_block(bit + OFF_DATA * 8, BYTES_PER_SECTOR, &mut data);

    Some(Sector {
        track: track_number,
        sector,
        togap,
        label,
        data,
        header_checksum_ok: header_checksum == header_computed,
        data_checksum_ok: data_checksum == data_computed,
        start_bit: bit,
    })
}

#[cfg(test)]
mod tests;
