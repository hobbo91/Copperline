// SPDX-License-Identifier: GPL-3.0-or-later

//! Turning a captured revolution into MFM cells.
//!
//! The interface hands back intervals in its own sample ticks; the data
//! separator wants nanoseconds. That conversion is the whole of this module,
//! and it is done in floating point on purpose: a 72 MHz flux timer ticks every
//! 13.9 ns, so rounding a tick to whole nanoseconds would throw away six
//! percent of the interval and drag every recovered cell off its window.
//!
//! Cell recovery itself is not done here. It is [`crate::floppy`]'s separator,
//! the same one that reads flux disk images, so a physical disk and a captured
//! one are decoded by identical code.

use super::Revolution;
use crate::floppy::{flux_to_mfm_cells, FluxCells};
use anyhow::{ensure, Result};

/// MFM cells recovered from one revolution, with the timing each was measured
/// at.
pub struct RecoveredTrack {
    pub words: Vec<u16>,
    pub bit_len: u32,
    /// Measured duration of each cell. The head is paced by these, so a disk
    /// running off nominal is read at the rate it is really turning.
    pub bitcell_ns: Vec<u32>,
    /// How long the revolution took, index to index.
    pub revolution_ns: f64,
}

impl RecoveredTrack {
    /// Mean of the cell times the separator actually measured.
    ///
    /// On a healthy DD disk this lands near 2 us, and how far off it lands is
    /// the disk's real rate. Deliberately averaged over the measurements rather
    /// than derived from the revolution length and the cell count: those two
    /// were used to fix each other, so their ratio would report the nominal
    /// figure back whatever the disk was doing.
    pub fn mean_cell_ns(&self) -> f64 {
        if self.bitcell_ns.is_empty() {
            return 0.0;
        }
        let total: u64 = self.bitcell_ns.iter().map(|&ns| u64::from(ns)).sum();
        total as f64 / self.bitcell_ns.len() as f64
    }
}

/// Recover MFM cells from one captured revolution.
///
/// How many cells the revolution holds is left to the flux to decide, and is
/// deliberately not computed from the rotation period and a nominal cell time.
/// A disk whose cells are 1.97 us rather than 2.00 holds nearly 1.5% more of
/// them than that sum predicts, and forcing the count either fabricates cells to
/// reach it or discards real ones to fit. Neither is visible in the middle of a
/// track -- but the head reads a ring, and a sector straddling the index has the
/// damage land inside it, so that sector fails its checksum on every revolution,
/// looking exactly like a physical defect that no amount of re-reading clears.
///
/// The separator's own measurements carry the timing instead: each cell records
/// how long it was measured to be, and those sum to the rotation period without
/// anything being assumed about the rate.
pub fn recover_cells(revolution: &Revolution, ticks_per_sec: u32) -> Result<RecoveredTrack> {
    ensure!(
        ticks_per_sec > 0,
        "a capture with no sample rate cannot be timed"
    );
    ensure!(
        revolution.ticks > 0,
        "a revolution of no duration holds no cells"
    );

    let ns_per_tick = 1.0e9 / f64::from(ticks_per_sec);
    let revolution_ns = revolution.ticks as f64 * ns_per_tick;

    let intervals_ns = revolution
        .intervals
        .iter()
        .map(|&ticks| f64::from(ticks) * ns_per_tick);
    let trailing_ns = revolution.trailing_ticks as f64 * ns_per_tick;

    let FluxCells {
        words,
        bit_len,
        bitcell_ns,
    } = flux_to_mfm_cells(intervals_ns, trailing_ns, None)?;

    Ok(RecoveredTrack {
        words,
        bit_len,
        bitcell_ns,
        revolution_ns,
    })
}
