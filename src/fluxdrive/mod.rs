// SPDX-License-Identifier: GPL-3.0-or-later

//! Physical 3.5" floppy drives at flux level.
//!
//! A real Amiga drive does not hand Paula bytes, or even bits. The head is a
//! coil: as the disk turns, each reversal of magnetisation under it induces a
//! pulse, and the drive electronics present those pulses on `/RDATA`. All the
//! information on the disk is in the *timing between pulses*. Paula's data
//! separator measures those intervals against its bit-cell clock and recovers
//! MFM cells; nothing upstream of Paula knows what a sector is.
//!
//! This module reproduces that arrangement with a real drive on the end of a
//! USB interface. An interface implements [`FluxSource`] to move the head and
//! hand back a [`FluxCapture`] -- the raw intervals between flux reversals, in
//! the interface's own sample ticks, with the index pulses marked. Recovering
//! cells from those intervals is Copperline's job, in one data separator shared
//! with flux disk images, so a physical disk and a captured one are decoded by
//! exactly the same code.
//!
//! Interfaces differ only in how the flux is fetched, which is why that is the
//! only thing behind the trait. [`greaseweazle`] is the first implementation.

use anyhow::Result;

pub mod amigados;
pub mod cells;
pub mod greaseweazle;

/// Which side of the disk the head is reading.
///
/// The Amiga's two heads are selected by `/SIDE1` on the drive bus, driven from
/// CIA-B's PRB. Side 0 is the lower head.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Head {
    #[default]
    Lower,
    Upper,
}

impl Head {
    /// The head number as the drive bus encodes it.
    pub fn index(self) -> u8 {
        match self {
            Head::Lower => 0,
            Head::Upper => 1,
        }
    }

    pub fn from_index(index: u8) -> Self {
        if index == 0 {
            Head::Lower
        } else {
            Head::Upper
        }
    }
}

/// What the interface can currently say about the drive.
///
/// The floppy bus is sparser than it looks: there is no line that means "a disk
/// is present", and write protection is sensed by the drive refusing a write
/// rather than by anything an interface can always read back. Fields that
/// cannot be established without disturbing the disk are `Option`, and `None`
/// means "not knowable right now", never "false".
#[derive(Clone, Copy, Debug, Default)]
pub struct DriveStatus {
    /// Cylinder the head is over, if the interface has calibrated against
    /// `/TRK0` since it was opened.
    pub cylinder: Option<u8>,
    pub motor_on: bool,
    /// `/WRPROT` from the drive, where the interface can read the line.
    pub write_protected: Option<bool>,
}

/// Raw flux from one capture.
///
/// `intervals` holds the ticks between successive flux reversals, in
/// `ticks_per_sec` units -- the signal the drive puts on `/RDATA`, unquantised
/// and undecoded. `index_ticks` timestamps each index pulse on the same
/// timebase as the running sum of `intervals`, so a revolution is the flux
/// between two consecutive index marks.
///
/// A capture normally spans one more index-to-index revolution than was asked
/// for: the head is wherever it happens to be when the read starts, so the
/// leading partial revolution is not a whole track and is not counted.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FluxCapture {
    /// Sample rate of the interface's flux timer.
    pub ticks_per_sec: u32,
    /// Ticks between successive flux reversals.
    pub intervals: Vec<u32>,
    /// Cumulative tick timestamp of each index pulse.
    pub index_ticks: Vec<u64>,
}

/// One index-to-index revolution carved out of a [`FluxCapture`].
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Revolution {
    /// Ticks between successive flux reversals. The first entry is measured
    /// from the index pulse rather than from the preceding reversal, so the
    /// revolution begins exactly at the index.
    pub intervals: Vec<u32>,
    /// Exact index-to-index length. This is the disk's true rotation period as
    /// measured, which is what the bit-cell rate has to be derived from -- a
    /// drive running slightly off 300 rpm shifts every cell in the track.
    pub ticks: u64,
    /// Ticks from the last reversal in the revolution to the closing index
    /// pulse. No reversal happened in this window, so it carries no cell data,
    /// but it is part of the rotation period.
    pub trailing_ticks: u64,
}

impl FluxCapture {
    /// How many whole index-to-index revolutions the capture contains.
    pub fn revolutions(&self) -> usize {
        self.index_ticks.len().saturating_sub(1)
    }

    /// Total ticks of flux in the capture, index pulses aside.
    pub fn total_ticks(&self) -> u64 {
        self.intervals.iter().map(|&t| u64::from(t)).sum()
    }

    /// Carve out one whole revolution, `rev` counting from the first index
    /// pulse. Returns `None` past the last complete revolution.
    ///
    /// The reversal that closes a revolution and the one that opens the next
    /// are the same physical event only if it lands exactly on the index, so
    /// the boundary intervals are split at the index rather than assigned
    /// whole to either side. That keeps each revolution's ticks summing to the
    /// true rotation period, which is what the data separator needs.
    pub fn revolution(&self, rev: usize) -> Option<Revolution> {
        let start = *self.index_ticks.get(rev)?;
        let end = *self.index_ticks.get(rev + 1)?;
        if end <= start {
            return None;
        }
        let mut intervals = Vec::new();
        // Absolute timestamp of each reversal, walking the capture once.
        let mut at = 0u64;
        let mut last_in_rev = start;
        for &interval in &self.intervals {
            let previous = at;
            at += u64::from(interval);
            if at <= start {
                continue;
            }
            if at > end {
                break;
            }
            // The interval that straddles the opening index contributes only
            // the part after it.
            let from = previous.max(start);
            intervals.push((at - from) as u32);
            last_in_rev = at;
        }
        Some(Revolution {
            intervals,
            ticks: end - start,
            trailing_ticks: end - last_in_rev,
        })
    }

    /// Rotation period of each whole revolution, in seconds. A 3.5" drive
    /// nominally turns at 300 rpm, so these should sit near 200 ms; how far
    /// they actually sit from it is the drive's real speed error.
    pub fn revolution_seconds(&self) -> Vec<f64> {
        let ticks_per_sec = f64::from(self.ticks_per_sec.max(1));
        self.index_ticks
            .windows(2)
            .map(|w| (w[1] - w[0]) as f64 / ticks_per_sec)
            .collect()
    }
}

/// A physical drive on the end of an interface that can hand back raw flux.
///
/// The emulated machine owns head position and motor state: Copperline drives
/// these from CIA-B's PRB writes exactly as the guest's trackdisk driver steps
/// a real drive, so an implementation should do what it is told and not move
/// the head on its own initiative.
pub trait FluxSource {
    /// How to name this drive in logs and diagnostics.
    fn describe(&self) -> String;

    /// Step the head to `cylinder`, recalibrating against `/TRK0` if the
    /// interface needs to.
    fn seek(&mut self, cylinder: u8) -> Result<()>;

    /// Select which head reads.
    fn select_head(&mut self, head: Head) -> Result<()>;

    /// Spin the disk up or down. Flux cannot be read with the motor off, and a
    /// drive needs a moment at speed before its output is stable.
    fn motor(&mut self, on: bool) -> Result<()>;

    /// Capture at least `revolutions` whole index-to-index revolutions of flux
    /// from the current cylinder and head.
    fn read_flux(&mut self, revolutions: u8) -> Result<FluxCapture>;

    fn status(&mut self) -> Result<DriveStatus>;
}

#[cfg(test)]
mod tests;
