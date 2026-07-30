// SPDX-License-Identifier: GPL-3.0-or-later

//! A physical drive driven from the emulated machine, without stalling it.
//!
//! Reading a track off a real disk costs at least one rotation -- around 200 ms,
//! and more for several revolutions -- which is an eternity to a machine being
//! emulated in step with its own clocks. So the drive lives on its own thread:
//! the emulated side asks for the track under the head and carries on, and
//! collects the flux when it arrives.
//!
//! What the emulated machine does in the meantime is turn the platter with
//! nothing readable on it, which is what a real drive does while the head is
//! over a part of the disk it has not reached yet. The alternative -- stopping
//! the platter until the capture lands -- would make the guest wait out the
//! capture and then its own rotational latency afterwards, one after the other.
//!
//! Head position and motor state are not decided here. They follow the guest's
//! stepper and CIA-B writes, so this thread only ever does as it is told.

use super::cells::{recover_cells, RecoveredTrack};
use super::{FluxSource, Head};
use anyhow::{anyhow, Result};
use log::{debug, warn};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::sync::Arc;
use std::thread::JoinHandle;

/// What the emulated side asks the drive to do.
enum Command {
    Motor(bool),
    /// The guest has stepped its head to this cylinder; follow it there.
    ///
    /// Sent only while the drive is empty, where the guest steps a track in and
    /// out about once a second to poll for a disk. Each is carried out as asked,
    /// because each is a click: collapsing a pair into "where the head ended up"
    /// loses the movement between them, and with it the sound. There is nothing
    /// to starve, since a drive with no disk in it has nothing to read.
    Seek(u8),
    Capture {
        cylinder: u8,
        head: Head,
        revolutions: u8,
    },
    /// Read the drive's status lines.
    Status,
    /// Try to read a little flux purely to find out whether a disk is in there.
    Probe,
    Stop,
}

/// One track's worth of flux, recovered into cells.
pub struct CapturedTrack {
    pub cylinder: u8,
    pub head: Head,
    /// Each whole index-to-index revolution, decoded on its own. A marginal
    /// sector often reads on one and not another, which is what lets the
    /// guest's own re-read recover it.
    pub revolutions: Vec<RecoveredTrack>,
}

/// What comes back from the drive's thread.
enum Event {
    Captured(Box<CapturedTrack>),
    Status(super::DriveStatus),
    /// Whether a probe found a disk in the drive.
    Probed(bool),
    Failed {
        cylinder: u8,
        head: Head,
        error: String,
        /// The failure was "no index pulse", which is what an empty drive looks
        /// like: nothing is turning for the sensor to see.
        no_disk: bool,
    },
}

/// A physical drive, asked for tracks and answered asynchronously.
pub struct FluxDrive {
    commands: Sender<Command>,
    events: Receiver<Event>,
    worker: Option<JoinHandle<()>>,
    description: String,
    /// The capture in flight, if any. Only one is ever outstanding: the
    /// emulated side asks for the track under the head thousands of times a
    /// second, and every one of those must not become a queued rotation.
    pending: Option<(u8, Head)>,
    motor_on: bool,
    /// Whether a disk has been established to be in the drive. `None` until
    /// something is read or fails in a way that settles it, because the floppy
    /// bus never says outright.
    disk_present: Option<bool>,
    /// The disk's own write-protect tab, as the drive senses it.
    write_protected: Option<bool>,
    /// A status read is outstanding, so another would only queue behind it.
    status_pending: bool,
    probe_pending: bool,
    /// Whether this drive's change line is worth reading for presence.
    trust_change_pin: bool,
    /// The cylinder the real head was last sent to, so the guest's steps are
    /// forwarded once each rather than repeatedly.
    sent_cylinder: Option<u8>,
    /// Set when the thread has gone, so the bay can stop asking.
    lost: bool,
    stopping: Arc<AtomicBool>,
}

impl FluxDrive {
    /// Take over a drive and put it on its own thread.
    pub fn attach(source: Box<dyn FluxSource + Send>, trust_change_pin: bool) -> Self {
        Self::spawn(source, trust_change_pin)
    }

    fn spawn(mut source: Box<dyn FluxSource + Send>, trust_change_pin: bool) -> Self {
        let description = source.describe();
        let (commands, command_rx) = std::sync::mpsc::channel::<Command>();
        let (event_tx, events) = std::sync::mpsc::channel::<Event>();
        // Set the moment the machine is on its way out, so the drive stops as
        // promptly as a real one does instead of finishing everything it was
        // asked for first.
        let stopping = Arc::new(AtomicBool::new(false));
        let worker_stopping = Arc::clone(&stopping);

        let worker = std::thread::Builder::new()
            .name("fluxdrive".to_string())
            .spawn(move || {
                // The drive's own motor state, so a probe can put it back.
                let mut motor_running = false;
                while let Ok(command) = command_rx.recv() {
                    if worker_stopping.load(Ordering::Relaxed) {
                        break;
                    }
                    match command {
                        Command::Stop => break,
                        Command::Motor(on) => {
                            if let Err(err) = source.motor(on) {
                                warn!("fluxdrive: cannot switch the drive motor: {err:#}");
                            } else {
                                motor_running = on;
                            }
                        }
                        Command::Seek(cylinder) => {
                            // A step that will not go through is not worth
                            // reporting every time: the guest steps a drive it
                            // believes is at the end stop quite deliberately.
                            if let Err(err) = source.seek(cylinder) {
                                debug!("fluxdrive: cannot step to cylinder {cylinder}: {err:#}");
                            }
                        }
                        Command::Probe => {
                            // The only reliable way to tell: try to read, and
                            // see whether an index pulse ever comes round. The
                            // motor has to be turning for that, so spin it and
                            // put it back as it was.
                            let was_running = motor_running;
                            if !was_running && source.motor(true).is_err() {
                                continue;
                            }
                            let present = source.read_flux(1).is_ok();
                            if !was_running {
                                let _ = source.motor(false);
                            }
                            if event_tx.send(Event::Probed(present)).is_err() {
                                break;
                            }
                        }
                        Command::Status => {
                            let event = match source.status() {
                                Ok(status) => Event::Status(status),
                                Err(err) => {
                                    debug!("fluxdrive: cannot read the drive status: {err:#}");
                                    continue;
                                }
                            };
                            if event_tx.send(event).is_err() {
                                break;
                            }
                        }
                        Command::Capture {
                            cylinder,
                            head,
                            revolutions,
                        } => {
                            let outcome = capture(
                                source.as_mut(),
                                cylinder,
                                head,
                                revolutions,
                                &worker_stopping,
                            );
                            let event = match outcome {
                                Ok(track) => Event::Captured(Box::new(track)),
                                Err(err) => {
                                    let no_disk = err
                                        .downcast_ref::<super::greaseweazle::CommandError>()
                                        .is_some_and(|e| e.means_no_disk());
                                    Event::Failed {
                                        cylinder,
                                        head,
                                        error: format!("{err:#}"),
                                        no_disk,
                                    }
                                }
                            };
                            // A closed channel means the machine is gone; stop
                            // touching the disk.
                            if event_tx.send(event).is_err() {
                                break;
                            }
                        }
                    }
                }
                // Leave the drive as it was found rather than spinning on.
                if let Err(err) = source.motor(false) {
                    debug!("fluxdrive: cannot stop the drive motor on shutdown: {err:#}");
                }
            })
            .expect("spawn the flux drive thread");

        Self {
            commands,
            events,
            worker: Some(worker),
            description,
            pending: None,
            motor_on: false,
            disk_present: None,
            write_protected: None,
            status_pending: false,
            probe_pending: false,
            trust_change_pin,
            sent_cylinder: None,
            lost: false,
            stopping,
        }
    }

    /// Note where the guest's head now is, so the real one follows.
    pub fn seek(&mut self, cylinder: u8) {
        if self.lost {
            return;
        }
        if self.sent_cylinder == Some(cylinder) {
            return;
        }
        if self.commands.send(Command::Seek(cylinder)).is_err() {
            self.lost = true;
            return;
        }
        self.sent_cylinder = Some(cylinder);
    }

    /// Try to find out whether a disk is in the drive by reading a little of it.
    ///
    /// Costs a rotation and spins the spindle, so it is for when nothing else
    /// can answer: the change line is a latch that reads "changed" for ever on
    /// many drives, and the guest will not spin a drive it believes is empty.
    pub fn probe_for_disk(&mut self) {
        if self.lost || self.probe_pending || self.pending.is_some() {
            return;
        }
        if self.commands.send(Command::Probe).is_err() {
            self.lost = true;
            return;
        }
        self.probe_pending = true;
    }

    /// Ask for the drive's status lines, unless a read is already outstanding.
    pub fn request_status(&mut self) {
        if self.lost || self.status_pending || self.pending.is_some() {
            return;
        }
        if self.commands.send(Command::Status).is_err() {
            self.lost = true;
            return;
        }
        self.status_pending = true;
    }

    /// The disk's write-protect tab, where the drive can report it.
    pub fn write_protected(&self) -> Option<bool> {
        self.write_protected
    }

    pub fn describe(&self) -> &str {
        &self.description
    }

    /// Whether a disk has been established to be in the drive, or `None` while
    /// that is still unknown.
    pub fn disk_present(&self) -> Option<bool> {
        self.disk_present
    }

    /// Whether finding a disk needs the drive spun up and read.
    ///
    /// False where the change line is believed: the guest's own polling clears
    /// that latch, so asking costs nothing and takes no time. Probing does both
    /// -- it holds the drive for most of a second while it spins and reads, and
    /// anything the guest asked for meanwhile waits behind it, which for a drive
    /// being polled means its steps arrive in a clump instead of evenly.
    pub fn needs_probe(&self) -> bool {
        !self.trust_change_pin
    }

    /// Whether the drive's thread has gone, in which case nothing more will
    /// come back from it.
    pub fn lost(&self) -> bool {
        self.lost
    }

    /// Spin the disk up or down, as the guest's CIA-B writes say.
    pub fn set_motor(&mut self, on: bool) {
        if self.motor_on == on || self.lost {
            return;
        }
        self.motor_on = on;
        if self.commands.send(Command::Motor(on)).is_err() {
            self.lost = true;
        }
    }

    pub fn motor_on(&self) -> bool {
        self.motor_on
    }

    /// Whether a capture is already in flight.
    pub fn busy(&self) -> bool {
        self.pending.is_some()
    }

    /// Ask for the track under the head.
    ///
    /// Does nothing while another capture is outstanding, or with the motor off:
    /// a stopped disk produces no flux. Returns whether the drive was actually
    /// asked.
    pub fn request(&mut self, cylinder: u8, head: Head, revolutions: u8) -> bool {
        if self.lost || self.pending.is_some() || !self.motor_on {
            return false;
        }
        let command = Command::Capture {
            cylinder,
            head,
            revolutions,
        };
        if self.commands.send(command).is_err() {
            self.lost = true;
            return false;
        }
        // The capture seeks on the drive's thread, so record where that leaves
        // the head or the next step would be thought already sent.
        self.sent_cylinder = Some(cylinder);
        self.pending = Some((cylinder, head));
        true
    }

    /// Collect a finished capture, if one has arrived. Never blocks.
    ///
    /// What comes back is whatever the drive was asked for, which is not
    /// necessarily the track under the head any more: the guest steps while the
    /// disk is turning. It is returned regardless and tagged with the track it
    /// belongs to, because those cells cost a rotation of real time to fetch and
    /// remain a perfectly good reading of that track whether or not the head is
    /// still over it.
    pub fn poll(&mut self) -> Option<Box<CapturedTrack>> {
        loop {
            match self.events.try_recv() {
                Ok(Event::Captured(track)) => {
                    self.pending = None;
                    // Flux came back, so something is turning in there.
                    self.disk_present = Some(true);
                    return Some(track);
                }
                Ok(Event::Probed(present)) => {
                    self.probe_pending = false;
                    if self.disk_present != Some(present) {
                        log::info!(
                            "fluxdrive: probe found {}",
                            if present { "a disk" } else { "no disk" }
                        );
                    }
                    self.disk_present = Some(present);
                }
                Ok(Event::Status(status)) => {
                    self.status_pending = false;
                    // Believed as it stands, both ways. A disk leaving asserts
                    // the line at once, and an Amiga acts on that immediately --
                    // waiting to be sure would put a pause between the disk
                    // coming out and the drive starting to click, which is not
                    // what the machine does. A disk going in is resolved by the
                    // guest's own polling: its steps clear the latch, and the
                    // probe is there for a drive that never clears it.
                    if self.trust_change_pin {
                        if let Some(present) = status.disk_present {
                            self.disk_present = Some(present);
                        }
                    }
                    if let Some(protected) = status.write_protected {
                        self.write_protected = Some(protected);
                    }
                }
                Ok(Event::Failed {
                    cylinder,
                    head,
                    error,
                    no_disk,
                }) => {
                    self.pending = None;
                    if no_disk {
                        self.disk_present = Some(false);
                    }
                    // Not fatal on its own: a disk still coming up to speed
                    // reads again next time.
                    debug!(
                        "fluxdrive: cylinder {cylinder} head {} did not read: {error}",
                        head.index()
                    );
                }
                Err(TryRecvError::Empty) => return None,
                Err(TryRecvError::Disconnected) => {
                    self.lost = true;
                    self.pending = None;
                    return None;
                }
            }
        }
    }
}

/// Seek, read, and recover cells, all on the drive's own thread so the emulated
/// machine never waits for a rotation.
fn capture(
    source: &mut (dyn FluxSource + Send),
    cylinder: u8,
    head: Head,
    revolutions: u8,
    stopping: &AtomicBool,
) -> Result<CapturedTrack> {
    if stopping.load(Ordering::Relaxed) {
        return Err(anyhow!("the machine is stopping"));
    }
    source.seek(cylinder)?;
    source.select_head(head)?;
    let flux = source.read_flux(revolutions)?;

    let recovered: Vec<RecoveredTrack> = (0..flux.revolutions())
        .filter_map(|rev| {
            let revolution = flux.revolution(rev)?;
            match recover_cells(&revolution, flux.ticks_per_sec) {
                Ok(cells) => Some(cells),
                Err(err) => {
                    debug!("fluxdrive: revolution {rev} yielded no cells: {err:#}");
                    None
                }
            }
        })
        .collect();

    if recovered.is_empty() {
        return Err(anyhow!(
            "cylinder {cylinder} head {} produced no usable revolution",
            head.index()
        ));
    }
    Ok(CapturedTrack {
        cylinder,
        head,
        revolutions: recovered,
    })
}

impl Drop for FluxDrive {
    fn drop(&mut self) {
        // Tell the thread to give up on anything it has not started, so the
        // longest this can wait is the one rotation already under way.
        self.stopping.store(true, Ordering::Relaxed);
        let _ = self.commands.send(Command::Stop);
        if let Some(worker) = self.worker.take() {
            // A capture already under way finishes first; it is one rotation,
            // and abandoning the thread mid-command would leave the interface
            // half-way through a command with the port still open.
            if worker.join().is_err() {
                warn!("fluxdrive: the drive thread did not shut down cleanly");
            }
        }
    }
}

impl std::fmt::Debug for FluxDrive {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FluxDrive")
            .field("drive", &self.description)
            .field("motor_on", &self.motor_on)
            .field("pending", &self.pending)
            .field("lost", &self.lost)
            .finish()
    }
}
