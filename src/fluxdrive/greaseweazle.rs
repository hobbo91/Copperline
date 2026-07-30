// SPDX-License-Identifier: GPL-3.0-or-later

//! Greaseweazle flux interface.
//!
//! The board is a USB device that sits on a PC floppy cable and times the
//! drive's `/RDATA` line with a hardware timer. Reads come back as the raw
//! intervals between flux reversals, which is what [`FluxSource`] exists to
//! deliver: no cell recovery happens on the board.
//!
//! The command set is stable across board revisions (F1, F7, V4, Adafruit
//! Floppy), but their timers and capabilities are not, so nothing here assumes
//! a particular board. `GetInfo` reports the flux timer's sample rate, the
//! highest command the firmware understands, and the hardware model; the tick
//! rate used to interpret every interval comes from that report, never from a
//! constant, and commands added in later firmware are gated on the reported
//! command ceiling rather than tried blind.
//!
//! The wire protocol is documented by the Greaseweazle project.

use super::{DriveStatus, FluxCapture, FluxSource, Head};
use anyhow::{anyhow, bail, ensure, Context, Result};
use log::{debug, info, warn};
use std::io::{ErrorKind, Read, Write};
use std::time::{Duration, Instant};

/// Commands, as the firmware numbers them.
mod cmd {
    pub const GET_INFO: u8 = 0;
    pub const SEEK: u8 = 2;
    pub const HEAD: u8 = 3;
    pub const SET_PARAMS: u8 = 4;
    pub const GET_PARAMS: u8 = 5;
    pub const MOTOR: u8 = 6;
    pub const READ_FLUX: u8 = 7;
    pub const GET_FLUX_STATUS: u8 = 9;
    pub const SELECT: u8 = 12;
    pub const DESELECT: u8 = 13;
    pub const SET_BUS_TYPE: u8 = 14;
    pub const SET_PIN: u8 = 15;
    pub const GET_PIN: u8 = 20;
    pub const NO_CLICK_STEP: u8 = 22;
}

/// `GetInfo` sub-indexes.
mod get_info {
    pub const FIRMWARE: u8 = 0;
    pub const CURRENT_DRIVE: u8 = 7;
}

/// `{Get,Set}Params` sub-indexes.
mod params {
    pub const DELAYS: u8 = 0;
}

/// Command acknowledgements.
mod ack {
    pub const OKAY: u8 = 0;
    pub const BAD_COMMAND: u8 = 1;
    pub const NO_INDEX: u8 = 2;
    pub const NO_TRK0: u8 = 3;
    pub const FLUX_OVERFLOW: u8 = 4;
    pub const FLUX_UNDERFLOW: u8 = 5;
    pub const WRPROT: u8 = 6;
    pub const NO_UNIT: u8 = 7;
    pub const NO_BUS: u8 = 8;
    pub const BAD_UNIT: u8 = 9;
    pub const BAD_PIN: u8 = 10;
    pub const BAD_CYLINDER: u8 = 11;
    pub const OUT_OF_SRAM: u8 = 12;
    pub const OUT_OF_FLASH: u8 = 13;

    pub fn describe(code: u8) -> &'static str {
        match code {
            OKAY => "okay",
            BAD_COMMAND => "bad command",
            NO_INDEX => "no index pulse: the disk is not turning",
            NO_TRK0 => "track 0 not found",
            FLUX_OVERFLOW => "flux overflow: the host did not keep up",
            FLUX_UNDERFLOW => "flux underflow: the host did not keep up",
            WRPROT => "the disk is write protected",
            NO_UNIT => "no drive unit selected",
            NO_BUS => "no bus type set",
            BAD_UNIT => "invalid unit number",
            BAD_PIN => "invalid pin",
            BAD_CYLINDER => "invalid cylinder",
            OUT_OF_SRAM => "out of SRAM",
            OUT_OF_FLASH => "out of flash",
            _ => "unknown error",
        }
    }
}

/// Escape opcodes in the flux stream, each introduced by a `0xFF` byte.
mod flux_op {
    pub const INDEX: u8 = 1;
    pub const SPACE: u8 = 2;
    pub const ASTABLE: u8 = 3;
}

/// How the drive is wired to the board.
///
/// A PC cable has its select and motor lines swapped between the two drive
/// positions by the twist in the ribbon; Shugart cabling numbers up to four
/// units without one. Amiga drives are Shugart-wired, but a Greaseweazle is
/// most often used with a PC drive and cable, so both are supported.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusType {
    IbmPc,
    Shugart,
}

impl BusType {
    fn value(self) -> u8 {
        match self {
            BusType::IbmPc => 1,
            BusType::Shugart => 2,
        }
    }
}

/// Which drive on the cable to talk to.
///
/// Spelled the way the interface's users already spell it: `a`/`b` are the two
/// positions on a PC cable, `0`..`3` are Shugart unit numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriveSelect {
    pub bus: BusType,
    pub unit: u8,
}

impl Default for DriveSelect {
    fn default() -> Self {
        // The position a single drive on a PC cable ends up in.
        Self {
            bus: BusType::IbmPc,
            unit: 0,
        }
    }
}

impl DriveSelect {
    pub fn parse(spec: &str) -> Result<Self> {
        match spec.trim().to_ascii_lowercase().as_str() {
            "a" => Ok(Self {
                bus: BusType::IbmPc,
                unit: 0,
            }),
            "b" => Ok(Self {
                bus: BusType::IbmPc,
                unit: 1,
            }),
            unit @ ("0" | "1" | "2" | "3") => Ok(Self {
                bus: BusType::Shugart,
                unit: unit.as_bytes()[0] - b'0',
            }),
            other => bail!("unknown drive {other}: expected a or b (PC cable), or 0..3 (Shugart)"),
        }
    }
}

/// USB identity the board enumerates with, used to pick it out of the host's
/// serial ports when no port was named.
const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x4d69;
/// The board in bootloader mode, where it can be updated but not used.
const USB_PID_BOOTLOADER: u16 = 0x0001;

/// Baud rates the board treats as out-of-band control rather than as a line
/// speed. It is a USB CDC device, so the "baud rate" never reaches a UART;
/// selecting this one makes the firmware discard any half-finished command,
/// which is how a session recovers from an interrupted one.
const BAUD_CLEAR_COMMS: u32 = 10_000;
const BAUD_NORMAL: u32 = 9600;

/// Firmware older than this predates the command set used here.
const EARLIEST_SUPPORTED_FIRMWARE: (u8, u8) = (0, 31);

/// `/TRK0` on the floppy bus, which the drive asserts (low) only while the head
/// is over the outermost cylinder. The one position on the disk an interface can
/// establish absolutely -- everything else is counted in steps from it.
const PIN_TRK0: u8 = 26;
/// `/DENSEL`. A drive that can take both densities decides from this line which
/// write current and data rate to expect.
const PIN_DENSITY_SELECT: u8 = 2;
/// `/WRPROT`, asserted (low) while the disk's tab is open -- which on a 3.5"
/// disk means the shutter hole is *closed*. The drive senses it mechanically.
const PIN_WRITE_PROTECT: u8 = 28;
/// `/DSKCHG` on a PC drive: asserted (low) from the moment a disk is removed
/// until a step happens with one in place. An empty slot therefore holds it
/// asserted, which is the closest the bus comes to reporting whether a disk is
/// there at all. Not every drive fits this line, so it is read as "unknown when
/// unsupported" rather than relied upon.
const PIN_DISK_CHANGE: u8 = 34;

/// Interval between head steps to ask the interface for, in microseconds.
///
/// An Amiga's trackdisk steps every 3 ms, and the emulated stepper charges the
/// same. The interface's own default is far slower, which leaves the real head
/// still travelling long after the emulated one has arrived -- audible as a
/// drawn-out seek, and slow enough that a multi-cylinder move dominates the time
/// a track takes to read.
const STEP_INTERVAL_US: u16 = 3_000;

/// How long after the head moves before its output is worth reading.
///
/// The carriage rings for a moment after a step, and flux taken during it comes
/// off the wrong part of the disk. The interface applies its own settle time
/// inside a seek, but only when a seek actually moves the head -- and the head
/// may well have been moved a moment earlier by the guest's own stepper, leaving
/// the seek before a read with nothing to do. So the wait is kept here, where it
/// depends on when the head last moved rather than on who moved it.
const HEAD_SETTLE: Duration = Duration::from_millis(15);

/// A capture is asked for by revolution count, but the firmware also wants a
/// tick ceiling so a stopped disk cannot hang the read forever. Nothing is
/// gained by cutting it fine: this only bounds the wait.
const READ_TICK_CEILING_SECONDS: f64 = 2.0;

/// How long to allow a single command's acknowledgement. Seeks step the head
/// and are the slowest of them.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(4);

/// What the board says about itself.
#[derive(Clone, Debug)]
pub struct FirmwareInfo {
    pub major: u8,
    pub minor: u8,
    /// Highest command number this firmware understands. Later firmware adds
    /// commands, so this is what gates using them.
    pub max_cmd: u8,
    /// Sample rate of the flux timer. Every interval in a capture is in these
    /// ticks, and it differs between board revisions.
    pub sample_freq: u32,
    pub hw_model: u8,
    pub hw_submodel: u8,
    pub usb_speed: u8,
    /// True when the board is running its bootloader, where it accepts an
    /// update but cannot touch a drive.
    pub update_mode: bool,
}

impl FirmwareInfo {
    /// The board's commercial name, for logs. Unknown ids are reported as
    /// their raw values rather than guessed at, so a board newer than this
    /// code still identifies itself usefully.
    pub fn model_name(&self) -> String {
        let name = match (self.hw_model, self.hw_submodel) {
            (1, 0) => "F1",
            (1, 1) => "F1 Plus",
            (1, 2) => "F1 Plus (unbuffered)",
            (4, 0) => "V4",
            (4, 1) => "V4 Slim",
            (4, 2) => "V4.1",
            (7, 0) => "F7 v1",
            (7, 1) => "F7 Plus (v1)",
            (7, 2) => "F7 Lightning",
            (7, 3) => "F7 v2",
            (7, 4) => "F7 Plus (v2)",
            (7, 5) => "F7 Lightning Plus",
            (7, 6) => "F7 Slim",
            (7, 7) => "F7 v3 Thunderbolt",
            (8, _) => "Adafruit Floppy",
            _ => {
                return format!(
                    "unknown model {:#04x}.{:#04x}",
                    self.hw_model, self.hw_submodel
                )
            }
        };
        name.to_string()
    }

    fn supports(&self, command: u8) -> bool {
        command <= self.max_cmd
    }
}

/// A Greaseweazle, opened and ready to move a drive.
pub struct Greaseweazle {
    port: Box<dyn serialport::SerialPort>,
    info: FirmwareInfo,
    port_name: String,
    drive: DriveSelect,
    /// Where the head is, once a seek has established it.
    cylinder: Option<u8>,
    head: Head,
    motor_on: bool,
    /// When the head last actually moved, so a read can wait out the carriage
    /// settling however the move came about.
    moved_at: Option<Instant>,
}

/// Serial ports that look like a Greaseweazle, most likely first.
///
/// A board in bootloader mode enumerates with a different product id; it is
/// reported so that "found, but needs a firmware update" can be told apart
/// from "not plugged in".
pub fn available() -> Vec<Discovered> {
    let ports = match serialport::available_ports() {
        Ok(ports) => ports,
        Err(err) => {
            warn!("fluxdrive: cannot enumerate serial ports: {err}");
            return Vec::new();
        }
    };
    ports
        .into_iter()
        .filter_map(|port| match port.port_type {
            serialport::SerialPortType::UsbPort(usb) if usb.vid == USB_VID => {
                let bootloader = match usb.pid {
                    USB_PID => false,
                    USB_PID_BOOTLOADER => true,
                    _ => return None,
                };
                Some(Discovered {
                    port: port.port_name,
                    serial: usb.serial_number,
                    bootloader,
                })
            }
            _ => None,
        })
        .filter(|found| is_usable_node(&found.port))
        .collect()
}

/// Whether a serial device node is the one to open.
///
/// macOS exposes every serial device twice: `/dev/cu.*` is the call-out node and
/// `/dev/tty.*` the call-in node, which blocks on open until carrier detect is
/// asserted. A USB CDC device never asserts it, so opening the `tty` node hangs.
/// Only the `cu` node is usable, and skipping the other also stops one board
/// looking like two.
#[cfg(target_os = "macos")]
fn is_usable_node(port: &str) -> bool {
    !port.starts_with("/dev/tty.")
}

#[cfg(not(target_os = "macos"))]
fn is_usable_node(_port: &str) -> bool {
    true
}

#[derive(Clone, Debug)]
pub struct Discovered {
    pub port: String,
    pub serial: Option<String>,
    /// The board is in bootloader mode and cannot drive a disk until its
    /// firmware is updated.
    pub bootloader: bool,
}

impl Greaseweazle {
    /// Open a board and prepare it to drive `drive`.
    ///
    /// `port` names a serial device; `None` picks the only Greaseweazle
    /// attached, and refuses to guess when there is more than one.
    pub fn open(port: Option<&str>, drive: DriveSelect) -> Result<Self> {
        let port_name = match port {
            Some(name) => name.to_string(),
            None => {
                let found = available();
                let usable: Vec<_> = found.iter().filter(|d| !d.bootloader).collect();
                match usable.as_slice() {
                    [one] => one.port.clone(),
                    [] if found.is_empty() => {
                        bail!("no Greaseweazle found: check it is plugged in")
                    }
                    [] => bail!(
                        "the attached Greaseweazle is in bootloader mode and cannot \
                         drive a disk until its firmware is updated"
                    ),
                    many => bail!(
                        "{} Greaseweazles are attached ({}): name one explicitly",
                        many.len(),
                        many.iter()
                            .map(|d| d.port.as_str())
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                }
            }
        };

        let port = serialport::new(&port_name, BAUD_NORMAL)
            // Short enough that a wedged board is noticed, with the real
            // bounds imposed by per-command deadlines on top.
            .timeout(Duration::from_millis(200))
            .open()
            .map_err(|err| {
                // Only one program can have the interface at a time, and the
                // usual reason is another Copperline still holding it. Say so:
                // a machine that cannot open the drive otherwise looks exactly
                // like one with an empty drive.
                if err.kind() == serialport::ErrorKind::Io(ErrorKind::ResourceBusy)
                    || err.to_string().contains("busy")
                {
                    anyhow!(
                        "the Greaseweazle on {port_name} is already in use -- another \
                         Copperline, or another tool, still has it open"
                    )
                } else {
                    anyhow!("cannot open Greaseweazle on {port_name}: {err}")
                }
            })?;

        let mut gw = Self {
            port,
            info: FirmwareInfo {
                major: 0,
                minor: 0,
                max_cmd: 0,
                sample_freq: 0,
                hw_model: 0,
                hw_submodel: 0,
                usb_speed: 0,
                update_mode: false,
            },
            port_name,
            drive,
            cylinder: None,
            head: Head::Lower,
            motor_on: false,
            moved_at: None,
        };

        gw.clear_comms()?;
        gw.info = gw.read_firmware_info()?;

        ensure!(
            !gw.info.update_mode,
            "the Greaseweazle on {} is in bootloader mode and cannot drive a disk \
             until its firmware is updated",
            gw.port_name
        );
        ensure!(
            (gw.info.major, gw.info.minor) >= EARLIEST_SUPPORTED_FIRMWARE,
            "the Greaseweazle on {} runs firmware {}.{}, older than the {}.{} this \
             supports: update it",
            gw.port_name,
            gw.info.major,
            gw.info.minor,
            EARLIEST_SUPPORTED_FIRMWARE.0,
            EARLIEST_SUPPORTED_FIRMWARE.1
        );
        ensure!(
            gw.info.sample_freq > 0,
            "the Greaseweazle on {} reports no flux sample rate",
            gw.port_name
        );

        info!(
            "fluxdrive: Greaseweazle {} on {}, firmware {}.{}, flux timer {:.3} MHz",
            gw.info.model_name(),
            gw.port_name,
            gw.info.major,
            gw.info.minor,
            f64::from(gw.info.sample_freq) / 1.0e6,
        );

        gw.send(&[cmd::SET_BUS_TYPE, 3, drive.bus.value()])
            .with_context(|| format!("the board does not support {:?} cabling", drive.bus))?;
        gw.send(&[cmd::SELECT, 3, drive.unit])
            .context("cannot select the drive")?;
        // Establish the motor state rather than assume it: a previous session
        // that ended abruptly can leave the disk spinning, and a cached "off"
        // that was never sent would make the first spin-up a no-op.
        gw.send(&[cmd::MOTOR, 4, drive.unit, 0])
            .context("cannot stop the drive motor")?;
        gw.motor_on = false;
        // Step at the rate the emulated stepper does, so the real head keeps up
        // with the one the guest thinks it is moving.
        if let Err(err) = gw.set_step_interval(STEP_INTERVAL_US) {
            warn!("fluxdrive: cannot match the drive's step rate to the Amiga's: {err:#}");
        }

        Ok(gw)
    }

    pub fn firmware(&self) -> &FirmwareInfo {
        &self.info
    }

    /// Ticks per second of the flux timer, which every interval in a capture is
    /// measured in.
    pub fn ticks_per_sec(&self) -> u32 {
        self.info.sample_freq
    }

    /// Tell a dual-density drive which density is in it.
    ///
    /// Amiga disks are double density. Most 3.5" drives ignore the line and
    /// decide from the media hole, which is why this is not done as a matter of
    /// course, but a drive that honours it will not read a DD disk without it.
    pub fn set_density_select(&mut self, double_density: bool) -> Result<()> {
        // The line is active low: asserted selects high density.
        let level = u8::from(double_density);
        self.send(&[cmd::SET_PIN, 4, PIN_DENSITY_SELECT, level])
            .context("cannot set the density-select line")
    }

    /// Bring the comms channel to a known state.
    ///
    /// A previous session killed mid-command can leave the firmware waiting for
    /// the rest of it, which would desynchronise everything sent afterwards.
    /// Selecting the control baud rate makes it discard that partial command.
    fn clear_comms(&mut self) -> Result<()> {
        // On hosts that reject a non-standard rate this cannot be done, and
        // flushing is the best available. A board that was left clean -- the
        // usual case -- is unaffected either way.
        if let Err(err) = self.port.set_baud_rate(BAUD_CLEAR_COMMS) {
            debug!("fluxdrive: host rejected the comms-clear baud rate: {err}");
        } else if let Err(err) = self.port.set_baud_rate(BAUD_NORMAL) {
            debug!("fluxdrive: cannot restore the normal baud rate: {err}");
        }
        self.port
            .clear(serialport::ClearBuffer::All)
            .context("cannot flush the Greaseweazle serial port")?;
        Ok(())
    }

    fn read_firmware_info(&mut self) -> Result<FirmwareInfo> {
        self.send(&[cmd::GET_INFO, 3, get_info::FIRMWARE])
            .context("the device on this port does not answer as a Greaseweazle")?;
        let mut buf = [0u8; 32];
        self.read_exact(&mut buf, Instant::now() + COMMAND_TIMEOUT)
            .context("truncated firmware report")?;
        let sample_freq = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        // Firmware predating the hardware report ran on the F1 alone.
        let hw_model = if buf[8] == 0 { 1 } else { buf[8] };
        Ok(FirmwareInfo {
            major: buf[0],
            minor: buf[1],
            max_cmd: buf[3],
            sample_freq,
            hw_model,
            hw_submodel: buf[9],
            usb_speed: buf[10],
            // The firmware reports whether it is the main image; the
            // bootloader is everything else.
            update_mode: buf[2] == 0,
        })
    }

    /// Send one command and wait for its acknowledgement.
    ///
    /// Every command is `[opcode, length, ..params]` and is answered by
    /// `[opcode, ack]`, so a mismatched opcode in the reply means the stream
    /// has lost sync and nothing after it can be trusted.
    fn send(&mut self, command: &[u8]) -> Result<()> {
        self.send_with_deadline(command, Instant::now() + COMMAND_TIMEOUT)
    }

    fn send_with_deadline(&mut self, command: &[u8], deadline: Instant) -> Result<()> {
        debug_assert_eq!(
            usize::from(command[1]),
            command.len(),
            "command length byte must match the frame"
        );
        self.port
            .write_all(command)
            .with_context(|| format!("cannot send command {}", command[0]))?;
        self.port.flush().ok();
        let mut reply = [0u8; 2];
        self.read_exact(&mut reply, deadline)
            .with_context(|| format!("no reply to command {}", command[0]))?;
        ensure!(
            reply[0] == command[0],
            "Greaseweazle replied to command {} with {}: the command stream has lost sync",
            command[0],
            reply[0]
        );
        if reply[1] != ack::OKAY {
            return Err(CommandError {
                command: command[0],
                code: reply[1],
            }
            .into());
        }
        Ok(())
    }

    /// Fill `buf`, tolerating the short reads a serial port hands back and
    /// giving up at `deadline`.
    fn read_exact(&mut self, buf: &mut [u8], deadline: Instant) -> Result<()> {
        let mut filled = 0;
        while filled < buf.len() {
            if Instant::now() >= deadline {
                bail!(
                    "timed out reading from the Greaseweazle on {} ({} of {} bytes)",
                    self.port_name,
                    filled,
                    buf.len()
                );
            }
            match self.port.read(&mut buf[filled..]) {
                Ok(0) => {}
                Ok(n) => filled += n,
                Err(err) if err.kind() == ErrorKind::TimedOut => {}
                Err(err) if err.kind() == ErrorKind::Interrupted => {}
                Err(err) => {
                    return Err(err).with_context(|| {
                        format!("cannot read from the Greaseweazle on {}", self.port_name)
                    })
                }
            }
        }
        Ok(())
    }

    /// Read the encoded flux stream, which runs until a zero byte.
    ///
    /// Every byte the encoding can produce is non-zero, so a zero is
    /// unambiguously the end of the stream and the length need not be known in
    /// advance.
    fn read_flux_stream(&mut self, deadline: Instant) -> Result<Vec<u8>> {
        let mut stream = Vec::new();
        let mut chunk = [0u8; 4096];
        loop {
            if Instant::now() >= deadline {
                bail!(
                    "timed out reading flux from the Greaseweazle on {} after {} bytes",
                    self.port_name,
                    stream.len()
                );
            }
            match self.port.read(&mut chunk) {
                Ok(0) => continue,
                Ok(n) => {
                    stream.extend_from_slice(&chunk[..n]);
                    if stream.last() == Some(&0) {
                        return Ok(stream);
                    }
                }
                Err(err)
                    if err.kind() == ErrorKind::TimedOut
                        || err.kind() == ErrorKind::Interrupted => {}
                Err(err) => {
                    return Err(err).with_context(|| {
                        format!(
                            "cannot read flux from the Greaseweazle on {}",
                            self.port_name
                        )
                    })
                }
            }
        }
    }

    /// Whether the head is over cylinder 0, from the one absolute position
    /// signal the bus offers.
    fn at_track0(&mut self) -> Result<bool> {
        Ok(self.read_pin(PIN_TRK0)?.is_some_and(|asserted| asserted))
    }

    /// Raw level of a drive line, for comparing against other tools.
    pub fn pin_level(&mut self, pin: u8) -> Result<Option<bool>> {
        match self.send(&[cmd::GET_PIN, 3, pin]) {
            Ok(()) => {}
            Err(err) => {
                let unsupported = err
                    .downcast_ref::<CommandError>()
                    .is_some_and(|e| e.code == ack::BAD_PIN || e.code == ack::BAD_COMMAND);
                if unsupported {
                    return Ok(None);
                }
                return Err(err);
            }
        }
        let mut level = [0u8; 1];
        self.read_exact(&mut level, Instant::now() + COMMAND_TIMEOUT)?;
        Ok(Some(level[0] != 0))
    }

    /// Read one drive line, or `None` when this board cannot reach that pin.
    ///
    /// Answers whether the signal is *asserted*, having already accounted for
    /// the bus being active low.
    fn read_pin(&mut self, pin: u8) -> Result<Option<bool>> {
        match self.send(&[cmd::GET_PIN, 3, pin]) {
            Ok(()) => {}
            Err(err) => {
                let unsupported = err
                    .downcast_ref::<CommandError>()
                    .is_some_and(|e| e.code == ack::BAD_PIN || e.code == ack::BAD_COMMAND);
                if unsupported {
                    return Ok(None);
                }
                return Err(err);
            }
        }
        let mut level = [0u8; 1];
        self.read_exact(&mut level, Instant::now() + COMMAND_TIMEOUT)?;
        Ok(Some(level[0] == 0))
    }

    /// Ask the interface to step the head at the rate an Amiga does.
    ///
    /// The delay block grew across firmware revisions, so its length is
    /// discovered by asking for the longest and shortening until the board
    /// accepts it; the values are then written back with only the step interval
    /// changed, leaving the board's other timings as its owner set them.
    fn set_step_interval(&mut self, microseconds: u16) -> Result<()> {
        let mut delays = Vec::new();
        for size in [16u8, 14, 12, 10] {
            match self.send(&[cmd::GET_PARAMS, 4, params::DELAYS, size]) {
                Ok(()) => {
                    delays.resize(usize::from(size), 0);
                    self.read_exact(&mut delays, Instant::now() + COMMAND_TIMEOUT)?;
                    break;
                }
                Err(err) => {
                    let too_long = err
                        .downcast_ref::<CommandError>()
                        .is_some_and(|e| e.code == ack::BAD_COMMAND);
                    if !too_long {
                        return Err(err);
                    }
                }
            }
        }
        ensure!(
            delays.len() >= 4,
            "the board did not report its drive delays"
        );
        // Second of the little-endian 16-bit fields: select, step, settle, ...
        delays[2..4].copy_from_slice(&microseconds.to_le_bytes());

        let mut command = Vec::with_capacity(delays.len() + 3);
        command.push(cmd::SET_PARAMS);
        command.push(u8::try_from(delays.len() + 3).context("delay block is too long")?);
        command.push(params::DELAYS);
        command.extend_from_slice(&delays);
        self.send(&command)
            .context("cannot set the drive step interval")
    }
}

/// A command the board refused, carrying the code so callers can tell a
/// recoverable refusal (an overflowed capture) from a real fault.
#[derive(Debug, Clone, Copy)]
pub struct CommandError {
    pub command: u8,
    pub code: u8,
}

impl CommandError {
    /// Whether this failure means there is no disk in the drive.
    ///
    /// The floppy bus has no line for it. What a drive with an empty slot does
    /// is turn nothing, so no index hole ever passes the sensor -- which is the
    /// same evidence a real controller goes on.
    pub fn means_no_disk(&self) -> bool {
        self.code == ack::NO_INDEX
    }
}

impl std::fmt::Display for CommandError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Greaseweazle command {} failed: {}",
            self.command,
            ack::describe(self.code)
        )
    }
}

impl std::error::Error for CommandError {}

/// How many times to re-ask for a capture the host failed to drain in time.
///
/// An overflow is a host scheduling hiccup, not a fault of the disk: the board
/// buffers flux while USB catches up and gives up if it cannot. Re-reading gets
/// a different revolution of the same track, which is what a real drive's
/// controller would do anyway.
const FLUX_OVERFLOW_RETRIES: u32 = 5;

impl FluxSource for Greaseweazle {
    fn describe(&self) -> String {
        format!(
            "Greaseweazle {} on {} (firmware {}.{})",
            self.info.model_name(),
            self.port_name,
            self.info.major,
            self.info.minor
        )
    }

    fn seek(&mut self, cylinder: u8) -> Result<()> {
        if self.cylinder == Some(cylinder) {
            return Ok(());
        }
        self.send(&[cmd::SEEK, 3, cylinder])
            .with_context(|| format!("cannot seek to cylinder {cylinder}"))?;

        // A step count is only ever a count: if the head was not where it was
        // believed to be, every later position is wrong too. `/TRK0` is the one
        // place that can be checked, so check it whenever the head is there or
        // claims not to be.
        let at_track0 = self.at_track0()?;
        if at_track0 != (cylinder == 0) {
            // Some flippy-modified drives do not assert /TRK0 when stepping
            // inward from below cylinder 0. A fake outward step settles it
            // without moving the head; older firmware lacks the command, in
            // which case the disagreement stands.
            if cylinder == 0 && self.info.supports(cmd::NO_CLICK_STEP) {
                self.send(&[cmd::NO_CLICK_STEP, 2]).ok();
            }
            let at_track0 = self.at_track0()?;
            if at_track0 != (cylinder == 0) {
                self.cylinder = None;
                bail!(
                    "the drive reports track 0 {} after seeking to cylinder {cylinder}: \
                     the head position is not trustworthy",
                    if at_track0 { "asserted" } else { "absent" }
                );
            }
        }

        self.cylinder = Some(cylinder);
        self.moved_at = Some(Instant::now());
        Ok(())
    }

    fn select_head(&mut self, head: Head) -> Result<()> {
        self.send(&[cmd::HEAD, 3, head.index()])
            .context("cannot select the head")?;
        self.head = head;
        Ok(())
    }

    fn motor(&mut self, on: bool) -> Result<()> {
        if self.motor_on == on {
            return Ok(());
        }
        self.send(&[cmd::MOTOR, 4, self.drive.unit, u8::from(on)])
            .with_context(|| {
                format!(
                    "cannot turn the drive motor {}",
                    if on { "on" } else { "off" }
                )
            })?;
        self.motor_on = on;
        Ok(())
    }

    fn read_flux(&mut self, revolutions: u8) -> Result<FluxCapture> {
        ensure!(
            revolutions > 0,
            "a capture must cover at least one revolution"
        );
        ensure!(
            self.motor_on,
            "the drive motor is off: a stopped disk produces no flux"
        );
        // Wait out anything left of the carriage settling, whoever moved it.
        if let Some(moved_at) = self.moved_at {
            let waited = moved_at.elapsed();
            if waited < HEAD_SETTLE {
                std::thread::sleep(HEAD_SETTLE - waited);
            }
            self.moved_at = None;
        }

        // The head is wherever the disk happens to have reached, so the first
        // partial revolution is not a whole track. Ask for one more than is
        // wanted and let the caller take whole ones from the first index.
        let requested = u16::from(revolutions).saturating_add(1);
        let ceiling = (f64::from(self.info.sample_freq) * READ_TICK_CEILING_SECONDS) as u32;

        let mut command = [0u8; 8];
        command[0] = cmd::READ_FLUX;
        command[1] = 8;
        command[2..6].copy_from_slice(&ceiling.to_le_bytes());
        command[6..8].copy_from_slice(&requested.to_le_bytes());

        let mut attempt = 0;
        let stream = loop {
            let deadline = Instant::now()
                + COMMAND_TIMEOUT
                + Duration::from_secs_f64(READ_TICK_CEILING_SECONDS);
            let outcome = self
                .send_with_deadline(&command, deadline)
                .and_then(|()| self.read_flux_stream(deadline))
                .and_then(|stream| {
                    // The board reports how the capture went separately, after
                    // the stream: a truncated or overflowed read still looks
                    // like a well-formed stream on its own.
                    self.send(&[cmd::GET_FLUX_STATUS, 2])?;
                    Ok(stream)
                });
            match outcome {
                Ok(stream) => break stream,
                Err(err) => {
                    let recoverable = err
                        .downcast_ref::<CommandError>()
                        .is_some_and(|e| e.code == ack::FLUX_OVERFLOW);
                    if !recoverable || attempt >= FLUX_OVERFLOW_RETRIES {
                        return Err(err);
                    }
                    attempt += 1;
                    debug!(
                        "fluxdrive: flux capture overflowed, retrying ({attempt}/{FLUX_OVERFLOW_RETRIES})"
                    );
                    // Re-sync before trying again: the aborted read may have
                    // left bytes in flight.
                    self.clear_comms()?;
                }
            }
        };

        let capture = decode_flux_stream(&stream, self.info.sample_freq)?;
        ensure!(
            capture.revolutions() >= usize::from(revolutions),
            "the capture holds {} whole revolutions, not the {revolutions} asked for",
            capture.revolutions()
        );
        Ok(capture)
    }

    fn status(&mut self) -> Result<DriveStatus> {
        let mut status = DriveStatus {
            cylinder: self.cylinder,
            motor_on: self.motor_on,
            // The drive senses the tab mechanically and puts it on /WRPROT.
            // A board that cannot reach the pin leaves this unknown, which must
            // never be read as "writable".
            write_protected: self.read_pin(PIN_WRITE_PROTECT)?,
            // /DSKCHG stays asserted while the slot is empty, so an unasserted
            // line means a disk is in there and has been stepped on since. The
            // line only exists on a PC cable: Shugart wiring puts something else
            // on that pin, so there is nothing to read.
            disk_present: match self.drive.bus {
                BusType::IbmPc => self.read_pin(PIN_DISK_CHANGE)?.map(|changed| !changed),
                BusType::Shugart => None,
            },
        };
        // Later firmware can report the head position and motor state from the
        // board's own view, which catches a drive that was moved by something
        // else. Older firmware refuses this sub-index; that is a capability
        // gap, not a fault, so the locally tracked values stand.
        match self.send(&[cmd::GET_INFO, 3, get_info::CURRENT_DRIVE]) {
            Ok(()) => {
                let mut buf = [0u8; 32];
                self.read_exact(&mut buf, Instant::now() + COMMAND_TIMEOUT)?;
                let flags = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                let cylinder = i32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                const FLAG_CYL_VALID: u32 = 1;
                const FLAG_MOTOR_ON: u32 = 2;
                if flags & FLAG_CYL_VALID != 0 {
                    status.cylinder = u8::try_from(cylinder).ok();
                }
                status.motor_on = flags & FLAG_MOTOR_ON != 0;
            }
            Err(err) => {
                let unsupported = err
                    .downcast_ref::<CommandError>()
                    .is_some_and(|e| e.code == ack::BAD_COMMAND);
                if !unsupported {
                    return Err(err);
                }
            }
        }
        Ok(status)
    }
}

impl Drop for Greaseweazle {
    fn drop(&mut self) {
        // Leave the drive as it was found: a motor left spinning wears the
        // disk, and a still-selected unit keeps its LED on.
        if self.motor_on {
            let unit = self.drive.unit;
            if let Err(err) = self.send(&[cmd::MOTOR, 4, unit, 0]) {
                warn!("fluxdrive: cannot stop the drive motor: {err}");
            }
        }
        if let Err(err) = self.send(&[cmd::DESELECT, 2]) {
            debug!("fluxdrive: cannot deselect the drive: {err}");
        }
    }
}

impl std::fmt::Debug for Greaseweazle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Greaseweazle")
            .field("port", &self.port_name)
            .field("model", &self.info.model_name())
            .field("cylinder", &self.cylinder)
            .field("head", &self.head)
            .field("motor_on", &self.motor_on)
            .finish()
    }
}

/// Turn the board's encoded flux stream into intervals and index timestamps.
///
/// The encoding is byte-oriented and self-delimiting. A byte below 250 is a
/// whole interval in ticks; 250..254 begins a two-byte interval; 255 introduces
/// an opcode:
///
/// - `Index` marks an index pulse, at a stated tick offset from the flux
///   position reached so far. The pulse does not interrupt the interval it
///   falls inside -- the head keeps reading across the index -- so it is
///   recorded as a timestamp rather than as a break in the flux.
/// - `Space` carries ticks with no reversal in them, which is how a long gap
///   is expressed without a huge interval byte count.
/// - `Astable` marks a region with no recoverable clock. It appears in streams
///   written *to* a board, not read from one.
///
/// A trailing zero byte ends the stream.
pub fn decode_flux_stream(stream: &[u8], ticks_per_sec: u32) -> Result<FluxCapture> {
    let Some((&terminator, body)) = stream.split_last() else {
        bail!("empty flux stream");
    };
    ensure!(
        terminator == 0,
        "flux stream does not end with its terminator"
    );

    let mut intervals: Vec<u32> = Vec::new();
    let mut index_ticks: Vec<u64> = Vec::new();
    // Ticks accumulated since the last reversal, and the absolute position of
    // that reversal. Together they place an index pulse exactly.
    let mut pending: u64 = 0;
    let mut at: u64 = 0;

    let mut i = 0;
    while i < body.len() {
        let byte = body[i];
        i += 1;
        if byte == 0xFF {
            let opcode = *body
                .get(i)
                .ok_or_else(|| anyhow!("flux stream ends mid-opcode"))?;
            i += 1;
            let value = read_28bit(body, &mut i)?;
            match opcode {
                flux_op::INDEX => index_ticks.push(at + pending + u64::from(value)),
                flux_op::SPACE => pending += u64::from(value),
                flux_op::ASTABLE => {
                    // The value is the period of the astable region, not its
                    // duration -- that came from the `Space` before it -- so it
                    // contributes no time. Boards emit this when *writing*, and
                    // seeing it in a read means a firmware whose streams are
                    // not fully understood here.
                    warn!("fluxdrive: ignoring an astable marker in a flux capture");
                }
                other => bail!("unknown opcode {other} in the flux stream"),
            }
            continue;
        }
        let value = if byte < 250 {
            u64::from(byte)
        } else {
            // 250..254 is a high byte; the low byte follows, biased by one so
            // it can never be the stream terminator.
            let low = *body
                .get(i)
                .ok_or_else(|| anyhow!("flux stream ends mid-interval"))?;
            i += 1;
            ensure!(low != 0, "flux stream contains a zero interval low byte");
            250 + u64::from(byte - 250) * 255 + u64::from(low) - 1
        };
        pending += value;
        // Clamp and carry the same number forward, so the running sum of
        // `intervals` and the index timestamps stay on one timebase. At any
        // real sample rate a single interval cannot come near the ceiling.
        let interval = u32::try_from(pending).unwrap_or(u32::MAX);
        at += u64::from(interval);
        intervals.push(interval);
        pending = 0;
    }

    Ok(FluxCapture {
        ticks_per_sec,
        intervals,
        index_ticks,
    })
}

/// Read the 28-bit value that follows an opcode.
///
/// It is spread over four bytes, seven bits each, with the low bit of every
/// byte set so that no byte of the encoding can be mistaken for the stream's
/// zero terminator.
fn read_28bit(body: &[u8], i: &mut usize) -> Result<u32> {
    ensure!(
        *i + 4 <= body.len(),
        "flux stream ends inside a 28-bit value"
    );
    let mut value: u32 = 0;
    for shift in 0..4 {
        let bits = u32::from(body[*i + shift] & 0xFE) >> 1;
        value |= bits << (7 * shift);
    }
    *i += 4;
    Ok(value)
}
