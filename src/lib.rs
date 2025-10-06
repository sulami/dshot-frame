//! Support for the DShot ESC protocol
//!
//! DShot has two-byte frames, where the first 11 bits are the throttle speed, bit 12 is a
//! telemetry request flag, and the last four bits are a checksum.
//!
//! Throttle values below 48 are reserved for special commands.
//!
//! It is transmitted over the wire at a fixed speed, with ones and zeroes both being pulses, but
//! ones being twice as long as zeroes.
//!
//! ## Usage
//!
//! This example is adapted from an embassy-stm32 codebase:
//!
//! ```ignore
//! let mut pwm = SimplePwm::new(
//!     timer,
//!     Some(PwmPin::new_ch1(pin, OutputType::PushPull)),
//!     None,
//!     None,
//!     None,
//!     Hertz(150_000),
//!     CountingMode::EdgeAlignedUp,
//! );
//! let max_duty_cycle = pwm.get_max_duty() as u16;
//!
//! let frame = Frame::<NormalDshot>::new(1000, false).unwrap();
//! pwm.waveform_up(&mut dma, Ch1, &frame.duty_cycles()).await;
//! // Pull the line low after sending a frame.
//! pwm.set_duty(channel, 0);
//! pwm.enable(channel);
//! ```

#![no_std]

/// Defines the behavior of a DShot protocol variant.
pub trait DshotProtocol {
    /// Computes the 4-bit CRC for the upper 12 bits of the frame.
    fn compute_crc(value: u16) -> u16;

    /// Returns `true` if the signal is inverted (bidirectional mode).
    fn is_inverted() -> bool;

    fn get_translated_throttle(speed: u16) -> u16;
}

/// Standard (non-inverted) DShot protocol.
#[derive(Debug, Clone, Copy)]
pub struct NormalDshot;

impl DshotProtocol for NormalDshot {
    fn compute_crc(value: u16) -> u16 {
        (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
    }

    fn is_inverted() -> bool {
        false
    }

    fn get_translated_throttle(speed: u16) -> u16 {
        (speed + 48) << 5
    }
}

/// Bidirectional (inverted) DShot protocol.
#[derive(Debug, Clone, Copy)]
pub struct BidirectionalDshot;

impl DshotProtocol for BidirectionalDshot {
    fn compute_crc(value: u16) -> u16 {
        (!(value ^ (value >> 4) ^ (value >> 8))) & 0x0F
    }

    fn is_inverted() -> bool {
        true
    }

    fn get_translated_throttle(speed: u16) -> u16 {
        let mask = 0b111_1111_1111;
        (!(speed + 48) & mask) << 5
    }
}

/// A DShot frame parameterized by its protocol variant.
#[derive(Copy, Clone, Debug)]
pub struct Frame<P: DshotProtocol = NormalDshot> {
    inner: u16,
    _protocol: core::marker::PhantomData<P>,
}

impl<P: DshotProtocol> Frame<P> {
    /// Creates a new frame with the given speed (0-1999) and telemetry request.
    ///
    /// Returns [`None`] if the speed is out of bounds.
    ///
    /// ```
    /// # use dshot_frame::*;
    /// assert_eq!(Frame::<NormalDshot>::new(1000, false).unwrap().speed(), 1000);
    /// ```
    pub fn new(speed: u16, request_telemetry: bool) -> Option<Self> {
        if speed >= 2000 {
            return None;
        }

        let translated_throttle = P::get_translated_throttle(speed);
        let mut frame = Self {
            inner: translated_throttle,
            _protocol: core::marker::PhantomData,
        };
        if request_telemetry {
            frame.inner |= 0x10;
        }
        frame.compute_crc();
        Some(frame)
    }

    /// Creates a new frame with the given [`Command`] and telemetry request.
    pub fn command(command: Command, request_telemetry: bool) -> Self {
        let mut frame = Self {
            inner: (command as u16) << 5,
            _protocol: core::marker::PhantomData,
        };
        if request_telemetry {
            frame.inner |= 0x10;
        }
        frame.compute_crc();
        frame
    }

    /// Returns the speed value (0-1999).
    pub fn speed(&self) -> u16 {
        (self.inner >> 5) - 48
    }

    /// Returns whether telemetry is enabled.
    pub fn telemetry_enabled(&self) -> bool {
        self.inner & 0x10 != 0
    }

    /// Returns the CRC checksum.
    pub fn crc(&self) -> u16 {
        self.inner & 0x0F
    }

    /// Computes the CRC based on the first 12 bits and ORs it in.
    fn compute_crc(&mut self) {
        let value = self.inner >> 4;
        let crc = P::compute_crc(value);
        self.inner = (self.inner & !0x0F) | crc;
    }

    /// Returns the raw [`u16`].
    pub fn inner(&self) -> u16 {
        self.inner
    }

    /// Returns an array of duty cycles for use in PWM DMA.
    ///
    /// This contains an extra element that is always zero to ensure the PWM output gets pulled low
    /// at the end of the sequence. It can be sliced off if not needed.
    pub fn duty_cycles(&self, max_duty_cycle: u16) -> [u16; 17] {
        let mut value = self.inner;
        let mut rv = [max_duty_cycle * 3 / 4; 17];
        for item in rv.iter_mut() {
            let bit = value & 0x8000;
            if bit == 0 {
                *item = max_duty_cycle * 3 / 8;
            }
            value <<= 1;
        }
        rv[16] = 0;
        rv
    }
}

// Type aliases for convenience
pub type NormalFrame = Frame<NormalDshot>;
pub type BidirectionalFrame = Frame<BidirectionalDshot>;

/// Fixed commands that occupy the lower 48 speed values.
///
/// Some commands need to be sent multiple times to be acted upon to prevent accidental bit-flips
/// wreaking havoc.
#[derive(Copy, Clone, Debug)]
pub enum Command {
    MotorStop = 0,
    /// Wait at least 260ms before next command.
    Beep1,
    /// Wait at least 260ms before next command.
    Beep2,
    /// Wait at least 260ms before next command.
    Beep3,
    /// Wait at least 260ms before next command.
    Beep4,
    /// Wait at least 260ms before next command.
    Beep5,
    /// Wait at least 12ms before next command.
    ESCInfo,
    /// Needs 6 transmissions.
    SpinDirection1,
    /// Needs 6 transmissions.
    SpinDirection2,
    /// Needs 6 transmissions.
    ThreeDModeOn,
    /// Needs 6 transmissions.
    ThreeDModeOff,
    SettingsRequest,
    /// Needs 6 transmissions. Wait at least 35ms before next command.
    SettingsSave,
    /// Needs 6 transmissions.
    ExtendedTelemetryEnable,
    /// Needs 6 transmissions.
    ExtendedTelemetryDisable,

    // 15-19 are unassigned.
    /// Needs 6 transmissions.
    SpinDirectionNormal = 20,
    /// Needs 6 transmissions.
    SpinDirectonReversed,
    Led0On,
    Led1On,
    Led2On,
    Led3On,
    Led0Off,
    Led1Off,
    Led2Off,
    Led3Off,
    AudioStreamModeToggle,
    SilentModeToggle,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineTelemetryEnable,
    /// Needs 6 transmissions. Disables individual signal line commands.
    SignalLineTelemetryDisable,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineContinuousERPMTelemetry,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineContinuousERPMPeriodTelemetry,

    // 36-41 are unassigned.
    /// 1ºC per LSB.
    SignalLineTemperatureTelemetry = 42,
    /// 10mV per LSB, 40.95V max.
    SignalLineVoltageTelemetry,
    /// 100mA per LSB, 409.5A max.
    SignalLineCurrentTelemetry,
    /// 10mAh per LSB, 40.95Ah max.
    SignalLineConsumptionTelemetry,
    /// 100erpm per LSB, 409500erpm max.
    SignalLineERPMTelemetry,
    /// 16us per LSB, 65520us max.
    SignalLineERPMPeriodTelemetry,
}

/// eRPM telemetry frame as sent by ESC in bidirectional DShot mode.
///
/// Despite being used in bidirectional mode, this frame is **not inverted**,
/// and its CRC is computed using the standard (non-inverted) DShot algorithm.
///
/// Format (16 bits): `[3-bit shift][9-bit period_base][4-bit CRC]`
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ErpmTelemetry {
    /// Raw 3-bit shift value (0–7)
    pub shift: u8,
    /// Raw 9-bit period base (0–511)
    pub period_base: u16,
    /// Raw 4-bit CRC (for debugging)
    pub crc: u8,
}

impl ErpmTelemetry {
    /// Attempts to parse a 16-bit raw telemetry value.
    ///
    /// Returns `None` if the CRC is invalid.
    pub fn try_from_raw(raw: u16) -> Option<Self> {
        let payload = raw >> 4; // upper 12 bits
        let received_crc = (raw & 0x0F) as u8;
        let expected_crc = NormalDshot::compute_crc(payload) as u8;

        if received_crc != expected_crc {
            return None;
        }

        let shift = ((payload >> 9) & 0x07) as u8;
        let period_base = payload & 0x1FF;

        Some(Self {
            shift,
            period_base,
            crc: received_crc,
        })
    }

    /// Returns the period in microseconds.
    ///
    /// If `period_base` is zero, the period is considered infinite (motor stopped).
    pub fn period_us(&self) -> Option<u32> {
        if self.period_base == 0 {
            None // motor stopped
        } else {
            let period = (self.period_base as u32) << self.shift;
            if period == 0 {
                None
            } else {
                Some(period)
            }
        }
    }

    /// Returns the electrical RPM (eRPM).
    ///
    /// Computed as `60_000_000 / period_us`.
    /// Returns `0` if the motor is stopped (`period_base == 0`).
    pub fn erpm(&self) -> u32 {
        match self.period_us() {
            Some(period) if period > 0 => 60_000_000 / period,
            _ => 0,
        }
    }

    /// Returns the raw 16-bit value (for logging or retransmission).
    pub fn to_raw(&self) -> u16 {
        let payload = ((self.shift as u16) << 9) | (self.period_base & 0x1FF);
        let crc = self.crc as u16;
        (payload << 4) | crc
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX_DUTY_CYCLE: u16 = 100;
    const ZERO: u16 = 37;
    const ONE: u16 = 75;

    #[test]
    fn duty_cycles_works() {
        let frame = NormalFrame::new(999, false).unwrap();
        assert_eq!(
            frame.duty_cycles(MAX_DUTY_CYCLE),
            [
                ONE, ZERO, ZERO, ZERO, ZERO, ZERO, ONE, ZERO, ONE, ONE, ONE, ZERO, ZERO, ONE, ZERO,
                ZERO, 0
            ]
        );
    }

    #[test]
    fn duty_cycles_at_zero() {
        let frame = NormalFrame::command(Command::MotorStop, false);
        assert_eq!(
            frame.duty_cycles(MAX_DUTY_CYCLE),
            [
                ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,
                ZERO, ZERO, 0
            ]
        );
    }

    #[test]
    fn frame_constructs_correctly() {
        let frame = NormalFrame::new(998, false).unwrap();
        assert_eq!(frame.speed(), 998);
        assert!(!frame.telemetry_enabled());
        assert_eq!(frame.crc(), 0x06);
    }

    #[test]
    fn frame_constructs_correctly_with_telemetry() {
        let frame = NormalFrame::new(998, true).unwrap();
        assert_eq!(frame.speed(), 998);
        assert!(frame.telemetry_enabled());
        assert_eq!(frame.crc(), 0x07);
    }

    #[test]
    fn frame_constructs_correctly_off_centre() {
        let frame = NormalFrame::new(50, false).unwrap();
        assert_eq!(frame.speed(), 50);
    }

    #[test]
    fn frame_rejects_invalid_speed_values() {
        assert!(NormalFrame::new(2000, false).is_none())
    }

    #[test]
    fn bidirectional_throttle_works() {
        let thr = BidirectionalDshot::get_translated_throttle(999);
        assert_eq!(thr, 0b011_1110_1000_00000)
    }
}

