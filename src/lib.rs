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
//! let frame = Frame::new(1000, false).unwrap();
//! pwm.waveform_up(&mut dma, Ch1, &frame.duty_cycles()).await;
//! // Pull the line low after sending a frame.
//! pwm.set_duty(channel, 0);
//! pwm.enable(channel);
//! ```

// TODO Bidirectional DShot

#![no_std]

/// A frame of two bytes that get send over the wire.
#[derive(Copy, Clone, Debug)]
pub struct Frame {
    inner: u16,
}

impl Frame {
    /// Creates a new frame with the given speed (0-1999) and telemetry request.
    ///
    /// Returns [`None`] if the speed is out of bounds.
    ///
    /// ```
    /// # use dshot_frame::*;
    /// assert_eq!(Frame::new(1000, false).unwrap().speed(), 1000);
    /// ```
    pub fn new(speed: u16, request_telemetry: bool) -> Option<Self> {
        if speed >= 2000 {
            return None;
        }

        let translated_throttle = (speed + 48) << 5;
        let mut frame = Self {
            inner: translated_throttle,
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
        let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
        self.inner |= crc;
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

#[cfg(test)]
mod tests {
    use super::*;

    const MAX_DUTY_CYCLE: u16 = 100;
    const ZERO: u16 = 37;
    const ONE: u16 = 75;

    #[test]
    fn duty_cycles_works() {
        let frame = Frame::new(999, false).unwrap();
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
        let frame = Frame::command(Command::MotorStop, false);
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
        let frame = Frame::new(998, false).unwrap();
        assert_eq!(frame.speed(), 998);
        assert!(!frame.telemetry_enabled());
        assert_eq!(frame.crc(), 0x06);
    }

    #[test]
    fn frame_constructs_correctly_with_telemetry() {
        let frame = Frame::new(998, true).unwrap();
        assert_eq!(frame.speed(), 998);
        assert!(frame.telemetry_enabled());
        assert_eq!(frame.crc(), 0x07);
    }

    #[test]
    fn frame_constructs_correctly_off_centre() {
        let frame = Frame::new(50, false).unwrap();
        assert_eq!(frame.speed(), 50);
    }

    #[test]
    fn frame_rejects_invalid_speed_values() {
        assert!(Frame::new(2000, false).is_none())
    }
}
