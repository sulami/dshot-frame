//! Support for the DShot ESC protocol
//!
//! DShot has two-byte frames, where the first 11 bits are the throttle speed, bit 12 is a
//! telemetry request flag, and the last four bits are a checksum.
//!
//! Throttle values below 48 are reserved for special commands.
//!
//! It is transmitted over the wire at a fixed speed, with ones and zeroes both being pulses, but
//! ones being twice as long as zeroes.

// TODO Special commands
//      Bidirectional DShot
//      embedded-hal-async

#![no_std]

use core::time::Duration;

use embedded_hal::{delay::DelayNs, digital::OutputPin};

/// How long to stay low between frames.
///
/// This is actually just a lower bound.
pub const INTER_FRAME_PAUSE: Duration = Duration::from_micros(2);

/// The protocol speed.
pub enum Speed {
    DShot150,
    DShot300,
    DShot600,
}

impl Speed {
    /// The time to hold the pin high for a set bit.
    fn one_hold_time(&self) -> Duration {
        match self {
            Self::DShot150 => Duration::from_nanos(5_000),
            Self::DShot300 => Duration::from_nanos(2_500),
            Self::DShot600 => Duration::from_nanos(1_250),
        }
    }

    /// The time to hold the pin high for an unset bit.
    fn zero_hold_time(&self) -> Duration {
        self.one_hold_time() / 2
    }

    /// The total time of one bit, both high and low.
    fn bit_time(&self) -> Duration {
        self.one_hold_time() / 3 * 4
    }
}

/// An error that occurred during DShot operation.
#[derive(Debug)]
pub enum Error<PIN>
where
    PIN: OutputPin,
{
    /// The provided throttle value was greater than 1999.
    OutOfBoundsError,
    /// A wrapped error occurred while trying to pull the pin high or low.
    PinError(PIN::Error),
}

/// A stateful DShot controller.
pub struct DShotController<PIN, DELAY>
where
    PIN: OutputPin,
    DELAY: DelayNs,
{
    pin: PIN,
    delay: DELAY,
    speed: Speed,
    throttle: u16,
    telemetry_enabled: bool,
}

impl<PIN, DELAY> DShotController<PIN, DELAY>
where
    PIN: OutputPin,
    DELAY: DelayNs,
{
    /// Creates a new DShot controller.
    ///
    /// Throttle defaults to zero, telemetry to off.
    pub fn new(pin: PIN, delay: DELAY, speed: Speed) -> Self {
        Self {
            pin,
            delay,
            speed,
            throttle: 0,
            telemetry_enabled: false,
        }
    }

    /// Sets the throttle and sends it over the wire.
    pub fn set_throttle(&mut self, throttle: u16) -> Result<(), Error<PIN>> {
        if throttle >= 2000 {
            return Err(Error::OutOfBoundsError);
        }

        self.send_throttle_command(throttle)?;
        self.throttle = throttle;

        Ok(())
    }

    /// Sends the current throttle over the wire.
    fn send_throttle_command(&mut self, throttle: u16) -> Result<(), Error<PIN>> {
        let frame = Frame::new(throttle, self.telemetry_enabled);
        let mut bits = frame.inner();

        for _ in 0..16 {
            self.pin.set_high().map_err(Error::PinError)?;

            let bit = bits & 0x01;
            let up_time = match bit {
                0 => self.speed.zero_hold_time(),
                1 => self.speed.one_hold_time(),
                _ => unreachable!(),
            };
            self.delay.delay_ns(up_time.as_nanos() as u32);

            self.pin.set_low().map_err(Error::PinError)?;

            let down_time = self.speed.bit_time() - up_time;
            self.delay.delay_ns(down_time.as_nanos() as u32);

            bits >>= 1;
        }

        Ok(())
    }

    /// Enables telemetry requests.
    ///
    /// Resends the current throttle value to actually let the ESC know.
    pub fn enable_telemetry(&mut self) -> Result<(), Error<PIN>> {
        self.telemetry_enabled = true;
        self.send_throttle_command(self.throttle)
    }

    /// Disables telemetry requests.
    ///
    /// Resends the current throttle value to actually let the ESC know.
    pub fn disable_telemetry(&mut self) -> Result<(), Error<PIN>> {
        self.telemetry_enabled = false;
        self.send_throttle_command(self.throttle)
    }
}

/// A frame of two bytes that get send over the wire.
struct Frame(u16);

impl Frame {
    /// Creates a new frame with the given throttle (0-1999) and telemetry request.
    fn new(throttle: u16, request_telemetry: bool) -> Self {
        debug_assert!(throttle < 2000);
        let translated_throttle = (throttle + 48) << 5;
        let mut cmd = Self(translated_throttle);
        if request_telemetry {
            cmd.0 |= 0x10;
        }
        cmd.compute_crc();
        cmd
    }

    /// Returns the throttle (0-1999).
    fn throttle(&self) -> u16 {
        (self.0 >> 5) - 48
    }

    /// Returns whether telemetry is enabled.
    fn telemetry_enabled(&self) -> bool {
        self.0 & 0x10 != 0
    }

    /// Returns the CRC checksum.
    fn crc(&self) -> u16 {
        self.0 & 0x0F
    }

    /// Computes the CRC based on the first 12 bits and ORs it in.
    fn compute_crc(&mut self) {
        let value = self.0 >> 4;
        let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
        self.0 |= crc;
    }

    /// Returns the raw [`u16`].
    fn inner(&self) -> u16 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_constructs_correctly() {
        let frame = Frame::new(998, false);
        assert_eq!(frame.throttle(), 998);
        assert!(!frame.telemetry_enabled());
        assert_eq!(frame.crc(), 0x06);
    }

    #[test]
    fn test_frame_constructs_correctly_with_telemetry() {
        let frame = Frame::new(998, true);
        assert_eq!(frame.throttle(), 998);
        assert!(frame.telemetry_enabled());
        assert_eq!(frame.crc(), 0x07);
    }
}
