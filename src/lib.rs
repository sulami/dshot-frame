//! Support for the DShot ESC protocol
//!
//! This crate provides
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

#![no_std]

/// A frame of two bytes that get send over the wire.
pub struct Frame {
    inner: u16,
}

impl Frame {
    /// Creates a new frame with the given speed (0-1999) and telemetry request.
    ///
    /// Returns [`None`] if the speed is out of bounds.
    pub fn new(speed: u16, request_telemetry: bool) -> Option<Self> {
        if speed >= 2000 {
            return None;
        }

        let translated_throttle = (speed + 48) << 5;
        let mut cmd = Self {
            inner: translated_throttle,
        };
        if request_telemetry {
            cmd.inner |= 0x10;
        }
        cmd.compute_crc();
        Some(cmd)
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
        let mut value = self.inner.reverse_bits();
        let mut rv = [max_duty_cycle * 3 / 4; 17];
        for item in rv.iter_mut() {
            let bit = value & 0x0001;
            if bit != 0 {
                *item = max_duty_cycle * 3 / 8;
            }
            value >>= 1;
        }
        rv[16] = 0;
        rv
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bits_works() {
        let frame = Frame::new(999, false).unwrap();
        assert_eq!(
            frame.duty_cycles(100),
            [37, 75, 75, 75, 75, 75, 37, 75, 37, 37, 37, 75, 75, 37, 75, 75, 0]
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
    fn frame_rejects_invalid_speed_values() {
        assert!(Frame::new(2000, false).is_none())
    }
}
