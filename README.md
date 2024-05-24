# DShot Frame

This crate provides support for the DShot ESC protocol.

DShot has two-byte frames, where the first 11 bits are the throttle speed, bit
12 is a telemetry request flag, and the last four bits are a checksum.

Throttle values below 48 are reserved for special commands.

It is transmitted over the wire at a fixed speed, with ones and zeroes both
being pulses, but ones being twice as long as zeroes.

## Usage

This example is adapted from an embassy-stm32 codebase:

```rust
async fn dshot(/* ... */) {
    let mut pwm = SimplePwm::new(
        timer,
        Some(PwmPin::new_ch1(pin, OutputType::PushPull)),
        None,
        None,
        None,
        Hertz(150_000),
        CountingMode::EdgeAlignedUp,
    );
    let max_duty_cycle = pwm.get_max_duty() as u16;

    let frame = Frame::new(1000, false).unwrap();
    pwm.waveform_up(&mut dma, Ch1, &frame.duty_cycles()).await;
    // Pull the line low after sending a frame.
    pwm.set_duty(channel, 0);
    pwm.enable(channel);
}
```
