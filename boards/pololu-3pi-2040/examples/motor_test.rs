//! # Pico PWM Micro Servo Example
//!
//! Moves the micro servo on a Pico board using the PWM peripheral.
//!
//! This will move in different positions the motor attached to GP1.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use cortex_m::prelude::*;

// GPIO traits
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use pololu_3pi_2040::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pololu_3pi_2040::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pololu_3pi_2040::hal;

// For logging
use defmt::*;
use defmt_rtt as _;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        pololu_3pi_2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let sio = hal::Sio::new(pac.SIO);
    let pins = pololu_3pi_2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Motor configuration
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm7;
    pwm.set_div_int(10u8);
    pwm.set_div_frac(0u8);
    pwm.enable();
    let channel_right = &mut pwm.channel_a;
    channel_right.output_to(pins.right_motor_pwm);
    let channel_left = &mut pwm.channel_b;
    channel_left.output_to(pins.left_motor_pwm);
    let mut direction_right = pins.right_motor_dir.into_push_pull_output();
    let mut direction_left = pins.left_motor_dir.into_push_pull_output();

    // Encoder configuration

    loop {
        info!("forward");
        direction_right.set_high().unwrap();
        direction_left.set_high().unwrap();

        // lower bound
        // 0x0FFF -> low power
        // 0x8FFF -> high power
        channel_right.set_duty_cycle(0x4FFF).unwrap();
        channel_left.set_duty_cycle(0x4FFF).unwrap();

        delay.delay_ms(5000);

        info!("backward");
        direction_right.set_low().unwrap();
        direction_left.set_low().unwrap();

        channel_right.set_duty_cycle(0x0FFF).unwrap();
        channel_left.set_duty_cycle(0x0FFF).unwrap();

        delay.delay_ms(5000);
    }
}

// End of file
