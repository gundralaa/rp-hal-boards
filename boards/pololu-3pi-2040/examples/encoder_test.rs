//! # Pico PIO PWM Blink Example
//!
//! Fades the LED on a Pico board using the PIO peripheral with an pwm program.
//!
//! This will fade in the LED attached to GP25, which is the pin the Pico
//! uses for the on-board LED.
//!
//! This example uses a few advance pio tricks such as side setting pins and instruction injection.
//!
//! See the `Cargo.toml` file for Copyright and license details. Except for the pio program which is subject to a different license.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
// The macro for our start-up function
use pololu_3pi_2040::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use pololu_3pi_2040::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pololu_3pi_2040::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pololu_3pi_2040::hal;

// Import pio crates
use hal::pio::{PIOBuilder, Rx, ValidStateMachine};
use pio_proc::pio_file;

fn get_encoder_count<T: ValidStateMachine>(rx: &mut Rx<T>) -> u32 {
    let mut count = 0;
    for i in 0..4 {
        if let Some(value) = rx.read() {
            count = value;
        } else {
            break;
        }
    }
    count
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pololu_3pi_2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio_file!("./examples/encoder.pio", select_program("encoder"));
    let installed_r = pio0.install(&program.program).unwrap();

    let _r_enc_a: hal::gpio::Pin<_, hal::gpio::FunctionPio0, hal::gpio::PullUp> =
        pins.right_encoder_a.reconfigure();
    let _r_enc_b: hal::gpio::Pin<_, hal::gpio::FunctionPio0, hal::gpio::PullUp> =
        pins.right_encoder_b.reconfigure();
    let r_enc_a_pin_id = 8;
    let r_enc_b_pin_id = 9;

    let (mut sm_right, mut rx_right, _) = PIOBuilder::from_installed_program(installed_r)
        .in_pin_base(r_enc_a_pin_id)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .autopull(false)
        .build(sm0);

    sm_right.set_pindirs([
        (r_enc_a_pin_id, hal::pio::PinDir::Input),
        (r_enc_b_pin_id, hal::pio::PinDir::Input),
    ]);
    sm_right.start();

    info!("Configured!");
    loop {
        info!("Right encoder: {:08x}", get_encoder_count(&mut rx_right));
        delay.delay_ms(1000);
    }
}

// End of file
