//! # UART Hello World Example
//!
//! This application demonstrates how to use the UART Driver to send a simple
//! "Hello, World!" message over the serial connection.
//!
//! The pinouts are:
//!
//! * GPIO 28 - UART TX (out of the RP2040)
//! * GPIO 29 - UART RX (in to the RP2040)
//! * GPIO 25 - An LED we can blink (active high)
//!
//! UART Configuration: 115200 baud, 8 data bits, 1 stop bit, no parity
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use pololu_3pi_2040::entry;

// GPIO traits
use embedded_hal::digital::OutputPin;

// UART traits
use embedded_hal_nb::serial::Write;

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

// Time handling traits
use fugit::RateExtU32;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

// For logging
use defmt::*;
use defmt_rtt as _;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then sends "Hello, World!"
/// messages over UART in an infinite loop.
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

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pololu_3pi_2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure UART pins
    let uart_pins = (
        // UART TX (characters sent from RP2040) on GPIO 28
        pins.uart_tx.into_function(),
        // UART RX (characters received by RP2040) on GPIO 29
        pins.uart_rx.into_function(),
    );

    // Make a UART on the given pins
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Send initial startup message
    uart.write_full_blocking(b"Pololu 3pi 2040 UART Hello World!\r\n");
    uart.write_full_blocking(b"Starting UART communication...\r\n");
    uart.write_full_blocking(b"Baud rate: 115200\r\n");
    uart.write_full_blocking(b"TX: GPIO 28, RX: GPIO 29\r\n");
    uart.write_full_blocking(b"----------------------------------------\r\n");

    let mut counter = 0u32;

    // Main loop - send hello world messages and blink LED
    loop {
        // Blink LED to show activity
        led_pin.set_high().unwrap();
        info!("LED on, sending message {}", counter);
        
        // Send hello world message
        uart.write_full_blocking(b"Hello, World! ");
        uart.write_full_blocking(counter.to_string().as_bytes());
        uart.write_full_blocking(b" from Pololu 3pi 2040!\r\n");
        
        delay.delay_ms(500);
        
        led_pin.set_low().unwrap();
        info!("LED off");
        
        delay.delay_ms(500);
        
        counter = counter.wrapping_add(1);
        
        // Send periodic status message
        if counter % 10 == 0 {
            uart.write_full_blocking(b"Status: System running normally\r\n");
        }
    }
}

// End of file