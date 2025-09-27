//! # Pololu 3pi+ 2040 SH1106 OLED Display Example
//!
//! This example demonstrates how to use an SH1106 OLED display with the Pololu 3pi+ 2040 robot.
//! The display shows a simple counter that increments periodically.
//!
//! Hardware Connections:
//! - SH1106 VCC -> 3.3V
//! - SH1106 GND -> GND  
//! - SH1106 SCK -> GPIO2 (display_sck)
//! - SH1106 MOSI -> GPIO3 (rgb_led_display_data)
//! - SH1106 DC -> GPIO0 (button_c)
//! - SH1106 CS -> GPIO1 (unused pin, can be tied to GND)
//! - SH1106 RES -> GPIO25 (led pin, can be reconfigured)
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use panic_halt as _;
use pololu_3pi_2040::hal;
use pololu_3pi_2040::hal::pac;
use pololu_3pi_2040::hal::prelude::*;
use sh1106::Builder;

// For logging
use defmt::*;
use defmt_rtt as _;

/// Entry point to our bare-metal application.
#[pololu_3pi_2040::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
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

    // The delay object lets us wait for specified amounts of time
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pololu_3pi_2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure SPI for SH1106 display
    let spi_sck = pins.display_sck.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins
        .rgb_led_display_data
        .into_function::<hal::gpio::FunctionSpi>();
    let dc_pin = pins.button_c_display_dc.into_push_pull_output();
    let mut reset_pin = pins.display_reset.into_push_pull_output(); // Using bump_emitter pin as CS

    // Initialize SPI
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_sck));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        20_000_000u32.Hz(),
        embedded_hal::spi::MODE_0,
    );

    // Initialize SH1106 display
    let mut display: sh1106::mode::GraphicsMode<_> = Builder::new()
        .connect_spi(spi, dc_pin, sh1106::builder::NoOutputPin::new())
        .into();

    // Initialize display
    reset_pin.set_low().unwrap();
    delay.delay_us(10);
    reset_pin.set_high().unwrap();
    delay.delay_us(10);

    display.init().unwrap();
    display.flush().unwrap();

    // Create text style
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Counter for display
    let mut counter = 0;

    info!("Starting SH1106 display demo on Pololu 3pi+ 2040");

    loop {
        // Clear display
        display.clear();

        Text::new("SH1106 Display Demo", Point::new(0, 20), text_style)
            .draw(&mut display)
            .unwrap();

        // Display counter
        Text::new("COUNTER: ", Point::new(0, 30), text_style)
            .draw(&mut display)
            .unwrap();

        // Simple number formatting for no_std environment
        let mut buffer = [0u8; 10];
        let mut num = counter;
        let mut i = 0;

        if num == 0 {
            buffer[i] = b'0';
            i += 1;
        } else {
            while num > 0 && i < 10 {
                buffer[9 - i] = b'0' + (num % 10) as u8;
                num /= 10;
                i += 1;
            }
        }

        // Display the counter number
        let counter_num_text = core::str::from_utf8(&buffer[10 - i..10]).unwrap();
        Text::new(counter_num_text, Point::new(60, 30), text_style)
            .draw(&mut display)
            .unwrap();

        // Update display
        display.flush().unwrap();

        // Increment counter every 2 seconds
        delay.delay_ms(2000);
        counter += 1;

        info!("Counter: {}", counter);
    }
}

// End of file
