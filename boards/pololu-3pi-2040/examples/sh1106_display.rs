//! # Pololu 3pi+ 2040 SH1106 OLED Display Example
//!
//! This example demonstrates how to use an SH1106 OLED display with the Pololu 3pi+ 2040 robot.
//! The display shows robot status information including motor directions, encoder values,
//! and line sensor readings.
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
use embedded_hal::pwm::SetDutyCycle;
use panic_halt as _;
use pololu_3pi_2040::hal::prelude::*;
use pololu_3pi_2040::hal::pac;
use pololu_3pi_2040::hal;
use sh1106::{prelude::*, Builder};
use display_interface_spi::SPIInterface;

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
    let spi_mosi = pins.rgb_led_display_data.into_function::<hal::gpio::FunctionSpi>();
    let dc_pin = pins.button_c.into_push_pull_output();
    let cs_pin = pins.led.into_push_pull_output(); // Using LED pin as CS
    let res_pin = pins.led.into_push_pull_output(); // Using LED pin as RES

    // Initialize SPI
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_sck));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // Create display interface
    let interface = SPIInterface::new(spi, dc_pin, cs_pin);

    // Initialize SH1106 display
    let mut display = Builder::new()
        .size(DisplaySize::Display128x64)
        .connect_spi(interface)
        .into();

    // Reset display
    res_pin.set_low().unwrap();
    delay.delay_ms(10);
    res_pin.set_high().unwrap();
    delay.delay_ms(10);

    // Initialize display
    display.init().unwrap();

    // Configure motors
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

    // Create text style
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Motor state tracking
    let mut motor_state = 0;
    let mut counter = 0;

    info!("Starting SH1106 display demo on Pololu 3pi+ 2040");

    loop {
        // Clear display
        display.clear();

        // Display robot status
        Text::new("Pololu 3pi+ 2040", Point::new(0, 10), text_style)
            .draw(&mut display)
            .unwrap();

        Text::new("SH1106 Display Demo", Point::new(0, 20), text_style)
            .draw(&mut display)
            .unwrap();

        // Display motor state
        let motor_text = match motor_state {
            0 => "Motors: Forward",
            1 => "Motors: Backward", 
            2 => "Motors: Stop",
            _ => "Motors: Unknown",
        };
        Text::new(motor_text, Point::new(0, 30), text_style)
            .draw(&mut display)
            .unwrap();

        // Display counter
        let counter_text = format!("Counter: {}", counter);
        Text::new(&counter_text, Point::new(0, 40), text_style)
            .draw(&mut display)
            .unwrap();

        // Display line sensor info
        Text::new("Line sensors active", Point::new(0, 50), text_style)
            .draw(&mut display)
            .unwrap();

        // Update display
        display.flush().unwrap();

        // Control motors based on state
        match motor_state {
            0 => {
                // Forward
                direction_right.set_high().unwrap();
                direction_left.set_high().unwrap();
                channel_right.set_duty_cycle(0x4FFF).unwrap();
                channel_left.set_duty_cycle(0x4FFF).unwrap();
            }
            1 => {
                // Backward
                direction_right.set_low().unwrap();
                direction_left.set_low().unwrap();
                channel_right.set_duty_cycle(0x4FFF).unwrap();
                channel_left.set_duty_cycle(0x4FFF).unwrap();
            }
            2 => {
                // Stop
                channel_right.set_duty_cycle(0x0000).unwrap();
                channel_left.set_duty_cycle(0x0000).unwrap();
            }
            _ => {}
        }

        // Update state every 2 seconds
        delay.delay_ms(2000);
        motor_state = (motor_state + 1) % 3;
        counter += 1;

        info!("Motor state: {}, Counter: {}", motor_state, counter);
    }
}

// End of file