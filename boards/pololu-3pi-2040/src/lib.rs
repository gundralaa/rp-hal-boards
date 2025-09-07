#![no_std]

//! A Hardware Abstraction Layer for the Raspberry Pi Pico.
//!
//! This crate serves as a HAL (Hardware Abstraction Layer) for the Raspberry Pi Pico. Since the Raspberry Pi Pico
//! is based on the RP2040 chip, it re-exports the [rp2040_hal] crate which contains the tooling to work with the
//! rp2040 chip.
//!
//! # Examples:
//!
//! The following example turns on the onboard LED. Note that most of the logic works through the [rp2040_hal] crate.
//! ```ignore
//! #![no_main]
//! use rp_pico::entry;
//! use panic_halt as _;
//! use embedded_hal::digital::v2::OutputPin;
//! use rp_pico::hal::pac;
//! use rp_pico::hal;
//!
//! #[entry]
//! fn does_not_have_to_be_main() -> ! {
//!   let mut pac = pac::Peripherals::take().unwrap();
//!   let sio = hal::Sio::new(pac.SIO);
//!   let pins = rp_pico::Pins::new(
//!        pac.IO_BANK0,
//!        pac.PADS_BANK0,
//!        sio.gpio_bank0,
//!        &mut pac.RESETS,
//!   );
//!   let mut led_pin = pins.led.into_push_pull_output();
//!   led_pin.set_high().unwrap();
//!   loop {
//!   }
//! }
//! ```

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;

/// The `entry` macro declares the starting function to the linker.
/// This is similar to the `main` function in console applications.
///
/// It is based on the [cortex_m_rt](https://docs.rs/cortex-m-rt/latest/cortex_m_rt/attr.entry.html) crate.
///
/// # Examples
/// ```ignore
/// #![no_std]
/// #![no_main]
/// use rp_pico::entry;
/// #[entry]
/// fn you_can_use_a_custom_main_name_here() -> ! {
///   loop {}
/// }
/// ```
#[cfg(feature = "rt")]
pub use hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

hal::bsp_pins!(
    Gpio0 { name: button_c },
    Gpio2 {
        name: display_sck,
        aliases: { FunctionSpi, PullNone: Gp2Spi0Sck }
    },
    Gpio3 {
        name: rgb_led_display_data,
        aliases: { FunctionSpi, PullNone: Gp3Spi0Tx }
    },

    Gpio4 {
        name: imu_sda,
        aliases: { FunctionI2C, PullUp: Gp4I2C0Sda }
    },

    Gpio5 {
        name: imu_scl,
        aliases: { FunctionI2C, PullUp: Gp5I2C0Scl }
    },

    Gpio6 {
        name: rgb_led_sck,
        aliases: { FunctionSpi, PullNone: Gp6Spi0Sck }
    },

    Gpio7 {
        name: buzzer_pwm,
        aliases: { FunctionPwm, PullNone: Gp7Pwm3B }
    },

    Gpio8 {
        name: right_encoder_a,
        aliases: { FunctionPio0, PullNone: Gp8Pio0 }
    },

    Gpio9 {
        name: right_encoder_b,
        aliases: { FunctionPio0, PullNone: Gp9Pio0 }
    },

    Gpio10 { name: right_motor_dir },

    Gpio11 { name: left_motor_dir },

    Gpio12 {
        name: left_encoder_a,
        aliases: { FunctionPio0, PullNone: Gp12Pio0 }
    },

    Gpio13 {
        name: left_encoder_b,
        aliases: { FunctionPio0, PullNone: Gp13Pio0 }
    },

    Gpio14 {
        name: right_motor_pwm,
        aliases: { FunctionPwm, PullNone: Gp14Pwm7A }
    },

    Gpio15 {
        name: left_motor_pwm,
        aliases: { FunctionPwm, PullNone: Gp15Pwm7B }
    },

    Gpio16 {
        name: right_bump,
        aliases: { FunctionPio1, PullNone: Gp16Pio1 }
    },

    Gpio17 {
        name: left_bump,
        aliases: { FunctionPio1, PullNone: Gp17Pio1 }
    },

    Gpio18 {
        name: line_5,
        aliases: { FunctionPio1, PullNone: Gp18Pio1 }
    },

    Gpio19 {
        name: line_4,
        aliases: { FunctionPio1, PullNone: Gp19Pio1 }
    },

    Gpio20 {
        name: line_3,
        aliases: { FunctionPio1, PullNone: Gp20Pio1 }
    },

    Gpio21 {
        name: line_2,
        aliases: { FunctionPio1, PullNone: Gp21Pio1 }
    },

    Gpio22 {
        name: line_1,
        aliases: { FunctionPio1, PullNone: Gp22Pio1 }
    },

    Gpio23 { name: bump_emitter },

    Gpio25 { name: led },

    Gpio26 { name: line_emitter },

    Gpio28 { name: uart_tx,
        aliases: { FunctionUart, PullNone: Gp28Uart0Tx }
    },

    Gpio29 { name: uart_rx,
        aliases: { FunctionUart, PullNone: Gp29Uart0Rx }
    }
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
