//! # IMU Sensor Logger Example
//!
//! This application demonstrates how to read and log data from the onboard IMU sensors
//! on the Pololu 3pi+ 2040 robot. The example reads acceleration and gyroscope data
//! from the LSM6DSO sensor and magnetometer data from the sensor, then logs
//! all values periodically using defmt.
//!
//! ## Hardware Configuration
//!
//! The IMU sensors are connected via I2C:
//! * GPIO 4 - I2C SDA (data line) - connected to both LSM6DSO
//! * GPIO 5 - I2C SCL (clock line) - connected to both LSM6DSO
//! * GPIO 25 - LED indicator (blinks during operation)
//!
//! ## Sensors
//!
//! * **LSM6DSO**: STMicroelectronics 6-axis IMU providing:
//!   - 3-axis accelerometer (±2g to ±16g selectable)
//!   - 3-axis gyroscope (±125dps to ±2000dps selectable)
//!
//! ## I2C Configuration
//!
//! * Clock frequency: 400 kHz (fast mode)
//! * Standard I2C addressing
//!
//! ## Usage
//!
//! Run this example and monitor the defmt output to see continuous sensor readings.
//! The LED will blink to indicate active sensor reading.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use pololu_3pi_2040::entry;

// GPIO traits
use embedded_hal::digital::OutputPin;

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

// For logging
use defmt::*;
use defmt_rtt as _;

use lsm6dso::{self, AccelerometerOutput, AccelerometerScale, GyroscopeFullScale, GyroscopeOutput};

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, initializes the IMU sensors,
/// then continuously reads and logs sensor data.
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

    // Configure I2C pins
    let sda_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.imu_sda.reconfigure();
    let scl_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.imu_scl.reconfigure();

    // Initialize I2C
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // Set the LED to be an output for activity indication
    let mut led_pin = pins.led.into_push_pull_output();

    // Send initial startup message
    info!("Pololu 3pi+ 2040 IMU Sensor Logger");
    info!("Initializing LSM6DSO (accelerometer/gyroscope)...");

    // Initialize LSM6DSO sensor (accelerometer and gyroscope)
    // Default I2C address for LSM6DSO depends on SDO/SA0 pin: 0x6A or 0x6B
    let mut lsm6dso = lsm6dso::Lsm6dso::new(i2c, 0x6B);
    if let Err(e) = lsm6dso.check() {
        warn!("LSM6DSO WHO_AM_I check failed: {}", defmt::Debug2Format(&e));
    } else {
        info!("LSM6DSO initialized successfully");
    }

    // Configure LSM6DSO settings
    if let Err(e) = lsm6dso.set_accelerometer_scale(AccelerometerScale::G16) {
        error!(
            "Failed to set accelerometer scale: {}",
            defmt::Debug2Format(&e)
        );
    }
    if let Err(e) = lsm6dso.set_gyroscope_scale(GyroscopeFullScale::Dps2000) {
        error!("Failed to set gyroscope scale: {}", defmt::Debug2Format(&e));
    }
    if let Err(e) = lsm6dso.set_accelerometer_output(AccelerometerOutput::Rate104) {
        error!(
            "Failed to set accelerometer output rate: {}",
            defmt::Debug2Format(&e)
        );
    }
    if let Err(e) = lsm6dso.set_gyroscope_output(GyroscopeOutput::Rate104) {
        error!(
            "Failed to set gyroscope output rate: {}",
            defmt::Debug2Format(&e)
        );
    }

    info!("IMU sensors configured successfully");
    info!("Starting continuous sensor data logging...");
    info!("----------------------------------------");

    let mut counter = 0u32;

    // Main loop - read and log sensor data
    loop {
        // Blink LED to show activity
        led_pin.set_high().unwrap();

        // Log sensor reading header
        info!("=== Sensor Reading #{} ===", counter);

        // Read accelerometer data (m/s^2)
        match lsm6dso.read_accelerometer() {
            Ok((ax, ay, az)) => {
                info!("Accelerometer (m/s²): X={}, Y={}, Z={}", ax, ay, az);
            }
            Err(e) => {
                error!(
                    "Failed to read accelerometer data: {}",
                    defmt::Debug2Format(&e)
                );
            }
        }

        // Read gyroscope data (rad/s)
        match lsm6dso.read_gyro() {
            Ok((gx, gy, gz)) => {
                info!("Gyroscope (rad/s): X={}, Y={}, Z={}", gx, gy, gz);
            }
            Err(e) => {
                error!("Failed to read gyroscope data: {}", defmt::Debug2Format(&e));
            }
        }

        info!("");

        delay.delay_ms(100);
        led_pin.set_low().unwrap();

        // Wait before next reading (total cycle time ~1 second)
        delay.delay_ms(900);

        counter = counter.wrapping_add(1);

        // Send periodic status message
        if counter % 10 == 0 {
            info!("Status: {} sensor readings completed", counter);
            info!("----------------------------------------");
        }
    }
}

// End of file
