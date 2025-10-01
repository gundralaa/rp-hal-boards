//! Line sensors HAL usage example
//!
//! Demonstrates using the `line_sensors` module to read the 5 QTR-style
//! reflectance sensors on the Pololu 3pi+ 2040. Values are printed over RTT.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_halt as _;
use pololu_3pi_2040::entry;

use embedded_hal::digital::OutputPin;
use pololu_3pi_2040::hal;
use pololu_3pi_2040::hal::prelude::*;
use pololu_3pi_2040::pac;

use pololu_3pi_2040::line_sensors::{LineSensorDelays, LineSensorPioConfig, LineSensors};
use embedded_hal::delay::DelayNs;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    let sio = hal::Sio::new(pac.SIO);
    let mut delay = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = pololu_3pi_2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Enable the line emitter LED (critical for QTR sensors)
    let mut line_emitter = pins.line_emitter.into_push_pull_output();
    line_emitter.set_high().ok();

    // Configure PIO1 and a state machine
    let (mut pio1, sm0, _, _, _) = pac.PIO1.split(&mut pac.RESETS);

    // Choose timings (can be tuned here)
    let delays = LineSensorDelays { stabilize_ms: 100, precharge_us: 32, measurement_wait_ms: 10 };

    let pio_cfg = LineSensorPioConfig {
        clock_div_int: 15,
        clock_div_frac: 160,
        timeout_power: 10,
    };

    // Create HAL instance. Pins are consumed and owned by the reader.
    let mut sensors = LineSensors::new(
        &mut pio1,
        sm0,
        pins.right_bump,
        pins.left_bump,
        pins.line_5,
        pins.line_4,
        pins.line_3,
        pins.line_2,
        pins.line_1,
        delays,
        pio_cfg,
    );

    // Let the emitter light stabilize
    delay.delay_ms(delays.stabilize_ms);

    info!("Line sensors HAL configured; starting measurements");

    loop {
        let counts = sensors.read_once(&mut delay);
        info!(
            "QTR values: [L1:{}, L2:{}, L3:{}, L4:{}, L5:{}]",
            counts[0], counts[1], counts[2], counts[3], counts[4]
        );

        delay.delay_ms(250);
    }
}
