//! QTR Reflectance sensor reader example
//!
//! Uses PIO to read the Pololu QTR reflectance sensors and prints the
//! measured reflectance counts over RTT using `defmt`.

#![no_std]
#![no_main]


use defmt::info;
use defmt_rtt as _;
use panic_halt as _;

use pololu_3pi_2040::entry;

use embedded_hal::digital::OutputPin;
use pio::{
    Instruction, InstructionOperands, MovDestination, MovOperation, MovSource, OutDestination,
};
use pio_proc::pio_file;

use pololu_3pi_2040::hal;
use pololu_3pi_2040::hal::pio::{PIOBuilder, Rx, ValidStateMachine};
use pololu_3pi_2040::hal::prelude::*;

use pololu_3pi_2040::pac;

fn read_qtr_counts<T: ValidStateMachine>(rx: &mut Rx<T>) -> [u32; 5] {
    const TIMEOUT: u32 = 1024;
    let mut line_sensors = [TIMEOUT; 5];
    let mut last_state = 0xFF_u8;

    for i in 0..10000 {
        // Simple timeout with max iterations
        if let Some(data) = rx.read() {
            if data == 0xFFFFFFFF {
                //info!("read success");
                break; // End marker
            }

            let time_left = data & 0xFFFF;
            let state = ((data >> 16) & 0x1F) as u8;
            let new_zeros = last_state & !state;
            
            //info!("time left: {:08x}", time_left);
            //info!("state: {:08x}", state);
            //info!("trans: {:08x}", new_zeros);

            // Record discharge times for pins that just went LOW
            if new_zeros & (1 << 0) != 0 {
                line_sensors[4] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 1) != 0 {
                line_sensors[3] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 2) != 0 {
                line_sensors[2] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 3) != 0 {
                line_sensors[1] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 4) != 0 {
                line_sensors[0] = TIMEOUT - time_left;
            }

            last_state = state;
        }
        cortex_m::asm::delay(10);
    }
    
    line_sensors
}

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
    let pins = pololu_3pi_2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut line_emitter = pins.line_emitter.into_push_pull_output();
    line_emitter.set_high().unwrap();
    
    let (mut pio1, sm0, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let program = pio_file!(
        "./examples/qtr_light.pio",
        select_program("qtr_sensor_counter")
    );
    let installed = pio1.install(&program.program).unwrap();

    // Base pin is GPIO16 (first sensor pin)
    let base_pin = 18u8;
    let (mut sm, mut rx, _tx) = PIOBuilder::from_installed_program(installed)
        .out_pins(base_pin, 5)
        .in_pin_base(base_pin)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .autopush(true)
        .push_threshold(21) // Matches C code
        .clock_divisor_fixed_point(15, 160) // Matches C code: 8 MHz
        .buffers(hal::pio::Buffers::OnlyRx) // CRITICAL: Join FIFOs to RX only like C code
        .build(sm0);

    info!("QTR reader configured, emitter enabled");

    // Give the sensors some time to stabilize
    delay.delay_ms(100);

    let mut line_1 = pins.line_1;
    let mut line_2 = pins.line_2;
    let mut line_3 = pins.line_3;
    let mut line_4 = pins.line_4;
    let mut line_5 = pins.line_5;

    loop {
        
        sm.clear_fifos();

        // Charge the sensor capacitors
        let mut _line_1 = line_1.into_push_pull_output();
        let mut _line_2 = line_2.into_push_pull_output();
        let mut _line_3 = line_3.into_push_pull_output();
        let mut _line_4 = line_4.into_push_pull_output();
        let mut _line_5 = line_5.into_push_pull_output();

        _line_1.set_high().unwrap();
        _line_2.set_high().unwrap();
        _line_3.set_high().unwrap();
        _line_4.set_high().unwrap();
        _line_5.set_high().unwrap();

        // Wait for the capacitors to charge
        delay.delay_us(32);

        // Set the pins to floating input
        let mut _line_1 = _line_1.into_floating_input();
        let mut _line_2 = _line_2.into_floating_input();
        let mut _line_3 = _line_3.into_floating_input();
        let mut _line_4 = _line_4.into_floating_input();
        let mut _line_5 = _line_5.into_floating_input();

        sm.set_pindirs([
            (base_pin, hal::pio::PinDir::Input),
            (base_pin + 1, hal::pio::PinDir::Input),
            (base_pin + 2, hal::pio::PinDir::Input),
            (base_pin + 3, hal::pio::PinDir::Input),
            (base_pin + 4, hal::pio::PinDir::Input),
        ]);

        // Now start the properly initialized state machine
        // Measure the discharge count
        let sm_running = sm.start();

        delay.delay_ms(10);

        let counts = read_qtr_counts(&mut rx);
        let mut avg_counts = [0; 5];
        for i in 0..5 {
            avg_counts[i] = (avg_counts[i] * (3) + counts[i] * (7)) / 10;
        }
        info!(
            "QTR values: [L1:{}, L2:{}, L3:{}, L4:{}, L5:{}]",
            avg_counts[0], avg_counts[1], avg_counts[2], avg_counts[3], avg_counts[4]
        );

        // Stop the state machine
        sm = sm_running.stop();
        
        line_1 = _line_1.reconfigure();
        line_2 = _line_2.reconfigure();
        line_3 = _line_3.reconfigure();
        line_4 = _line_4.reconfigure();
        line_5 = _line_5.reconfigure();

        delay.delay_ms(500);
    }
}
