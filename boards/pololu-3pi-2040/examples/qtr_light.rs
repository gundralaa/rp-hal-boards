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

    for _ in 0..1000 {
        // Simple timeout with max iterations
        if let Some(data) = rx.read() {
            if data == 0xFFFFFFFF {
                break; // End marker
            }

            let time_left = data & 0xFFFF;
            let state = ((data >> 16) & 0x7F) as u8;
            let new_zeros = last_state & !state;

            // Record discharge times for pins that just went LOW
            if new_zeros & (1 << 2) != 0 {
                line_sensors[4] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 3) != 0 {
                line_sensors[3] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 4) != 0 {
                line_sensors[2] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 5) != 0 {
                line_sensors[1] = TIMEOUT - time_left;
            }
            if new_zeros & (1 << 6) != 0 {
                line_sensors[0] = TIMEOUT - time_left;
            }

            last_state = state;
        }
        cortex_m::asm::delay(100);
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

    // Configure and turn on the line emitter (critical for QTR sensors!)
    let mut line_emitter = pins.line_emitter.into_push_pull_output();
    line_emitter.set_high().unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Prepare PIO
    let (mut pio1, sm0, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let program = pio_file!(
        "./examples/qtr_light.pio",
        select_program("qtr_sensor_counter")
    );
    let installed = pio1.install(&program.program).unwrap();

    // Configure all 7 sensor pins for PIO (pins 16-22) initially
    let mut bump_right = pins.right_bump;
    let mut bump_left = pins.left_bump;
    let mut l5 = pins.line_5;
    let mut l4 = pins.line_4;
    let mut l3 = pins.line_3;
    let mut l2 = pins.line_2;
    let mut l1 = pins.line_1;

    // Base pin is GPIO16 (first sensor pin)
    let base_pin = 16u8;

    let (mut sm, mut rx, _tx) = PIOBuilder::from_installed_program(installed)
        .out_pins(base_pin, 7)
        .in_pin_base(base_pin)
        .in_shift_direction(hal::pio::ShiftDirection::Right)
        .autopush(false)
        .push_threshold(23) // Matches C code
        .clock_divisor_fixed_point(15, 160) // Matches C code: 8 MHz
        .buffers(hal::pio::Buffers::OnlyRx) // CRITICAL: Join FIFOs to RX only like C code
        .build(sm0);

    info!("QTR reader configured, emitter enabled");

    // Give the sensors some time to stabilize
    delay.delay_ms(100);

    loop {
        info!("Starting new QTR measurement");

        // CRITICAL: Pre-charge the sensor capacitors by driving pins HIGH
        // This matches the C code behavior in lines 55-56 and 66

        // Configure pins as outputs and drive HIGH to charge capacitors
        let mut pin16 = bump_right.into_push_pull_output();
        let mut pin17 = bump_left.into_push_pull_output();
        let mut pin18 = l5.into_push_pull_output();
        let mut pin19 = l4.into_push_pull_output();
        let mut pin20 = l3.into_push_pull_output();
        let mut pin21 = l2.into_push_pull_output();
        let mut pin22 = l1.into_push_pull_output();

        // Drive all pins HIGH (charge phase)
        pin16.set_high().unwrap();
        pin17.set_high().unwrap();
        pin18.set_high().unwrap();
        pin19.set_high().unwrap();
        pin20.set_high().unwrap();
        pin21.set_high().unwrap();
        pin22.set_high().unwrap();

        // Critical 32μs delay to charge the capacitors (matches C code line 66)
        delay.delay_us(32);

        // Reconfigure pins back to PIO function
        bump_right = pin16.reconfigure();
        bump_left = pin17.reconfigure();
        l5 = pin18.reconfigure();
        l4 = pin19.reconfigure();
        l3 = pin20.reconfigure();
        l2 = pin21.reconfigure();
        l1 = pin22.reconfigure();

        // Clear FIFOs before starting
        sm.clear_fifos();

        // CRITICAL: Initialize Y register to 1023 using blocking execution (matches C code lines 79-84)
        // mov osr, !null - loads 0xFFFFFFFF into OSR

        sm.exec_instruction(Instruction {
            operands: InstructionOperands::MOV {
                destination: MovDestination::OSR,
                op: MovOperation::Invert,
                source: MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        // out y, 10 - moves lower 10 bits (1023) from OSR to Y register
        sm.exec_instruction(Instruction {
            operands: InstructionOperands::OUT {
                destination: OutDestination::Y,
                bit_count: 10,
            },
            delay: 0,
            side_set: None,
        });

        // mov osr, null - clears OSR to 0
        sm.exec_instruction(Instruction {
            operands: InstructionOperands::MOV {
                destination: MovDestination::OSR,
                op: MovOperation::None,
                source: MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        // Now start the properly initialized state machine
        let sm_running = sm.start();

        // Wait for the measurement to complete
        delay.delay_ms(10);

        let counts = read_qtr_counts(&mut rx);
        info!(
            "QTR values: [L1:{}, L2:{}, L3:{}, L4:{}, L5:{}]",
            counts[0], counts[1], counts[2], counts[3], counts[4]
        );

        // Stop the state machine
        sm = sm_running.stop();

        delay.delay_ms(500);
    }
}
