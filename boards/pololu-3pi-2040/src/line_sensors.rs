//! Line sensor HAL for Pololu 3pi+ 2040 (QTR-style reflectance sensors)
//!
//! This module provides a small, typed wrapper around the RP2040 PIO-based
//! reader used for the Pololu QTR reflectance sensors on the 3pi+ 2040 robot.
//! It closely follows the logic from the C reference and the `qtr_light` example,
//! but bundles setup and measurement into a reusable API with tunable timings.
//!
//! Notes:
//! - Uses PIO1 and a single state machine supplied by the caller
//! - Reads the 7 inputs on GPIO 16..22 (2 bump + 5 line sensors)
//! - Returns only the 5 line sensor readings in L1..L5 order
//! - Expects the line emitter LED to be handled by the caller
//!
//! Default timings match the Pololu example: 32 µs precharge, 8 MHz PIO clock,
//! and a timeout of 1024 PIO loop iterations. You can adjust delays at init.

use crate::{
    hal,
    hal::pio::{Buffers, PIOBuilder, Rx, ShiftDirection, StateMachine, StateMachineIndex, Stopped},
};
use hal::gpio::{bank0::*, FunctionNull, FunctionPio1, PullDown, Pin};

use embedded_hal::{delay::DelayNs, digital::OutputPin};
use pio::{Instruction, InstructionOperands, MovDestination, MovOperation, MovSource, OutDestination};
use pio_proc::pio_file;

/// Tunable timing parameters for the sensor measurement loop.
#[derive(Clone, Copy)]
pub struct LineSensorDelays {
    /// Waiting time after enabling emitter before first measurement (ms)
    pub stabilize_ms: u32,
    /// Time to charge the sensor capacitors by driving pins HIGH (µs)
    pub precharge_us: u32,
    /// Wait time after starting the PIO state machine to ensure completion (ms)
    pub measurement_wait_ms: u32,
}

impl Default for LineSensorDelays {
    fn default() -> Self {
        Self {
            stabilize_ms: 100,
            precharge_us: 32,
            measurement_wait_ms: 10,
        }
    }
}

/// Static configuration for the PIO program.
#[derive(Clone, Copy)]
pub struct LineSensorPioConfig {
    /// PIO clock divisor: integer part.
    pub clock_div_int: u16,
    /// PIO clock divisor: fractional part (out of 256).
    pub clock_div_frac: u8,
    /// Timeout power-of-two for the PIO decrementing counter Y.
    /// Y will be initialized to (2^timeout_power - 1).
    /// Default 10 => timeout of 1024 ticks.
    pub timeout_power: u8,
}

impl Default for LineSensorPioConfig {
    fn default() -> Self {
        Self {
            // 125 MHz / (15 + 160/256) ~= 8 MHz PIO clock
            clock_div_int: 15,
            clock_div_frac: 160,
            timeout_power: 10, // 2^10 = 1024
        }
    }
}

/// HAL wrapper that owns the PIO state machine, RX FIFO and pins.
///
/// T is the concrete state machine token type (e.g., `(PIO1, SM0)`).
pub struct LineSensors<SMI: StateMachineIndex> {
    sm: Option<StateMachine<(crate::pac::PIO1, SMI), Stopped>>,
    rx: Rx<(crate::pac::PIO1, SMI)>,

    // Sensor pins configured for PIO1 function (store with PullDown, matches board defaults)
    bump_right: Option<Pin<Gpio16, FunctionPio1, PullDown>>,
    bump_left: Option<Pin<Gpio17, FunctionPio1, PullDown>>,
    l5: Option<Pin<Gpio18, FunctionPio1, PullDown>>,
    l4: Option<Pin<Gpio19, FunctionPio1, PullDown>>,
    l3: Option<Pin<Gpio20, FunctionPio1, PullDown>>,
    l2: Option<Pin<Gpio21, FunctionPio1, PullDown>>,
    l1: Option<Pin<Gpio22, FunctionPio1, PullDown>>,

    delays: LineSensorDelays,
    pio_cfg: LineSensorPioConfig,
}

impl<SMI: StateMachineIndex> LineSensors<SMI> {
    /// Install the QTR PIO program into `pio1`, build a state machine on `sm`,
    /// and create a new `LineSensors` HAL instance.
    ///
    /// The emitter LED is not controlled here; enable it separately before use.
    pub fn new(
        pio1: &mut hal::pio::PIO<crate::pac::PIO1>,
        sm: hal::pio::UninitStateMachine<(crate::pac::PIO1, SMI)>,
        bump_right: Pin<Gpio16, FunctionNull, PullDown>,
        bump_left: Pin<Gpio17, FunctionNull, PullDown>,
        l5: Pin<Gpio18, FunctionNull, PullDown>,
        l4: Pin<Gpio19, FunctionNull, PullDown>,
        l3: Pin<Gpio20, FunctionNull, PullDown>,
        l2: Pin<Gpio21, FunctionNull, PullDown>,
        l1: Pin<Gpio22, FunctionNull, PullDown>,
        delays: LineSensorDelays,
        pio_cfg: LineSensorPioConfig,
    ) -> Self {
        // Embed and install the PIO program (path relative to this crate root)
        let program = pio_file!("qtr_ex.pio", select_program("qtr_sensor_counter"));
        let installed = pio1.install(&program.program).expect("Failed to install PIO program");

        // Base pin is GPIO 16 (first sensor pin)
        let base_pin: u8 = 16;

        // Build the state machine matching the reference configuration
        let (sm, rx, _tx) = PIOBuilder::from_installed_program(installed)
            .out_pins(base_pin, 7)
            .in_pin_base(base_pin)
            .in_shift_direction(ShiftDirection::Right)
            // Match the example/C reference
            .autopush(false)
            .push_threshold(23)
            .clock_divisor_fixed_point(pio_cfg.clock_div_int, pio_cfg.clock_div_frac)
            .buffers(Buffers::OnlyRx)
            .build(sm);

        // Convert pins to PIO1 function
        let bump_right = bump_right.into_function::<FunctionPio1>();
        let bump_left = bump_left.into_function::<FunctionPio1>();
        let l5 = l5.into_function::<FunctionPio1>();
        let l4 = l4.into_function::<FunctionPio1>();
        let l3 = l3.into_function::<FunctionPio1>();
        let l2 = l2.into_function::<FunctionPio1>();
        let l1 = l1.into_function::<FunctionPio1>();

        Self {
            sm: Some(sm),
            rx,
            bump_right: Some(bump_right),
            bump_left: Some(bump_left),
            l5: Some(l5),
            l4: Some(l4),
            l3: Some(l3),
            l2: Some(l2),
            l1: Some(l1),
            delays,
            pio_cfg,
        }
    }

    /// Perform a single measurement and return raw counts for the 5 line sensors
    /// in L1..L5 order. Higher values indicate higher reflectance.
    ///
    /// The caller should ensure the emitter LED is enabled before calling this
    /// (and ideally wait for `delays.stabilize_ms` after enabling once).
    pub fn read_once<D: DelayNs>(&mut self, delay: &mut D) -> [u16; 5] {
        // 1) Precharge phase: temporarily drive the pins HIGH
        let mut pin16 = self.bump_right.take().unwrap().into_push_pull_output();
        let mut pin17 = self.bump_left.take().unwrap().into_push_pull_output();
        let mut pin18 = self.l5.take().unwrap().into_push_pull_output();
        let mut pin19 = self.l4.take().unwrap().into_push_pull_output();
        let mut pin20 = self.l3.take().unwrap().into_push_pull_output();
        let mut pin21 = self.l2.take().unwrap().into_push_pull_output();
        let mut pin22 = self.l1.take().unwrap().into_push_pull_output();

        // Drive all pins HIGH (charge phase)
        let _ = pin16.set_high();
        let _ = pin17.set_high();
        let _ = pin18.set_high();
        let _ = pin19.set_high();
        let _ = pin20.set_high();
        let _ = pin21.set_high();
        let _ = pin22.set_high();

        delay.delay_us(self.delays.precharge_us);

        // Return pins to PIO function
        self.bump_right = Some(pin16.reconfigure());
        self.bump_left = Some(pin17.reconfigure());
        self.l5 = Some(pin18.reconfigure());
        self.l4 = Some(pin19.reconfigure());
        self.l3 = Some(pin20.reconfigure());
        self.l2 = Some(pin21.reconfigure());
        self.l1 = Some(pin22.reconfigure());

        // 2) Clear FIFOs and initialize Y to (2^timeout_power) - 1
        self.sm.as_mut().expect("sm present").clear_fifos();

        // mov osr, !null (0xFFFF_FFFF)
        self.sm.as_mut().expect("sm present").exec_instruction(Instruction {
            operands: InstructionOperands::MOV {
                destination: MovDestination::OSR,
                op: MovOperation::Invert,
                source: MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        // out y, timeout_power
        self.sm.as_mut().expect("sm present").exec_instruction(Instruction {
            operands: InstructionOperands::OUT {
                destination: OutDestination::Y,
                bit_count: self.pio_cfg.timeout_power,
            },
            delay: 0,
            side_set: None,
        });

        // mov osr, null (clear OSR)
        self.sm.as_mut().expect("sm present").exec_instruction(Instruction {
            operands: InstructionOperands::MOV {
                destination: MovDestination::OSR,
                op: MovOperation::None,
                source: MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        });

        // 3) Start SM and wait for completion window
        let sm_stopped = self.sm.take().unwrap();
        let sm_running = sm_stopped.start();
        delay.delay_ms(self.delays.measurement_wait_ms);

        // 4) Drain RX FIFO and reconstruct discharge times
        let timeout: u32 = (1u32 << self.pio_cfg.timeout_power) as u32;
        let mut line_sensors = [timeout as u16; 5];
        let mut last_state: u8 = 0xFF;

        // Poll RX FIFO for a bounded number of entries
        for _ in 0..1000u32 {
            if let Some(data) = self.rx.read() {
                if data == 0xFFFF_FFFF { break; }

                let time_left = data & 0xFFFF;
                let state = ((data >> 16) & 0x7F) as u8;
                let new_zeros = last_state & !state;

                // Bits 2..6 correspond to L5..L1 (per reference mapping)
                if new_zeros & (1 << 2) != 0 { line_sensors[4] = (timeout - time_left) as u16; }
                if new_zeros & (1 << 3) != 0 { line_sensors[3] = (timeout - time_left) as u16; }
                if new_zeros & (1 << 4) != 0 { line_sensors[2] = (timeout - time_left) as u16; }
                if new_zeros & (1 << 5) != 0 { line_sensors[1] = (timeout - time_left) as u16; }
                if new_zeros & (1 << 6) != 0 { line_sensors[0] = (timeout - time_left) as u16; }

                last_state = state;
            }
        }

        // 5) Stop SM and return results
        self.sm = Some(sm_running.stop());

        line_sensors
    }
}
