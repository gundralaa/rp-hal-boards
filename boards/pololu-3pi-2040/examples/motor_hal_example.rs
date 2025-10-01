#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;

use pololu_3pi_2040::hal::prelude::*;
use pololu_3pi_2040::hal;
use pololu_3pi_2040::pac;

use defmt::*;
use defmt_rtt as _;
use panic_halt as _;

fn expo_curve(x: f32) -> f32 {
	let a = 1.8f32;
	(x.powf(a)).min(1.0).max(0.0)
}

#[pololu_3pi_2040::entry]
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

	let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
	let sio = hal::Sio::new(pac.SIO);
	let pins = pololu_3pi_2040::Pins::new(
		pac.IO_BANK0,
		pac.PADS_BANK0,
		sio.gpio_bank0,
		&mut pac.RESETS,
	);

	let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
	let pwm7 = pwm_slices.pwm7;

	let right_pwm_pin = pins.right_motor_pwm;
	let left_pwm_pin = pins.left_motor_pwm;
	let right_dir = pins.right_motor_dir.into_push_pull_output();
	let left_dir = pins.left_motor_dir.into_push_pull_output();

	let cfg = pololu_3pi_2040::motor::MotorsConfig {
		div_int: 10,
		div_frac: 0,
		top: None,
		phase_correct: false,
		max_duty: 0x8FFF, // cap to limit power
		power_curve: expo_curve,
	};

	let mut motors = pololu_3pi_2040::motor::new_from_pwm7(
		pwm7,
		right_pwm_pin,
		left_pwm_pin,
		right_dir,
		left_dir,
		cfg,
	);

	info!("forward slow");
	motors.set_both(0.2, pololu_3pi_2040::motor::Direction::Forward, 0.2, pololu_3pi_2040::motor::Direction::Forward);
	delay.delay_ms(3000);

	info!("turn in place");
	motors.set_left(0.5, pololu_3pi_2040::motor::Direction::Forward);
	motors.set_right(0.5, pololu_3pi_2040::motor::Direction::Backward);
	delay.delay_ms(3000);

	info!("reverse slow");
	motors.set_both(0.3, pololu_3pi_2040::motor::Direction::Backward, 0.3, pololu_3pi_2040::motor::Direction::Backward);
	delay.delay_ms(3000);

	loop {
		motors.set_both(0.0, pololu_3pi_2040::motor::Direction::Forward, 0.0, pololu_3pi_2040::motor::Direction::Forward);
		delay.delay_ms(1000);
	}
}

