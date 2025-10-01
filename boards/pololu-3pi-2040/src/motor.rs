#![allow(dead_code)]

use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

use crate::hal;

/// Motor rotation direction
pub enum Direction {
	Forward,
	Backward,
}

/// Configuration for the motors PWM and curve
pub struct MotorsConfig {
	/// Integer clock divider for PWM slice
	pub div_int: u8,
	/// Fractional clock divider for PWM slice (0..=15)
	pub div_frac: u8,
	/// Optional TOP value for the PWM counter. If `None`, leave default.
	pub top: Option<u16>,
	/// Enable phase-correct PWM if true
	pub phase_correct: bool,
	/// Max duty used when mapping power to duty (e.g. `u16::MAX`)
	pub max_duty: u16,
	/// Power curve mapping from [0,1] -> [0,1]
	pub power_curve: fn(f32) -> f32,
}

impl Default for MotorsConfig {
	fn default() -> Self {
		Self {
			div_int: 10,
			div_frac: 0,
			top: None,
			phase_correct: false,
			max_duty: u16::MAX,
			power_curve: |x: f32| x,
		}
	}
}

/// Represents a single motor with its PWM channel and direction pin
pub struct Motor<Channel, DirPin, PwmPin>
where
	Channel: SetDutyCycle<Duty = u16>,
	DirPin: OutputPin,
{
	channel: Channel,
	dir_pin: DirPin,
	_pwm_pin: PwmPin,
	max_duty: u16,
	power_curve: fn(f32) -> f32,
}

impl<Channel, DirPin, PwmPin> Motor<Channel, DirPin, PwmPin>
where
	Channel: SetDutyCycle<Duty = u16>,
	DirPin: OutputPin,
{
	#[inline]
	pub fn set(&mut self, speed_01: f32, direction: Direction) {
		let clamped = if speed_01 < 0.0 { 0.0 } else if speed_01 > 1.0 { 1.0 } else { speed_01 };
		let adjusted = (self.power_curve)(clamped);
		let adjusted = if adjusted < 0.0 { 0.0 } else if adjusted > 1.0 { 1.0 } else { adjusted };
		let duty = (adjusted * self.max_duty as f32) as u16;
		let _ = self.channel.set_duty_cycle(duty);
		match direction {
			Direction::Forward => {
				let _ = self.dir_pin.set_high();
			}
			Direction::Backward => {
				let _ = self.dir_pin.set_low();
			}
		}
	}

	#[inline]
	pub fn set_power_curve(&mut self, curve: fn(f32) -> f32) {
		self.power_curve = curve;
	}

	#[inline]
	pub fn set_max_duty(&mut self, max_duty: u16) {
		self.max_duty = max_duty;
	}
}

/// Dual motors convenience wrapper for the Pololu 3pi+ (right/left)
pub struct Motors<RightChan, LeftChan, RightDir, LeftDir, RightPwmPin, LeftPwmPin>
where
	RightChan: SetDutyCycle<Duty = u16>,
	LeftChan: SetDutyCycle<Duty = u16>,
	RightDir: OutputPin,
	LeftDir: OutputPin,
{
	pub right: Motor<RightChan, RightDir, RightPwmPin>,
	pub left: Motor<LeftChan, LeftDir, LeftPwmPin>,
}

impl<RightChan, LeftChan, RightDir, LeftDir, RightPwmPin, LeftPwmPin>
	Motors<RightChan, LeftChan, RightDir, LeftDir, RightPwmPin, LeftPwmPin>
where
	RightChan: SetDutyCycle<Duty = u16>,
	LeftChan: SetDutyCycle<Duty = u16>,
	RightDir: OutputPin,
	LeftDir: OutputPin,
{
	#[inline]
	pub fn set_right(&mut self, speed_01: f32, direction: Direction) {
		self.right.set(speed_01, direction);
	}

	#[inline]
	pub fn set_left(&mut self, speed_01: f32, direction: Direction) {
		self.left.set(speed_01, direction);
	}

	#[inline]
	pub fn set_both(&mut self, left_speed_01: f32, left_dir: Direction, right_speed_01: f32, right_dir: Direction) {
		self.left.set(left_speed_01, left_dir);
		self.right.set(right_speed_01, right_dir);
	}

	#[inline]
	pub fn set_power_curve(&mut self, curve: fn(f32) -> f32) {
		self.left.set_power_curve(curve);
		self.right.set_power_curve(curve);
	}

	#[inline]
	pub fn set_max_duty(&mut self, max_duty: u16) {
		self.left.set_max_duty(max_duty);
		self.right.set_max_duty(max_duty);
	}
}

/// Construct `Motors` from PWM slice 7 and the associated pins.
///
/// This consumes the PWM7 slice and the motor pins, configures the slice,
/// attaches the channels to the pins, and returns a `Motors` handle.
pub fn new_from_pwm7<Id, Mode, RightPwmPin, LeftPwmPin, RightDir, LeftDir>(
	mut pwm7: hal::pwm::Slice<Id, Mode>,
	right_pwm_pin: RightPwmPin,
	left_pwm_pin: LeftPwmPin,
	right_dir: RightDir,
	left_dir: LeftDir,
	config: MotorsConfig,
) -> Motors<
		hal::pwm::Channel<Id, hal::pwm::A>,
		hal::pwm::Channel<Id, hal::pwm::B>,
		RightDir,
		LeftDir,
		RightPwmPin,
		LeftPwmPin,
	>
where
	RightDir: OutputPin,
	LeftDir: OutputPin,
{
	if config.phase_correct {
		pwm7.set_ph_correct();
	}
	pwm7.set_div_int(config.div_int);
	pwm7.set_div_frac(config.div_frac);
	if let Some(top) = config.top {
		pwm7.set_top(top);
	}
	pwm7.enable();

	let mut channel_right = pwm7.channel_a;
	let mut channel_left = pwm7.channel_b;

	let right_pwm_pin = channel_right.output_to(right_pwm_pin);
	let left_pwm_pin = channel_left.output_to(left_pwm_pin);

	Motors {
		right: Motor {
			channel: channel_right,
			dir_pin: right_dir,
			_pwm_pin: right_pwm_pin,
			max_duty: config.max_duty,
			power_curve: config.power_curve,
		},
		left: Motor {
			channel: channel_left,
			dir_pin: left_dir,
			_pwm_pin: left_pwm_pin,
			max_duty: config.max_duty,
			power_curve: config.power_curve,
		},
	}
}

