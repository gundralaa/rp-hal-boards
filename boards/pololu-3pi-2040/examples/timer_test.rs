//! QTR Reflectance sensor reader example
//!
//! Uses a single PIO state machine 
//! To read all the Pololu QTR line reflectance sensors
//! 

#![no_std]
#![no_main]

use defmt::{info, error};

use defmt_rtt as _;
use panic_halt as _;

use pololu_3pi_2040::{
    entry,
    hal::{
        timer::*,
        gpio::*,
        watchdog::Watchdog,
        clocks::*,
        Sio,
        Timer,
    },
    pac::{
        interrupt,
        
        NVIC,
        Interrupt,
        Peripherals,
        CorePeripherals,
    },

    Pins,
};

use embedded_hal::digital::{OutputPin, StatefulOutputPin};

use core::cell::RefCell;
use critical_section::Mutex;

use fugit::MicrosDurationU32;

type LedPin = Pin<bank0::Gpio25, FunctionSioOutput, PullDown>;

static LED_PIN: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static ALARM_0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static ALARM_1: Mutex<RefCell<Option<Alarm1>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIMER_IRQ_0() {
    info!("TIMER_IRQ_0");
    critical_section::with(|cs| {
        let mut alarm_0 = ALARM_0.borrow(cs).take().unwrap();
        let mut alarm_1 = ALARM_1.borrow(cs).take().unwrap();

        alarm_0.clear_interrupt();
        let _ = alarm_0.schedule(MicrosDurationU32::secs(3));

        let _ = alarm_1.schedule(MicrosDurationU32::millis(100));
        alarm_1.enable_interrupt();

        ALARM_0.borrow(cs).replace(Some(alarm_0));
        ALARM_1.borrow(cs).replace(Some(alarm_1));
    });
}

#[interrupt]
fn TIMER_IRQ_1() {
    static mut COUNT: u32 = 0;
    critical_section::with(|cs| {
        let mut led_pin = LED_PIN.borrow(cs).take().unwrap();
        let mut alarm_1 = ALARM_1.borrow(cs).take().unwrap();

        alarm_1.clear_interrupt();
        let _ = alarm_1.schedule(MicrosDurationU32::millis(100));
        led_pin.toggle().unwrap();

        *COUNT += 1;
        info!("TIMER_IRQ_1: {}", *COUNT);
        if *COUNT >= 10 {
            alarm_1.disable_interrupt();
            *COUNT = 0;
        }

        ALARM_1.borrow(cs).replace(Some(alarm_1));
        LED_PIN.borrow(cs).replace(Some(led_pin));
    });
}

#[entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
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
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.toggle().unwrap();

    let mut alarm0 = timer.alarm_0().unwrap();
    let mut alarm1 = timer.alarm_1().unwrap();

    let mut delay = cortex_m::delay::Delay::new(
        core.SYST, 
        clocks.system_clock.freq().to_Hz()
    );

    critical_section::with(|cs| {
        let _ = alarm0.schedule(MicrosDurationU32::secs(5));
        alarm0.enable_interrupt();
        
        let _ = alarm1.schedule(MicrosDurationU32::secs(10));
        alarm1.enable_interrupt();
        
        LED_PIN.borrow(cs).replace(Some(led_pin));
        ALARM_0.borrow(cs).replace(Some(alarm0));
        ALARM_1.borrow(cs).replace(Some(alarm1));

    });

    unsafe {
        NVIC::unmask(Interrupt::TIMER_IRQ_1);
        NVIC::unmask(Interrupt::TIMER_IRQ_0);
    }

    loop {
        info!("hello, world! {:08x}", timer.get_counter().ticks());
        delay.delay_ms(1000);
    }

}