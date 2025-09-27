# SH1106 OLED Display Controller - SPI Interface Documentation

## Table of Contents
1. [Overview](#overview)
2. [Hardware Connections](#hardware-connections)
3. [Pololu 3pi+ 2040 Specific Setup](#pololu-3pi-2040-specific-setup)
4. [SPI Communication Protocol](#spi-communication-protocol)
5. [Initialization Sequence](#initialization-sequence)
6. [Rust Implementation](#rust-implementation)
7. [Command Reference](#command-reference)
8. [Troubleshooting](#troubleshooting)
9. [References](#references)

## Overview

The SH1106 is a 128x64 pixel OLED display controller commonly used in small monochrome displays. It supports multiple communication interfaces including SPI, I2C, and parallel interfaces. This documentation focuses on the SPI interface implementation.

### Key Features
- **Resolution**: 128x64 pixels
- **Interface**: SPI (4-wire), I2C, or Parallel
- **Color**: Monochrome (white/black)
- **Power Supply**: 3.3V or 5V (depending on module)
- **Memory**: 1KB display RAM (128x64 bits)

### Display Specifications
- **Active Area**: 128x64 pixels
- **Pixel Size**: Typically 0.15mm x 0.15mm
- **Contrast**: Adjustable via software
- **Refresh Rate**: Up to 100Hz

## Hardware Connections

### SPI Pin Configuration

The SH1106 supports 4-wire SPI communication. Here's the typical pin configuration:

| OLED Pin | Function | Description | Microcontroller Connection |
|----------|----------|-------------|---------------------------|
| VCC | Power Supply | 3.3V or 5V | 3.3V/5V |
| GND | Ground | Common ground | GND |
| D0/SCK | SPI Clock | Clock signal | SPI SCK |
| D1/MOSI | SPI Data | Data input | SPI MOSI |
| DC | Data/Command | Command/Data select | GPIO pin |
| CS | Chip Select | SPI chip select | GPIO pin |
| RES | Reset | Reset signal | GPIO pin |

### Connection Example for RP2040

```rust
// Typical pin assignments for RP2040
let spi_sck = pins.gp10.into_function::<hal::gpio::FunctionSpi>();  // SCK
let spi_mosi = pins.gp11.into_function::<hal::gpio::FunctionSpi>(); // MOSI
let dc_pin = pins.gp12.into_push_pull_output();                    // DC
let cs_pin = pins.gp13.into_push_pull_output();                   // CS
let res_pin = pins.gp14.into_push_pull_output();                  // RES
```

## Pololu 3pi+ 2040 Specific Setup

The Pololu 3pi+ 2040 robot provides specific pins that can be used for SH1106 display connection. Here's the recommended pin mapping:

### Pin Assignments for Pololu 3pi+ 2040

| SH1106 Pin | Function | Pololu 3pi+ 2040 Pin | GPIO | Notes |
|------------|----------|----------------------|------|-------|
| VCC | Power Supply | 3.3V | - | Use 3.3V rail |
| GND | Ground | GND | - | Common ground |
| SCK | SPI Clock | GPIO2 (display_sck) | GP2 | Pre-configured for SPI |
| MOSI | SPI Data | GPIO3 (rgb_led_display_data) | GP3 | Pre-configured for SPI |
| DC | Data/Command | GPIO0 (button_c) | GP0 | Reconfigure as output |
| CS | Chip Select | GPIO1 (unused) | GP1 | Or tie to GND |
| RES | Reset | GPIO25 (led) | GP25 | Reconfigure as output |

### Hardware Connections

```rust
// Pololu 3pi+ 2040 specific pin configuration
let spi_sck = pins.display_sck.into_function::<hal::gpio::FunctionSpi>();
let spi_mosi = pins.rgb_led_display_data.into_function::<hal::gpio::FunctionSpi>();
let dc_pin = pins.button_c.into_push_pull_output();
let cs_pin = pins.led.into_push_pull_output(); // Using LED pin as CS
let res_pin = pins.led.into_push_pull_output(); // Using LED pin as RES
```

### Wiring Diagram

```
SH1106 OLED Display    Pololu 3pi+ 2040
===================    ==================
VCC                    -> 3.3V
GND                    -> GND
SCK                    -> GPIO2 (display_sck)
MOSI                   -> GPIO3 (rgb_led_display_data)
DC                     -> GPIO0 (button_c)
CS                     -> GPIO1 (or GND)
RES                    -> GPIO25 (led)
```

### Important Notes for Pololu 3pi+ 2040

1. **Pin Conflicts**: The `display_sck` and `rgb_led_display_data` pins are shared with the RGB LED display. Ensure only one device uses these pins at a time.

2. **Power Supply**: The robot operates on battery power, so consider power consumption when adding external displays.

3. **Physical Mounting**: The SH1106 display can be mounted on the robot chassis for status display during operation.

4. **SPI Bus Sharing**: If using multiple SPI devices, ensure proper chip select management.

### Power Considerations

- **Voltage Levels**: Most SH1106 modules operate at 3.3V logic levels
- **Current Consumption**: Typically 20-40mA during operation
- **Power Management**: The controller supports sleep mode for power saving

## SPI Communication Protocol

### SPI Configuration

The SH1106 uses SPI Mode 0 (CPOL=0, CPHA=0):
- **Clock Polarity (CPOL)**: 0 (idle low)
- **Clock Phase (CPHA)**: 0 (data sampled on rising edge)
- **Bit Order**: MSB first
- **Clock Speed**: Up to 10MHz (check module specifications)

### Data/Command Control

The DC (Data/Command) pin determines how the data is interpreted:

| DC Pin State | Data Interpretation |
|--------------|-------------------|
| LOW (0) | Command |
| HIGH (1) | Display Data |

### Communication Flow

1. **Chip Select**: Pull CS low to select the device
2. **Data/Command**: Set DC pin appropriately
3. **Data Transfer**: Send command or data via SPI
4. **Chip Deselect**: Pull CS high to deselect the device

## Initialization Sequence

The SH1106 requires a specific initialization sequence to operate correctly:

### Hardware Reset
```rust
// Reset sequence
res_pin.set_low().unwrap();
delay.delay_ms(10);
res_pin.set_high().unwrap();
delay.delay_ms(10);
```

### Software Initialization Commands

```rust
// Essential initialization commands
let init_commands = [
    0xAE, // Display OFF
    0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
    0xA8, 0x3F, // Set multiplex ratio (1 to 64)
    0xD3, 0x00, // Set display offset
    0x40, // Set start line address
    0x8D, 0x14, // Charge pump setting
    0x20, 0x00, // Memory addressing mode
    0xA1, // Set segment re-map
    0xC8, // Set COM output scan direction
    0xDA, 0x12, // Set COM pins hardware configuration
    0x81, 0xCF, // Set contrast control
    0xD9, 0xF1, // Set pre-charge period
    0xDB, 0x40, // Set VCOMH deselect level
    0xA4, // Set entire display ON/OFF
    0xA6, // Set normal/inverse display
    0xAF, // Display ON
];
```

## Rust Implementation

### Running the Example

To run the SH1106 display example on your Pololu 3pi+ 2040:

1. **Add Dependencies**: Add these to your `Cargo.toml`:

```toml
[dependencies]
pololu-3pi-2040 = { path = "boards/pololu-3pi-2040" }
embedded-hal = "1.0"
embedded-graphics = "0.7"
display-interface-spi = "0.4"
sh1106 = "0.7"
defmt = "0.3"
defmt-rtt = "0.7"
panic-halt = "0.2"
```

2. **Build and Flash**:

```bash
cargo run --example sh1106_display --release
```

3. **Expected Behavior**: The robot will cycle through different motor states (forward, backward, stop) while displaying status information on the SH1106 OLED display.

### Dependencies

For the Pololu 3pi+ 2040, add these dependencies to your `Cargo.toml`:

```toml
[dependencies]
pololu-3pi-2040 = { path = "boards/pololu-3pi-2040" }
embedded-hal = "1.0"
embedded-graphics = "0.7"
display-interface-spi = "0.4"
sh1106 = "0.7"
defmt = "0.3"
defmt-rtt = "0.7"
panic-halt = "0.2"
```

### Basic Implementation

```rust
#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use panic_halt as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::Pins,
    pac,
    spi::Spi,
    watchdog::Watchdog,
    Sio,
};
use sh1106::{prelude::*, Builder};

#[link_section = ".boot2"]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

    // Configure SPI pins
    let spi_sck = pins.gp10.into_function::<rp2040_hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gp11.into_function::<rp2040_hal::gpio::FunctionSpi>();
    let dc_pin = pins.gp12.into_push_pull_output();
    let cs_pin = pins.gp13.into_push_pull_output();
    let res_pin = pins.gp14.into_push_pull_output();

    // Initialize SPI
    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_sck));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // Create display interface
    let interface = display_interface_spi::SPIInterface::new(spi, dc_pin, cs_pin);

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

    // Clear display
    display.clear();

    // Create text style
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Draw text
    Text::new("Hello SH1106!", Point::new(0, 20), text_style)
        .draw(&mut display)
        .unwrap();

    // Update display
    display.flush().unwrap();

    loop {
        // Main application loop
        delay.delay_ms(1000);
    }
}
```

### Pololu 3pi+ 2040 Example

Here's a complete example for the Pololu 3pi+ 2040 robot that displays robot status information:

```rust
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

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let sio = hal::Sio::new(pac.SIO);
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
    let cs_pin = pins.led.into_push_pull_output();
    let res_pin = pins.led.into_push_pull_output();

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
```

### Advanced Features

#### Scrolling
```rust
// Enable horizontal scrolling
display.scroll_right(0, 7).unwrap();
```

#### Contrast Control
```rust
// Set contrast (0-255)
display.set_contrast(128).unwrap();
```

#### Power Management
```rust
// Enter sleep mode
display.sleep().unwrap();

// Wake from sleep
display.wake().unwrap();
```

## Command Reference

### Display Control Commands

| Command | Description | Parameters |
|---------|-------------|------------|
| 0xAE | Display OFF | None |
| 0xAF | Display ON | None |
| 0xA4 | Entire display ON | None |
| 0xA5 | Entire display OFF | None |
| 0xA6 | Normal display | None |
| 0xA7 | Inverse display | None |

### Addressing Commands

| Command | Description | Parameters |
|---------|-------------|------------|
| 0x20 | Set memory addressing mode | 0x00=Horizontal, 0x01=Vertical, 0x02=Page |
| 0x21 | Set column address | Start, End |
| 0x22 | Set page address | Start, End |
| 0x40-0x7F | Set start line | Line number |

### Hardware Configuration

| Command | Description | Parameters |
|---------|-------------|------------|
| 0xA8 | Set multiplex ratio | 0x3F for 64 rows |
| 0xD3 | Set display offset | Offset value |
| 0xDA | Set COM pins | 0x12 for sequential |
| 0xC0/C8 | Set COM scan direction | None |

### Timing and Power

| Command | Description | Parameters |
|---------|-------------|------------|
| 0xD5 | Set display clock | Divide ratio |
| 0xD9 | Set pre-charge period | Period value |
| 0xDB | Set VCOMH deselect | Level value |
| 0x8D | Charge pump setting | 0x14 for enable |

## Troubleshooting

### Common Issues

#### Display Not Responding
- **Check power supply**: Ensure 3.3V/5V is connected correctly
- **Verify SPI connections**: Check SCK, MOSI, DC, CS, RES pins
- **Reset sequence**: Ensure proper hardware reset is performed
- **SPI configuration**: Verify SPI mode and clock speed

#### Garbled Display
- **Initialization sequence**: Ensure all init commands are sent
- **Memory addressing**: Check addressing mode setting
- **Contrast setting**: Adjust contrast value
- **Clock speed**: Reduce SPI clock speed if too high

#### Partial Display Update
- **Page addressing**: Ensure correct page boundaries
- **Column addressing**: Check column address settings
- **Display buffer**: Verify buffer management

### Debugging Tips

1. **Use oscilloscope**: Monitor SPI signals for proper timing
2. **Check voltages**: Verify power supply and logic levels
3. **Test with known good code**: Use reference implementation
4. **Incremental testing**: Test each component separately

### Performance Optimization

- **Use DMA**: Implement SPI DMA for faster transfers
- **Buffer management**: Optimize display buffer updates
- **Partial updates**: Only update changed regions
- **Power management**: Use sleep mode when possible

## References

### Datasheets and Specifications
- [SH1106 Controller Datasheet](https://www.smart-prototyping.com/Other-Products/1_3-inch-OLED-Display-SH1106-SPI-I2C-128-64)
- [OLED Display Module Specifications](https://www.waveshare.com/wiki/1.3inch_OLED_Module)

### Software Libraries
- [SH1106 Rust Driver](https://crates.io/crates/sh1106)
- [Embedded Graphics](https://crates.io/crates/embedded-graphics)
- [RP2040 HAL](https://crates.io/crates/rp2040-hal)

### Community Resources
- [RP2040 Community Forum](https://forums.raspberrypi.org/)
- [Rust Embedded Working Group](https://github.com/rust-embedded)
- [SH1106 MicroPython Library](https://github.com/robert-hh/SH1106)

---

*This documentation provides a comprehensive guide for implementing SH1106 OLED display control over SPI using Rust and the RP2040 microcontroller. For additional support or questions, refer to the community resources listed above.*