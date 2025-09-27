# SH1106 OLED Display Controller - SPI Interface Documentation

## Table of Contents
1. [Overview](#overview)
2. [Hardware Connections](#hardware-connections)
3. [SPI Communication Protocol](#spi-communication-protocol)
4. [Initialization Sequence](#initialization-sequence)
5. [Rust Implementation](#rust-implementation)
6. [Command Reference](#command-reference)
7. [Troubleshooting](#troubleshooting)
8. [References](#references)

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

### Dependencies

Add these dependencies to your `Cargo.toml`:

```toml
[dependencies]
rp2040-hal = "0.14"
embedded-hal = "1.0"
embedded-graphics = "0.7"
display-interface-spi = "0.4"
sh1106 = "0.7"
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