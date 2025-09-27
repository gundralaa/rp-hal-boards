# UART Hello World Example for Pololu 3pi 2040

This example demonstrates how to implement a simple UART "Hello, World!" application for the Pololu 3pi 2040 robot using the rp-hal crate.

## Features

- Sends "Hello, World!" messages over UART
- Blinks the onboard LED to indicate activity
- Uses GPIO 28 (TX) and GPIO 29 (RX) for UART communication
- Configures UART at 9600 baud rate
- Includes counter to track message numbers
- Sends periodic status messages

## Hardware Setup

### UART Pins
- **GPIO 28**: UART TX (Transmit)
- **GPIO 29**: UART RX (Receive)
- **GPIO 25**: Onboard LED (for visual feedback)

### Connection
Connect a USB-to-Serial adapter or serial terminal to:
- TX (GPIO 28) → RX of your serial device
- RX (GPIO 29) → TX of your serial device
- GND → GND

## Software Requirements

- Rust toolchain with `thumbv6m-none-eabi` target
- `probe-rs` for flashing the device
- Serial terminal program (e.g., `minicom`, `screen`, or PuTTY)

## Building and Flashing

1. **Install Rust target** (if not already installed):
   ```bash
   rustup target add thumbv6m-none-eabi
   ```

2. **Install probe-rs** (if not already installed):
   ```bash
   cargo install --locked probe-rs
   ```

3. **Build and flash the example**:
   ```bash
   cd boards/pololu-3pi-2040
   cargo run --example uart_hello_world
   ```

## Expected Output

When running, you should see output similar to:
```
Pololu 3pi 2040 UART Hello World!
Starting UART communication...
Baud rate: 9600
TX: GPIO 28, RX: GPIO 29
----------------------------------------
Hello, World! 0 from Pololu 3pi 2040!
Hello, World! 1 from Pololu 3pi 2040!
Hello, World! 2 from Pololu 3pi 2040!
...
Status: System running normally
```

## Serial Terminal Configuration

Configure your serial terminal with these settings:
- **Baud Rate**: 9600
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

## Code Structure

The example follows the standard rp-hal pattern:

1. **Initialization**: Sets up clocks, watchdog, and GPIO pins
2. **UART Configuration**: Configures UART0 with proper pins and settings
3. **Main Loop**: Sends messages and blinks LED periodically

## Key Components

- **UART Peripheral**: Uses `hal::uart::UartPeripheral` for communication
- **Pin Configuration**: Uses board-specific pin definitions from `pololu_3pi_2040::Pins`
- **Blocking I/O**: Uses `write_full_blocking()` for reliable message transmission
- **LED Feedback**: Blinks onboard LED to show activity

## Troubleshooting

- **No output**: Check UART pin connections and baud rate settings
- **Garbled output**: Verify baud rate matches between device and terminal
- **Build errors**: Ensure all dependencies are properly installed
- **Flash errors**: Check USB connection and probe-rs installation

## References

- [rp-hal Documentation](https://docs.rs/rp2040-hal/)
- [Pololu 3pi 2040 Documentation](https://www.pololu.com/docs/0j86)
- [RP2040 Datasheet](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf)