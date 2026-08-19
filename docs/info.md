## How it works

This project implements an I2C-controlled GPIO and event controller for Tiny Tapeout.

The device uses the standard Tiny Tapeout digital interface with 8 dedicated inputs, 8 dedicated outputs, and 8 bidirectional I/O pins.

The pin assignment is:

- `ui_in[7:0]`: 8-bit dedicated input port.
- `uo_out[7:0]`: 8-bit dedicated output port.
- `uio[0]`: active-low interrupt output (`IRQ_N`) or GPIO0 when interrupts are disabled.
- `uio[1]`: bidirectional GPIO1.
- `uio[2]`: dedicated I2C SCL.
- `uio[3]`: dedicated I2C SDA.
- `uio[4]` to `uio[7]`: bidirectional GPIO4 to GPIO7.

The I2C target uses the fixed 7-bit address `0x20`.

SCL and SDA are permanently reserved for I2C and cannot be reconfigured as GPIO pins. SDA uses open-drain behavior by driving only LOW and releasing the output through `uio_oe` for the HIGH state.

The controller provides:

- 8 dedicated digital inputs.
- 8 dedicated digital outputs.
- 5 bidirectional GPIO pins while the interrupt output is enabled.
- 6 bidirectional GPIO pins when the interrupt output is disabled and `uio[0]` is used as GPIO0.
- Configurable GPIO direction and output values.
- Interrupt detection on rising and falling edges.
- Per-port interrupt masks.
- Latched interrupt status registers.
- Write-one-to-clear interrupt flags.
- Input and GPIO snapshots captured when an event occurs.
- A 16-bit event counter.
- A 32-byte scratchpad memory accessible through I2C.
- Multi-byte I2C read and write operations with automatic register-address increment.

After reset, the interrupt function is enabled by default. `IRQ_N` remains released in its inactive state and is actively driven LOW when a configured input event is detected.

The main register map is:

| Address | Register | Access |
|---|---|---|
| `0x00` | DEVICE_ID | Read |
| `0x01` | VERSION | Read |
| `0x10` | INPUT_VALUE | Read |
| `0x11` | OUTPUT_VALUE | Read/Write |
| `0x12` | UIO_VALUE | Read |
| `0x13` | GPIO_OUTPUT | Read/Write |
| `0x14` | GPIO_DIRECTION | Read/Write |
| `0x20` | IRQ_STATUS_INPUT | Read |
| `0x21` | IRQ_STATUS_GPIO | Read |
| `0x22` | IRQ_MASK_INPUT | Read/Write |
| `0x23` | IRQ_MASK_GPIO | Read/Write |
| `0x24` | IRQ_RISE_INPUT | Read/Write |
| `0x25` | IRQ_FALL_INPUT | Read/Write |
| `0x26` | IRQ_RISE_GPIO | Read/Write |
| `0x27` | IRQ_FALL_GPIO | Read/Write |
| `0x28` | IRQ_CONTROL | Read/Write |
| `0x29` | IRQ_CLEAR_INPUT | Write |
| `0x2A` | IRQ_CLEAR_GPIO | Write |
| `0x2B` | INPUT_SNAPSHOT | Read |
| `0x2C` | GPIO_SNAPSHOT | Read |
| `0x2D` | EVENT_COUNT_L | Read |
| `0x2E` | EVENT_COUNT_H | Read |
| `0x40`-`0x5F` | 32-byte scratchpad | Read/Write |

## How to test

The project includes a Cocotb regression test in the `test` directory.

Create and activate a Python virtual environment and install the test dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r test/requirements.txt
