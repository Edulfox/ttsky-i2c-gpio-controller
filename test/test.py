# Copyright (c) 2026 Eduardo Norambuena. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Comprehensive RTL regression for the Tiny Tapeout I2C GPIO controller.

The regression verifies the complete behavior implemented by the current RTL:

* reset defaults and asynchronous reset behavior
* fixed 7-bit I2C address (0x20)
* ACK/NACK, START, STOP and repeated START
* current-address and auto-increment reads
* multi-byte writes and reads
* operation at I2C Standard-mode (100 kHz) and Fast-mode (400 kHz)
* 8 dedicated inputs and 8 dedicated outputs
* all available bidirectional GPIO pins
* permanent reservation of SCL/SDA
* SDA open-drain behavior and no SCL clock stretching in v1
* IRQ enabled by default after reset
* IRQ/GPIO0 alternate-function behavior
* per-pin IRQ masks and rising/falling edge selection
* dedicated-input and bidirectional-GPIO interrupts
* write-one-to-clear interrupt status
* input/GPIO event snapshots
* 16-bit event counter
* complete 32-byte scratchpad and address boundary behavior
* read-only and unmapped register behavior
* recovery from reset during an active I2C transaction

Scratchpad power-up contents are intentionally NOT tested because the RTL does
not reset the scratchpad memory. Every scratchpad test writes before reading.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer


# -----------------------------------------------------------------------------
# Global configuration
# -----------------------------------------------------------------------------

PROJECT_CLOCK_HZ = 50_000_000
PROJECT_CLOCK_PERIOD_NS = 20

I2C_ADDRESS = 0x20
I2C_STANDARD_HZ = 100_000
I2C_FAST_HZ = 400_000

GPIO_ALLOWED_MASK = 0xF3   # UIO0,1,4,5,6,7
GPIO_IRQ_ON_MASK = 0xF2    # UIO1,4,5,6,7 (UIO0 is IRQ_N)
I2C_PIN_MASK = 0x0C        # UIO2=SCL, UIO3=SDA


# -----------------------------------------------------------------------------
# Register map
# -----------------------------------------------------------------------------

REG_DEVICE_ID = 0x00
REG_VERSION = 0x01

REG_INPUT_VALUE = 0x10
REG_OUTPUT_VALUE = 0x11
REG_UIO_VALUE = 0x12
REG_GPIO_OUTPUT = 0x13
REG_GPIO_DIRECTION = 0x14

REG_IRQ_STATUS_IN = 0x20
REG_IRQ_STATUS_GPIO = 0x21
REG_IRQ_MASK_IN = 0x22
REG_IRQ_MASK_GPIO = 0x23
REG_IRQ_RISE_IN = 0x24
REG_IRQ_FALL_IN = 0x25
REG_IRQ_RISE_GPIO = 0x26
REG_IRQ_FALL_GPIO = 0x27
REG_IRQ_CONTROL = 0x28
REG_IRQ_CLEAR_IN = 0x29
REG_IRQ_CLEAR_GPIO = 0x2A
REG_INPUT_SNAPSHOT = 0x2B
REG_GPIO_SNAPSHOT = 0x2C
REG_EVENT_COUNT_L = 0x2D
REG_EVENT_COUNT_H = 0x2E

SCRATCHPAD_BASE = 0x40
SCRATCHPAD_LAST = 0x5F


# -----------------------------------------------------------------------------
# Utility helpers
# -----------------------------------------------------------------------------

def signal_int(signal):
    """Return a resolved cocotb signal as an integer."""
    return int(signal.value)


def bit(value, index):
    return (value >> index) & 1


async def wait_sync(dut, cycles=6):
    """Wait long enough for the two-stage input synchronizers and detector."""
    await ClockCycles(dut.clk, cycles)


async def start_project_clock(dut):
    """Start the 50 MHz Tiny Tapeout project clock."""
    dut.clk.value = 0
    clock = Clock(dut.clk, PROJECT_CLOCK_PERIOD_NS, unit="ns")
    cocotb.start_soon(clock.start())
    return clock


class I2CMaster:
    """Open-drain I2C controller model for the Tiny Tapeout UIO bus.

    The RTL synchronizes SCL/SDA to the 50 MHz project clock.  Therefore the
    testbench must preserve a real LOW interval after every SCL falling edge
    before changing/releasing SDA.  This also models the I2C requirement that
    data changes while SCL is LOW and prevents artificial START/STOP edges.
    """

    def __init__(self, dut, frequency_hz=I2C_FAST_HZ):
        self.dut = dut
        self.external_uio = 0x01  # External pull-up on IRQ_N by default.
        self.scl = 1
        self.master_sda_release = True
        self.observed_target_sda_drive = False
        self.set_frequency(frequency_hz)

    def set_frequency(self, frequency_hz):
        """Configure an I2C clock with a 60% LOW / 40% HIGH duty cycle.

        At 100 kHz this produces tLOW=6 us, tHIGH=4 us.
        At 400 kHz this produces tLOW=1.5 us, tHIGH=1.0 us.

        A portion of tLOW is reserved after each falling edge so the RTL's
        synchronizers can observe the edge and, when required, assert/release
        the target ACK before the controller changes SDA.
        """
        self.frequency_hz = int(frequency_hz)
        assert self.frequency_hz > 0

        self.period_ns = int(round(1_000_000_000 / self.frequency_hz))
        self.high_ns = int(round(self.period_ns * 0.40))
        self.low_ns = self.period_ns - self.high_ns

        # Keep SDA unchanged long enough after SCL falls for the two-stage
        # synchronizer + edge detector in i2c_target.v.  20% of tLOW is ample
        # at both supported bus rates, with a hard minimum of six project clocks.
        minimum_guard_ns = 6 * PROJECT_CLOCK_PERIOD_NS
        self.fall_guard_ns = max(
            minimum_guard_ns,
            int(round(self.low_ns * 0.20)),
        )

        if self.fall_guard_ns >= self.low_ns:
            self.fall_guard_ns = self.low_ns // 2

        self.low_setup_ns = self.low_ns - self.fall_guard_ns

        assert self.low_setup_ns > 0
        assert self.high_ns > 0

    def set_external_uio(self, value):
        """Set externally driven/pulled values for UIO pins other than SCL/SDA."""
        self.external_uio = value & 0xFF

    def _resolve_physical_uio(self):
        """Resolve controller drive, external levels and ASIC output-enable."""
        physical = self.external_uio & 0xFF

        # Controller side of SCL.
        if self.scl:
            physical |= 1 << 2
        else:
            physical &= ~(1 << 2)

        # Controller side of SDA (open-drain).
        if self.master_sda_release:
            physical |= 1 << 3
        else:
            physical &= ~(1 << 3)

        oe = signal_int(self.dut.uio_oe)
        out = signal_int(self.dut.uio_out)

        # Permanent electrical invariants of the current design.
        assert (out & I2C_PIN_MASK) == 0, (
            f"I2C output paths must never drive HIGH: uio_out=0x{out:02X}"
        )
        assert bit(oe, 2) == 0, (
            f"SCL clock stretching is not implemented in v1: uio_oe=0x{oe:02X}"
        )

        if bit(oe, 3):
            self.observed_target_sda_drive = True
            assert bit(out, 3) == 0, "SDA may only be actively driven LOW"

        # Tiny Tapeout UIO behavior: OE=1 means ASIC drives the physical pin.
        for index in range(8):
            if bit(oe, index):
                if bit(out, index):
                    physical |= 1 << index
                else:
                    physical &= ~(1 << index)

        self.dut.uio_in.value = physical
        return physical

    async def _hold(self, scl, master_sda_release, duration_ns):
        self.scl = int(bool(scl))
        self.master_sda_release = bool(master_sda_release)
        self._resolve_physical_uio()
        await Timer(int(duration_ns), unit="ns")

    async def idle(self, periods=1):
        """Hold the bus idle and refresh the physical UIO model."""
        for _ in range(periods):
            await self._hold(1, True, self.period_ns)

    async def start(self):
        """Generate START or repeated START without changing SDA on an SCL edge.

        The sequence intentionally mirrors a real controller:
          SCL LOW, SDA released -> SCL HIGH -> SDA LOW -> SCL LOW.
        """
        # Prepare SDA HIGH while SCL is LOW.  If this follows an ACK clock,
        # fall_guard_ns has already elapsed, so this completes the configured
        # tLOW interval.
        await self._hold(0, True, self.low_setup_ns)

        # Bring SCL HIGH with SDA already stable HIGH.
        await self._hold(1, True, self.high_ns)

        # SDA falling while SCL HIGH -> START / repeated START.
        await self._hold(1, False, self.high_ns)

        # Enter LOW and keep SDA unchanged long enough for the target to
        # synchronize the falling edge before the first data bit is prepared.
        await self._hold(0, False, self.fall_guard_ns)

    async def stop(self):
        """Generate a legal STOP condition."""
        # Prepare SDA LOW during SCL LOW.
        await self._hold(0, False, self.low_setup_ns)

        # Raise SCL while SDA remains LOW.
        await self._hold(1, False, self.high_ns)

        # SDA LOW -> HIGH while SCL HIGH = STOP.
        await self._hold(1, True, self.high_ns)

    async def write_bit(self, value):
        """Transmit one controller-driven data bit."""
        release = bool(value)

        # Change SDA only while SCL is LOW, then satisfy the remainder of tLOW.
        await self._hold(0, release, self.low_setup_ns)

        # Sample window.
        await self._hold(1, release, self.high_ns)

        # Falling edge.  Keep SDA unchanged during the post-fall guard so the
        # synchronized target sees the edge before the next SDA change.
        await self._hold(0, release, self.fall_guard_ns)

    async def read_ack(self):
        """Clock and sample the target ACK bit."""
        # The previous data bit already supplied fall_guard_ns.  By the time
        # this call releases SDA, the target has had time to assert its ACK.
        await self._hold(0, True, self.low_setup_ns)

        await self._hold(1, True, self.high_ns)
        ack = bit(signal_int(self.dut.uio_in), 3) == 0

        # Complete the ACK clock and allow the target to release ACK / advance
        # its state before the controller prepares the next byte.
        await self._hold(0, True, self.fall_guard_ns)
        return ack

    async def write_byte(self, value):
        for index in range(7, -1, -1):
            await self.write_bit(bit(value, index))
        return await self.read_ack()

    async def read_byte(self, send_ack):
        """Read one target-driven byte, then transmit controller ACK/NACK."""
        value = 0

        for _ in range(8):
            # Controller releases SDA while target transmits.
            await self._hold(0, True, self.low_setup_ns)
            await self._hold(1, True, self.high_ns)

            value = (value << 1) | bit(signal_int(self.dut.uio_in), 3)

            await self._hold(0, True, self.fall_guard_ns)

        # Controller ACK = LOW, NACK = released HIGH.
        master_release = not send_ack

        await self._hold(0, master_release, self.low_setup_ns)
        await self._hold(1, master_release, self.high_ns)
        await self._hold(0, master_release, self.fall_guard_ns)

        return value

    async def probe_address(self, address, read=False):
        """Probe one address without leaving a read target driving SDA.

        A successful read address enters target transmit mode immediately after
        ACK.  Therefore, when probing a valid read address, clock one byte and
        NACK it before STOP instead of trying to STOP in the middle of a byte.
        """
        await self.start()
        ack = await self.write_byte((address << 1) | int(bool(read)))

        if ack and read:
            await self.read_byte(send_ack=False)

        await self.stop()
        return ack

    async def set_register_pointer(self, address):
        await self.start()
        assert await self.write_byte((I2C_ADDRESS << 1) | 0), "No ACK on I2C write address"
        assert await self.write_byte(address), "No ACK on register pointer"
        await self.stop()

    async def write_registers(self, start_address, data):
        await self.start()

        assert await self.write_byte((I2C_ADDRESS << 1) | 0), "No ACK on I2C write address"
        assert await self.write_byte(start_address), "No ACK on register pointer"

        for offset, value in enumerate(data):
            assert await self.write_byte(value & 0xFF), (
                f"No ACK writing 0x{value:02X} at register "
                f"0x{(start_address + offset) & 0xFF:02X}"
            )

        await self.stop()

    async def read_registers(self, start_address, length):
        assert length > 0

        # Set register pointer.
        await self.start()
        assert await self.write_byte((I2C_ADDRESS << 1) | 0), "No ACK on I2C write address"
        assert await self.write_byte(start_address), "No ACK on register pointer"

        # Repeated START and read direction.
        await self.start()
        assert await self.write_byte((I2C_ADDRESS << 1) | 1), "No ACK on I2C read address"

        result = []
        for index in range(length):
            result.append(
                await self.read_byte(send_ack=(index != length - 1))
            )

        await self.stop()
        return result

    async def read_current(self, length):
        """Read starting from the target's existing internal register pointer."""
        assert length > 0

        await self.start()
        assert await self.write_byte((I2C_ADDRESS << 1) | 1), "No ACK on I2C read address"

        result = []
        for index in range(length):
            result.append(
                await self.read_byte(send_ack=(index != length - 1))
            )

        await self.stop()
        return result


async def reset_dut(dut, master):
    """Apply reset and establish a clean external bus state."""
    dut.ena.value = 1
    dut.ui_in.value = 0x00

    # Before UIO outputs are resolved, drive an idle physical bus directly:
    # UIO0 pull-up, UIO2 SCL HIGH, UIO3 SDA HIGH.
    dut.uio_in.value = 0x0D

    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 6)

    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    master.set_external_uio(0x01)
    await master.idle(periods=2)
    await wait_sync(dut)


async def setup_test(dut, i2c_frequency=I2C_FAST_HZ):
    await start_project_clock(dut)
    master = I2CMaster(dut, i2c_frequency)
    await reset_dut(dut, master)
    return master


async def write_reg(master, address, value):
    await master.write_registers(address, [value])


async def read_reg(master, address):
    return (await master.read_registers(address, 1))[0]


async def read_event_count(master):
    low, high = await master.read_registers(REG_EVENT_COUNT_L, 2)
    return low | (high << 8)


async def clear_all_interrupts(master):
    await write_reg(master, REG_IRQ_CLEAR_IN, 0xFF)
    await write_reg(master, REG_IRQ_CLEAR_GPIO, GPIO_ALLOWED_MASK)


# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------

@cocotb.test()
async def test_01_reset_defaults_and_register_map(dut):
    """Verify every reset-controlled register and physical reset state."""
    master = await setup_test(dut)

    assert signal_int(dut.uo_out) == 0x00, "Dedicated outputs must reset LOW"
    assert signal_int(dut.uio_oe) == 0x00, "All UIO drivers must be released after reset"

    assert await read_reg(master, REG_DEVICE_ID) == 0x47
    assert await read_reg(master, REG_VERSION) == 0x01

    assert await read_reg(master, REG_OUTPUT_VALUE) == 0x00
    assert await read_reg(master, REG_GPIO_OUTPUT) == 0x00
    assert await read_reg(master, REG_GPIO_DIRECTION) == 0x00

    expected_irq = [
        0x00,               # 0x20 STATUS INPUT
        0x00,               # 0x21 STATUS GPIO
        0xFF,               # 0x22 MASK INPUT
        GPIO_ALLOWED_MASK,  # 0x23 MASK GPIO
        0xFF,               # 0x24 RISE INPUT
        0xFF,               # 0x25 FALL INPUT
        GPIO_ALLOWED_MASK,  # 0x26 RISE GPIO
        GPIO_ALLOWED_MASK,  # 0x27 FALL GPIO
        0x01,               # 0x28 IRQ enabled
        0x00,               # 0x29 CLEAR INPUT reads zero
        0x00,               # 0x2A CLEAR GPIO reads zero
        0x00,               # 0x2B INPUT SNAPSHOT
        0x00,               # 0x2C GPIO SNAPSHOT
        0x00,               # 0x2D EVENT COUNT L
        0x00,               # 0x2E EVENT COUNT H
    ]

    actual_irq = await master.read_registers(REG_IRQ_STATUS_IN, len(expected_irq))
    assert actual_irq == expected_irq, (
        f"IRQ reset map mismatch: expected {expected_irq}, got {actual_irq}"
    )

    # An unmapped location must read zero and ignore writes.
    assert await read_reg(master, 0x30) == 0x00
    await write_reg(master, 0x30, 0xA5)
    assert await read_reg(master, 0x30) == 0x00

    # Read-only registers must ignore writes.
    input_before = await read_reg(master, REG_INPUT_VALUE)

    readonly_expectations = {
        REG_DEVICE_ID: 0x47,
        REG_VERSION: 0x01,
        REG_INPUT_VALUE: input_before,
        REG_IRQ_STATUS_IN: 0x00,
        REG_IRQ_STATUS_GPIO: 0x00,
        REG_INPUT_SNAPSHOT: 0x00,
        REG_GPIO_SNAPSHOT: 0x00,
        REG_EVENT_COUNT_L: 0x00,
        REG_EVENT_COUNT_H: 0x00,
    }

    for address, expected in readonly_expectations.items():
        await write_reg(master, address, 0xA5)
        actual = await read_reg(master, address)
        assert actual == expected, (
            f"Read-only register 0x{address:02X} changed: "
            f"expected 0x{expected:02X}, got 0x{actual:02X}"
        )

    # REG_UIO_VALUE is also read-only. Compare only the GPIO-capable bits,
    # because SCL/SDA naturally change while the register itself is read.
    master.set_external_uio(0x01 | GPIO_IRQ_ON_MASK)
    await master.idle(periods=2)
    await wait_sync(dut)
    await write_reg(master, REG_UIO_VALUE, 0x00)
    uio_after = await read_reg(master, REG_UIO_VALUE)
    # Compare only free GPIO input pins. UIO0 is IRQ_N while IRQ is enabled
    # and may legitimately be LOW if one of these transitions latched an event.
    assert (uio_after & GPIO_IRQ_ON_MASK) == GPIO_IRQ_ON_MASK

    dut._log.info("PASS: reset defaults and complete register-map baseline")


@cocotb.test()
async def test_02_i2c_addressing_protocol_and_pointer(dut):
    """Verify fixed address, ACK/NACK, repeated START and pointer behavior."""
    master = await setup_test(dut)

    assert not await master.probe_address(0x1F, read=False), "Wrong write address unexpectedly ACKed"
    assert not await master.probe_address(0x21, read=False), "Wrong write address unexpectedly ACKed"
    assert not await master.probe_address(0x21, read=True), "Wrong read address unexpectedly ACKed"

    assert await master.probe_address(I2C_ADDRESS, read=False), "Correct write address did not ACK"
    assert await master.probe_address(I2C_ADDRESS, read=True), "Correct read address did not ACK"

    # Repeated START + auto-increment read.
    identity = await master.read_registers(REG_DEVICE_ID, 2)
    assert identity == [0x47, 0x01]

    # Current-address read after explicitly setting the pointer with STOP.
    await master.set_register_pointer(REG_VERSION)
    assert await master.read_current(1) == [0x01]

    # The previous read auto-increments the pointer to 0x02, which is unmapped.
    assert await master.read_current(1) == [0x00]

    # Bus must remain usable after NACKed transactions.
    assert await read_reg(master, REG_DEVICE_ID) == 0x47

    dut._log.info("PASS: I2C addressing, ACK/NACK, START/STOP, repeated START and pointer")


@cocotb.test()
async def test_03_i2c_standard_and_fast_mode(dut):
    """Verify useful transactions at 100 kHz and 400 kHz."""
    master = await setup_test(dut, I2C_STANDARD_HZ)

    for frequency in (I2C_STANDARD_HZ, I2C_FAST_HZ):
        master.set_frequency(frequency)
        await reset_dut(dut, master)

        assert await read_reg(master, REG_DEVICE_ID) == 0x47

        value = 0x5A if frequency == I2C_STANDARD_HZ else 0xA5
        await write_reg(master, REG_OUTPUT_VALUE, value)
        assert signal_int(dut.uo_out) == value
        assert await read_reg(master, REG_OUTPUT_VALUE) == value

        scratch_address = SCRATCHPAD_BASE + (0 if frequency == I2C_STANDARD_HZ else 1)
        await write_reg(master, scratch_address, value ^ 0xFF)
        assert await read_reg(master, scratch_address) == (value ^ 0xFF)

    dut._log.info("PASS: I2C Standard-mode 100 kHz and Fast-mode 400 kHz")


@cocotb.test()
async def test_04_dedicated_8bit_input_output_ports(dut):
    """Verify all eight dedicated output bits and all eight dedicated input bits."""
    master = await setup_test(dut)

    # Disable IRQ generation for this pure I/O test.
    await write_reg(master, REG_IRQ_MASK_IN, 0x00)
    await write_reg(master, REG_IRQ_MASK_GPIO, 0x00)

    output_patterns = [
        0x00, 0xFF, 0xAA, 0x55, 0xA5, 0x5A, 0x01, 0x80,
    ] + [1 << index for index in range(8)]

    for value in output_patterns:
        await write_reg(master, REG_OUTPUT_VALUE, value)
        assert signal_int(dut.uo_out) == value, f"uo_out mismatch for 0x{value:02X}"
        assert await read_reg(master, REG_OUTPUT_VALUE) == value

    input_patterns = [
        0x00, 0xFF, 0xAA, 0x55, 0xA5, 0x5A,
    ] + [1 << index for index in range(8)]

    for value in input_patterns:
        dut.ui_in.value = value
        await wait_sync(dut)
        actual = await read_reg(master, REG_INPUT_VALUE)
        assert actual == value, f"INPUT_VALUE expected 0x{value:02X}, got 0x{actual:02X}"

    dut._log.info("PASS: complete 8-bit dedicated input/output ports")


@cocotb.test()
async def test_05_bidirectional_gpio_and_physical_oe(dut):
    """Verify every GPIO-capable UIO pin in input, output LOW and output HIGH modes."""
    master = await setup_test(dut)

    # IRQ off makes UIO0 available as GPIO0.
    await write_reg(master, REG_IRQ_CONTROL, 0x00)
    await write_reg(master, REG_IRQ_MASK_GPIO, 0x00)

    # Attempts to write I2C bits are always masked.
    await write_reg(master, REG_GPIO_OUTPUT, 0xFF)
    await write_reg(master, REG_GPIO_DIRECTION, 0xFF)

    assert await read_reg(master, REG_GPIO_OUTPUT) == GPIO_ALLOWED_MASK
    assert await read_reg(master, REG_GPIO_DIRECTION) == GPIO_ALLOWED_MASK

    await master.idle()
    assert (signal_int(dut.uio_oe) & 0xFF) == GPIO_ALLOWED_MASK
    assert (signal_int(dut.uio_out) & GPIO_ALLOWED_MASK) == GPIO_ALLOWED_MASK

    # Exercise each physical GPIO output individually HIGH and LOW.
    for index in (0, 1, 4, 5, 6, 7):
        mask = 1 << index

        await write_reg(master, REG_GPIO_DIRECTION, mask)
        await write_reg(master, REG_GPIO_OUTPUT, mask)
        await master.idle()

        oe = signal_int(dut.uio_oe)
        out = signal_int(dut.uio_out)
        assert (oe & GPIO_ALLOWED_MASK) == mask
        assert bit(out, index) == 1

        # The input path must also observe the driven physical pin.
        await wait_sync(dut)
        uio_value = await read_reg(master, REG_UIO_VALUE)
        assert bit(uio_value, index) == 1

        await write_reg(master, REG_GPIO_OUTPUT, 0x00)
        await master.idle()
        assert bit(signal_int(dut.uio_out), index) == 0

    # Return all GPIO to input / Hi-Z.
    await write_reg(master, REG_GPIO_DIRECTION, 0x00)
    assert (signal_int(dut.uio_oe) & GPIO_ALLOWED_MASK) == 0

    # Verify external HIGH levels on every available input GPIO.
    master.set_external_uio(GPIO_ALLOWED_MASK)
    await master.idle(periods=2)
    await wait_sync(dut)
    uio_value = await read_reg(master, REG_UIO_VALUE)
    assert (uio_value & GPIO_ALLOWED_MASK) == GPIO_ALLOWED_MASK

    # Verify external LOW levels.
    master.set_external_uio(0x00)
    await master.idle(periods=2)
    await wait_sync(dut)
    uio_value = await read_reg(master, REG_UIO_VALUE)
    assert (uio_value & GPIO_ALLOWED_MASK) == 0x00

    dut._log.info("PASS: all bidirectional GPIO pins and physical output-enable behavior")


@cocotb.test()
async def test_06_scl_sda_are_fixed_and_open_drain(dut):
    """Prove SCL/SDA cannot become GPIO and SDA never actively drives HIGH."""
    master = await setup_test(dut)

    # Try to force all UIO pins into output mode with HIGH data.
    await write_reg(master, REG_GPIO_OUTPUT, 0xFF)
    await write_reg(master, REG_GPIO_DIRECTION, 0xFF)

    assert await read_reg(master, REG_GPIO_OUTPUT) == GPIO_ALLOWED_MASK
    assert await read_reg(master, REG_GPIO_DIRECTION) == GPIO_ALLOWED_MASK

    # SCL/SDA remain excluded from general-purpose OE.
    oe = signal_int(dut.uio_oe)
    assert (oe & I2C_PIN_MASK) == 0

    # Return GPIO to input so only I2C itself may use SDA OE. Releasing GPIO
    # outputs can legitimately change their physical input level, so allow that
    # transition to settle and clear any GPIO event before testing I2C isolation.
    await write_reg(master, REG_GPIO_DIRECTION, 0x00)
    await master.idle(periods=2)
    await wait_sync(dut)
    await write_reg(master, REG_IRQ_CLEAR_GPIO, GPIO_ALLOWED_MASK)
    await wait_sync(dut)

    master.observed_target_sda_drive = False

    # Force ACKs and both 0/1 target data bits.
    assert await read_reg(master, REG_DEVICE_ID) == 0x47
    assert master.observed_target_sda_drive, "Target never pulled SDA LOW during I2C activity"

    # I2C activity itself must never appear as a GPIO interrupt source.
    assert await read_reg(master, REG_IRQ_STATUS_GPIO) == 0x00

    # After STOP, SDA must be released and SCL must remain input-only.
    await master.idle(periods=2)
    oe = signal_int(dut.uio_oe)
    out = signal_int(dut.uio_out)
    assert bit(oe, 2) == 0
    assert bit(oe, 3) == 0
    assert bit(out, 2) == 0
    assert bit(out, 3) == 0

    dut._log.info("PASS: fixed SCL/SDA assignment and SDA open-drain invariant")


@cocotb.test()
async def test_07_full_32byte_scratchpad_and_boundaries(dut):
    """Verify every scratchpad byte, multi-byte auto-increment and upper boundary."""
    master = await setup_test(dut)

    pattern_a = [((index * 37) + 0x5A) & 0xFF for index in range(32)]
    pattern_b = [value ^ 0xFF for value in reversed(pattern_a)]

    await master.write_registers(SCRATCHPAD_BASE, pattern_a)
    assert await master.read_registers(SCRATCHPAD_BASE, 32) == pattern_a

    await master.write_registers(SCRATCHPAD_BASE, pattern_b)
    assert await master.read_registers(SCRATCHPAD_BASE, 32) == pattern_b

    # Verify the last implemented byte and the first unmapped address.
    await master.write_registers(SCRATCHPAD_LAST, [0xE1, 0xE2])
    boundary = await master.read_registers(SCRATCHPAD_LAST, 2)
    assert boundary == [0xE1, 0x00], f"Scratchpad boundary mismatch: {boundary}"

    # Writes beyond 0x5F are ignored.
    await write_reg(master, 0x60, 0xCC)
    assert await read_reg(master, 0x60) == 0x00

    # The scratchpad is deliberately not connected to rst_n. Once written,
    # asserting the logic reset does not explicitly clear it.
    await write_reg(master, SCRATCHPAD_BASE, 0x7D)
    await reset_dut(dut, master)
    assert await read_reg(master, SCRATCHPAD_BASE) == 0x7D

    dut._log.info("PASS: complete 32-byte scratchpad, auto-increment and boundary behavior")


@cocotb.test()
async def test_08_irq_defaults_enable_and_gpio0_mux(dut):
    """Verify IRQ default enable and complete IRQ_N/GPIO0 alternate-function behavior."""
    master = await setup_test(dut)

    assert await read_reg(master, REG_IRQ_CONTROL) == 0x01
    assert bit(signal_int(dut.uio_oe), 0) == 0, "Inactive IRQ_N must be released"

    # Only bit 0 of IRQ_CONTROL is meaningful.
    await write_reg(master, REG_IRQ_CONTROL, 0xFE)
    assert await read_reg(master, REG_IRQ_CONTROL) == 0x00

    # GPIO0 can now be a normal push-pull output.
    await write_reg(master, REG_GPIO_OUTPUT, 0x01)
    await write_reg(master, REG_GPIO_DIRECTION, 0x01)
    await master.idle()

    assert bit(signal_int(dut.uio_oe), 0) == 1
    assert bit(signal_int(dut.uio_out), 0) == 1

    # Re-enable IRQ with no pending status. IRQ must override GPIO0 and release it.
    await write_reg(master, REG_IRQ_CONTROL, 0xFF)
    await master.idle()

    assert await read_reg(master, REG_IRQ_CONTROL) == 0x01
    assert bit(signal_int(dut.uio_oe), 0) == 0
    assert bit(signal_int(dut.uio_out), 0) == 0, "IRQ output data path must be LOW/open-drain"

    # Disable IRQ again: the previously configured GPIO0 output must reappear.
    await write_reg(master, REG_IRQ_CONTROL, 0x00)
    await master.idle()
    assert bit(signal_int(dut.uio_oe), 0) == 1
    assert bit(signal_int(dut.uio_out), 0) == 1

    dut._log.info("PASS: IRQ enabled by reset and IRQ_N/GPIO0 mux behavior")


@cocotb.test()
async def test_09_irq_all_dedicated_input_bits_both_edges(dut):
    """Verify rising and falling IRQ generation on every dedicated input bit."""
    master = await setup_test(dut)

    await clear_all_interrupts(master)
    dut.ui_in.value = 0x00
    await wait_sync(dut)
    await clear_all_interrupts(master)

    expected_count = 0

    for index in range(8):
        mask = 1 << index

        # Rising edge.
        dut.ui_in.value = mask
        await wait_sync(dut)
        expected_count += 1

        status = await read_reg(master, REG_IRQ_STATUS_IN)
        assert status == mask, (
            f"Rising IRQ status for INPUT{index}: expected 0x{mask:02X}, got 0x{status:02X}"
        )
        assert bit(signal_int(dut.uio_oe), 0) == 1, "IRQ_N was not asserted"
        assert bit(signal_int(dut.uio_out), 0) == 0
        assert await read_reg(master, REG_INPUT_SNAPSHOT) == mask

        await write_reg(master, REG_IRQ_CLEAR_IN, mask)
        assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x00
        assert bit(signal_int(dut.uio_oe), 0) == 0

        # Falling edge.
        dut.ui_in.value = 0x00
        await wait_sync(dut)
        expected_count += 1

        status = await read_reg(master, REG_IRQ_STATUS_IN)
        assert status == mask, (
            f"Falling IRQ status for INPUT{index}: expected 0x{mask:02X}, got 0x{status:02X}"
        )
        assert await read_reg(master, REG_INPUT_SNAPSHOT) == 0x00

        await write_reg(master, REG_IRQ_CLEAR_IN, mask)
        assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x00

    assert await read_event_count(master) == expected_count

    dut._log.info("PASS: rising/falling IRQ on all eight dedicated input pins")


@cocotb.test()
async def test_10_irq_all_bidirectional_gpio_inputs(dut):
    """Verify rising/falling IRQ generation on every free GPIO input while IRQ is enabled."""
    master = await setup_test(dut)

    await write_reg(master, REG_GPIO_DIRECTION, 0x00)
    await write_reg(master, REG_IRQ_MASK_GPIO, GPIO_IRQ_ON_MASK)
    await write_reg(master, REG_IRQ_RISE_GPIO, GPIO_IRQ_ON_MASK)
    await write_reg(master, REG_IRQ_FALL_GPIO, GPIO_IRQ_ON_MASK)

    master.set_external_uio(0x01)
    await master.idle(periods=2)
    await wait_sync(dut)
    await clear_all_interrupts(master)

    expected_count = 0

    for index in (1, 4, 5, 6, 7):
        mask = 1 << index

        # Rising edge. Keep the IRQ pull-up on bit 0.
        master.set_external_uio(0x01 | mask)
        await master.idle(periods=2)
        await wait_sync(dut)
        expected_count += 1

        status = await read_reg(master, REG_IRQ_STATUS_GPIO)
        assert status == mask, (
            f"Rising GPIO IRQ for UIO{index}: expected 0x{mask:02X}, got 0x{status:02X}"
        )
        snapshot = await read_reg(master, REG_GPIO_SNAPSHOT)
        assert snapshot & mask
        assert bit(signal_int(dut.uio_oe), 0) == 1

        await write_reg(master, REG_IRQ_CLEAR_GPIO, mask)
        assert await read_reg(master, REG_IRQ_STATUS_GPIO) == 0x00

        # Falling edge.
        master.set_external_uio(0x01)
        await master.idle(periods=2)
        await wait_sync(dut)
        expected_count += 1

        status = await read_reg(master, REG_IRQ_STATUS_GPIO)
        assert status == mask, (
            f"Falling GPIO IRQ for UIO{index}: expected 0x{mask:02X}, got 0x{status:02X}"
        )
        snapshot = await read_reg(master, REG_GPIO_SNAPSHOT)
        assert (snapshot & mask) == 0

        await write_reg(master, REG_IRQ_CLEAR_GPIO, mask)

    assert await read_event_count(master) == expected_count

    dut._log.info("PASS: rising/falling IRQ on all five GPIO inputs available with IRQ enabled")


@cocotb.test()
async def test_11_gpio0_interrupt_status_while_irq_disabled(dut):
    """Verify GPIO0 can capture an event while IRQ output is disabled, then assert when re-enabled."""
    master = await setup_test(dut)

    await write_reg(master, REG_IRQ_CONTROL, 0x00)
    await write_reg(master, REG_GPIO_DIRECTION, 0x00)
    await write_reg(master, REG_IRQ_MASK_GPIO, 0x01)
    await write_reg(master, REG_IRQ_RISE_GPIO, 0x01)
    await write_reg(master, REG_IRQ_FALL_GPIO, 0x01)

    # Establish GPIO0 LOW while IRQ mode is disabled.
    master.set_external_uio(0x00)
    await master.idle(periods=2)
    await wait_sync(dut)
    await clear_all_interrupts(master)

    # Rising GPIO0 event is latched, but IRQ_N cannot assert while irq_enable=0.
    master.set_external_uio(0x01)
    await master.idle(periods=2)
    await wait_sync(dut)

    assert await read_reg(master, REG_IRQ_STATUS_GPIO) == 0x01
    assert bit(signal_int(dut.uio_oe), 0) == 0

    # Re-enabling IRQ converts the pending status into an active-low IRQ.
    await write_reg(master, REG_IRQ_CONTROL, 0x01)
    await master.idle()
    assert bit(signal_int(dut.uio_oe), 0) == 1
    assert bit(signal_int(dut.uio_out), 0) == 0

    await write_reg(master, REG_IRQ_CLEAR_GPIO, 0x01)
    assert await read_reg(master, REG_IRQ_STATUS_GPIO) == 0x00
    assert bit(signal_int(dut.uio_oe), 0) == 0

    dut._log.info("PASS: GPIO0 event latch while IRQ disabled and assertion after re-enable")


@cocotb.test()
async def test_12_irq_masks_edge_selection_and_partial_w1c(dut):
    """Verify programmable masks, rising/falling selection and partial W1C semantics."""
    master = await setup_test(dut)

    # Only INPUT2 is unmasked; rising only.
    await write_reg(master, REG_IRQ_MASK_IN, 0x04)
    await write_reg(master, REG_IRQ_RISE_IN, 0x04)
    await write_reg(master, REG_IRQ_FALL_IN, 0x00)

    dut.ui_in.value = 0x00
    await wait_sync(dut)
    await clear_all_interrupts(master)

    # INPUT1 is masked, INPUT2 is enabled.
    dut.ui_in.value = 0x06
    await wait_sync(dut)
    assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x04
    await write_reg(master, REG_IRQ_CLEAR_IN, 0x04)

    # Falling edge is disabled.
    dut.ui_in.value = 0x00
    await wait_sync(dut)
    assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x00

    # Falling-only now.
    await write_reg(master, REG_IRQ_RISE_IN, 0x00)
    await write_reg(master, REG_IRQ_FALL_IN, 0x04)

    dut.ui_in.value = 0x04
    await wait_sync(dut)
    assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x00

    dut.ui_in.value = 0x00
    await wait_sync(dut)
    assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x04
    await write_reg(master, REG_IRQ_CLEAR_IN, 0x04)

    # Partial W1C: leave one event pending and keep IRQ asserted.
    await write_reg(master, REG_IRQ_MASK_IN, 0x03)
    await write_reg(master, REG_IRQ_RISE_IN, 0x03)
    await write_reg(master, REG_IRQ_FALL_IN, 0x00)

    dut.ui_in.value = 0x00
    await wait_sync(dut)
    await clear_all_interrupts(master)

    dut.ui_in.value = 0x03
    await wait_sync(dut)
    assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x03
    assert bit(signal_int(dut.uio_oe), 0) == 1

    await write_reg(master, REG_IRQ_CLEAR_IN, 0x01)
    assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x02
    assert bit(signal_int(dut.uio_oe), 0) == 1, "IRQ must remain active while a status bit is pending"

    await write_reg(master, REG_IRQ_CLEAR_IN, 0x02)
    assert await read_reg(master, REG_IRQ_STATUS_IN) == 0x00
    assert bit(signal_int(dut.uio_oe), 0) == 0

    # GPIO configuration registers themselves must mask I2C bits 2 and 3.
    await write_reg(master, REG_IRQ_MASK_GPIO, 0xFF)
    await write_reg(master, REG_IRQ_RISE_GPIO, 0xFF)
    await write_reg(master, REG_IRQ_FALL_GPIO, 0xFF)
    assert await read_reg(master, REG_IRQ_MASK_GPIO) == GPIO_ALLOWED_MASK
    assert await read_reg(master, REG_IRQ_RISE_GPIO) == GPIO_ALLOWED_MASK
    assert await read_reg(master, REG_IRQ_FALL_GPIO) == GPIO_ALLOWED_MASK

    dut._log.info("PASS: IRQ masks, edge selection, GPIO masks and partial W1C")


@cocotb.test()
async def test_13_snapshots_and_simultaneous_event_count(dut):
    """Verify both snapshots and that simultaneous multi-bit events count once."""
    master = await setup_test(dut)

    await write_reg(master, REG_GPIO_DIRECTION, 0x00)
    await clear_all_interrupts(master)

    dut.ui_in.value = 0x00
    master.set_external_uio(0x01)
    await master.idle(periods=2)
    await wait_sync(dut)
    await clear_all_interrupts(master)

    # Change dedicated inputs and multiple free GPIO pins essentially together.
    input_pattern = 0xA5
    gpio_pattern = 0xB2  # Only UIO1,4,5,7 from the monitored GPIO mask.

    dut.ui_in.value = input_pattern
    master.set_external_uio(0x01 | gpio_pattern)
    await master.idle(periods=2)
    await wait_sync(dut)

    assert await read_reg(master, REG_IRQ_STATUS_IN) == input_pattern
    assert await read_reg(master, REG_IRQ_STATUS_GPIO) == (gpio_pattern & GPIO_IRQ_ON_MASK)

    assert await read_reg(master, REG_INPUT_SNAPSHOT) == input_pattern
    gpio_snapshot = await read_reg(master, REG_GPIO_SNAPSHOT)
    assert (gpio_snapshot & GPIO_IRQ_ON_MASK) == (gpio_pattern & GPIO_IRQ_ON_MASK)

    # All synchronized changes occurred in the same detector cycle.
    assert await read_event_count(master) == 1

    dut._log.info("PASS: event snapshots and simultaneous-event counter semantics")


@cocotb.test()
async def test_14_event_counter_16bit_high_byte(dut):
    """Drive enough events to verify the event counter's high byte."""
    master = await setup_test(dut)

    # Disable physical IRQ output while preserving event detection.
    await write_reg(master, REG_IRQ_CONTROL, 0x00)
    await write_reg(master, REG_IRQ_MASK_IN, 0x01)
    await write_reg(master, REG_IRQ_RISE_IN, 0x01)
    await write_reg(master, REG_IRQ_FALL_IN, 0x01)

    dut.ui_in.value = 0x00
    await wait_sync(dut)
    await clear_all_interrupts(master)

    transitions = 260
    current = 0

    for _ in range(transitions):
        current ^= 1
        dut.ui_in.value = current
        await ClockCycles(dut.clk, 4)

    count = await read_event_count(master)
    assert count == transitions, f"Expected event_count={transitions}, got {count}"
    assert await read_reg(master, REG_EVENT_COUNT_L) == (transitions & 0xFF)
    assert await read_reg(master, REG_EVENT_COUNT_H) == ((transitions >> 8) & 0xFF)

    dut._log.info("PASS: 16-bit event counter including high-byte increment")


@cocotb.test()
async def test_15_gpio_outputs_are_not_monitored_as_gpio_inputs(dut):
    """Verify pins configured as outputs are removed from GPIO change monitoring."""
    master = await setup_test(dut)

    await write_reg(master, REG_IRQ_MASK_GPIO, GPIO_IRQ_ON_MASK)
    await write_reg(master, REG_IRQ_RISE_GPIO, GPIO_IRQ_ON_MASK)
    await write_reg(master, REG_IRQ_FALL_GPIO, GPIO_IRQ_ON_MASK)

    # UIO4 as output.
    await write_reg(master, REG_GPIO_DIRECTION, 0x10)
    await clear_all_interrupts(master)

    # Toggle its output repeatedly. It must not create GPIO input events.
    for value in (0x10, 0x00, 0x10, 0x00):
        await write_reg(master, REG_GPIO_OUTPUT, value)
        await master.idle()
        await wait_sync(dut)

    assert (await read_reg(master, REG_IRQ_STATUS_GPIO) & 0x10) == 0

    # External attempts to toggle the same pin are overridden by the ASIC output
    # in the physical model and also excluded by gpio_monitor_mask.
    master.set_external_uio(0x11)
    await master.idle(periods=2)
    await wait_sync(dut)
    assert (await read_reg(master, REG_IRQ_STATUS_GPIO) & 0x10) == 0

    dut._log.info("PASS: output-configured GPIO pins are excluded from IRQ monitoring")


@cocotb.test()
async def test_16_asynchronous_reset_during_active_i2c(dut):
    """Assert reset while the target is actively ACKing and verify full recovery."""
    master = await setup_test(dut)

    # Put a non-reset value on a reset-controlled output first.
    await write_reg(master, REG_OUTPUT_VALUE, 0xA5)
    assert signal_int(dut.uo_out) == 0xA5

    # Send a correct address manually, then enter the ACK LOW phase and hold it.
    await master.start()
    address_byte = (I2C_ADDRESS << 1) | 0
    for index in range(7, -1, -1):
        await master.write_bit(bit(address_byte, index))

    # Release SDA while SCL remains LOW. The target should pull SDA LOW for ACK.
    await master._hold(0, True, master.low_setup_ns)
    assert bit(signal_int(dut.uio_oe), 3) == 1, "Target did not enter ACK drive state"
    assert bit(signal_int(dut.uio_out), 3) == 0

    # Asynchronous active-low reset must immediately release SDA and reset outputs.
    dut.rst_n.value = 0
    await Timer(1, unit="ns")

    assert signal_int(dut.uo_out) == 0x00
    assert signal_int(dut.uio_oe) == 0x00

    # Restore an idle external bus, release reset and verify the I2C FSM recovers.
    dut.uio_in.value = 0x0D
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    master.set_external_uio(0x01)
    await master.idle(periods=2)
    await wait_sync(dut)

    assert await read_reg(master, REG_DEVICE_ID) == 0x47
    await write_reg(master, REG_OUTPUT_VALUE, 0x5A)
    assert signal_int(dut.uo_out) == 0x5A

    dut._log.info("PASS: asynchronous reset during active I2C and post-reset recovery")
