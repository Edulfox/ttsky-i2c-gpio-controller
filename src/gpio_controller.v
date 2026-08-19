// Copyright (c) 2026 Eduardo Norambuena. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module gpio_controller (
    input  wire [7:0] gpio_output,
    input  wire [7:0] gpio_direction,
    input  wire       irq_enable,
    input  wire       irq_active,
    input  wire       scl_drive_low,
    input  wire       sda_drive_low,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe
);

    // UIO0: IRQ_N when IRQ is enabled, otherwise GPIO0.
    // IRQ_N is open-drain: drive LOW when active, release otherwise.
    assign uio_out[0] = irq_enable ? 1'b0 : gpio_output[0];
    assign uio_oe[0]  = irq_enable ? irq_active : gpio_direction[0];

    // UIO1: General-purpose bidirectional GPIO.
    assign uio_out[1] = gpio_output[1];
    assign uio_oe[1]  = gpio_direction[1];

    // UIO2: I2C SCL, permanently reserved.
    // The current implementation does not stretch the clock, but the
    // open-drain output path is kept available for future clock stretching.
    assign uio_out[2] = 1'b0;
    assign uio_oe[2]  = scl_drive_low;

    // UIO3: I2C SDA, permanently reserved and open-drain.
    assign uio_out[3] = 1'b0;
    assign uio_oe[3]  = sda_drive_low;

    // UIO4..UIO7: General-purpose bidirectional GPIO.
    assign uio_out[4] = gpio_output[4];
    assign uio_out[5] = gpio_output[5];
    assign uio_out[6] = gpio_output[6];
    assign uio_out[7] = gpio_output[7];

    assign uio_oe[4] = gpio_direction[4];
    assign uio_oe[5] = gpio_direction[5];
    assign uio_oe[6] = gpio_direction[6];
    assign uio_oe[7] = gpio_direction[7];

endmodule

`default_nettype wire
