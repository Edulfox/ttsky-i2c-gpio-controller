// Copyright (c) 2026 Eduardo Norambuena. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module tt_um_eduardon_i2c_gpio_controller (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,

    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,

    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire sda_drive_low;
    wire scl_drive_low;

    wire       bus_write;
    wire [7:0] bus_waddr;
    wire [7:0] bus_wdata;
    wire [7:0] bus_raddr;
    wire [7:0] bus_rdata;

    wire [7:0] input_value_sync;
    wire [7:0] uio_value_sync;

    wire [7:0] output_port;
    wire [7:0] gpio_output;
    wire [7:0] gpio_direction;

    wire       irq_enable;

    wire [7:0] irq_mask_input;
    wire [7:0] irq_mask_gpio;

    wire [7:0] irq_rise_input;
    wire [7:0] irq_fall_input;

    wire [7:0] irq_rise_gpio;
    wire [7:0] irq_fall_gpio;

    wire [7:0] irq_clear_input;
    wire [7:0] irq_clear_gpio;

    wire [7:0] irq_status_input;
    wire [7:0] irq_status_gpio;

    wire [7:0] input_snapshot;
    wire [7:0] gpio_snapshot;

    wire [15:0] event_count;

    wire irq_active;

    // Clock stretching is not used in version 1.
    assign scl_drive_low = 1'b0;

    assign uo_out = output_port;

    i2c_target #(
        .I2C_ADDRESS(7'h20)
    ) i2c (
        .clk           (clk),
        .rst_n         (rst_n),

        .scl_in        (uio_in[2]),
        .sda_in        (uio_in[3]),

        .sda_drive_low (sda_drive_low),

        .bus_write     (bus_write),
        .bus_waddr     (bus_waddr),
        .bus_wdata     (bus_wdata),

        .bus_raddr     (bus_raddr),
        .bus_rdata     (bus_rdata)
    );

    interrupt_controller interrupts (
        .clk              (clk),
        .rst_n            (rst_n),

        .input_port_async (ui_in),
        .uio_port_async   (uio_in),

        .gpio_direction   (gpio_direction),

        .irq_enable       (irq_enable),

        .irq_mask_input   (irq_mask_input),
        .irq_mask_gpio    (irq_mask_gpio),

        .irq_rise_input   (irq_rise_input),
        .irq_fall_input   (irq_fall_input),

        .irq_rise_gpio    (irq_rise_gpio),
        .irq_fall_gpio    (irq_fall_gpio),

        .irq_clear_input  (irq_clear_input),
        .irq_clear_gpio   (irq_clear_gpio),

        .input_port_sync  (input_value_sync),
        .uio_port_sync    (uio_value_sync),

        .irq_status_input (irq_status_input),
        .irq_status_gpio  (irq_status_gpio),

        .input_snapshot   (input_snapshot),
        .gpio_snapshot    (gpio_snapshot),

        .event_count      (event_count),

        .irq_active       (irq_active)
    );

    register_bank registers (
        .clk              (clk),
        .rst_n            (rst_n),

        .bus_write        (bus_write),
        .bus_waddr        (bus_waddr),
        .bus_wdata        (bus_wdata),

        .bus_raddr        (bus_raddr),
        .bus_rdata        (bus_rdata),

        .input_value      (input_value_sync),
        .uio_value        (uio_value_sync),

        .irq_status_input (irq_status_input),
        .irq_status_gpio  (irq_status_gpio),

        .input_snapshot   (input_snapshot),
        .gpio_snapshot    (gpio_snapshot),

        .event_count      (event_count),

        .output_port      (output_port),

        .gpio_output      (gpio_output),
        .gpio_direction   (gpio_direction),

        .irq_enable       (irq_enable),

        .irq_mask_input   (irq_mask_input),
        .irq_mask_gpio    (irq_mask_gpio),

        .irq_rise_input   (irq_rise_input),
        .irq_fall_input   (irq_fall_input),

        .irq_rise_gpio    (irq_rise_gpio),
        .irq_fall_gpio    (irq_fall_gpio),

        .irq_clear_input  (irq_clear_input),
        .irq_clear_gpio   (irq_clear_gpio)
    );

    gpio_controller gpio (
        .gpio_output    (gpio_output),
        .gpio_direction (gpio_direction),

        .irq_enable     (irq_enable),
        .irq_active     (irq_active),

        .scl_drive_low  (scl_drive_low),
        .sda_drive_low  (sda_drive_low),

        .uio_out        (uio_out),
        .uio_oe         (uio_oe)
    );

    // ena is always high while the project is selected.
    wire _unused = &{
        ena,
        1'b0
    };

endmodule

`default_nettype wire
