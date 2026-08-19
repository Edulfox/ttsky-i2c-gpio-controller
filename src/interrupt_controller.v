// Copyright (c) 2026 Eduardo Norambuena. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module interrupt_controller (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] input_port_async,
    input  wire [7:0] uio_port_async,
    input  wire [7:0] gpio_direction,

    input  wire       irq_enable,
    input  wire [7:0] irq_mask_input,
    input  wire [7:0] irq_mask_gpio,
    input  wire [7:0] irq_rise_input,
    input  wire [7:0] irq_fall_input,
    input  wire [7:0] irq_rise_gpio,
    input  wire [7:0] irq_fall_gpio,

    input  wire [7:0] irq_clear_input,
    input  wire [7:0] irq_clear_gpio,

    output wire [7:0] input_port_sync,
    output wire [7:0] uio_port_sync,

    output reg  [7:0] irq_status_input,
    output reg  [7:0] irq_status_gpio,
    output reg  [7:0] input_snapshot,
    output reg  [7:0] gpio_snapshot,
    output reg [15:0] event_count,
    output wire       irq_active
);

    reg [7:0] input_meta;
    reg [7:0] input_sync;
    reg [7:0] uio_meta;
    reg [7:0] uio_sync;

    reg [7:0] input_previous;
    reg [7:0] uio_previous;
    reg [1:0] sync_valid;

    wire [7:0] input_rising;
    wire [7:0] input_falling;
    wire [7:0] uio_rising;
    wire [7:0] uio_falling;
    wire [7:0] gpio_monitor_mask;
    wire [7:0] input_events;
    wire [7:0] gpio_events;
    wire       any_event;

    assign input_port_sync = input_sync;
    assign uio_port_sync   = uio_sync;

    assign input_rising  =  input_sync & ~input_previous;
    assign input_falling = ~input_sync &  input_previous;
    assign uio_rising    =  uio_sync & ~uio_previous;
    assign uio_falling   = ~uio_sync &  uio_previous;

    // UIO2 and UIO3 are permanently reserved for I2C.
    // UIO0 is monitored as GPIO only when IRQ_N is disabled.
    assign gpio_monitor_mask[0] = ~irq_enable & ~gpio_direction[0];
    assign gpio_monitor_mask[1] = ~gpio_direction[1];
    assign gpio_monitor_mask[2] = 1'b0;
    assign gpio_monitor_mask[3] = 1'b0;
    assign gpio_monitor_mask[4] = ~gpio_direction[4];
    assign gpio_monitor_mask[5] = ~gpio_direction[5];
    assign gpio_monitor_mask[6] = ~gpio_direction[6];
    assign gpio_monitor_mask[7] = ~gpio_direction[7];

    assign input_events =
        ((input_rising & irq_rise_input) |
         (input_falling & irq_fall_input)) & irq_mask_input;

    assign gpio_events =
        (((uio_rising & irq_rise_gpio) |
          (uio_falling & irq_fall_gpio)) & irq_mask_gpio) &
        gpio_monitor_mask;

    assign any_event = (|input_events) | (|gpio_events);

    assign irq_active =
        irq_enable &&
        ((|irq_status_input) | (|irq_status_gpio));

    // Synchronize all external data inputs into the project clock domain.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_meta <= 8'h00;
            input_sync <= 8'h00;
            uio_meta   <= 8'h00;
            uio_sync   <= 8'h00;
        end else begin
            input_meta <= input_port_async;
            input_sync <= input_meta;

            uio_meta   <= uio_port_async;
            uio_sync   <= uio_meta;
        end
    end

    // Detect changes only after the synchronizers have settled.
    // This prevents initial external levels from creating a false interrupt.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_previous   <= 8'h00;
            uio_previous     <= 8'h00;
            sync_valid       <= 2'b00;

            irq_status_input <= 8'h00;
            irq_status_gpio  <= 8'h00;

            input_snapshot   <= 8'h00;
            gpio_snapshot    <= 8'h00;

            event_count      <= 16'h0000;
        end else begin
            sync_valid <= {sync_valid[0], 1'b1};

            if (&sync_valid) begin

                // Write-one-to-clear requests are applied first.
                // A new event in the same cycle wins and remains pending.
                irq_status_input <=
                    (irq_status_input & ~irq_clear_input) |
                    input_events;

                irq_status_gpio <=
                    (irq_status_gpio & ~irq_clear_gpio) |
                    gpio_events;

                if (any_event) begin
                    input_snapshot <= input_sync;
                    gpio_snapshot  <= uio_sync;
                    event_count    <= event_count + 16'h0001;
                end
            end else begin

                irq_status_input <=
                    irq_status_input & ~irq_clear_input;

                irq_status_gpio <=
                    irq_status_gpio & ~irq_clear_gpio;
            end

            input_previous <= input_sync;
            uio_previous   <= uio_sync;
        end
    end

endmodule

`default_nettype wire
