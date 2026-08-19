// Copyright (c) 2026 Eduardo Norambuena. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module register_bank (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       bus_write,
    input  wire [7:0] bus_waddr,
    input  wire [7:0] bus_wdata,
    input  wire [7:0] bus_raddr,
    output reg  [7:0] bus_rdata,

    input  wire [7:0] input_value,
    input  wire [7:0] uio_value,

    input  wire [7:0] irq_status_input,
    input  wire [7:0] irq_status_gpio,
    input  wire [7:0] input_snapshot,
    input  wire [7:0] gpio_snapshot,
    input  wire [15:0] event_count,

    output reg  [7:0] output_port,
    output reg  [7:0] gpio_output,
    output reg  [7:0] gpio_direction,

    output reg        irq_enable,
    output reg  [7:0] irq_mask_input,
    output reg  [7:0] irq_mask_gpio,
    output reg  [7:0] irq_rise_input,
    output reg  [7:0] irq_fall_input,
    output reg  [7:0] irq_rise_gpio,
    output reg  [7:0] irq_fall_gpio,
    output reg  [7:0] irq_clear_input,
    output reg  [7:0] irq_clear_gpio
);

    localparam [7:0] REG_DEVICE_ID       = 8'h00;
    localparam [7:0] REG_VERSION         = 8'h01;

    localparam [7:0] REG_INPUT_VALUE     = 8'h10;
    localparam [7:0] REG_OUTPUT_VALUE    = 8'h11;
    localparam [7:0] REG_UIO_VALUE       = 8'h12;
    localparam [7:0] REG_GPIO_OUTPUT     = 8'h13;
    localparam [7:0] REG_GPIO_DIRECTION  = 8'h14;

    localparam [7:0] REG_IRQ_STATUS_IN   = 8'h20;
    localparam [7:0] REG_IRQ_STATUS_GPIO = 8'h21;
    localparam [7:0] REG_IRQ_MASK_IN     = 8'h22;
    localparam [7:0] REG_IRQ_MASK_GPIO   = 8'h23;
    localparam [7:0] REG_IRQ_RISE_IN     = 8'h24;
    localparam [7:0] REG_IRQ_FALL_IN     = 8'h25;
    localparam [7:0] REG_IRQ_RISE_GPIO   = 8'h26;
    localparam [7:0] REG_IRQ_FALL_GPIO   = 8'h27;
    localparam [7:0] REG_IRQ_CONTROL     = 8'h28;
    localparam [7:0] REG_IRQ_CLEAR_IN    = 8'h29;
    localparam [7:0] REG_IRQ_CLEAR_GPIO  = 8'h2A;
    localparam [7:0] REG_INPUT_SNAPSHOT  = 8'h2B;
    localparam [7:0] REG_GPIO_SNAPSHOT   = 8'h2C;
    localparam [7:0] REG_EVENT_COUNT_L   = 8'h2D;
    localparam [7:0] REG_EVENT_COUNT_H   = 8'h2E;

    localparam [7:0] SCRATCHPAD_BASE     = 8'h40;
    localparam [7:0] SCRATCHPAD_LAST     = 8'h5F;

    wire       scratch_write;
    wire [7:0] scratch_read_data;

    assign scratch_write =
        bus_write &&
        (bus_waddr >= SCRATCHPAD_BASE) &&
        (bus_waddr <= SCRATCHPAD_LAST);

    scratchpad_ram scratchpad (
        .clk          (clk),
        .write_enable (scratch_write),
        .write_addr   (bus_waddr[4:0]),
        .write_data   (bus_wdata),
        .read_addr    (bus_raddr[4:0]),
        .read_data    (scratch_read_data)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_port    <= 8'h00;
            gpio_output    <= 8'h00;
            gpio_direction <= 8'h00;

            // IRQ_N is selected on UIO0 immediately after reset.
            irq_enable <= 1'b1;

            // Detect both edges by default.
            irq_mask_input <= 8'hFF;
            irq_mask_gpio  <= 8'hF3;

            irq_rise_input <= 8'hFF;
            irq_fall_input <= 8'hFF;

            irq_rise_gpio <= 8'hF3;
            irq_fall_gpio <= 8'hF3;

            irq_clear_input <= 8'h00;
            irq_clear_gpio  <= 8'h00;

        end else begin

            // Clear requests are one-clock pulses.
            irq_clear_input <= 8'h00;
            irq_clear_gpio  <= 8'h00;

            if (bus_write) begin
                case (bus_waddr)

                    REG_OUTPUT_VALUE: begin
                        output_port <= bus_wdata;
                    end

                    REG_GPIO_OUTPUT: begin

                        // UIO2 and UIO3 are reserved for I2C.
                        gpio_output <= bus_wdata & 8'hF3;
                    end

                    REG_GPIO_DIRECTION: begin

                        // 0=input/Hi-Z, 1=output.
                        // UIO2 and UIO3 cannot be changed into GPIO.
                        gpio_direction <= bus_wdata & 8'hF3;
                    end

                    REG_IRQ_MASK_IN: begin
                        irq_mask_input <= bus_wdata;
                    end

                    REG_IRQ_MASK_GPIO: begin
                        irq_mask_gpio <= bus_wdata & 8'hF3;
                    end

                    REG_IRQ_RISE_IN: begin
                        irq_rise_input <= bus_wdata;
                    end

                    REG_IRQ_FALL_IN: begin
                        irq_fall_input <= bus_wdata;
                    end

                    REG_IRQ_RISE_GPIO: begin
                        irq_rise_gpio <= bus_wdata & 8'hF3;
                    end

                    REG_IRQ_FALL_GPIO: begin
                        irq_fall_gpio <= bus_wdata & 8'hF3;
                    end

                    REG_IRQ_CONTROL: begin
                        irq_enable <= bus_wdata[0];
                    end

                    REG_IRQ_CLEAR_IN: begin
                        irq_clear_input <= bus_wdata;
                    end

                    REG_IRQ_CLEAR_GPIO: begin
                        irq_clear_gpio <= bus_wdata & 8'hF3;
                    end

                    default: begin
                    end
                endcase
            end
        end
    end

    always @(*) begin

        if ((bus_raddr >= SCRATCHPAD_BASE) &&
            (bus_raddr <= SCRATCHPAD_LAST)) begin

            bus_rdata = scratch_read_data;

        end else begin

            case (bus_raddr)

                REG_DEVICE_ID:
                    bus_rdata = 8'h47;

                REG_VERSION:
                    bus_rdata = 8'h01;

                REG_INPUT_VALUE:
                    bus_rdata = input_value;

                REG_OUTPUT_VALUE:
                    bus_rdata = output_port;

                REG_UIO_VALUE:
                    bus_rdata = uio_value;

                REG_GPIO_OUTPUT:
                    bus_rdata = gpio_output;

                REG_GPIO_DIRECTION:
                    bus_rdata = gpio_direction;

                REG_IRQ_STATUS_IN:
                    bus_rdata = irq_status_input;

                REG_IRQ_STATUS_GPIO:
                    bus_rdata = irq_status_gpio;

                REG_IRQ_MASK_IN:
                    bus_rdata = irq_mask_input;

                REG_IRQ_MASK_GPIO:
                    bus_rdata = irq_mask_gpio;

                REG_IRQ_RISE_IN:
                    bus_rdata = irq_rise_input;

                REG_IRQ_FALL_IN:
                    bus_rdata = irq_fall_input;

                REG_IRQ_RISE_GPIO:
                    bus_rdata = irq_rise_gpio;

                REG_IRQ_FALL_GPIO:
                    bus_rdata = irq_fall_gpio;

                REG_IRQ_CONTROL:
                    bus_rdata = {7'b0000000, irq_enable};

                REG_IRQ_CLEAR_IN:
                    bus_rdata = 8'h00;

                REG_IRQ_CLEAR_GPIO:
                    bus_rdata = 8'h00;

                REG_INPUT_SNAPSHOT:
                    bus_rdata = input_snapshot;

                REG_GPIO_SNAPSHOT:
                    bus_rdata = gpio_snapshot;

                REG_EVENT_COUNT_L:
                    bus_rdata = event_count[7:0];

                REG_EVENT_COUNT_H:
                    bus_rdata = event_count[15:8];

                default:
                    bus_rdata = 8'h00;

            endcase
        end
    end

endmodule

`default_nettype wire
