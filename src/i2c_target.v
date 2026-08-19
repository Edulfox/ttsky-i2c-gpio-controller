// Copyright (c) 2026 Eduardo Norambuena. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module i2c_target #(
    parameter [6:0] I2C_ADDRESS = 7'h20
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       scl_in,
    input  wire       sda_in,

    output reg        sda_drive_low,

    output reg        bus_write,
    output reg  [7:0] bus_waddr,
    output reg  [7:0] bus_wdata,

    output wire [7:0] bus_raddr,
    input  wire [7:0] bus_rdata
);

    localparam [3:0] ST_IDLE       = 4'd0;
    localparam [3:0] ST_ADDRESS    = 4'd1;
    localparam [3:0] ST_ADDR_ACK   = 4'd2;
    localparam [3:0] ST_REG_ADDR   = 4'd3;
    localparam [3:0] ST_REG_ACK    = 4'd4;
    localparam [3:0] ST_WRITE_DATA = 4'd5;
    localparam [3:0] ST_WRITE_ACK  = 4'd6;
    localparam [3:0] ST_READ_DATA  = 4'd7;
    localparam [3:0] ST_READ_ACK   = 4'd8;
    localparam [3:0] ST_IGNORE     = 4'd9;

    reg scl_meta;
    reg scl_sync;
    reg scl_previous;

    reg sda_meta;
    reg sda_sync;
    reg sda_previous;

    reg [3:0] state;
    reg [2:0] bit_count;

    reg [7:0] rx_shift;
    reg [7:0] tx_shift;
    reg [7:0] reg_pointer;

    reg address_match;
    reg rw_bit;

    reg       ack_phase;
    reg [1:0] read_ack_phase;
    reg       master_ack;

    wire scl_rising;
    wire scl_falling;

    wire sda_rising;
    wire sda_falling;

    wire start_condition;
    wire stop_condition;

    assign scl_rising =
        scl_sync & ~scl_previous;

    assign scl_falling =
        ~scl_sync & scl_previous;

    assign sda_rising =
        sda_sync & ~sda_previous;

    assign sda_falling =
        ~sda_sync & sda_previous;

    assign start_condition =
        sda_falling & scl_sync;

    assign stop_condition =
        sda_rising & scl_sync;

    assign bus_raddr = reg_pointer;

    // Synchronize asynchronous I2C lines.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_meta     <= 1'b1;
            scl_sync     <= 1'b1;
            scl_previous <= 1'b1;

            sda_meta     <= 1'b1;
            sda_sync     <= 1'b1;
            sda_previous <= 1'b1;
        end else begin
            scl_meta     <= scl_in;
            scl_sync     <= scl_meta;
            scl_previous <= scl_sync;

            sda_meta     <= sda_in;
            sda_sync     <= sda_meta;
            sda_previous <= sda_sync;
        end
    end

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            state <= ST_IDLE;

            bit_count <= 3'd0;

            rx_shift <= 8'h00;
            tx_shift <= 8'h00;

            reg_pointer <= 8'h00;

            address_match <= 1'b0;
            rw_bit        <= 1'b0;

            ack_phase      <= 1'b0;
            read_ack_phase <= 2'd0;
            master_ack     <= 1'b0;

            sda_drive_low <= 1'b0;

            bus_write <= 1'b0;
            bus_waddr <= 8'h00;
            bus_wdata <= 8'h00;

        end else begin

            bus_write <= 1'b0;

            // START and repeated START have priority.
            if (start_condition) begin

                state <= ST_ADDRESS;

                bit_count <= 3'd0;
                rx_shift  <= 8'h00;

                address_match <= 1'b0;
                rw_bit        <= 1'b0;

                ack_phase      <= 1'b0;
                read_ack_phase <= 2'd0;

                sda_drive_low <= 1'b0;

            end else if (stop_condition) begin

                state <= ST_IDLE;

                bit_count <= 3'd0;

                ack_phase      <= 1'b0;
                read_ack_phase <= 2'd0;

                sda_drive_low <= 1'b0;

            end else begin

                case (state)

                    ST_IDLE: begin
                        sda_drive_low <= 1'b0;
                    end

                    ST_ADDRESS: begin

                        if (scl_rising) begin

                            rx_shift <= {
                                rx_shift[6:0],
                                sda_sync
                            };

                            if (bit_count == 3'd7) begin

                                address_match <=
                                    (rx_shift[6:0] == I2C_ADDRESS);

                                rw_bit <= sda_sync;

                                bit_count <= 3'd0;
                                ack_phase <= 1'b0;

                                state <= ST_ADDR_ACK;

                            end else begin

                                bit_count <=
                                    bit_count + 3'd1;

                            end
                        end
                    end

                    ST_ADDR_ACK: begin

                        if (scl_falling) begin

                            if (!ack_phase) begin

                                // ACK only when address matches.
                                sda_drive_low <= address_match;
                                ack_phase     <= 1'b1;

                            end else begin

                                ack_phase <= 1'b0;

                                if (!address_match) begin

                                    sda_drive_low <= 1'b0;
                                    state <= ST_IGNORE;

                                end else if (!rw_bit) begin

                                    sda_drive_low <= 1'b0;

                                    rx_shift  <= 8'h00;
                                    bit_count <= 3'd0;

                                    state <= ST_REG_ADDR;

                                end else begin

                                    // Read transaction.
                                    tx_shift <= bus_rdata;

                                    // Open-drain:
                                    // bit=0 -> drive LOW
                                    // bit=1 -> release SDA
                                    sda_drive_low <=
                                        ~bus_rdata[7];

                                    reg_pointer <=
                                        reg_pointer + 8'h01;

                                    bit_count <= 3'd0;

                                    state <= ST_READ_DATA;

                                end
                            end
                        end
                    end

                    ST_REG_ADDR: begin

                        if (scl_rising) begin

                            rx_shift <= {
                                rx_shift[6:0],
                                sda_sync
                            };

                            if (bit_count == 3'd7) begin

                                reg_pointer <= {
                                    rx_shift[6:0],
                                    sda_sync
                                };

                                bit_count <= 3'd0;
                                ack_phase <= 1'b0;

                                state <= ST_REG_ACK;

                            end else begin

                                bit_count <=
                                    bit_count + 3'd1;

                            end
                        end
                    end

                    ST_REG_ACK: begin

                        if (scl_falling) begin

                            if (!ack_phase) begin

                                sda_drive_low <= 1'b1;
                                ack_phase <= 1'b1;

                            end else begin

                                sda_drive_low <= 1'b0;
                                ack_phase <= 1'b0;

                                rx_shift  <= 8'h00;
                                bit_count <= 3'd0;

                                state <= ST_WRITE_DATA;

                            end
                        end
                    end

                    ST_WRITE_DATA: begin

                        if (scl_rising) begin

                            rx_shift <= {
                                rx_shift[6:0],
                                sda_sync
                            };

                            if (bit_count == 3'd7) begin

                                bus_waddr <= reg_pointer;

                                bus_wdata <= {
                                    rx_shift[6:0],
                                    sda_sync
                                };

                                bus_write <= 1'b1;

                                reg_pointer <=
                                    reg_pointer + 8'h01;

                                bit_count <= 3'd0;
                                ack_phase <= 1'b0;

                                state <= ST_WRITE_ACK;

                            end else begin

                                bit_count <=
                                    bit_count + 3'd1;

                            end
                        end
                    end

                    ST_WRITE_ACK: begin

                        if (scl_falling) begin

                            if (!ack_phase) begin

                                sda_drive_low <= 1'b1;
                                ack_phase <= 1'b1;

                            end else begin

                                sda_drive_low <= 1'b0;
                                ack_phase <= 1'b0;

                                rx_shift <= 8'h00;
                                bit_count <= 3'd0;

                                state <= ST_WRITE_DATA;

                            end
                        end
                    end

                    ST_READ_DATA: begin

                        if (scl_rising) begin

                            if (bit_count == 3'd7) begin

                                bit_count <= 3'd0;

                                read_ack_phase <= 2'd0;

                                state <= ST_READ_ACK;

                            end else begin

                                bit_count <=
                                    bit_count + 3'd1;

                            end
                        end

                        if (scl_falling) begin

                            tx_shift <= {
                                tx_shift[6:0],
                                1'b0
                            };

                            sda_drive_low <=
                                ~tx_shift[6];

                        end
                    end

                    ST_READ_ACK: begin

                        // Release SDA for controller ACK/NACK.
                        if (
                            scl_falling &&
                            (read_ack_phase == 2'd0)
                        ) begin

                            sda_drive_low <= 1'b0;
                            read_ack_phase <= 2'd1;

                        end

                        // Sample ACK/NACK.
                        if (
                            scl_rising &&
                            (read_ack_phase == 2'd1)
                        ) begin

                            master_ack <= ~sda_sync;
                            read_ack_phase <= 2'd2;

                        end

                        // Continue or stop after ACK/NACK.
                        if (
                            scl_falling &&
                            (read_ack_phase == 2'd2)
                        ) begin

                            read_ack_phase <= 2'd0;

                            if (master_ack) begin

                                tx_shift <= bus_rdata;

                                sda_drive_low <=
                                    ~bus_rdata[7];

                                reg_pointer <=
                                    reg_pointer + 8'h01;

                                bit_count <= 3'd0;

                                state <= ST_READ_DATA;

                            end else begin

                                sda_drive_low <= 1'b0;
                                state <= ST_IDLE;

                            end
                        end
                    end

                    ST_IGNORE: begin
                        sda_drive_low <= 1'b0;
                    end

                    default: begin
                        state <= ST_IDLE;
                        sda_drive_low <= 1'b0;
                    end

                endcase
            end
        end
    end

endmodule

`default_nettype wire
