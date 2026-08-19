// Copyright (c) 2026 Eduardo Norambuena. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module scratchpad_ram (
    input  wire       clk,
    input  wire       write_enable,
    input  wire [4:0] write_addr,
    input  wire [7:0] write_data,
    input  wire [4:0] read_addr,
    output wire [7:0] read_data
);

    // 32-byte software scratchpad. The contents are intentionally not reset;
    // firmware must write a location before relying on its value.
    reg [7:0] memory [0:31];

    always @(posedge clk) begin
        if (write_enable)
            memory[write_addr] <= write_data;
    end

    assign read_data = memory[read_addr];

endmodule

`default_nettype wire
