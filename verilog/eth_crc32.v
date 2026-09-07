`timescale 1ns / 1ps
// Purpose: calculate the Ethernet CRC-32 used in the four-byte Frame Check
// Sequence (FCS). Ethernet initializes the CRC register to all ones, processes
// each byte least-significant bit first, and transmits the complemented result.
// The reflected polynomial 0xEDB88320 is the bit-reversed representation of
// the Ethernet generator polynomial 0x04C11DB7.
//
// This detects corrupted Ethernet frames; it does not repair them. A receiver
// recalculates the CRC and discards a frame whose FCS does not match.
// Reference: IEEE Std 802.3, MAC frame FCS/CRC-32 definition.

module eth_crc32 (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire [7:0]  data,
    output reg  [31:0] crc
);
    integer i;
    reg [31:0] c;
    reg         mix;


// Process all eight bits during one FPGA clock. `mix` says whether the incoming
// bit differs from the CRC feedback bit and therefore whether the polynomial
// must be XORed into the shifted remainder.
    always @(posedge clk) begin
        if (rst)
            crc <= 32'hFFFF_FFFF;
        else if (en) begin
            c = crc;
            for (i = 0; i < 8; i = i + 1) begin
                mix = c[0] ^ data[i];
                c = {1'b0, c[31:1]};
                if (mix)
                // XOR is the subtraction operation used by polynomial division
                // over GF(2); there are no carries or borrows.
                    c = c ^ 32'hEDB8_8320;
            end
            crc <= c;
        end
    end
endmodule
