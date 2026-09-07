`timescale 1ns / 1ps
// Purpose: execute the ordered register/value script in ov7670_config_rom.
// Each normal word is {register_address, register_value}; 16'hFFF0 waits 50 ms
// and 16'hFFFF ends the script. Those two words are project-local control tokens,
// not OV7670 registers.
//
// Ordering matters because reset must occur first, its settling delay must pass,
// and later writes may select a mode before tuning registers for that mode.
// References are recorded in ov7670_config_rom.v, where the values are defined.

module ov7670_config #(
    parameter CLK_FREQ = 25_000_000
) (
    input  wire        clk,
    input  wire        start,
    input  wire        sccb_ready,
    input  wire [15:0] rom_data,
    output reg  [7:0]  rom_addr,
    output reg  [7:0]  sccb_reg,
    output reg  [7:0]  sccb_val,
    output reg         sccb_start,
    output reg         done
);
    localparam IDLE     = 2'd0;
    localparam SEND     = 2'd1;
    localparam WAIT_T   = 2'd2;
    localparam FINISH   = 2'd3;

    localparam DELAY_50MS = CLK_FREQ / 20;

    reg [1:0]  state, ret_state;
    reg [31:0] timer;

    initial begin
        rom_addr    = 8'd0;
        done        = 1'b0;
        sccb_start  = 1'b0;
        state       = IDLE;
    end

    always @(posedge clk) begin
        case (state)
            IDLE: begin
                rom_addr   <= 8'd0;
                sccb_start <= 1'b0;
                if (start) begin
                    done  <= 1'b0;
                    state <= SEND;
                end
            end

            SEND: begin
                sccb_start <= 1'b0;
                case (rom_data)
                    16'hFFFF: state <= FINISH;
                    16'hFFF0: begin
                        rom_addr   <= rom_addr + 1'b1;
                        timer      <= DELAY_50MS;
                        ret_state  <= SEND;
                        state      <= WAIT_T;
                    end
                    default: begin
                        if (sccb_ready) begin
                            sccb_reg   <= rom_data[15:8];
                            sccb_val   <= rom_data[7:0];
                            sccb_start <= 1'b1;
                            rom_addr   <= rom_addr + 1'b1;
                            timer      <= 32'd0;
                            ret_state  <= SEND;
                            state      <= WAIT_T;
                        end
                    end
                endcase
            end

            WAIT_T: begin
                sccb_start <= 1'b0;
                if (timer == 32'd0)
                    state <= ret_state;
                else
                    timer <= timer - 1'b1;
            end

            FINISH: begin
                done  <= 1'b1;
                state <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
endmodule
