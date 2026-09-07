`timescale 1ns / 1ps
// Purpose: ordered OV7670 initialization script for 640x480 RGB565 output.
// The ROM address is only the script step number; it is not a camera register
// address. Each data word is {OV7670 register, value}. The sequencer consumes
// entries from address 0 upward, so reset/delay/mode selection precede window,
// color, automatic-control, and image-tuning writes.
//
// Sources:
//   OV7670 datasheet (documented register names and bit fields)
//   https://strawberry-linux.com/pub/OV7670.pdf
//   Linux ov7670 driver (OmniVision default/window/matrix tables and reserved
//   values, including register A4 = 88)
//   https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov7670.c
//
// Some undocumented/reserved "magic" values come from OmniVision's reference
// sequence as preserved by the Linux driver, rather than being derivable from
// the public datasheet. 16'hFFF0 (delay) and 16'hFFFF (end) are local sentinels.

module ov7670_config_rom (
    input  wire        clk,
    input  wire [7:0]  addr,
    output reg  [15:0] data
);
    always @(posedge clk) begin
        case (addr)
            8'd0:  data <= 16'h12_80; // COM7 reset
            8'd1:  data <= 16'hFF_F0;
            8'd2:  data <= 16'h11_80; // CLKRC use XCLK, no prescale
            8'd3:  data <= 16'h12_04; // COM7 640x480 RGB
            8'd4:  data <= 16'h3A_04; // TSLB
            8'd5:  data <= 16'h0C_00; // COM3 no scaling
            8'd6:  data <= 16'h3E_00; // COM14
            8'd7:  data <= 16'h04_00; // COM1
            8'd8:  data <= 16'h40_D0; // COM15 RGB565 full range
            8'd9:  data <= 16'h8C_00; // RGB444 off
            8'd10: data <= 16'h15_00; // COM10 keep PCLK running in blanking
            8'd11: data <= 16'h17_13; // HSTART (Linux 640x480 window)
            8'd12: data <= 16'h18_01; // HSTOP
            8'd13: data <= 16'h32_B6; // HREF
            8'd14: data <= 16'h19_02; // VSTART
            8'd15: data <= 16'h1A_7A; // VSTOP
            8'd16: data <= 16'h03_0A; // VREF
            8'd17: data <= 16'h0F_4B; // COM6
            8'd18: data <= 16'h1E_00; // MVFP
            8'd19: data <= 16'h33_0B; // CHLF
            8'd20: data <= 16'h3C_78; // COM12
            8'd21: data <= 16'h69_00; // GFIX
            8'd22: data <= 16'h74_10;
            8'd23: data <= 16'hB0_84; // required for color
            8'd24: data <= 16'hB1_0C;
            8'd25: data <= 16'hB2_0E;
            8'd26: data <= 16'hB3_82;
            8'd27: data <= 16'h70_3A; // SCALING_XSC (640x480 dummy)
            8'd28: data <= 16'h71_35; // SCALING_YSC
            8'd29: data <= 16'h72_11;
            8'd30: data <= 16'h73_F0;
            8'd31: data <= 16'hA2_02;
            8'd32: data <= 16'h7A_20; // gamma
            8'd33: data <= 16'h7B_10;
            8'd34: data <= 16'h7C_1E;
            8'd35: data <= 16'h7D_35;
            8'd36: data <= 16'h7E_5A;
            8'd37: data <= 16'h7F_69;
            8'd38: data <= 16'h80_76;
            8'd39: data <= 16'h81_80;
            8'd40: data <= 16'h82_88;
            8'd41: data <= 16'h83_8F;
            8'd42: data <= 16'h84_96;
            8'd43: data <= 16'h85_A3;
            8'd44: data <= 16'h86_AF;
            8'd45: data <= 16'h87_C4;
            8'd46: data <= 16'h88_D7;
            8'd47: data <= 16'h89_E8;
            8'd48: data <= 16'h13_80; // COM8 off while loading AGC/AWB
            8'd49: data <= 16'h00_00;
            8'd50: data <= 16'h10_00;
            8'd51: data <= 16'h0D_40;
            8'd52: data <= 16'h14_38; // COM9 16x gain
            8'd53: data <= 16'hA5_05;
            8'd54: data <= 16'hAB_07;
            8'd55: data <= 16'h24_95;
            8'd56: data <= 16'h25_33;
            8'd57: data <= 16'h26_E3;
            8'd58: data <= 16'h9F_78;
            8'd59: data <= 16'hA0_68;
            8'd60: data <= 16'hA1_03;
            8'd61: data <= 16'hA6_D8;
            8'd62: data <= 16'hA7_D8;
            8'd63: data <= 16'hA8_F0;
            8'd64: data <= 16'hA9_90;
            8'd65: data <= 16'hAA_94;
            8'd66: data <= 16'h43_0A; // AWB
            8'd67: data <= 16'h44_F0;
            8'd68: data <= 16'h45_34;
            8'd69: data <= 16'h46_58;
            8'd70: data <= 16'h47_28;
            8'd71: data <= 16'h48_3A;
            8'd72: data <= 16'h59_88;
            8'd73: data <= 16'h5A_88;
            8'd74: data <= 16'h5B_44;
            8'd75: data <= 16'h5C_67;
            8'd76: data <= 16'h5D_49;
            8'd77: data <= 16'h5E_0E;
            8'd78: data <= 16'h6C_0A;
            8'd79: data <= 16'h6D_55;
            8'd80: data <= 16'h6E_11;
            8'd81: data <= 16'h6F_9F; // advanced AWB
            8'd82: data <= 16'h6A_40;
            8'd83: data <= 16'h01_40; // BLUE
            8'd84: data <= 16'h02_60; // RED
            8'd85: data <= 16'h4F_B3; // RGB565 matrix
            8'd86: data <= 16'h50_B3;
            8'd87: data <= 16'h51_00;
            8'd88: data <= 16'h52_3D;
            8'd89: data <= 16'h53_A7;
            8'd90: data <= 16'h54_E4;
            8'd91: data <= 16'h58_9E;
            8'd92: data <= 16'h3D_C3; // COM13 gamma + UV sat
            8'd93: data <= 16'h41_38; // COM16 AWB gain + denoise
            8'd94: data <= 16'h75_05;
            8'd95: data <= 16'h76_E1;
            8'd96: data <= 16'h4C_00;
            8'd97: data <= 16'h77_01;
            8'd98: data <= 16'h4B_09;
            8'd99: data <= 16'hC9_60; // saturation
            8'd100: data <= 16'h56_40; // contrast
            8'd101: data <= 16'h34_11;
            8'd102: data <= 16'h3B_12; // COM11 auto 50/60
            // Reserved-register value copied from OmniVision's default sequence
            // as preserved in Linux ov7670.c; the public datasheet does not
            // explain how to derive 0x88. ROM address 103 is only its sequence
            // position after the preceding initialization operations.
            8'd103: data <= 16'hA4_88;
            8'd104: data <= 16'h13_E7; // COM8 AGC AWB AEC
            default: data <= 16'hFF_FF;
        endcase
    end
endmodule
