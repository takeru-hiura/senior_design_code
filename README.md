# FPGA OV7670 Real-time Edge Detection Project

Live 640x480 edge maps from an OV7670 on a Digilent Nexys4 DDR
(Artix-7 XC7A100T), processed by a streaming Sobel accelerator in FPGA RTL and
sent to a PC over the board's Ethernet port. 

The camera keeps its proven RGB565 SCCB configuration. Each captured pixel is
converted to 8-bit grayscale, passed through a two-line-buffer 3x3 Sobel
accelerator, and thresholded in hardware. The selected grayscale or Sobel image
is stored in block RAM and transmitted to the PC. Image capture, grayscale
conversion, and Sobel processing all run at the camera pixel clock. The PC remains responsible for license plate decoding. 


## Verilog module map

| File | Function |
| --- | --- |
| `verilog/ov7670_nexys4ddr_top.v` | Top-level clocking, reset, mode selection, and module integration |
| `verilog/ov7670_capture.v` | RGB565 capture and hardware grayscale conversion |
| `verilog/sobel_stream.v` | Streaming 3x3 Sobel edge detector with two line buffers |
| `verilog/frame_buffer.v` | Dual-clock 640x480, 4-bit frame buffer |
| `verilog/eth_video_udp.v` | Video packetization into Ethernet/IPv4/UDP frames |
| `verilog/eth_rmii_tx.v` | 100 Mbps RMII transmitter |
| `verilog/eth_crc32.v` | Ethernet frame CRC |
| `verilog/eth_phy_link.v` | LAN8720A PHY initialization and link configuration |
| `verilog/ov7670_config.v`, `verilog/ov7670_sccb.v` | Camera register configuration over SCCB |
| `verilog/plate_uart.v`, `verilog/uart_rx.v` | Recognition-result receive path from the PC |
| `verilog/ssd1306.v`, `verilog/i2c_write.v` | OLED controller and I2C writer |
| `verilog/clocks.v` | FPGA clock generation |

## BRAM optimization

Sobel operates on the original 8-bit grayscale stream, but the full frame is
stored using the upper four bits of each selected pixel. On readback, the FPGA
replicates the nibble (`abcd` becomes `abcdabcd`) to restore the full 0-255
range before UDP transmission. Binary Sobel values remain exactly black or
white. This reduces frame-buffer BRAM substantially while keeping the 640x480
resolution.

## Hardware

- Nexys4 DDR, JP1 = JTAG
- OV7670 Camera Module
- Short female-to-female jumpers
- SSD1406 Camera Module

### Wiring (match signal names)

Pmod pins 5/11 = GND, 6/12 = 3.3 V.

| OV7670 | Pmod | FPGA |
| --- | --- | --- |
| D0–D3 | JA1–JA4 | C17, D18, E18, G17 |
| D4–D7 | JA7–JA10 | D17, E17, F18, G18 |
| SIOC (SCL) | JB1 | D14 |
| SIOD (SDA) | JB2 | F16 |
| VSYNC | JB3 | G16 |
| HREF | JB4 | H14 |
| XCLK | JB7 | E16 |
| RESET | JB8 | F13  |
| PWDN | JB9 | G13|
| PCLK| JB10 |  |
| 3.3 V | JA6  | |
| GND | JA5  | |

## Build

Part: `xc7a100tcsg324-1`.


Open `vivado`, generate the bitstream, and program the board.

## SSD1306 OLED (Pmod JD)

I2C address **0x3C**. **3.3 V only**. 
| SSD1306 | Pmod JD | FPGA |
| --- | --- | --- |
| SCL | JD1 | H4 |
| SDA | JD2 | H1 |
| GND | JD5 or JD11 | |
| 3.3 V | JD6 or JD12 | |

After programming, the panel shows `PLATE WAITING` until the PC sends a
newline-terminated ASCII string at **115200 8N1** on the board USB-UART (same
USB cable used to program). The PC runs
[FastALPR](https://github.com/ankandrew/fast-alpr) on the Ethernet video and
writes the plate back over serial.

## License plate recognition

1. Ethernet as in the section below (`192.168.1.2`, UDP 5000).
2. Board USB connected so computer shows a USB Serial / FTDI COM port.
3. Rebuild and program FPGA.
4. On the PC:

```bat
python host_pc\alpr_ov7670.py --bind 0.0.0.0
```



## Image format

The FPGA expands the RGB565 components and computes grayscale as
`(R + 2G + B) / 4`. It then computes saturated `|Gx| + |Gy|` with a streaming
3x3 Sobel kernel and emits a binary edge map using threshold 80. The frame
buffer stores four bits per pixel; the UDP payload remains one byte per pixel.

## Ethernet stream

**100 Mbps** RMII to the on-board LAN8720A. The transmitted payload is one byte
per pixel, and each packet carries 192 pixels. 

