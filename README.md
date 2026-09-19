# FPGA OV7670 Real-time Edge Detection Project

The Sobel-only V1, V2, and V3 optimization comparison, including simulation,
Vivado reporting, power-estimation, and meeting instructions, is documented in
[`docs/SOBEL_OPTIMIZATION_TASK.md`](docs/SOBEL_OPTIMIZATION_TASK.md).

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
| `hdl/ov7670_nexys4ddr_top.v` | Top-level clocking, reset, mode selection, and module integration |
| `hdl/ov7670_capture.v` | RGB565 capture and hardware grayscale conversion |
| `hdl/sobel_stream.v` | Streaming 3x3 Sobel edge detector with two line buffers |
| `hdl/sobel_roi_locator.v` | Scores plate-shaped Sobel windows and publishes the best ROI |
| `hdl/frame_buffer.v` | Dual-clock 640x480, 4-bit frame buffer |
| `hdl/eth_video_udp.v` | Video and ROI packetization into Ethernet/IPv4/UDP frames |
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
| PCLK| JB10 | H16 |
| 3.3 V | JA6  | Pmod JA Pin 6 |
| GND | JA5  | Pmod JA Pin 5 |

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

If the OLED is completely dark, check it before starting the PC script. Power
cycle the board, program a known working bitstream, and allow a few seconds for
initialization. `PLATE WAITING` should appear without Ethernet, camera video,
or ALPR. Check the OLED's VCC-to-GND voltage is 3.3 V, its ground is shared with
the board, and SCL/SDA still reach JD1/JD2. If the waiting text appears but a
recognized plate does not, check the USB-UART port and send a test line at
115200 baud. A persistent dark screen with an older bitstream points to power,
wiring, display address/type, or the FPGA I2C connection rather than ALPR.

## License plate recognition

1. Ethernet as in the section below (`192.168.1.2`, UDP 5000).
2. Board USB connected so computer shows a USB Serial / FTDI COM port.
3. Rebuild and program FPGA.
4. On the PC:

```bat
python pc\alpr_ov7670.py --bind 0.0.0.0
```

### Sobel-assisted ROI mode

The Sobel accelerator now runs continuously and scores 128x64 plate-shaped
windows. Its best bounding box and edge score are included in format-3 N4VD
packets. The PC draws the padded proposal in yellow, runs FastALPR on that
grayscale crop, and automatically retries the complete grayscale frame when
the crop contains no recognized plate.

Keep **SW0 off** during recognition so the frame buffer contains grayscale.
Sobel and the ROI locator still operate internally with SW0 off. Turn SW0 on
only when you want to inspect the binary edge map; pretrained FastALPR accuracy
will be lower in that diagnostic display mode.

From the `Functioning` directory, install the PC dependencies once and run:

```bash
python3 -m pip install -r pc/requirements-alpr.txt
python3 pc/alpr_ov7670.py --bind 0.0.0.0
```

On macOS the receiver uses ONNX Runtime's CPU provider for FastALPR. The
CoreML provider fails when this detector produces a zero-length output for a
frame with no plate. This choice may make inference slower, but it lets the
receiver keep running through empty frames.

Useful comparisons:

```bash
# Ignore the FPGA ROI and run the former full-frame behavior.
python3 pc/alpr_ov7670.py --no-roi

# Increase crop padding if the yellow box clips a plate.
python3 pc/alpr_ov7670.py --roi-padding 0.30

# Compare the faster, smaller OCR model.
python3 pc/alpr_ov7670.py --ocr-model cct-xs-v2-global-model
```

The first camera frame has no previous Sobel result, so its ROI is intentionally
invalid. Format-2 packets from an older bitstream remain accepted and simply
use full-frame recognition.

Before programming the board, the ROI scorer has a small self-checking HDL
simulation. From the `Functioning` directory:

```bash
vivado -mode batch -source tcl/create_vivado_project.tcl
vivado -mode batch -source tcl/run_roi_sim.tcl
```

The second command should print `PASS sobel_roi_locator test`. Open
`vivado/ov7670_nexys4ddr.xpr`, generate the bitstream, program the FPGA, leave
SW0 off, and then start `pc/alpr_ov7670.py`. A yellow rectangle labeled
`Sobel ROI` confirms that format-3 ROI metadata is reaching the PC. Recognition
labels ending in `ROI` were obtained from the grayscale crop; labels without it
came from the automatic full-frame fallback.



## Image format

The FPGA expands the RGB565 components and computes grayscale as
`(R + 2G + B) / 4`. It then computes saturated `|Gx| + |Gy|` with a streaming
3x3 Sobel kernel and emits a binary edge map using threshold 80 to the ROI
locator. The frame buffer stores four bits per selected pixel; the UDP video
payload remains one byte per pixel.

## Ethernet stream

**100 Mbps** RMII to the on-board LAN8720A. The transmitted payload is one byte
per pixel, and each packet carries 192 pixels. Format 3 adds five network-order
16-bit values after the original 16-byte N4VD header: inclusive ROI coordinates
`x0, y0, x1, y1`, followed by the Sobel edge score. A zero score means that no
window passed the hardware threshold.
