# OV7670 license plate capture on Nexys4 DDR

This project captures 640 × 480 RGB565 video from an OV7670 on a Nexys4 DDR
(`xc7a100tcsg324-1`). The FPGA converts each pixel to grayscale and sends it
to a PC over the board's 100 Mbps Ethernet port. One streaming Sobel filter
scores plate-shaped regions and sends the best ROI coordinates with the video.
The PC runs FastALPR on that grayscale crop, retries the full frame when the
crop finds no plate, and writes recognized text to an SSD1306 OLED over the
board's USB-UART connection.

The Sobel image is used for ROI selection only. The UDP image is always
grayscale; SW0 is unused.

## Hardware connections

Set the Nexys4 DDR JP1 jumper to JTAG. Pmod power is **3.3 V**. Connect the
OV7670 as follows; `cam_pclk` must use JB10/H16.

| OV7670 | Nexys4 DDR Pmod |
| --- | --- |
| D0–D7 | JA1–JA4, JA7–JA10, in order |
| SIOC, SIOD | JB1, JB2 |
| VSYNC, HREF | JB3, JB4 |
| XCLK, RESET, PWDN, PCLK | JB7, JB8, JB9, JB10 |
| 3.3 V, GND | JA6, JA5 |

Connect the SSD1306 OLED at I2C address `0x3C`:

| SSD1306 | Nexys4 DDR Pmod JD |
| --- | --- |
| SCL, SDA | JD1, JD2 |
| GND, 3.3 V | JD5/JD11, JD6/JD12 |

Connect board Ethernet to the PC network adapter and the board USB cable to
the PC for programming and serial text output.

## Build and run

1. In Vivado, create a project for `xc7a100tcsg324-1`. Add every `.v` file in
   `verilog/` as a design source and `verilog/nexys4ddr.xdc` as a constraint
   file. Set `ov7670_nexys4ddr_top` as the top module, then generate and
   program the bitstream.
2. Set the PC's Ethernet adapter to static IPv4 address `192.168.1.2` on a
   `/24` subnet. In `verilog/eth_video_udp.v`, set `DEST_MAC` to that adapter's
   actual MAC address before building. The FPGA uses source IP
   `192.168.1.10` and sends UDP to port `5000`.
3. On the PC, use Python 3.10 or newer and install the receiver dependencies:

   ```sh
   python -m pip install numpy opencv-python fast-alpr pyserial
   ```

4. Start the receiver from this directory:

   ```sh
   python host_pc/alpr_ov7670.py --bind 0.0.0.0
   ```

The receiver tries to find the Digilent USB-UART port. If needed, pass
`--serial COM7` on Windows or the corresponding serial device path on macOS
or Linux. The OLED shows `PLATE WAITING` until a plate is recognized. Press
Esc to close the receiver.

## Data path and limits

- `ov7670_capture.v` assembles RGB565 bytes and computes approximately
  `(R + 2G + B) / 4`.
- `sobel_stream.v` applies one 3 × 3 Sobel filter to the 8-bit grayscale
  stream and thresholds its edge magnitude at 80.
- `sobel_roi_locator.v` chooses the highest scoring 128 × 64 region from
  32 × 32 edge tiles. The PC pads the ROI before recognition.
- `frame_buffer.v` stores the high four grayscale bits per pixel in block RAM;
  the packetizer expands each nibble to one byte. Each UDP packet carries 192
  pixels and format-3 ROI metadata.

The frame buffer is a single buffer shared by camera writes and Ethernet
reads, so a transmitted image can contain pixels from two camera frames.
Packet loss can also leave gaps in the PC image. The receiver shows partial
frames instead of waiting indefinitely for every packet.
