# FPGA Real-Time License Plate Preprocessing and Recognition

Real-time image acquisition, edge detection, ROI proposal, and license-plate recognition using a **Digilent Nexys4 DDR (Artix-7 XC7A100T)** and an **OV7670 camera**.

The FPGA performs the time-critical streaming image-processing work:

- OV7670 RGB565 capture
- hardware grayscale conversion
- streaming 3×3 Sobel edge detection
- Sobel-based license-plate ROI proposal
- 4-bit grayscale frame buffering
- Ethernet/IPv4/UDP packetization
- UART receive and SSD1306 OLED output

A host PC reconstructs the grayscale video stream, runs **FastALPR** on the FPGA-proposed ROI, falls back to the full frame when needed, and sends recognized plate text back to the FPGA over USB-UART.

---

## System Architecture

```text
                          FPGA
                 ┌──────────────────────┐
OV7670 RGB565 ──►│ ov7670_capture       │
                 │ RGB565 → grayscale   │
                 └──────────┬───────────┘
                            │ 8-bit grayscale
                 ┌──────────┴───────────┐
                 │                      │
                 ▼                      ▼
        ┌────────────────┐     ┌────────────────┐
        │ sobel_stream   │     │ frame_buffer   │
        │ 3×3 Sobel      │     │ 4 bits/pixel   │
        └───────┬────────┘     └───────┬────────┘
                │ binary edges         │ grayscale frame
                ▼                      ▼
        ┌────────────────┐     ┌────────────────┐
        │sobel_roi_locator│     │ eth_video_udp │
        │128×64 ROI       │────►│ UDP packetizer│
        └────────────────┘     └───────┬────────┘
                                       │ 100 Mbps RMII
                                       ▼
                                    Host PC
                                       │
                          OpenCV + FastALPR
                                       │
                              recognized plate
                                       │ USB-UART
                                       ▼
                              FPGA → SSD1306
```

The main hardware/software partition is intentional: the FPGA handles deterministic, high-throughput pixel processing and ROI generation, while the PC handles neural-network detection and OCR.

---

## Repository Structure

| Path | Purpose |
| --- | --- |
| `verilog/ov7670_nexys4ddr_top.v` | Top-level integration, reset sequencing, clock-domain crossings, camera, Ethernet, UART, and OLED wiring |
| `verilog/ov7670_capture.v` | Reconstructs two-byte RGB565 pixels and converts them to 8-bit grayscale |
| `verilog/sobel_stream.v` | Streaming 3×3 Sobel accelerator using two line buffers |
| `verilog/sobel_roi_locator.v` | Scores plate-shaped Sobel windows and publishes the best ROI |
| `verilog/frame_buffer.v` | Dual-clock 640×480 frame buffer using 4 bits per pixel |
| `verilog/eth_video_udp.v` | Ethernet/IPv4/UDP packetization and ROI metadata transport |
| `verilog/eth_rmii_tx.v` | 100 Mbps RMII Ethernet transmitter |
| `verilog/eth_crc32.v` | Ethernet CRC generation |
| `verilog/eth_phy_link.v` | LAN8720A PHY initialization/configuration |
| `verilog/ov7670_config.v` | Camera configuration controller |
| `verilog/ov7670_config_rom.v` | OV7670 register configuration data |
| `verilog/ov7670_sccb.v` | SCCB camera register writer |
| `verilog/clocks.v` | 100 MHz → 25 MHz and 50 MHz clock generation |
| `verilog/uart_rx.v` | 115200-baud UART receiver |
| `verilog/plate_uart.v` | Sanitizes received plate text for the OLED |
| `verilog/i2c_write.v` | I²C byte writer |
| `verilog/ssd1306.v` | SSD1306 OLED controller |
| `verilog/nexys4ddr.xdc` | Nexys4 DDR pin constraints |
| `host_pc/alpr_ov7670.py` | UDP receiver, frame reconstruction, ROI handling, FastALPR, and UART return path |

---

## Camera Capture and Grayscale Conversion

The OV7670 is configured for **RGB565**. Each pixel arrives over the 8-bit camera bus in two PCLK cycles.

`ov7670_capture.v` stores the first byte, combines it with the second byte, expands the RGB565 components, and computes a hardware-friendly grayscale approximation:

```text
gray = (R + 2G + B) / 4
```

This approximation gives green a larger luminance contribution while avoiding general multipliers. The division by four is implemented by selecting the upper bits of the sum.

A one-cycle `wr_en` pulse marks every valid completed grayscale pixel and acts as the valid signal for the downstream streaming datapath.

---

## Streaming Sobel Accelerator

`sobel_stream.v` processes the grayscale camera stream without storing a full image for convolution.

A 3×3 Sobel window requires the current row plus the previous two rows:

```text
top_l   top_c   top_r
mid_l   mid_c   mid_r
bot_l   bot_c   bot_r
```

The implementation uses:

- two 640-pixel line buffers for vertical history
- six shift registers for horizontal history
- one new grayscale input as the bottom-right sample

This allows the accelerator to accept **one valid pixel per input clock after warm-up**.

The Sobel kernels are:

```text
Gx =  -1  0  1        Gy =  -1 -2 -1
      -2  0  2              0  0  0
      -1  0  1              1  2  1
```

Instead of the exact Euclidean magnitude,

```text
sqrt(Gx² + Gy²)
```

the hardware uses:

```text
|Gx| + |Gy|
```

This avoids multipliers and square-root logic while still providing a useful edge-strength estimate. The result is saturated to 8 bits and thresholded at **80**.

With `BINARY_OUT = 1`:

- edge pixel → `0xFF`
- non-edge pixel → `0x00`

The first two rows and first two columns are forced to zero because a complete causal 3×3 neighborhood is not yet available.

---

## Sobel-Based ROI Proposal

The Sobel edge stream feeds `sobel_roi_locator.v`.

Instead of buffering the entire edge image or running connected-component analysis, the image is divided into **32×32 tiles**.

For a 640-pixel-wide image:

```text
640 / 32 = 20 tile columns
```

The ROI engine counts edge pixels in each tile and evaluates every **4×2 tile window**:

```text
4 × 32 = 128 pixels wide
2 × 32 =  64 pixels high
```

Each candidate ROI is therefore **128×64**, giving it an intentionally license-plate-like aspect ratio.

The score is the total number of binary Sobel edge pixels inside the eight tiles. The highest-scoring candidate is retained and accepted only when:

```text
score >= 350
```

At the next frame boundary, the winning coordinates are published as:

```text
x0, y0, x1, y1, score
```

The ROI therefore corresponds to the frame that just completed and remains stable while the next frame is processed.

---

## Frame-Buffer Memory Optimization

The complete 640×480 grayscale frame is stored for Ethernet transmission.

An 8-bit grayscale frame would require:

```text
640 × 480 × 8 = 2,457,600 bits
```

The current implementation stores only the upper four grayscale bits:

```text
640 × 480 × 4 = 1,228,800 bits
```

This approximately halves the full-frame storage requirement.

On readback, the nibble is replicated:

```text
abcd → abcdabcd
```

For example:

```text
0xA → 0xAA
```

The UDP interface therefore remains 8 bits per pixel while the FPGA stores only four bits per pixel internally.

The full frame buffer is explicitly mapped toward **block RAM**. The much smaller Sobel line buffers are directed toward **distributed RAM**.

### Single-Buffer Tradeoff

The current design uses one frame buffer. The camera can therefore overwrite locations while Ethernet is reading them.

A future improvement would be a ping-pong/double-buffer architecture:

```text
camera writes Buffer A
Ethernet reads Buffer B
        ↓
      swap
```

This would improve frame consistency at the cost of substantially more memory.

---

## Clocking and Clock-Domain Crossings

The Nexys4 DDR provides a **100 MHz** board clock.

`clocks.v` uses an MMCM to generate:

- **25 MHz** for camera control, framebuffer reads, UART, OLED, and related logic
- **50 MHz** for 100 Mbps RMII Ethernet

The OV7670 supplies its own pixel clock, which is buffered through `IBUF` and `BUFG` and used for camera capture and streaming image processing.

The design therefore contains multiple clock domains:

```text
100 MHz board clock
25 MHz control / framebuffer domain
50 MHz Ethernet domain
camera PCLK domain
```

Two-stage synchronizers and toggle-based event synchronization are used where control signals cross between unrelated domains.

For example, the packetizer flips `fill_tog` when a 192-pixel packet is ready. The Ethernet domain synchronizes that toggle and detects a change with XOR, avoiding the risk of missing a narrow single-cycle pulse.

---

## Ethernet Video Transport

Video is transmitted through the Nexys4 DDR's LAN8720A PHY using **100 Mbps RMII**.

Each grayscale frame contains:

```text
640 × 480 = 307,200 pixels
```

The packetizer sends **192 pixels per UDP packet**:

```text
307,200 / 192 = 1,600 packets per frame
```

The framebuffer-side packetizer follows this state sequence:

```text
IDLE → PRIME → FILL → KICK → WAIT
```

`PRIME` accounts for the one-cycle synchronous block-RAM read latency before the 192-byte payload is staged.

Each UDP payload begins with the project-specific `N4VD` header containing:

- magic value `N4VD`
- frame ID
- packet index
- packet count
- image width and height
- format version
- ROI coordinates
- ROI edge score

The remaining payload contains 192 grayscale pixels.

### RMII Output Timing

The Ethernet reference clock is generated with an ODDR. The current implementation intentionally shifts the PHY sampling relationship relative to TXD/TXEN updates to provide sufficient I/O timing margin. This change was introduced after earlier 100 Mbps transmission attempts exposed a hold-timing problem.

---

## PC-Side License Plate Recognition

Run:

```bash
python3 host_pc/alpr_ov7670.py --bind 0.0.0.0
```

The Python program:

1. listens for UDP packets on port **5000**
2. validates the `N4VD` header
3. reconstructs the linear 640×480 grayscale frame
4. reads FPGA-generated ROI metadata
5. adds configurable padding around the proposed ROI
6. runs FastALPR on the ROI
7. retries the full frame if the ROI produces no plate
8. displays the result with OpenCV
9. sends new recognized plate text back to the FPGA over USB-UART

The fallback path prevents an incorrect Sobel ROI from completely suppressing recognition.

Useful options:

```bash
# Ignore FPGA ROI metadata and always use the full frame
python3 host_pc/alpr_ov7670.py --no-roi

# Change ROI padding
python3 host_pc/alpr_ov7670.py --roi-padding 0.30

# Select another FastALPR OCR model
python3 host_pc/alpr_ov7670.py --ocr-model cct-xs-v2-global-model
```

The default UDP configuration in the RTL is:

```text
FPGA IP: 192.168.1.10
PC IP:   192.168.1.2
UDP port: 5000
```

The destination MAC address in `eth_video_udp.v` must match the receiving network adapter.

---

## UART and OLED Feedback

When FastALPR recognizes a plate, the PC sends a newline-terminated ASCII string back to the Nexys4 DDR at:

```text
115200 baud, 8N1
```

`plate_uart.v`:

- converts lowercase letters to uppercase
- accepts A–Z, 0–9, spaces, and hyphens
- ignores unsupported characters
- stores up to 16 characters
- pulses the OLED update signal when CR/LF terminates the plate string

The SSD1306 OLED uses I²C address **0x3C** and displays `PLATE WAITING` until the first valid result arrives.

---

## Hardware

- Digilent Nexys4 DDR
- Xilinx Artix-7 XC7A100T-CSG324
- OV7670 camera module
- LAN8720A Ethernet PHY integrated on the Nexys4 DDR
- SSD1306 I²C OLED
- USB connection for JTAG programming and UART

Vivado device:

```text
xc7a100tcsg324-1
```

### OV7670 Wiring

| OV7670 | Pmod | FPGA pin |
| --- | --- | --- |
| D0–D3 | JA1–JA4 | C17, D18, E18, G17 |
| D4–D7 | JA7–JA10 | D17, E17, F18, G18 |
| SIOC | JB1 | D14 |
| SIOD | JB2 | F16 |
| VSYNC | JB3 | G16 |
| HREF | JB4 | H14 |
| XCLK | JB7 | E16 |
| RESET | JB8 | F13 |
| PWDN | JB9 | G13 |
| PCLK | JB10 | H16 |
| 3.3 V | JA6 | — |
| GND | JA5 | — |

### SSD1306 OLED

| SSD1306 | Pmod JD | FPGA pin |
| --- | --- | --- |
| SCL | JD1 | H4 |
| SDA | JD2 | H1 |
| GND | JD5 or JD11 | — |
| 3.3 V | JD6 or JD12 | — |

The OLED is **3.3 V only**.

---

## Building the FPGA Design

1. Create or open a Vivado project targeting:

   ```text
   xc7a100tcsg324-1
   ```

2. Add the Verilog sources from `verilog/`.

3. Add `verilog/nexys4ddr.xdc` as the constraints file.

4. Set:

   ```text
   ov7670_nexys4ddr_top
   ```

   as the top module.

5. Run synthesis, implementation, timing analysis, and bitstream generation.

6. Program the Nexys4 DDR through JTAG.

---

## Host Setup

The host script uses Python 3.10+.

Install the packages required by `host_pc/alpr_ov7670.py`:

```bash
python3 -m pip install numpy opencv-python pyserial fast-alpr
```

Then configure the PC Ethernet adapter for the expected static address and run:

```bash
python3 host_pc/alpr_ov7670.py --bind 0.0.0.0
```

If the OLED does not update, verify that the correct Digilent/FTDI UART interface is selected and that the PC can send a newline-terminated plate string at 115200 baud.

---

## Key Design Decisions

| Design problem | Implementation choice |
| --- | --- |
| RGB565 to grayscale | `(R + 2G + B) / 4` to avoid general multipliers |
| 3×3 streaming convolution | Two line buffers plus horizontal shift registers |
| Sobel magnitude | `|Gx| + |Gy|` instead of square root |
| Edge map | Binary threshold at 80 |
| Plate proposal | Highest-edge-density 128×64 tiled window |
| Frame memory | 4-bit grayscale storage to reduce BRAM use |
| Video transport | 192 pixels per UDP packet over 100 Mbps RMII |
| CDC event transfer | Multi-stage synchronizers and toggle handshakes |
| ML workload | FastALPR on the host PC |
| Recognition robustness | ROI first, automatic full-frame fallback |
| User output | UART result returned to SSD1306 OLED |

---

## Future Improvements

Potential next steps include:

- ping-pong frame buffering to eliminate read/write tearing
- deeper timing and resource optimization of the Sobel datapath
- more sophisticated ROI scoring using aspect ratio and spatial edge distribution
- hardware-assisted packet buffering
- additional FPGA-side preprocessing before recognition
- automated Vivado build and regression scripts
- expanded self-checking simulations for the complete video pipeline

---

## Project Goal

The project demonstrates an end-to-end FPGA/host vision system in which the FPGA does more than simply capture video. It performs real-time image preprocessing, memory-conscious streaming computation, ROI generation, clock-domain crossing, and network transport before handing higher-level recognition to the host.
