## Nexys4 DDR Rev. C — OV7670 grayscale camera over Ethernet
## Part: xc7a100tcsg324-1
##
## Purpose: map top-level ports to board/Pmod pins and declare clock-domain
## timing assumptions. Internal PULLUP properties support the open-drain SCCB,
## I2C and MDIO buses used by this build; external pull-ups are still preferable
## for robust hardware because FPGA pull-ups are weak and device dependent.
##
## Board source: Digilent Nexys4 DDR Reference Manual / master XDC
## https://digilent.com/reference/_media/nexys4-ddr/nexys4ddr_rm.pdf
## Ethernet source: Microchip LAN8720A datasheet
## https://ww1.microchip.com/downloads/en/devicedoc/8720a.pdf

set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports CLK100MHZ]
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 5.000} [get_ports CLK100MHZ]

set_property -dict {PACKAGE_PIN C12 IOSTANDARD LVCMOS33} [get_ports CPU_RESETN]

## SW0: 0 streams FPGA grayscale, 1 streams FPGA Sobel edge output.
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports edge_enable]

## Camera on Pmod JA (data) + JB (clocks / SCCB). PCLK must be JB10 / H16.
set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS33} [get_ports {cam_d[0]}]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports {cam_d[1]}]
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33} [get_ports {cam_d[2]}]
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports {cam_d[3]}]
set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33} [get_ports {cam_d[4]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports {cam_d[5]}]
set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33} [get_ports {cam_d[6]}]
set_property -dict {PACKAGE_PIN G18 IOSTANDARD LVCMOS33} [get_ports {cam_d[7]}]
set_property -dict {PACKAGE_PIN D14 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports cam_sioc]
set_property -dict {PACKAGE_PIN F16 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports cam_siod]
set_property -dict {PACKAGE_PIN G16 IOSTANDARD LVCMOS33} [get_ports cam_vsync]
set_property -dict {PACKAGE_PIN H14 IOSTANDARD LVCMOS33} [get_ports cam_href]
set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports cam_xclk]
set_property -dict {PACKAGE_PIN F13 IOSTANDARD LVCMOS33} [get_ports cam_reset]
set_property -dict {PACKAGE_PIN G13 IOSTANDARD LVCMOS33} [get_ports cam_pwdn]
set_property -dict {PACKAGE_PIN H16 IOSTANDARD LVCMOS33} [get_ports cam_pclk]

create_clock -period 40.000 -name cam_pclk -waveform {0.000 20.000} [get_ports cam_pclk]
## Camera PCLK is unrelated in phase/frequency to the MMCM clocks. CDC paths
## need explicit synchronization in HDL and are not timed synchronously here.
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks sys_clk_pin] \
    -group [get_clocks cam_pclk]

## SSD1306 I2C OLED on Pmod JD (Nexys4 DDR RM Table 5).
## JD1=H4 SCL, JD2=H1 SDA, JD5/JD11=GND, JD6/JD12=3.3 V.
set_property -dict {PACKAGE_PIN H4 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports oled_scl]
set_property -dict {PACKAGE_PIN H1 IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports oled_sda]

## USB-UART: PC TX -> FPGA RX (Digilent UART_TXD_IN).
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports uart_rxd]
set_false_path -from [get_ports uart_rxd]

## LAN8720A RMII (on-board Ethernet). FPGA drives 50 MHz REFCLK.
## MDIO A9 / MDC C9 — required to advertise 100 Mbps (PHY address 1).
set_property -dict {PACKAGE_PIN D5  IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports eth_refclk]
set_property -dict {PACKAGE_PIN B3  IOSTANDARD LVCMOS33} [get_ports eth_rstn]
set_property -dict {PACKAGE_PIN C9  IOSTANDARD LVCMOS33} [get_ports eth_mdc]
set_property -dict {PACKAGE_PIN A9  IOSTANDARD LVCMOS33 PULLUP TRUE} [get_ports eth_mdio]
set_property -dict {PACKAGE_PIN B9  IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports eth_txen]
set_property -dict {PACKAGE_PIN A10 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {eth_txd[0]}]
set_property -dict {PACKAGE_PIN A8  IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {eth_txd[1]}]

## Reset and UART are asynchronous external controls/data. Their first internal
## stages are synchronizers or reset logic, so ordinary setup timing is waived.
set_false_path -from [get_ports CPU_RESETN]
set_false_path -from [get_ports edge_enable]
set_false_path -to   [get_ports eth_rstn]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
