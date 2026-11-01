# ============================================================================
# 征战Core (XC7A35TFGG484-2) Board I/O Constraints
# ADC mapping: U15B 61..100, Bank 35 (user confirmed NC end P1/R1, 2026-10-17).
# Keys/LEDs follow Core V1 schematic page 2; KEY2 (SW1/H5) cycles timebase.
# ============================================================================

# ---- Board config ----
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullnone [current_design]

# ---- 50 MHz System Clock ----
# From schematic page 2: need to verify exact pin
# Common 50MHz oscillator on Artix-7 boards
set_property PACKAGE_PIN Y18 [get_ports clk_50m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]

# ---- Reset ----
set_property PACKAGE_PIN B21 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

# ---- LEDs ----
set_property PACKAGE_PIN AA8 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN AB8 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN Y8 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN R14 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

# ---- Push Buttons ----
set_property PACKAGE_PIN G17 [get_ports {key_n[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[0]}]
set_property PACKAGE_PIN G18 [get_ports {key_n[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[1]}]
set_property PACKAGE_PIN H5 [get_ports {key_n[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[2]}]
set_property PACKAGE_PIN L13 [get_ports {key_n[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_n[3]}]

# ---- Slide Switches ----
set_property PACKAGE_PIN G22 [get_ports {sw[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[0]}]
set_property PACKAGE_PIN D22 [get_ports {sw[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[1]}]
set_property PACKAGE_PIN E22 [get_ports {sw[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sw[2]}]

# Direct board USB UART: FPGA AB6 -> CH340E RXD, per retained board schematic.
# AA6 is CH340E TXD and is intentionally NOT driven/used here. M15 stays unused.
set_property PACKAGE_PIN AB6 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
set_property DRIVE 8 [get_ports uart_tx]
set_property SLEW SLOW [get_ports uart_tx]

# ---- HDMI pins are in hdmi_tx.xdc ----

# User-confirmed NC end P1/R1; module occupies U15B pins 61..100.
# Module component view: left=even, right=odd. GND=100, 5V=99.
# Power and NC have no FPGA ports.
# module row 3 left CLK_A -> U15B.66
set_property PACKAGE_PIN E3 [get_ports {adc_clk}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_clk}]
# module row 3 right PWDN_A -> U15B.65
set_property PACKAGE_PIN F3 [get_ports {adc_pwdn}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_pwdn}]
# module row 4 left OEB_A -> U15B.68
set_property PACKAGE_PIN D2 [get_ports {adc_oeb}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_oeb}]
# module row 4 right OTR_A -> U15B.67
set_property PACKAGE_PIN E2 [get_ports {adc_otr}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_otr}]
# module row 5 left A_D11 -> U15B.70
set_property PACKAGE_PIN G4 [get_ports {adc_d0[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[11]}]
# module row 5 right A_D10 -> U15B.69
set_property PACKAGE_PIN H4 [get_ports {adc_d0[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[10]}]
# module row 6 left A_D9 -> U15B.72
set_property PACKAGE_PIN G3 [get_ports {adc_d0[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[9]}]
# module row 6 right A_D8 -> U15B.71
set_property PACKAGE_PIN H3 [get_ports {adc_d0[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[8]}]
# module row 7 left A_D7 -> U15B.74
set_property PACKAGE_PIN J4 [get_ports {adc_d0[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[7]}]
# module row 7 right A_D6 -> U15B.73
set_property PACKAGE_PIN K4 [get_ports {adc_d0[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[6]}]
# module row 8 left A_D5 -> U15B.76
set_property PACKAGE_PIN J2 [get_ports {adc_d0[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[5]}]
# module row 8 right A_D4 -> U15B.75
set_property PACKAGE_PIN K2 [get_ports {adc_d0[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[4]}]
# module row 9 left A_D3 -> U15B.78
set_property PACKAGE_PIN K3 [get_ports {adc_d0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[3]}]
# module row 9 right A_D2 -> U15B.77
set_property PACKAGE_PIN L3 [get_ports {adc_d0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[2]}]
# module row 10 left A_D1 -> U15B.80
set_property PACKAGE_PIN M2 [get_ports {adc_d0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[1]}]
# module row 10 right A_D0 -> U15B.79
set_property PACKAGE_PIN M3 [get_ports {adc_d0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d0[0]}]
# module row 11 left OTR_B -> U15B.82
set_property PACKAGE_PIN A1 [get_ports {adc_otr_b}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_otr_b}]
# module row 11 right B_D11 -> U15B.81
set_property PACKAGE_PIN B1 [get_ports {adc_d1[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[11]}]
# module row 12 left B_D10 -> U15B.84
set_property PACKAGE_PIN B2 [get_ports {adc_d1[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[10]}]
# module row 12 right B_D9 -> U15B.83
set_property PACKAGE_PIN C2 [get_ports {adc_d1[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[9]}]
# module row 13 left B_D8 -> U15B.86
set_property PACKAGE_PIN D1 [get_ports {adc_d1[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[8]}]
# module row 13 right B_D7 -> U15B.85
set_property PACKAGE_PIN E1 [get_ports {adc_d1[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[7]}]
# module row 14 left B_D6 -> U15B.88
set_property PACKAGE_PIN F1 [get_ports {adc_d1[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[6]}]
# module row 14 right B_D5 -> U15B.87
set_property PACKAGE_PIN G1 [get_ports {adc_d1[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[5]}]
# module row 15 left B_D4 -> U15B.90
set_property PACKAGE_PIN G2 [get_ports {adc_d1[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[4]}]
# module row 15 right B_D3 -> U15B.89
set_property PACKAGE_PIN H2 [get_ports {adc_d1[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[3]}]
# module row 16 left B_D2 -> U15B.92
set_property PACKAGE_PIN J1 [get_ports {adc_d1[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[2]}]
# module row 16 right B_D1 -> U15B.91
set_property PACKAGE_PIN K1 [get_ports {adc_d1[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[1]}]
# module row 17 left B_D0 -> U15B.94
set_property PACKAGE_PIN L1 [get_ports {adc_d1[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_d1[0]}]
# module row 17 right OEB_B -> U15B.93
set_property PACKAGE_PIN M1 [get_ports {adc_oeb_b}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_oeb_b}]
# module row 18 left PWDN_B -> U15B.96
set_property PACKAGE_PIN N2 [get_ports {adc_pwdn_b}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_pwdn_b}]
# module row 18 right CLK_B -> U15B.95
set_property PACKAGE_PIN P2 [get_ports {adc_clk_b}]
set_property IOSTANDARD LVCMOS33 [get_ports {adc_clk_b}]

# SPI is archived under the MCU project; its former pins are not top ports.
set_property PULLUP true [get_ports {rst_n key_n[*]}]
set_property PULLDOWN true [get_ports {sw[*]}]
