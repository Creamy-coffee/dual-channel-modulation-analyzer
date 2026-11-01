# Single UART: FPGA AB5 TX -> adapter RXD; adapter TXD -> FPGA AA5 RX.
# Frame-boundary fixes are retained; no second RX arbitration or TX mirror.
set_property PACKAGE_PIN AB5 [get_ports v32_tx]
set_property IOSTANDARD LVCMOS33 [get_ports v32_tx]
set_property DRIVE 8 [get_ports v32_tx]
set_property SLEW SLOW [get_ports v32_tx]
set_property PULLUP true [get_ports v32_tx]
set_property PACKAGE_PIN AA5 [get_ports v32_rx]
set_property IOSTANDARD LVCMOS33 [get_ports v32_rx]
set_property PULLUP true [get_ports v32_rx]
set_false_path -from [get_ports v32_rx] -to [get_cells -hier -filter {NAME =~ *u_ab5_rx/rx_meta_reg}]
