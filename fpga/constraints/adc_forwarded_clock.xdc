# Dual parallel ports: data changes only after ADC RISING clock edges.
# Clock forwarding, FPGA buffers and PCB/ADC external delay are all analyzed.
# Keep inherited max 11.5 ns. Use 2 ns minimum (datasheet), do not assume
# the former unmeasured 4.5 ns PCB minimum.
create_generated_clock -name adc_fwd_clk -source [get_pins adc_clk_fwd/C] -divide_by 1 [get_ports adc_clk]
create_generated_clock -name adc_b_fwd_clk -source [get_pins adc_clk_b_fwd/C] -divide_by 1 [get_ports adc_clk_b]
set_property SLEW FAST [get_ports {adc_clk adc_clk_b}]
set_input_delay -max 11.5 -clock adc_fwd_clk [get_ports {adc_d0[*] adc_otr}]
set_input_delay -min 2.0 -clock adc_fwd_clk [get_ports {adc_d0[*] adc_otr}]
set_input_delay -max 11.5 -clock adc_b_fwd_clk [get_ports {adc_d1[*] adc_otr_b}]
set_input_delay -min 2.0 -clock adc_b_fwd_clk [get_ports {adc_d1[*] adc_otr_b}]

# Physical ADC clock is advanced 60 degrees (MMCM phase 300).
# The first internal rising edge after launch MUST still see the PREVIOUS
# word; the following edge captures the launched word. Both setup and hold
# therefore bound one 15.385 ns data-valid window. This is NOT a CE/2-rate bus.
# Setup-only multiplier selects that window; its stricter default hold edge
# MUST remain. DO NOT add the usual "-hold 1" (that would hide early changes).
# AMD UG903: Multicycle Paths and Clock Phase-Shift, setup-only example.
set adc_capture_clock [get_clocks -of_objects [get_pins clocks/u_sys/O]]
set_multicycle_path 2 -setup -from [get_clocks {adc_fwd_clk adc_b_fwd_clk}] -to $adc_capture_clock
