# Fixed 50 MHz board oscillator; MMCM output clocks are auto-derived by Vivado.
create_clock -name sys_50m -period 20.000 [get_ports clk_50m]
# ADC forwarded clock is declared in adc_forwarded_clock.xdc at the ODDR output.
# Never declare ADC inputs asynchronous or put the ADC data bus on a false path.
