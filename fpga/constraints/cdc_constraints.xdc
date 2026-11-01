# CDC constraints, rewritten as pure XDC commands (Vivado synthesis's XDC
# interpreter skips Tcl if/proc, which would silently drop these constraints).
set_false_path -to [get_pins -quiet -hier -regexp {.*(key_meta_reg|switch_meta_reg)\[[0-9]+\]/D}]
# External reset asserts asynchronously; internal deassertion is synchronized.
set_false_path -from [get_ports rst_n]
# UART-only daq_top has no display mailboxes or manual-export synchronizer. The three old
# bundled-data max-delay constraints matched no endpoints (Vivado 12-4739).
# If a display clock/domain is restored, add its mailbox settling constraints
# with verified source/destination registers in that display project's XDC.
# No global asynchronous clock groups and no exceptions on the ADC data bus.

