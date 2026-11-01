# Canonical source set. This stage produces a routed candidate, never a release bit.
set root [file normalize [file dirname [info script]]]
set ::env(UNIFIED_ID) contest_rebuild
set tag $::env(UNIFIED_ID)
set build $root/build/unified_$tag
set reports $root/reports/implementation/$tag
file mkdir $reports
set_param general.maxThreads 1
create_project signal_analyzer_unified $build -part xc7a35tfgg484-2 -force
set_property target_language Verilog [current_project]
foreach f [glob $root/rtl/*.v] {
 if {[file tail $f] ni {afc_controller.v afc_observer.v}} {add_files -norecurse $f}
}
add_files -norecurse [glob $root/rtl/*.mem]
add_files -fileset constrs_1 -norecurse [glob $root/constraints/*.xdc]
foreach f [get_files -filter {FILE_TYPE == XDC}] {
 if {[file tail $f] eq "base_clocks.xdc"} {set_property PROCESSING_ORDER EARLY $f}
 if {[file tail $f] in {adc_forwarded_clock.xdc hold_margin.xdc}} {set_property PROCESSING_ORDER LATE $f}
}
# The adaptive low-carrier observer plus COM9 text exceeds the A35T LUT limit
# (21 LUTs over at placement). Prune the independent debug display, preserving
# the Linux measurement/filter/waveform precision and all binary functions.
set cfg "ADC_FS_HZ=65000000 BINARY_POINTS=1024 BINARY_BAUD=2000000 REF_FREQ_HZ=30000000 AFC_ENABLE=1 SCAN_ENABLE=0 FFT_ENABLE=1 LOW_FFT_ENABLE=1 BW_MODE=2 BW_MAX_GEAR=2 AUTO_TIMEBASE=2 MAX_DISP_M=15 ET_ENABLE=1 ET_AW=6 ET_AVG_SHIFT=4 AA_ORDER=3 DDC_ENABLE=1 COM9_TEXT_ENABLE=0"
set_property generic $cfg [get_filesets sources_1]
set_property top daq_top [get_filesets sources_1]
update_compile_order -fileset sources_1
foreach token [split $cfg] {
 if {[lsearch -exact [split [get_property generic [get_filesets sources_1]]] $token]<0} {error "Generic mismatch $token"}
}
set run [get_runs impl_1]
# The integrated DDC is LUT-limited on A35T. Use area mapping together with
# explicit FIR BRAM and bit-exact CIC rounding; timing remains a release gate.
set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE AreaOptimized_high [get_runs synth_1]
# The 128-byte V32MEAS! frame added 2026-10-20 pushed the fft_find drift
# comparator (interp_freq_packed -> lost_count CE) to WNS -0.168 with the old
# placement.  ExtraTimingOpt is a placement-only strategy change: it does not
# touch RTL, so every simulation proof keyed on the rtl/ hashes stays valid.
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true $run
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE ExploreWithAggressiveHoldFix $run
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraTimingOpt $run
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE Explore $run
launch_runs synth_1 -jobs 1
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
open_run synth_1
report_utilization -hierarchical -file $reports/synth_utilization.rpt
report_utilization -file $reports/synth_resources.rpt
report_timing_summary -delay_type min_max -file $reports/synth_timing.rpt
write_checkpoint -force $reports/synth.dcp
puts "HARMONIC_TOP_SYNTH_READY $tag"

launch_runs impl_1 -to_step route_design -jobs 1
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
open_run impl_1
phys_opt_design -directive ExploreWithAggressiveHoldFix
report_timing_summary -delay_type min_max -report_unconstrained -file $reports/final_timing.rpt
report_utilization -file $reports/final_utilization.rpt
report_drc -file $reports/final_drc.rpt
if {[get_property SLACK [get_timing_paths -delay_type max -max_paths 1]] < 0} {error "Setup violation"}
if {[get_property SLACK [get_timing_paths -delay_type min -max_paths 1]] < 0} {error "Hold violation"}
set_property BITSTREAM.CONFIG.USR_ACCESS 0xa222905e [current_design]
write_bitstream -force $reports/rebuilt_a222905e.bit
