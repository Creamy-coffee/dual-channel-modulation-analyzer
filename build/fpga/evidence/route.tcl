set root {D:/FPGA_bag/Project/signal_analyzer_base_v1/firmware/experimental/harmonic_rounding_20261029}
set tag a222905e5799
set build $root/build/unified_$tag
set reports {D:/FPGA_bag/Project/signal_analyzer_base_v1/firmware/experimental/harmonic_rounding_20261029/reports/a222905e5799}
set_param general.maxThreads 1
open_project $build/signal_analyzer_unified.xpr
launch_runs impl_1 -to_step route_design -jobs 1
wait_on_run impl_1
if {[string match -nocase *error* [get_property STATUS [get_runs impl_1]]]} {error "Implementation failed"}
set routed $build/signal_analyzer_unified.runs/impl_1/daq_top_routed.dcp
if {![file exists $routed]} {error "Routed checkpoint missing"}
# Generic properties cannot be queried after opening a checkpoint.
set actual_cfg [get_property generic [get_filesets sources_1]]
open_checkpoint $routed
phys_opt_design -directive ExploreWithAggressiveHoldFix
report_timing_summary -delay_type min_max -report_unconstrained -file $reports/timing.rpt
report_utilization -hierarchical -file $reports/utilization.rpt
report_drc -file $reports/drc.rpt
set wns [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set whs [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
set errors [llength [get_drc_violations -quiet -filter {SEVERITY == Error}]]
set bb [llength [get_cells -hier -quiet -filter {IS_BLACKBOX == 1}]]
set cnn [llength [get_cells -hier -quiet -filter {NAME =~ *cnn*}]]
set fft [llength [get_cells -hier -quiet -filter {REF_NAME =~ fft_frame_4ch*}]]
set cic [llength [get_cells -hier -quiet -filter {REF_NAME =~ cic3_decimate_2ch*}]]
set banks [llength [get_cells -hier -quiet -filter {REF_NAME =~ fft_capture_bank*}]]
set fine [llength [get_cells -hier -quiet -filter {REF_NAME =~ afc_precision_ctrl*}]]
set et [llength [get_cells -hier -quiet -filter {REF_NAME =~ et_display*}]]
set binary [llength [get_cells -hier -quiet -filter {REF_NAME =~ v32cap3_tx*}]]
set ddcn [llength [get_cells -hier -quiet -filter {REF_NAME =~ ddc_nco_mixer*}]]
set iqst [llength [get_cells -hier -quiet -filter {REF_NAME =~ analysis_stream_tx*}]]
set command_rx [llength [get_cells -hier -quiet -filter {REF_NAME =~ ab5_uart_commands*}]]
set f [open $reports/build_audit.txt w]
puts $f "SOURCE_ID=$tag\nGENERIC=$actual_cfg\nWNS=$wns\nWHS=$whs\nDRC_ERRORS=$errors\nBLACKBOX=$bb\nCNN=$cnn\nFFT_ENGINES=$fft\nCIC_STAGES=$cic\nCAPTURE_BANKS=$banks\nFINE=$fine\nET=$et\nV32CAP3=$binary\nDDC_NCO=$ddcn\nANALYSIS_STREAM=$iqst"
puts $f "COMMAND_RX=$command_rx"
foreach p {uart_tx v32_tx v32_rx adc_clk adc_clk_b} {puts $f "$p=[get_property PACKAGE_PIN [get_ports $p]]"}
close $f
write_checkpoint -force $reports/routed.dcp
if {$wns<0 || $whs<0 || $errors!=0 || $bb!=0 || $cnn!=0 || $fft!=1 || $cic!=4 || $banks!=5 || $fine!=1 || $et!=0 || $binary!=1 || $ddcn!=1 || $iqst!=1 || $command_rx!=1} {
 error "Candidate failed physical/source audit; no bitstream released"
}
foreach {p pin} {uart_tx AB6 v32_tx AB5 v32_rx AA5 adc_clk E3 adc_clk_b P2} {
 if {[get_property PACKAGE_PIN [get_ports $p]] ne $pin} {error "Pin mismatch $p"}
}
puts "UNIFIED_ROUTE_READY $tag"

set polar [llength [get_cells -hier -quiet -filter {REF_NAME =~ analysis_polar_stream*}]]
set arb [llength [get_cells -hier -quiet -filter {REF_NAME =~ parallel_frame_arbiter*}]]
if {$polar!=1 || $arb!=1} {error "Missing parallel modulation hardware"}
puts "PARALLEL_CANDIDATE_ROUTE_READY"
