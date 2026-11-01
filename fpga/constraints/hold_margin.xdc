# ============================================================================
#  Hold-margin budget
# ----------------------------------------------------------------------------
#  Problem this file fixes
#    Implementation closes with WNS ~ 1.41 ns but WHS ~ 0.06 ns. The worst
#    hold path is inside the lock-in CIC integrator chain
#    (lockin_core_2ch: s2_reg -> s3_reg, 58-bit recursive adders): data path
#    0.434 ns against 0.272 ns of clock skew, i.e. the slack is decided by
#    skew, not by logic depth.
#
#  Why directives alone did not fix it
#    phys_opt_design -hold_fix / ExploreWithAggressiveHoldFix only insert LUT1
#    buffers on paths that actually VIOLATE hold. With WHS already positive
#    (0.057 ns) the router had nothing flagged, so applying
#    ExploreWithAggressiveHoldFix (post-place and post-route) moved WHS by
#    only +0.006 ns -- measured, see reports/build_audit_merged_ref1m.txt.
#
#  What this file does instead
#    Adds an explicit hold uncertainty budget, so the hold requirement becomes
#    "+0.15 ns of margin" instead of "just barely positive". The paths then
#    appear as violations and the hold-fix machinery engages. The reported WHS
#    after this file is the slack ABOVE the 0.150 ns budget we added -- it is
#    NOT a "real" hold margin. The delivered hold margin is the constraint-
#    reported WHS (e.g. +0.030 ns for the ET bit). Do not add the budget back
#    and call it a measured/real margin: that is only the requirement with the
#    budget removed, not an independent timing result.
#
#  Clock object: resolved from the BUFG output pin, exactly like
#  adc_forwarded_clock.xdc does, so it does not depend on the net name.
# ============================================================================

set sys_clk [get_clocks -of_objects [get_pins clocks/u_sys/O]]
set_clock_uncertainty -hold 0.150 $sys_clk
