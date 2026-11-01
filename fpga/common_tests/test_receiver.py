import importlib.util
from pathlib import Path
root=Path(__file__).resolve().parent.parent
sp=importlib.util.spec_from_file_location('receiver',root/'scripts/receive.py')
m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m)
# The reference frame is the REAL wire capture from tests/tb_adc_display.sv,
# regenerated whenever that testbench or vofa_base_display.v changes.  It is a
# proto 6 line from an FFT-finder build: the sweep fields (scan/slock/samp/
# spass) stay 0 because the shared FFT won the arbitration, and the finder
# fields carry the values the testbench drives (see tb_adc_display.sv).
line=(root/'tests/chain_uart.bin').read_text().splitlines()[0]
assert m.decode(line)['usable'] is False
row=m.decode(line)
# ---- proto 6: the 4096-point shared-FFT finder telemetry -------------------
assert row['protocol'] == 6
assert row['finder_state'] == 4                    # ffs: 4 = TRACK
assert row['finder_bin0'] == 126 and row['finder_bin1'] == 126
assert row['finder_freq0_hz'] == 1999700 and row['finder_freq1_hz'] == 2000300
assert row['finder_mag0'] == 1280000 and row['finder_mag1'] == 1275000
assert row['finder_frames'] == 7 and row['finder_lost'] == 0
valid=line.replace(' mv=0',' mv=1').replace(' rv=0',' rv=1').replace(' qual=00',' qual=03').replace(' flags=128',' flags=000')
# proto 6 + ffs=4: the finder has parked, so a reading is usable even though
# slock stays 0 for the whole run - the sweep is not the finder any more.
assert m.decode(valid)['usable'] is True
# ... and while the finder is still acquiring (ffs 1..3) or has just lost the
# carrier (ffs 5), the very same flags must never produce a reading.
for st in ('1','2','3','5'):
    assert m.decode(valid.replace(' ffs=4',' ffs=%s'%st))['usable'] is False, st
# The sweep build reports the same proto with ffs=0 and live sweep fields:
# slock=1 must still be the "parked" condition there, and ffs=0 alone is not.
sweep=valid.replace(' ffs=4',' ffs=0')
assert m.decode(sweep)['usable'] is False
assert m.decode(sweep.replace(' slock=0',' slock=1'))['usable'] is True
assert m.decode(sweep.replace(' slock=1',' slock=0'))['usable'] is False
# proto 4: a frame taken while a gear change is still settling is not a reading
# either, even though every other flag looks healthy.
row=m.decode(valid.replace(' bw=0',' bw=5'))
assert row['usable'] is False and row['bw_gear'] == 1 and row['bw_settling'] is True
assert m.decode(valid)['bw_gear'] == 0 and m.decode(valid)['bw_settling'] is False
for before,after in [('qual=03','qual=01'),('ar=1','ar=0'),('fv=3','fv=0'),('clip=00','clip=01'),('mv=1','mv=0')]:
    row=m.decode(valid.replace(before,after))
    assert not row['usable'] and row['phase_diff_deg'] is None and row['ratio'] is None
# proto 5: two different generators must never yield a ratio or a phase
# difference (fm bit8 = channel frequency mismatch).  proto 6 carries the
# same field, so it needs the same rule.
row=m.decode(valid)
assert row['usable'] and row['ch_same_source'] is True
row=m.decode(valid.replace(' fm=000',' fm=257'))
assert row['usable'] is False and row['ch_same_source'] is False
assert row['ratio'] is None and row['phase_diff_deg'] is None
# ... but the per-channel amplitudes stay available for a level reading.
assert row['amp0_peak_counts'] is None
# display telemetry is decoded, including the auto/manual distinction.
# tb packs {sat, auto, m}: 0b110000 = auto with saturation, 0b100000 = manual
# with saturation and m=0 (a 16x-decimated field would be 0b110100 = 52).
row=m.decode(valid.replace(' tb=16',' tb=48'))
assert row['tb_saturated'] is True and row['tb_auto'] is True and row['display_decim_log2'] == 0
assert m.decode(valid)['display_decim_log2'] == 0 and m.decode(valid)['tb_auto'] is True
try:
    m.decode(valid.replace('proto=6','proto=1'))
except ValueError:
    pass
else:
    raise AssertionError('Old protocol accepted')
# ---- proto 7 / 8: equivalent-time display telemetry -----------------------
# TWO field sets carry the ET tail, and the protocol number distinguishes them
# because the ladder must stay cumulative (proto>=7 implies etm= exists;
# proto>=6 implies ffs= exists).  Both are built here from the captured line so
# the field ORDER is the one the RTL really emits: 61 (dn), 71 (proto), 72..76.
ET_TAIL=' etm=3 etb=0064 etc=0064 etw=0000000300 eth=00000019565'
_pre,_rest=valid.split(' ffs=',1)
_fft_span=' ffs='+_rest.split(' proto=')[0]
# proto 7 = ET build WITHOUT the finder, so the only "parked" evidence left is
# slock (proto 3+).  The captured line has slock=0 because the FFT won the
# arbitration there; an ET-only build has no FFT to win, so its sweep does the
# searching and slock is the flag that settles it.
et7=_pre.replace(' slock=0',' slock=1')+' proto=7'+ET_TAIL
et8=valid.replace('proto=6','proto=8')+ET_TAIL   # ET + finder
row=m.decode(et7)
assert row['protocol'] == 7
assert row['finder_state'] is None   # proto 7 must NOT claim the FFT fields
assert row['et_mode'] is True and row['et_locked'] is True and row['et_phase_ref_ok'] is True
assert row['et_bins'] == 64 and row['et_filled_bins'] == 64
assert row['et_period_wraps'] == 300 and row['et_averaged_samples'] == 19565
assert m.decode(et8)['protocol'] == 8
assert m.decode(et8)['finder_state'] == 4
# ET is a DISPLAY mode: it must not invalidate a healthy measurement, and the
# row count reported in dn is the ET bin count (dn is reused, not added).
assert row['usable'] is True
assert m.decode(et7.replace(' dn=1024',' dn=0064'))['wave_rows'] == 64
# etm bit1 clear = display on but the AFC is not locked: the phase bins are
# then not a valid time base even though every measurement flag is healthy.
row=m.decode(et7.replace('etm=3','etm=1'))
assert row['et_mode'] is True and row['et_locked'] is False and row['et_phase_ref_ok'] is False
assert row['usable'] is True              # the lock-in reading itself is unaffected
# etm bit0 clear = ET display off (the wave: rows are real-time again).
row=m.decode(et7.replace('etm=3','etm=0'))
assert row['et_mode'] is False and row['et_phase_ref_ok'] is False
# proto 7/8 are cumulative: the proto-3/4/5/6 gates must still apply to them.
assert m.decode(et8.replace(' ffs=4',' ffs=2'))['usable'] is False
assert m.decode(et7.replace(' bw=0',' bw=5'))['bw_settling'] is True
assert m.decode(et7.replace(' fm=000',' fm=257'))['ch_same_source'] is False
for bad in ('proto=11','proto=1'):
    try:
        m.decode(et7.replace('proto=7',bad))
    except ValueError:
        pass
    else:
        raise AssertionError('%s accepted'%bad)
assert m.decode('wave:+0001,-0002') is None
print('PASS receiver qualification, single-channel failure, tracking, clipping, stale result,')
print('     FFT-finder lock/loss gating (proto 6), two-generator rejection (proto 5),')
print('     equivalent-time coverage/unlocked-reference handling (proto 7 and 8)')
print('     and protocol rejection')
