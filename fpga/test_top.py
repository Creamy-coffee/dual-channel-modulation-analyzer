"""Production top integration; real UART wire decoding, synthetic ADC only."""
from pathlib import Path
import hashlib,json,subprocess,shutil
from receiver import Receiver
H=Path(__file__).resolve().parent; B=H/'top_build';B.mkdir(exist_ok=True)
lin=lambda p:'/mnt/'+p.drive[0].lower()+p.as_posix()[2:]
files=[*sorted((H/'rtl').glob('*.v')),*sorted((H/'rtl').glob('*.mem'))]
inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
for p in (H/'rtl').glob('*.mem'):shutil.copy2(p,B/p.name)
commands=[['verilator','--binary','--timing','-DSIMULATION','--top-module','tb_harmonic_top',
 '-j','2','-O3','-Wno-fatal','--Mdir',lin(B/'obj'),lin(H/'tests/tb_harmonic_top.sv'),
 lin(H.parents[1]/'tests/sim_prims.v'),*[lin(p) for p in (H/'rtl').glob('*.v')]],
 [lin(B/'obj/Vtb_harmonic_top')]]
for cmd,name in zip(commands,['compile.log','run.log']):
 with (B/name).open('w') as log:
  proc=subprocess.run(['wsl.exe','-d','FPGA-Build-2404','--cd',lin(B),'--',*cmd],
                      stdout=log,stderr=subprocess.STDOUT,timeout=600)
 assert proc.returncode==0,str(B/name)+'\n'+(B/name).read_text(errors='replace')[-2500:]
assert 'HARMONIC_TOP_PASS' in (B/'run.log').read_text(errors='replace')
data=(B/'harmonic_top.bin').read_bytes();r=Receiver();frames=[]
for i in range(0,len(data),137):frames+=r.feed(data[i:i+137])
assert frames and not r.crc_errors and not r.invalid_frames
f=frames[0]
assert f['channel']==0 and f['reference_fcw']==0x08000000 and f['decimation']==1
assert f['valid_mask']==31 and f['thd_valid'] and not f['frontend_uncalibrated']
assert f['version']==2 and f['thd_computed_by']=='Linux' and f['fpga_thd_q24']==0
assert abs(f['thd_percent']-11.1803398875)<.3,f
# Existing packet parser must still receive intact WAVE/MEAS/ANA traffic.
import sys
sys.path.insert(0,str(H.parent/'screen_adaptation_20261025/linux'))
from mixed_protocol import Parser
p=Parser();events=[]
for i in range(0,len(data),137):events+=p.feed(data[i:i+137])
assert not p.crc_errors and not p.format_errors
counts={k:sum(e['type']==k for e in events) for k in ['analysis','waveform','measurement','spectrum']}
assert counts['analysis']>0 and counts['waveform']>0,counts
assert {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}==inputs
(B/'validation.json').write_text(json.dumps({'pass':True,'inputs':inputs,'harmonics':frames,
 'legacy_counts':counts,'crc_errors':r.crc_errors,'scope':'synthetic ADC, production RTL/UART simulation; no physical THD calibration'},indent=2)+'\n')
print('HARMONIC_TOP_PROTOCOL_PASS',counts,'THD=',f['thd_percent'])
