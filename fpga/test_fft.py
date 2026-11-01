"""Linux host: independent ideal DFT and exact integer-power statistics audit."""
import cmath
import csv
import json
import math
from pathlib import Path
import re
import subprocess

home=Path(__file__).resolve().parent
rtl=home/'rtl';out=home/'tests_build';out.mkdir(exist_ok=True)
sources=['fft_frame_4ch.v','fft_tdp_ram.v','fft_twiddle_rom.v','hann_window_2ch.v',
         'harmonic_stream_stats.v','fixed_divider_generic.v','isqrt_u64.v']
subprocess.run(['iverilog','-g2012','-s','tb_harmonic_fft','-o',str(out/'fft.vvp'),str(home/'tests/tb_harmonic_fft.sv'),*[str(rtl/s) for s in sources]],check=True)
all_results=[]
for fundamental_bin in [128.0,128.27]:
    n=4096
    samples=[round(700*math.sin(2*math.pi*fundamental_bin*k/n+.23)+70*math.sin(4*math.pi*fundamental_bin*k/n+.37)+35*math.sin(6*math.pi*fundamental_bin*k/n-.19)) for k in range(n)]
    (home/'tests/stimulus.mem').write_text(''.join(f'{v&4095:03x}\n' for v in samples))
    fcw=round(fundamental_bin*2**32/n)
    p=subprocess.run(['vvp',str(out/'fft.vvp'),f'+REF={fcw}'],cwd=rtl,text=True,capture_output=True,check=True)
    (out/f'fft_{fundamental_bin}.log').write_text(p.stdout+p.stderr)
    m=re.search(r'HARMONIC_FFT_RESULT thd_q24=(\d+) flags=(\d+)',p.stdout);assert m,p.stdout
    powers={int(k):int(v) for k,v in csv.reader((out/'fft_powers.csv').open())}
    centers=[round(fundamental_bin*h) for h in range(1,6)]
    corrected=[]
    for center in centers:
        band=sum(powers[k] for k in range(center-3,center+4))
        noise=sum(powers[k] for k in [*range(center-6,center-3),*range(center+4,center+7)])
        corrected.append(max(0,6*band-7*noise))
    exact=math.isqrt((sum(corrected[1:])<<48)//corrected[0])
    assert int(m[1])==exact,(int(m[1]),exact)
    # Independent DFT starts from the actual ADC quantization and window ROM.
    half=[int(s,16) for s in (rtl/'hann_4096_half_q15.mem').read_text().split()]
    coeff=half+half[::-1]
    windowed=[(x*w)//32768 for x,w in zip(samples,coeff)]
    wanted={k for center in centers for k in range(center-6,center+7)}
    gold={k:abs(sum(x*cmath.exp(-2j*math.pi*k*j/n) for j,x in enumerate(windowed))*16/n)**2 for k in wanted}
    gold_corrected=[max(0,6*sum(gold[k] for k in range(c-3,c+4))-7*sum(gold[k] for k in [*range(c-6,c-3),*range(c+4,c+7)])) for c in centers]
    ideal=math.sqrt(sum(gold_corrected[1:])/gold_corrected[0])
    measured=exact/2**24
    # Quantized 16-bit FFT has its own error; report it separately from exact
    # power-statistics correctness. This is a bounded synthetic test, no ADC spec.
    assert abs(measured-ideal)<.001,(measured,ideal)
    r={'fundamental_bin':fundamental_bin,'RTL_THD_percent':measured*100,'independent_DFT_percent':ideal*100,
       'difference_percentage_points':(measured-ideal)*100,'exact_power_statistics_match':True,
       'flags':int(m[2]),'limit':'Synthetic ADC samples, existing fixed-point FFT, not physical THD calibration.'}
    all_results.append(r);print(json.dumps(r),flush=True)
(out/'fft_validation.json').write_text(json.dumps(all_results,indent=2))
