import importlib.util,struct,zlib,json,hashlib
from pathlib import Path
r=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('checker',r/'scripts/check_v32_stream.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def frame(n=1024,dec=128,flags=0,start=0):
    h=struct.pack('<8s8I',m.MAGIC,3,1,65000000,dec,n,flags,start&0xffffffff,start>>32)
    raw=h+struct.pack('<hh',-2048,2047)*n
    return raw+struct.pack('<I',zlib.crc32(raw)&0xffffffff)
tests=[]
for n in (256,512,1024,2048,4096):
    d=m.audit(frame(n));assert d['pass_check'] and d['frames'][0]['points']==n;tests.append('points '+str(n))
for data,label in [(b'','empty'),(frame()[:-1],'truncated frame'),(frame()[:20],'truncated header'),(frame(dec=3),'invalid rate'),(frame(flags=1),'unsupported spectrum flag')]:
    assert not m.audit(data)['pass_check'],label;tests.append(label)
b=bytearray(frame());b[80]^=1;assert not m.audit(b)['pass_check'];tests.append('bad CRC')
assert not m.audit(frame(dec=32768),max_decimation=256)['pass_check'];tests.append('host rate limit')
d=m.audit(frame(start=123)+frame(start=65000123));assert d['pass_check'] and d['frames'][1]['raw_clock_start_delta']==65000000;tests.append('sparse frame indices')
sample=r/'references/low_frequency_review_20261020/samples/test_500hz_dec32.bin'
if sample.exists():
    d=m.audit(sample.read_bytes(),max_decimation=256);assert d['pass_check'] and len(d['frames'])==1
    f=d['frames'][0];assert f['points']==4096 and f['decimation']==32;tests.append('supplied synthetic sample')
    (r/'reports/attachment_v32_sample_audit.json').write_text(json.dumps(d,indent=2),encoding='utf-8')
(r/'reports/v32_checker_validation.json').write_text(json.dumps(dict(pass_check=True,tests=tests,
    inputs={'scripts/check_v32_stream.py':hashlib.sha256((r/'scripts/check_v32_stream.py').read_bytes()).hexdigest()}),indent=2),encoding='utf-8')
print('PASS V32 checker:',len(tests),'cases')
