"""Draft V32HRM1! v1/v2 decoder; framing/CRC is independent of GUI and pyserial.

Feed the same single-reader byte stream to this and the existing parsers.
No claim that a current production bit already sends this new family.
"""
import math, struct, zlib

MAGIC=b'V32HRM1!'
LENGTH=216
THD_VALID=1<<31
INVALID_THD=1|2|4|8|32|128
FRONTEND_UNCALIBRATED=256
HOST_THD=512

def decode(packet):
    if len(packet)!=LENGTH or packet[:8]!=MAGIC:
        raise ValueError('family/length')
    version,length=struct.unpack_from('<II',packet,8)
    if version not in (1,2) or length!=LENGTH:
        raise ValueError('version/length field')
    if zlib.crc32(packet[:-4])!=struct.unpack_from('<I',packet,LENGTH-4)[0]:
        raise ValueError('CRC')
    fields=struct.unpack_from('<14I',packet,16)
    rid,cfg,fs,nfft,fcw,dec,age,flags,ch,window,mask,nmask,thd,drops=fields
    wire_flags=flags;wire_thd=thd
    if bool(flags&HOST_THD)!=(version==2) or (version==2 and thd!=0):
        raise ValueError('THD computation/version metadata')
    if not fs or nfft<256 or nfft>4096 or nfft&(nfft-1) or dec not in (1,16,256,4096,16384):
        raise ValueError('rate/FFT metadata')
    if ch>1 or window>2 or mask&~31 or nmask&~31 or nmask&~mask:
        raise ValueError('channel/window/mask')
    band=struct.unpack_from('<5Q',packet,72)
    noise=struct.unpack_from('<5Q',packet,112)
    signal=struct.unpack_from('<5Q',packet,152)
    peaks=struct.unpack_from('<5I',packet,192)
    if any(x>=1<<40 for x in band+noise+signal) or any(x>=nfft//2 for x in peaks):
        raise ValueError('power/bin range')
    if tuple(max(6*b-7*n,0) for b,n in zip(band,noise))!=signal:
        raise ValueError('incoherent power statistics')
    valid=bool(flags&THD_VALID)
    if valid and (mask!=31 or flags&INVALID_THD or not signal[0] or window!=0):
        raise ValueError('incoherent THD validity')
    if valid:
        exact_thd=math.isqrt((sum(signal[1:])<<48)//signal[0])
        if version==1 and exact_thd!=thd:
            raise ValueError('incoherent THD result')
        if version==2:
            if exact_thd>0xffffffff:
                valid=False;flags=(flags&~THD_VALID)|32
            else:thd=exact_thd
    below_noise=bool(wire_flags&THD_VALID) and (nmask&30)==30
    if below_noise:
        valid=False;flags=(flags&~THD_VALID)|16
    harmonics=[]
    base_usable=bool(mask&1) and not (nmask&1) and not (flags&(1|2|8|128))
    for i,p in enumerate(signal):
        measured=bool(mask&(1<<i))
        below=bool(nmask&(1<<i))
        db=None if not base_usable or not measured or below or not p or not signal[0] else 10*math.log10(p/signal[0])
        harmonics.append({'order':i+1,'available':measured,'below_noise':below,
                          'dBc':db,'band_power':band[i],'noise_power':noise[i],
                          'signal_power_x6':p,'peak_bin':peaks[i]})
    return dict(version=version,result_id=rid,cfg_id=cfg,adc_fs=fs,nfft=nfft,reference_fcw=fcw,
                reference_hz=fcw*fs/(1<<32),decimation=dec,age_ticks=age,age_seconds=age/fs,
                flags=flags,channel=ch,window=window,valid_mask=mask,noise_mask=nmask,
                thd_valid=valid,thd_q24=thd,thd_percent=thd*100/(1<<24) if valid else None,
                thd_computed_by='Linux' if version==2 else 'FPGA',wire_flags=wire_flags,fpga_thd_q24=wire_thd,
                thd_below_noise=below_noise,
                frontend_uncalibrated=bool(flags&FRONTEND_UNCALIBRATED),
                dropped_frames=drops,harmonics=harmonics)

class Receiver:
    def __init__(self):
        self.buffer=bytearray();self.crc_errors=0;self.invalid_frames=0;self.frames=0
    def feed(self,data):
        self.buffer.extend(data);out=[]
        while True:
            at=self.buffer.find(MAGIC)
            if at<0:
                self.buffer[:]=self.buffer[-7:];break
            if at:del self.buffer[:at]
            if len(self.buffer)<16:break
            version,length=struct.unpack_from('<II',self.buffer,8)
            if version not in (1,2) or length!=LENGTH:
                self.invalid_frames+=1;del self.buffer[0];continue
            if len(self.buffer)<LENGTH:break
            candidate=bytes(self.buffer[:LENGTH])
            try:frame=decode(candidate)
            except ValueError as error:
                self.invalid_frames+=1
                if str(error)=='CRC':self.crc_errors+=1
                del self.buffer[0];continue
            del self.buffer[:LENGTH];self.frames+=1;out.append(frame)
        return out
