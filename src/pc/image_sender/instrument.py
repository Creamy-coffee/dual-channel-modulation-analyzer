import ctypes as C
import time,struct,hashlib
from instrument_lock import InstrumentLease, InstrumentBusyError
import threading

class Visa:
    def __init__(self, log):
        self.log = log
        self.dll = None
        self.rm, self.session = C.c_uint32(), C.c_uint32()
        self.lease = None
        self._io_lock = threading.RLock()
        try:
            # Held for the entire save/configure/transmit/restore transaction.
            self.lease = InstrumentLease()
            self.dll = C.WinDLL('visa64.dll')
            u = C.c_uint32
            self.dll.viOpenDefaultRM.argtypes = [C.POINTER(u)]
            self.dll.viOpen.argtypes = [u, C.c_char_p, u, u, C.POINTER(u)]
            self.dll.viSetAttribute.argtypes = [u, u, C.c_size_t]
            self.dll.viWrite.argtypes = [u, C.c_void_p, u, C.POINTER(u)]
            self.dll.viRead.argtypes = [u, C.c_void_p, u, C.POINTER(u)]
            self.dll.viClose.argtypes = [u]
            self.check(self.dll.viOpenDefaultRM(C.byref(self.rm)))
            # VI_EXCLUSIVE_LOCK=1: VISA also rejects I/O from other sessions,
            # including clients that do not use our Windows named mutex.
            self.check(self.dll.viOpen(self.rm, b'USB0::0x6656::0x0834::AWGK125060042::INSTR', 1, 0, C.byref(self.session)))
            self.check(self.dll.viSetAttribute(self.session, 0x3FFF001A, 5000))
            if 'UTG2062X,AWGK125060042' not in self.query('*IDN?'):
                raise RuntimeError('发生器身份回复不匹配，已关闭本次连接')
        except BaseException:
            self.close()
            raise

    @staticmethod
    def check(status):
        if status < 0:
            raise RuntimeError(f'VISA status {status:#x}')

    def write(self, command):
        with self._io_lock:
            data = (command + '\n').encode('ascii')
            count = C.c_uint32()
            self.check(self.dll.viWrite(self.session, data, len(data), C.byref(count)))
            if count.value != len(data):
                raise RuntimeError('Short VISA write')
            self.log.append(dict(command=command))
            time.sleep(.025)

    def query(self, command):
        with self._io_lock:
            self.write(command)
            data = C.create_string_buffer(65536)
            count = C.c_uint32()
            self.check(self.dll.viRead(self.session, data, len(data), C.byref(count)))
            value = data.raw[:count.value].decode('ascii').strip()
            self.log[-1]['response'] = value
            if not value:
                raise RuntimeError('发生器返回空回复：'+command)
            if command.upper() != '*IDN?' and 'UTG2062X,AWGK125060042' in value:
                raise RuntimeError('发生器查询回复错位：'+command+' 收到了仪器身份')
            return value

    def close(self):
        # Keep ownership until every VISA handle has been closed. Constructor
        # failure must release it too; close is intentionally idempotent.
        try:
            if self.dll:
                for handle in (self.session, self.rm):
                    if handle.value:
                        try:
                            self.dll.viClose(handle)
                        finally:
                            handle.value = 0
        finally:
            if self.lease:
                lease, self.lease = self.lease, None
                lease.close()

FIELDS = ['OUTPut', 'MODE', 'BASE:WAVe', 'BASE:FREQuency', 'AMPLitude:UNIT',
          'BASE:AMPLitude', 'BASE:OFFSet', 'BASE:PHASe', 'LOAD', 'MODulate:TYPe',
          'MODulate:SOURce', 'MODulate:WAVe', 'MODulate:FREQuency',
          'MODulate:RATio', 'MODulate:BITRatio', 'MODulate:DEPTh', 'PNCode',
          'FM:FREQuency:DEV', 'FSK:FREQuency1', 'PSK:PHASe1', 'PSK:PHASe2',
          'PSK:PHASe3', 'PSK:PHASe4']

def bsv(samples,channel=2):
    header=(f'VPP:0.166667\r\nOFFSET:0.000000\r\nCHANNEL:{channel}\r\n'
            'RATEPOS:0.000030518509475997\r\nRATENEG:0.000030518509475997\r\n'
            'MAX:32767.000000\r\nMIN:-32767.000000\r\n').encode('ascii')
    data=f'[HEAD]:{len(header)}\r\n'.encode()+header+f'[DATA]:{len(samples)}\r\n'.encode()
    return data+struct.pack('<'+'h'*len(samples),*samples)

def write_raw(v,data):
    with v._io_lock:
        buffer=C.create_string_buffer(data);count=C.c_uint32()
        v.check(v.dll.viWrite(v.session,buffer,len(data),C.byref(count)))
        if count.value!=len(data):raise RuntimeError('Short binary transfer')

def arb_select(v,prefix,field,value):
    source,name=value.split(',',1);name=name.strip('"')
    assert not any(x in name for x in ('"','\n','\r'))
    v.write(prefix+field+' '+source+',"'+name+'"')

def restore(v,original,extra):
    p=':CHANnel2:';v.write(p+'OUTPut OFF')
    # File loading can affect parameter metadata; restore selectors first.
    arb_select(v,p,'MODulate:ARB',extra['MODulate:ARB'])
    if v.query(p+'BASE:ARB?')!=extra['BASE:ARB']:arb_select(v,p,'BASE:ARB',extra['BASE:ARB'])
    if 'PM:PHASe:DEV' in extra:
        v.write(p+'MODulate:TYPe PM');v.write(p+'PM:PHASe:DEV '+extra['PM:PHASe:DEV'])
    v.write(p+'MODulate:TYPe QPSK');v.write(p+'PNCode '+original['PNCode'])
    for f in ('PSK:PHASe1','PSK:PHASe2','PSK:PHASe3','PSK:PHASe4'):v.write(p+f+' '+original[f])
    v.write(p+'MODulate:TYPe FSK');v.write(p+'FSK:FREQuency1 '+original['FSK:FREQuency1'])
    order=['BASE:WAVe','AMPLitude:UNIT','BASE:FREQuency','BASE:AMPLitude','BASE:OFFSet','BASE:PHASe','MODulate:TYPe']
    order += [f for f in FIELDS if f not in order+['OUTPut','MODE','LOAD','FSK:FREQuency1','PSK:PHASe1','PSK:PHASe2','PSK:PHASe3','PSK:PHASe4']]
    for f in order+['MODE']:v.write(p+f+' '+original[f])
    assert v.query(p+'BASE:AMPLitude?')==original['BASE:AMPLitude']
    assert v.query(p+'BASE:OFFSet?')==original['BASE:OFFSet']
    v.write(p+'OUTPut '+original['OUTPut'])
    actual={f:v.query(p+f+'?') for f in FIELDS}
    actual_extra={f:v.query(p+f+'?') for f in extra}
    return dict(verified=actual==original and actual_extra==extra,readback=actual,extra=actual_extra)
