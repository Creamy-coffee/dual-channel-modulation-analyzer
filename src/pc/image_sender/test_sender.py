"""Hardware-free checks: real packet encoder/BSV and worker cleanup paths."""
import base64,json,struct,tempfile,threading,unittest,zlib
from pathlib import Path
from unittest.mock import patch
import server

class FakeVisa:
    fail=False
    def __init__(self,log):self.log=log;self.values={};self.closed=False
    def write(self,c):
        self.log.append({'command':c})
        if self.fail and c.endswith('MODE MODulation'):raise RuntimeError('injected write failure')
        k,_,v=c.partition(' ');self.values[k]=v
    def query(self,c):
        k=c[:-1];default='0'
        if k.endswith('AMPLitude:UNIT'):default='Vpp'
        elif k.endswith('BASE:AMPLitude'):default='0.3333333'
        elif k.endswith('MODulate:ARB') or k.endswith('BASE:ARB'):default='INTernal,ACosH.bsv'
        return self.values.get(k,default).replace('"','')
    def close(self):self.closed=True

class TestSender(unittest.TestCase):
    def test_real_wire_packets(self):
        for size in (32,64,128):
            pixels=bytes((i*31)&255 for i in range(size*size//8))
            packets=server.make_packets(pixels,size,size,123)
            self.assertEqual(len(packets),size*size//256)
            rebuilt=b''
            for packet in packets:
                self.assertEqual(zlib.crc32(packet[:64]),struct.unpack('<I',packet[64:])[0])
                rebuilt+=packet[32:64]
                syms=server.symbols_for([packet])+bytes([0,1,1,0])*112
                self.assertEqual(len(syms),2048)
                encoded=server.bsv([32767 if s else -32767 for s in syms for _ in range(2)])
                self.assertIn(b'[DATA]:4096\r\n',encoded)
            self.assertEqual(rebuilt,pixels)
    def test_worker_restore_and_failure(self):
        for mode in ('ook','fsk','bpsk'):
            for fail in (False,True):
                with tempfile.TemporaryDirectory() as d:
                    FakeVisa.fail=fail
                    req={'mode':mode,'size':32,'rounds':1,'pixels':base64.b64encode(bytes(128)).decode()}
                    server.STATE.update(busy=True,error=None,progress=0)
                    server.STOP.clear()
                    with patch.object(server,'HOME',Path(d)),patch.object(server,'write_raw'),patch.object(server,'restore',return_value={'verified':True}) as restore,patch.object(server.STOP,'wait',return_value=False):
                        server.transmit(req,FakeVisa)
                        restore.assert_called_once()
                    self.assertFalse(server.STATE['busy']);self.assertTrue(server.STATE['restored'])
                    self.assertEqual(bool(server.STATE['error']),fail)
    def test_validation(self):
        for size in (0,31,256,'32',True):
            with self.assertRaises(ValueError):server.validate({'mode':'ook','size':size,'pixels':''})
    def test_landscape_dimensions(self):
        raw=bytes(128*64//8)
        req={"mode":"ook","size":128,"height":64,"pixels":base64.b64encode(raw).decode()}
        self.assertEqual(server.validate(req)[3],raw)
        packets=server.make_packets(raw,128,64,7)
        self.assertEqual(len(packets),32)
        self.assertEqual(struct.unpack_from("<HH",packets[0],10),(128,64))
        req["height"]=63
        with self.assertRaises(ValueError):server.validate(req)

    def test_large_image_reuses_four_slots(self):
        with tempfile.TemporaryDirectory() as d:
            FakeVisa.fail=False;log=[];fake=FakeVisa(log)
            req={'mode':'fsk','size':128,'rounds':1,'pixels':base64.b64encode(bytes(2048)).decode()}
            server.STATE.update(busy=True,error=None,progress=0);server.STOP.clear()
            with patch.object(server,'HOME',Path(d)),patch.object(server,'write_raw'),patch.object(server,'restore',return_value={'verified':True}),patch.object(server.STOP,'wait',return_value=False):server.transmit(req,lambda _:fake)
            self.assertIsNone(server.STATE['error'])
            uploads=[r['command'] for r in log if r['command'].startswith(':WARB2:MODulate ')]
            self.assertEqual(len(uploads),64);self.assertEqual(len(set(uploads)),4)
            self.assertEqual(len(list(Path(d).rglob('packet*.bsv'))),64)

    def test_128x256_reuses_four_slots(self):
        with tempfile.TemporaryDirectory() as d:
            FakeVisa.fail=False;log=[];fake=FakeVisa(log)
            req={'mode':'ook','size':128,'height':256,'rounds':1,'pixels':base64.b64encode(bytes(4096)).decode()}
            server.STATE.update(busy=True,error=None,progress=0);server.STOP.clear()
            with patch.object(server,'HOME',Path(d)),patch.object(server,'write_raw'),patch.object(server,'restore',return_value={'verified':True}),patch.object(server.STOP,'wait',return_value=False):server.transmit(req,lambda _:fake)
            self.assertIsNone(server.STATE['error'])
            self.assertEqual(server.STATE['packets'],128)
            uploads=[r['command'] for r in log if r['command'].startswith(':WARB2:MODulate ')]
            self.assertEqual(len(uploads),128);self.assertEqual(len(set(uploads)),4)
            self.assertEqual((next(Path(d).rglob('image.pbm'))).read_bytes()[:11],b'P4\n128 256\n')
    def test_rejected_slot_never_enables_output(self):
        class RejectVisa(FakeVisa):
            def query(self,c):return 'INTernal,ACosH.bsv' if c.endswith('MODulate:ARB?') else super().query(c)
        with tempfile.TemporaryDirectory() as d:
            FakeVisa.fail=False;log=[];fake=RejectVisa(log)
            server.STATE.update(busy=True,error=None,progress=0);server.STOP.clear()
            req={'mode':'fsk','size':32,'rounds':1,'pixels':base64.b64encode(bytes(128)).decode()}
            with patch.object(server,'HOME',Path(d)),patch.object(server,'write_raw'),patch.object(server,'restore',return_value={'verified':True}) as recovery:server.transmit(req,lambda _:fake)
            recovery.assert_called_once();self.assertIn('波形未选中',server.STATE['error'])
            self.assertNotIn(':CHANnel2:OUTPut ON',[r['command'] for r in log])

if __name__=='__main__':unittest.main()
