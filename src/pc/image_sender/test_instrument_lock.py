"""Windows cross-process exclusion and failed constructor cleanup checks."""
import ctypes as C
import subprocess,sys,unittest
from pathlib import Path
from unittest.mock import patch
from instrument_lock import InstrumentLease,InstrumentBusyError
from instrument import Visa
class Function:
    def __init__(self,call):self.call=call
    def __call__(self,*args):return self.call(*args)
class FakeDLL:
    def __init__(self):
        self.closed=[];self.access=[]
        self.viOpenDefaultRM=Function(self.rm)
        self.viOpen=Function(self.open)
        self.viSetAttribute=Function(lambda *args:0)
        self.viWrite=Function(lambda *args:0)
        self.viRead=Function(lambda *args:0)
        self.viClose=Function(lambda h:self.closed.append(h.value) or 0)
    def rm(self,out):out._obj.value=11;return 0
    def open(self,rm,resource,access,timeout,out):
        self.access.append((access,timeout));out._obj.value=22;return 0
class FakeLease:
    def __init__(self):self.closed=False
    def close(self):self.closed=True
class TestInstrumentLock(unittest.TestCase):
    def child(self,code):return subprocess.run([sys.executable,'-c',code],cwd=Path(__file__).parent,capture_output=True,text=True,timeout=10)
    def test_same_process_and_release(self):
        owner=InstrumentLease()
        try:
            with self.assertRaises(InstrumentBusyError):InstrumentLease()
        finally:owner.close()
        owner.close();new=InstrumentLease();new.close()
    def test_kernel_initialization_failure_releases_local_lock(self):
        with patch.object(C,'WinDLL',side_effect=OSError('injected')):
            with self.assertRaises(OSError):InstrumentLease()
        lease=InstrumentLease();lease.close()
    def test_cross_process_busy(self):
        owner=InstrumentLease()
        try:
            result=self.child('from instrument_lock import *\ntry: InstrumentLease()\nexcept InstrumentBusyError: print("BUSY_OK")\nelse: raise RuntimeError("second process acquired owned generator")')
            self.assertEqual(result.returncode,0,result.stderr);self.assertIn('BUSY_OK',result.stdout)
        finally:owner.close()
        result=self.child('from instrument_lock import InstrumentLease; x=InstrumentLease(); x.close(); print("RELEASE_OK")')
        self.assertEqual(result.returncode,0,result.stderr)
    def test_constructor_failure_cleanup(self):
        dll=FakeDLL();lease=FakeLease()
        with patch('instrument.InstrumentLease',return_value=lease),patch.object(C,'WinDLL',return_value=dll),patch.object(Visa,'query',return_value='WRONG'):
            with self.assertRaises(RuntimeError):Visa([])
        self.assertEqual(dll.access,[(1,0)]);self.assertEqual(dll.closed,[22,11]);self.assertTrue(lease.closed)
    def test_idempotent_close(self):
        dll=FakeDLL();lease=FakeLease()
        with patch('instrument.InstrumentLease',return_value=lease),patch.object(C,'WinDLL',return_value=dll),patch.object(Visa,'query',return_value='UTG2062X,AWGK125060042'):
            v=Visa([]);v.close();v.close()
        self.assertEqual(dll.closed,[22,11]);self.assertTrue(lease.closed)
if __name__=='__main__':unittest.main()
