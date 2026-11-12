"""One owner for the physical generator, across threads and Python processes."""
import ctypes as C
from ctypes import wintypes as W
import threading

_PROCESS_LOCK = threading.Lock()

class InstrumentBusyError(RuntimeError):
    pass

class InstrumentLease:
    NAME = r"Global\UTG2062X_AWGK125060042_CONTROL"

    def __init__(self):
        self.handle = None
        self.local_owned = _PROCESS_LOCK.acquire(blocking=False)
        if not self.local_owned:
            raise InstrumentBusyError('发生器正在被其他任务使用，请等图片发送或测试结束后重试')
        try:
            self.kernel = C.WinDLL('kernel32', use_last_error=True)
            self.kernel.CreateMutexW.argtypes = [C.c_void_p, W.BOOL, W.LPCWSTR]
            self.kernel.CreateMutexW.restype = W.HANDLE
            self.kernel.WaitForSingleObject.argtypes = [W.HANDLE, W.DWORD]
            self.kernel.WaitForSingleObject.restype = W.DWORD
            self.kernel.ReleaseMutex.argtypes = [W.HANDLE]
            self.kernel.ReleaseMutex.restype = W.BOOL
            self.kernel.CloseHandle.argtypes = [W.HANDLE]
            self.kernel.CloseHandle.restype = W.BOOL
            h = self.kernel.CreateMutexW(None, False, self.NAME)
            if not h:
                raise C.WinError(C.get_last_error())
            result = self.kernel.WaitForSingleObject(h, 0)
            # An abandoned mutex is now owned by this thread as well.
            if result not in (0, 0x80):
                self.kernel.CloseHandle(h)
                if result == 0x102:
                    raise InstrumentBusyError('发生器正在被其他任务使用，请等图片发送或测试结束后重试')
                raise OSError('Cannot acquire instrument mutex: '+hex(result))
            self.handle = h
        except BaseException:
            self.local_owned=False
            _PROCESS_LOCK.release()
            raise

    def close(self):
        if self.handle:
            h, self.handle = self.handle, None
            try:
                if not self.kernel.ReleaseMutex(h):
                    raise C.WinError(C.get_last_error())
            finally:
                self.kernel.CloseHandle(h)
                if self.local_owned:
                    self.local_owned=False
                    _PROCESS_LOCK.release()
