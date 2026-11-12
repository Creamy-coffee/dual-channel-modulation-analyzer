import unittest
from unittest.mock import patch
import control_web as c
import threading,urllib.request,urllib.error,json
class Checks(unittest.TestCase):
    def test_http_session_guide_and_auth(self):
        http=c.ThreadingHTTPServer(('127.0.0.1',0),c.Handler)
        thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        base='http://127.0.0.1:'+str(http.server_port)
        try:
            with urllib.request.urlopen(base+'/api/session') as r:self.assertEqual(json.load(r)['token'],c.sender.TOKEN)
            with urllib.request.urlopen(base+'/api/guide') as r:self.assertIn('AI 使用指南',r.read().decode())
            req=urllib.request.Request(base+'/api/screen',data=b'{}',method='POST')
            with self.assertRaises(urllib.error.HTTPError) as err:urllib.request.urlopen(req)
            self.assertEqual(err.exception.code,403)
        finally:http.shutdown();http.server_close();thread.join()
    def test_invalid_channel(self):
        for x in (0,3,True,'2'):
            with self.assertRaises(ValueError):c.generator({'channel':x})
    def test_invalid_values_do_not_open(self):
        base=dict(channel=2,frequency=1e6,amplitude=.3,phase=0,mode='CONTinue',wave='SINe',output=False)
        with patch.object(c,'Visa') as visa:
            for k,v in [('frequency',float('nan')),('frequency',31e6),('amplitude',5),('mode','bad'),('output','ON')]:
                req=dict(base);req[k]=v
                with self.assertRaises(ValueError):c.generator(req,True)
            visa.assert_not_called()
    def test_busy_rejected(self):
        c.sender.STATE['busy']=True
        try:
            with self.assertRaises(RuntimeError):c.generator({})
        finally:c.sender.STATE['busy']=False
    def test_failure_releases(self):
        with patch.object(c,'Visa',side_effect=RuntimeError('offline')):
            with self.assertRaises(RuntimeError):c.generator({})
        self.assertFalse(c.sender.STATE['busy'])
if __name__=='__main__':unittest.main()
