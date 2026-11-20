import unittest
import base64,io,subprocess,tempfile
import struct,zlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import control_web as c
import threading,urllib.request,urllib.error,json
class Checks(unittest.TestCase):
    def setUp(self):
        self.home_dir=tempfile.TemporaryDirectory()
        self.home_patch=patch.object(c.sender,'HOME',Path(self.home_dir.name))
        self.home_patch.start()
    def tearDown(self):
        self.home_patch.stop()
        self.home_dir.cleanup()
    def test_http_session_guide_and_auth(self):
        http=c.ThreadingHTTPServer(('127.0.0.1',0),c.Handler)
        thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        base='http://127.0.0.1:'+str(http.server_port)
        try:
            with patch.object(c.sender,'HOME',Path(__file__).parent):
                with urllib.request.urlopen(base+'/') as r:
                    page=r.read().decode()
                    self.assertIn('板端波形文件',page)
                    self.assertIn('/api/waveforms/download',page)
                    self.assertNotIn('__TOKEN__',page)
            with urllib.request.urlopen(base+'/api/session') as r:self.assertEqual(json.load(r)['token'],c.sender.TOKEN)
            with tempfile.TemporaryDirectory() as directory:
                (Path(directory)/'AI_API_GUIDE.md').write_text('AI 使用指南',encoding='utf-8')
                with patch.object(c.sender,'HOME',Path(directory)):
                    with urllib.request.urlopen(base+'/api/guide') as r:self.assertIn('AI 使用指南',r.read().decode())
            req=urllib.request.Request(base+'/api/screen',data=b'{}',method='POST')
            with self.assertRaises(urllib.error.HTTPError) as err:urllib.request.urlopen(req)
            self.assertEqual(err.exception.code,403)
            with patch.object(c,'list_waveform_files',return_value={'files':[],'count':0,'roots':list(c.WAVEFORM_ROOTS),'truncated':False}):
                req=urllib.request.Request(base+'/api/waveforms/list',data=b'{}',headers={'Content-Type':'application/json','X-Session-Token':c.sender.TOKEN},method='POST')
                with urllib.request.urlopen(req) as r:self.assertEqual(json.load(r)['count'],0)
            with patch.object(c,'screenshot',return_value={'image':'data:image/png;base64,AA=='}):
                req=urllib.request.Request(base+'/api/screen',data=b'{}',headers={'Content-Type':'application/json','X-Session-Token':c.sender.TOKEN},method='POST')
                with urllib.request.urlopen(req) as r:self.assertEqual(json.load(r)['image'],'data:image/png;base64,AA==')
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
        with tempfile.TemporaryDirectory() as directory,patch.object(c.sender,'HOME',Path(directory)):
            with patch.object(c,'Visa',side_effect=RuntimeError('offline')):
                with self.assertRaises(RuntimeError):c.generator({})
        self.assertFalse(c.sender.STATE['busy'])
    def test_waveform_scan_accepts_unicode_names_and_rejects_untrusted_paths(self):
        paths=[('/userdata/captures/带 空格.raw',4),('/data/private.raw',1),
               ('/userdata/../tmp/escape.raw',1),('/userdata/readme.txt',3)]
        output=''.join(f'{size}\t{base64.b64encode(path.encode()).decode()}\n' for path,size in paths).encode()
        with patch.object(c,'adb',return_value=output) as adb:
            result=c.list_waveform_files()
        adb.assert_called_once_with('shell',c.WAVEFORM_FIND,timeout=60)
        self.assertEqual(result['count'],1)
        self.assertEqual(result['files'][0]['path'],'/userdata/captures/带 空格.raw')
        self.assertEqual(result['files'][0]['size'],4)
        self.assertEqual(c._waveform_entry(result['files'][0]['id'])['path'],'/userdata/captures/带 空格.raw')
    def test_waveform_scan_reports_truncated_results(self):
        output=''.join(f'1\t{base64.b64encode(f"/tmp/{i}.raw".encode()).decode()}\n' for i in range(c.WAVEFORM_LIMIT+1)).encode()
        with patch.object(c,'adb',return_value=output):result=c.list_waveform_files()
        self.assertEqual(result['count'],c.WAVEFORM_LIMIT)
        self.assertTrue(result['truncated'])
    def test_ppm_to_png_converts_rgb_rows_and_rejects_truncation(self):
        ppm=b'P6\n2 1\n255\n'+bytes((255,0,0,0,255,0))
        png=c._ppm_to_png(ppm)
        self.assertTrue(png.startswith(b'\x89PNG\r\n\x1a\n'))
        pos=8;chunks=[]
        while pos<len(png):
            size=struct.unpack('!I',png[pos:pos+4])[0];kind=png[pos+4:pos+8];payload=png[pos+8:pos+8+size]
            self.assertEqual(struct.unpack('!I',png[pos+8+size:pos+12+size])[0],zlib.crc32(kind+payload)&0xffffffff)
            chunks.append((kind,payload));pos+=size+12
        self.assertEqual(chunks[0][0],b'IHDR')
        self.assertEqual(struct.unpack('!II',chunks[0][1][:8]),(2,1))
        self.assertEqual(zlib.decompress(chunks[1][1]),b'\x00'+bytes((255,0,0,0,255,0)))
        with self.assertRaises(ValueError):c._ppm_to_png(ppm[:-1])
    def test_waveform_download_checks_size_and_streams_bytes(self):
        payload=b'wave\x00'
        file_id='A1234567890_'
        with c.WAVEFORM_LOCK:
            c.WAVEFORM_CACHE.clear()
            c.WAVEFORM_CACHE[file_id]={'path':'/userdata/captures/test.raw','size':len(payload),'name':'test.raw'}
        class Handler:
            def __init__(self):self.headers_out={};self.status=None;self.wfile=io.BytesIO()
            def send_response(self,value):self.status=value
            def send_header(self,key,value):self.headers_out[key]=value
            def end_headers(self):pass
        def fake_pull(args,**kwargs):
            Path(args[-1]).write_bytes(payload)
            return SimpleNamespace(returncode=0,stderr=b'')
        h=Handler()
        with patch.object(c,'adb',return_value=str(len(payload)).encode()),patch.object(c.subprocess,'run',side_effect=fake_pull) as pull:
            c.send_waveform_file(h,file_id)
        self.assertEqual(h.status,200)
        self.assertEqual(h.headers_out['Content-Length'],str(len(payload)))
        self.assertEqual(h.wfile.getvalue(),payload)
        pull.assert_called_once()
    def test_waveform_download_rejects_stale_id_without_adb(self):
        with c.WAVEFORM_LOCK:c.WAVEFORM_CACHE.clear()
        with patch.object(c,'adb') as adb:
            with self.assertRaises(ValueError):c._waveform_entry('A1234567890_')
        adb.assert_not_called()
if __name__=='__main__':unittest.main()
