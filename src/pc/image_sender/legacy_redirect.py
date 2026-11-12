"""Retire the old sender URL without creating a second instrument controller."""
import json
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
class Handler(BaseHTTPRequestHandler):
    def message(self,status):
        data=json.dumps({'busy':False,'phase':'旧版已停用，请刷新页面进入修复版','progress':0,
                         'error':'请刷新当前网页，将自动转到 http://127.0.0.1:8768/ 。此旧页面不会操作发生器。'},ensure_ascii=False).encode()
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
    def do_GET(self):
        if self.path=='/api/status':return self.message(200)
        self.send_response(302);self.send_header('Location','http://127.0.0.1:8768/');self.send_header('Cache-Control','no-store');self.end_headers()
    def do_POST(self):self.message(409)
    def log_message(self,*args):pass
if __name__=='__main__':ThreadingHTTPServer(('127.0.0.1',8766),Handler).serve_forever()
