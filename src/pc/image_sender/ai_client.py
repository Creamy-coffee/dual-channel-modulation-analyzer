"""Standard-library local client. Construction never changes an instrument."""
import urllib.request,json,base64
from pathlib import Path
class Client:
    def __init__(self,url='http://127.0.0.1:8770'):
        self.url=url.rstrip('/')
        self.token=self.get('/api/session')['token']
    def get(self,path):
        with urllib.request.urlopen(self.url+path,timeout=10) as r:return json.load(r)
    def post(self,path,payload,timeout=120):
        req=urllib.request.Request(self.url+path,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json','X-Session-Token':self.token},method='POST')
        with urllib.request.urlopen(req,timeout=timeout) as r:return json.load(r)
    def status(self):return self.get('/api/status')
    def read(self,channel=2):return self.post('/api/generator/read',dict(channel=channel))
    def apply(self,**parameters):return self.post('/api/generator/apply',parameters)
    def screenshot(self,path='screen.png'):
        r=self.post('/api/screen',{})
        Path(path).write_bytes(base64.b64decode(r['image'].split(',',1)[1],validate=True))
        return str(Path(path).resolve())
    def send_pixels(self,pixels,width,height,mode='ook',rounds=1):
        return self.post('/api/start',dict(pixels=base64.b64encode(pixels).decode(),size=width,height=height,mode=mode,rounds=rounds))
    def stop(self):return self.post('/api/stop',{})
if __name__=='__main__':
    print(json.dumps(Client().status(),ensure_ascii=False,indent=2))
