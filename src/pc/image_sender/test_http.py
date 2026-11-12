"""Read-only smoke check against the running local server; never starts RF."""
import json,re,subprocess,urllib.request,urllib.error,tempfile
from pathlib import Path
base='http://127.0.0.1:8768'
html=urllib.request.urlopen(base).read().decode()
assert '__TOKEN__' not in html
assert not json.loads(urllib.request.urlopen(base+'/api/status').read())['busy']
with tempfile.TemporaryDirectory() as d:
    js=Path(d)/'check.js';js.write_text(re.search(r'<script>(.*?)</script>',html,re.S).group(1),encoding='utf-8')
    subprocess.run(['node','--check',str(js)],check=True)
request=urllib.request.Request(base+'/api/start',data=b'{}',headers={'Content-Type':'application/json'})
try:
    urllib.request.urlopen(request)
    raise AssertionError('Missing token was accepted')
except urllib.error.HTTPError as exc:assert exc.code==403
token=re.search("X-Session-Token':'([^']+)'",html).group(1)
request=urllib.request.Request(base+'/api/start',data=b'{}',headers={'Content-Type':'application/json','X-Session-Token':token})
try:
    urllib.request.urlopen(request)
    raise AssertionError('Invalid payload was accepted')
except urllib.error.HTTPError as exc:assert exc.code==400
assert not json.loads(urllib.request.urlopen(base+'/api/status').read())['busy']
print('HTTP, JS syntax, token and payload validation PASS; no RF operation')
