"""Local unified control for the generator, RK3506 screen and stored captures."""
import base64,json,math,mimetypes,os,re,secrets,shlex,shutil,socket,subprocess,time,threading,webbrowser,zlib,struct
from pathlib import Path
from http.server import ThreadingHTTPServer
from urllib.parse import quote
import server as sender
from instrument import Visa
from tempfile import TemporaryDirectory

def find_adb():
    override=os.environ.get('V32_ADB')
    if override:return Path(override)
    located=shutil.which('adb')
    if located:return Path(located)
    roots=[os.environ.get('ANDROID_SDK_ROOT'),os.environ.get('ANDROID_HOME'),
           os.environ.get('LOCALAPPDATA') and str(Path(os.environ['LOCALAPPDATA'])/'Android'/'Sdk')]
    roots.extend((str(Path.home()/'adb-tools'),str(Path.home()/'AppData'/'Local'/'Android'/'Sdk')))
    for root in roots:
        if root:
            candidate=Path(root)/'platform-tools'/'adb.exe'
            if candidate.is_file():return candidate
    return Path('adb')
ADB=find_adb()
ADB_SERIAL=os.environ.get('V32_ADB_SERIAL','6ed3ab1135df1ef9')
SCREEN_LOCK=threading.Lock()
WAVEFORM_LOCK=threading.Lock()
WAVEFORM_CACHE={}
WAVEFORM_ROOTS=('/userdata','/tmp','/mnt/udisk')
WAVEFORM_EXTENSIONS=('.bin','.raw','.csv','.wav','.wfm','.json','.jsonl')
WAVEFORM_LIMIT=1000
WAVEFORM_MAX_DOWNLOAD=256*1024*1024
def adb(*args,timeout=15):
    r=subprocess.run([str(ADB),'-s',ADB_SERIAL,*args],capture_output=True,timeout=timeout)
    if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace') or r.stdout.decode(errors='replace'))
    return r.stdout

WAVEFORM_FIND=(
    "find /userdata /tmp /mnt/udisk -maxdepth 8 -type f "
    "\\( -iname '*.bin' -o -iname '*.raw' -o -iname '*.csv' -o "
    "-iname '*.wav' -o -iname '*.wfm' -o -iname '*.json' -o -iname '*.jsonl' \\) "
    "-print 2>/dev/null | head -n 1001 | while IFS= read -r file; do "
    "size=$(wc -c < \"$file\" 2>/dev/null) || continue; "
    "encoded=$(printf '%s' \"$file\" | base64 | tr -d '\\r\\n') || continue; "
    "printf '%s\\t%s\\n' \"$size\" \"$encoded\"; done"
)

def list_waveform_files():
    """List existing capture-like files without accepting a caller-supplied path."""
    output=adb('shell',WAVEFORM_FIND,timeout=60).decode('utf-8','replace')
    entries=[]
    cache={}
    records=output.splitlines()
    truncated=len(records)>WAVEFORM_LIMIT
    for line in records[:WAVEFORM_LIMIT]:
        try:
            size_text,encoded_path=line.split('\t',1)
            size=int(size_text.strip())
            path=base64.b64decode(encoded_path,validate=True).decode('utf-8')
        except (ValueError,TypeError,UnicodeDecodeError):continue
        if size<0 or any(ord(char)<32 for char in path):continue
        if not any(path.startswith(root+'/') for root in WAVEFORM_ROOTS):continue
        if any(part in ('','.','..') for part in path.split('/')[1:]):continue
        if not path.lower().endswith(WAVEFORM_EXTENSIONS):continue
        name=path.rsplit('/',1)[-1]
        if path=='/tmp/scope_history.csv':
            category='示波器 History 导出'
            description='屏端 History 页点 CSV 保存的单条原始采集记录'
            priority=0
        elif name=='raw.bin' and '/' in path[1:]:
            category='串口原始抓包'
            description='AB5/V32CAP3 接收的原始串口字节，通常与同目录日志配套'
            priority=1
        elif name=='frames.jsonl':
            category='帧解析日志'
            description='逐帧记录 CRC、帧类型与接收顺序等信息'
            priority=1
        elif name=='health.jsonl':
            category='链路健康日志'
            description='按时间统计吞吐、有效帧、CRC 错误与丢弃字节'
            priority=1
        elif name=='summary.json':
            category='抓包摘要'
            description='同目录串口采集的总时长、字节数与帧统计'
            priority=1
        elif path.startswith('/tmp/'):
            category='临时实验采样'
            description='算法验证或诊断用的临时采样文件，不是 History 页面导出'
            priority=2
        elif path.startswith('/userdata/'):
            category='板端测试数据'
            description='板上实验或诊断文件；请结合文件名及同目录记录判断'
            priority=3
        else:
            category='采集/测试文件'
            description='保存于已挂载的外部存储'
            priority=4
        file_id=secrets.token_urlsafe(12)
        cache[file_id]={'path':path,'size':size,'name':name}
        entries.append({'id':file_id,'path':path,'name':name,'size':size,
                        'category':category,'description':description,'priority':priority})
    entries.sort(key=lambda item:(item['priority'],item['path'].lower()))
    for item in entries:item.pop('priority')
    with WAVEFORM_LOCK:WAVEFORM_CACHE.clear();WAVEFORM_CACHE.update(cache)
    return {'files':entries,'count':len(entries),'roots':list(WAVEFORM_ROOTS),'truncated':truncated}

def _waveform_entry(file_id):
    if not isinstance(file_id,str) or not re.fullmatch(r'[A-Za-z0-9_-]{12,32}',file_id):
        raise ValueError('文件编号无效，请刷新文件列表')
    with WAVEFORM_LOCK:entry=WAVEFORM_CACHE.get(file_id)
    if not entry:raise ValueError('文件列表已过期，请刷新后重试')
    return dict(entry)

def send_waveform_file(handler,file_id):
    entry=_waveform_entry(file_id)
    path=entry['path'];size=entry['size'];name=entry['name']
    if size>WAVEFORM_MAX_DOWNLOAD:raise ValueError('文件超过 256 MiB 下载上限')
    current=adb('shell',f'wc -c < {shlex.quote(path)}').decode().strip()
    if not current.isdigit() or int(current)!=size:raise RuntimeError('板端文件大小已变化，请刷新文件列表')
    with TemporaryDirectory(prefix='rk3506-waveform-') as directory:
        local_path=Path(directory)/'capture'
        result=subprocess.run([str(ADB),'-s',ADB_SERIAL,'pull',path,str(local_path)],
                              capture_output=True,timeout=300)
        if result.returncode:
            raise RuntimeError(result.stderr.decode('utf-8','replace').strip() or 'ADB 下载失败')
        if not local_path.is_file() or local_path.stat().st_size!=size:
            raise RuntimeError('下载后的文件大小校验失败')
        handler.send_response(200)
        handler.send_header('Content-Type',mimetypes.guess_type(name)[0] or 'application/octet-stream')
        handler.send_header('Content-Length',str(size))
        handler.send_header('Content-Disposition',f"attachment; filename*=UTF-8''{quote(name,safe='')}")
        handler.send_header('Cache-Control','no-store')
        handler.send_header('X-Content-Type-Options','nosniff')
        handler.end_headers()
        with local_path.open('rb') as source:
            while True:
                block=source.read(1024*1024)
                if not block:break
                try:handler.wfile.write(block)
                except (BrokenPipeError,ConnectionResetError):break
def number(req,key,low,high):
    x=float(req[key])
    if not math.isfinite(x) or not low<=x<=high:raise ValueError(f'{key}应在{low}..{high}之间')
    return x
FIELDS=['OUTPut','MODE','BASE:WAVe','BASE:FREQuency','BASE:AMPLitude','BASE:OFFSet','BASE:PHASe','AMPLitude:UNIT','MODulate:TYPe','MODulate:FREQuency','MODulate:DEPTh','FM:FREQuency:DEV','PM:PHASe:DEV']
def settings(v,ch):return {f:v.query(f':CHANnel{ch}:{f}?') for f in FIELDS}

# ---------------------------------------------------------------- 拍摄脚本分段
# scenes.json 由 video/script-3min.md 逐段翻译而来：每段给出该段拍摄时
# 发生器两路应该是什么状态，以及对应屏页与镜头提示。
SCENES_FILE=sender.HOME/'scenes.json'
def load_scenes():
    return json.loads(SCENES_FILE.read_text(encoding='utf-8'))

def apply_scene(v,log,scene):
    """按段下发两路参数，并逐项回读核验。

    先全部关输出 → 设参数 → 再开输出，避免中途出现半旧半新的状态被拍进镜头。
    """
    gens=scene.get('gen') or []
    if not isinstance(gens,list) or not gens:raise ValueError('该段没有发生器参数')
    for g in gens:
        ch=g.get('ch')
        if type(ch)!=int or ch not in (1,2):raise ValueError('通道错误')
        hz=float(g['hz']);vpp=float(g['vpp']);wave=g.get('wave','SINe');phase=float(g.get('phase',0));on=bool(g.get('on',True))
        if not math.isfinite(hz) or not 1<=hz<=30000000:raise ValueError('频率超出 1 Hz..30 MHz')
        if not math.isfinite(vpp) or not 0.001<=vpp<=0.5:raise ValueError('幅度应在 0.001..0.5 Vpp')
        if wave not in ('SINe','SQUare','RAMP'):raise ValueError('波形错误')
        if not math.isfinite(phase) or not -180<=phase<=180:raise ValueError('相位超出 ±180°')
        for c in [f':CHANnel{ch}:OUTPut OFF']:
            v.write(c)
    for g in gens:
        ch=g['ch'];hz=float(g['hz']);vpp=float(g['vpp']);wave=g.get('wave','SINe');phase=float(g.get('phase',0));on=bool(g.get('on',True))
        for field,value in [('AMPLitude:UNIT','VPP'),('BASE:WAVe',wave),
                            ('BASE:FREQuency','%d'%hz),('BASE:AMPLitude','%.7f'%vpp),
                            ('BASE:OFFSet','0'),('BASE:PHASe','%.3f'%phase),
                            ('MODE','CONTinue'),('OUTPut','ON' if on else 'OFF')]:
            v.write(f':CHANnel{ch}:{field} {value}')
    # 回读核验：频率/幅度/相位/输出状态必须与下发值一致
    checked=[]
    for g in gens:
        ch=g['ch'];after=settings(v,ch)
        hz=float(g['hz']);vpp=float(g['vpp']);phase=float(g.get('phase',0));on=bool(g.get('on',True))
        prob=[]
        if abs(float(after['BASE:FREQuency'])-hz)>max(.01,hz*1e-7):prob.append('频率')
        if abs(float(after['BASE:AMPLitude'])-vpp)>1e-6:prob.append('幅度')
        if abs(float(after['BASE:PHASe'])-phase)>0.01:prob.append('相位')
        if after['OUTPut']!=('1' if on else '0'):prob.append('输出状态')
        if after['BASE:WAVe'].upper()!=wave.upper():prob.append('波形')
        if prob:raise RuntimeError(f'CH{ch} 回读不一致：'+ '、'.join(prob) + '（'+json.dumps(after,ensure_ascii=False)+'）')
        checked.append({'ch':ch,'hz':hz,'vpp':vpp,'phase':phase,'on':on,'wave':wave})
    return checked

def generator(req,apply=False):
    ch=req.get('channel',2)
    if type(ch)!=int or ch not in (1,2):raise ValueError('通道错误')
    commands=[]
    if apply:
        hz=number(req,'frequency',1,30000000);amp=number(req,'amplitude',.001,.5);phase=number(req,'phase',-180,180)
        mode=req.get('mode');wave=req.get('wave')
        if mode not in ('CONTinue','AM','FM','PM') or wave not in ('SINe','SQUare','RAMP'):raise ValueError('模式或波形错误')
        if type(req.get('output'))!=bool:raise ValueError('输出状态错误')
        commands=[('OUTPut','OFF'),('AMPLitude:UNIT','VPP'),('BASE:WAVe',wave),('BASE:FREQuency',hz),('BASE:AMPLitude',amp),('BASE:OFFSet',0),('BASE:PHASe',phase)]
        if mode!='CONTinue':
            rate=number(req,'mod_frequency',.1,100000);dev=number(req,'deviation',0,100 if mode=='AM' else 180 if mode=='PM' else 100000)
            commands += [('MODulate:TYPe',mode),('MODulate:SOURce','INTernal'),('MODulate:WAVe','SINe'),('MODulate:FREQuency',rate),({'AM':'MODulate:DEPTh','FM':'FM:FREQuency:DEV','PM':'PM:PHASe:DEV'}[mode],dev)]
        commands += [('MODE','CONTinue' if mode=='CONTinue' else 'MODulation'),('OUTPut','ON' if req['output'] else 'OFF')]
    with sender.LOCK:
        if sender.STATE['busy']:raise RuntimeError('发生器正在执行任务，请等待完成或停止图片发送')
        sender.STATE['busy']=True
    v=None;log=[]
    try:
        v=Visa(log)
        before=settings(v,ch)
        if apply:
            try:
                for field,value in commands:v.write(f':CHANnel{ch}:{field} {value}')
                after=settings(v,ch)
                expected_mode='CONTINUE' if mode=='CONTinue' else 'MODULATION'
                if abs(float(after['BASE:FREQuency'])-hz)>max(.01,hz*1e-7) or abs(float(after['BASE:AMPLitude'])-amp)>1e-6 or after['OUTPut']!=('1' if req['output'] else '0') or after['BASE:WAVe'].upper()!=wave.upper() or after['MODE'].upper()!=expected_mode or abs(float(after['BASE:PHASe'])-phase)>0.01:raise RuntimeError('参数回读不匹配')
                if mode!='CONTinue':
                    field={'AM':'MODulate:DEPTh','FM':'FM:FREQuency:DEV','PM':'PM:PHASe:DEV'}[mode]
                    if after['MODulate:TYPe'].upper()!=mode or abs(float(after['MODulate:FREQuency'])-rate)>max(.001,rate*1e-6) or abs(float(after[field])-dev)>max(.001,dev*1e-6):raise RuntimeError('调制参数回读不匹配')
            except Exception:
                v.write(f':CHANnel{ch}:OUTPut OFF')
                raise
        else:after=before
        return dict(identity=v.query('*IDN?'),channel=ch,settings=after)
    finally:
        try:
            if v:v.close()
        finally:
            with sender.LOCK:sender.STATE['busy']=False
        folder=sender.HOME/'runs'/'control';folder.mkdir(parents=True,exist_ok=True)
        (folder/f'{time.time_ns()}.json').write_text(json.dumps(log,ensure_ascii=False,indent=2),encoding='utf-8')
def scene_apply(req):
    """按拍摄脚本的某一段，一次性下发两路发生器参数并回读核验。"""
    scenes=load_scenes()['scenes']
    idx=req.get('index')
    if type(idx)!=int or not 0<=idx<len(scenes):raise ValueError('段落序号错误')
    scene=scenes[idx]
    with sender.LOCK:
        if sender.STATE['busy']:raise RuntimeError('发生器正在执行任务，请等待完成或停止图片发送')
        sender.STATE['busy']=True
    v=None;log=[]
    try:
        v=Visa(log)
        checked=apply_scene(v,log,scene)
        return dict(ok=True,no=scene['no'],title=scene['title'],time=scene['time'],
                    page=scene['page'],note=scene['note'],shot=scene['shot'],applied=checked)
    finally:
        try:
            if v:v.close()
        finally:
            with sender.LOCK:sender.STATE['busy']=False
        folder=sender.HOME/'runs'/'control';folder.mkdir(parents=True,exist_ok=True)
        (folder/f'{time.time_ns()}.json').write_text(json.dumps(log,ensure_ascii=False,indent=2),encoding='utf-8')

def scene_quiet(req):
    """关掉一路输出（演示「拔掉一路信号」→ 读数应变「—」+相位变化超限）。"""
    ch=req.get('channel')
    if type(ch)!=int or ch not in (1,2):raise ValueError('通道错误')
    with sender.LOCK:
        if sender.STATE['busy']:raise RuntimeError('发生器正在执行任务，请等待完成或停止图片发送')
        sender.STATE['busy']=True
    v=None;log=[]
    try:
        v=Visa(log)
        v.write(f':CHANnel{ch}:OUTPut OFF')
        after=settings(v,ch)
        if after['OUTPut']!='0':raise RuntimeError(f'CH{ch} 输出未能关闭')
        return dict(ok=True,channel=ch,output=False,settings=after)
    finally:
        try:
            if v:v.close()
        finally:
            with sender.LOCK:sender.STATE['busy']=False
        folder=sender.HOME/'runs'/'control';folder.mkdir(parents=True,exist_ok=True)
        (folder/f'{time.time_ns()}.json').write_text(json.dumps(log,ensure_ascii=False,indent=2),encoding='utf-8')

def mode_apply(req):
    """段六用的调制模式预设（AM / FM / PM / 连续波）。"""
    modes=load_scenes()['modes']
    idx=req.get('index')
    if type(idx)!=int or not 0<=idx<len(modes):raise ValueError('模式序号错误')
    m=modes[idx]
    ch=m.get('ch',1)
    if ch not in (1,2):raise ValueError('通道错误')
    payload=dict(channel=ch,frequency=m['hz'],amplitude=m['vpp'],phase=0,
                 wave='SINe',mode=('CONTinue' if m['mode']=='CONT' else m['mode']),
                 mod_frequency=m['mod_freq'],deviation=m['deviation'],output=True)
    out=generator(payload,True)
    out['label']=m['label']
    return out

def _ppm_to_png(data):
    if not data.startswith(b'P6'):
        raise ValueError('屏端返回的不是 P6 PPM 图像')
    pos=2
    def token():
        nonlocal pos
        while pos<len(data):
            if data[pos] in b' \t\r\n':pos+=1;continue
            if data[pos]==ord('#'):
                end=data.find(b'\n',pos)
                if end<0:raise ValueError('PPM 注释不完整')
                pos=end+1;continue
            break
        start=pos
        while pos<len(data) and data[pos] not in b' \t\r\n#':pos+=1
        if start==pos:raise ValueError('PPM 文件头不完整')
        try:return int(data[start:pos])
        except ValueError as exc:raise ValueError('PPM 文件头格式错误') from exc
    width=token();height=token();maximum=token()
    if not 0<width<=8192 or not 0<height<=8192 or width*height>16*1024*1024 or maximum!=255:
        raise ValueError('PPM 尺寸或色深不受支持')
    if pos>=len(data) or data[pos] not in b' \t\r\n':raise ValueError('PPM 像素数据分隔符缺失')
    if data[pos:pos+2]==b'\r\n':pos+=2
    else:pos+=1
    expected=width*height*3
    if len(data)-pos!=expected:raise ValueError(f'PPM 像素数据长度错误：期望 {expected}，得到 {len(data)-pos}')
    rows=b''.join(b'\x00'+data[pos+y*width*3:pos+(y+1)*width*3] for y in range(height))
    def chunk(kind,payload):
        body=kind+payload
        return struct.pack('!I',len(payload))+body+struct.pack('!I',zlib.crc32(body)&0xffffffff)
    ihdr=struct.pack('!IIBBBBB',width,height,8,2,0,0,0)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',ihdr)+chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b'')

def screenshot():
    with SCREEN_LOCK:
        pid=adb('shell','pidof waveform_demo').decode().split()
        if len(pid)!=1 or not pid[0].isdigit():raise RuntimeError('屏幕程序必须只有一个实例')
        adb('shell',f'rm -f /tmp/wave_frame.ppm; kill -USR1 {pid[0]}')
        for _ in range(30):
            if adb('shell','test ! -f /tmp/wave_frame.ppm || wc -c < /tmp/wave_frame.ppm').strip()==b'1843216':break
            time.sleep(.1)
        else:raise RuntimeError('屏幕截图超时，请检查屏端程序')
        data=adb('exec-out','cat /tmp/wave_frame.ppm')
        png=_ppm_to_png(data)
        return dict(image='data:image/png;base64,'+base64.b64encode(png).decode())
class Handler(sender.Handler):
    def do_GET(self):
        if self.path=='/api/session':
            # base_url 用实际 Host，手机通过热点 IP 访问时返回的也是可用的地址
            host=self.headers.get('Host') or f'127.0.0.1:8770'
            self.reply(dict(api_version='1.0',token=sender.TOKEN,base_url='http://'+host,guide='/api/guide'))
        elif self.path=='/api/guide':
            data=(sender.HOME/'AI_API_GUIDE.md').read_bytes()
            self.send_response(200);self.send_header('Content-Type','text/plain; charset=utf-8');self.end_headers();self.wfile.write(data)
        elif self.path=='/':
            data=(sender.HOME/'control.html').read_text(encoding='utf-8').replace('__TOKEN__',sender.TOKEN).encode()
            self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        elif self.path=='/script':
            data=(sender.HOME/'script.html').read_text(encoding='utf-8').replace('__TOKEN__',sender.TOKEN).encode()
            self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        elif self.path=='/api/scenes':
            self.reply(load_scenes())
        elif self.path=='/images':
            self.path='/';super().do_GET()
        else:super().do_GET()
    def do_POST(self):
        if self.path not in ('/api/generator/read','/api/generator/apply','/api/screen',
                             '/api/scene/apply','/api/mode/apply','/api/scene/quiet',
                             '/api/waveforms/list','/api/waveforms/download'):
            return super().do_POST()
        if self.headers.get('X-Session-Token')!=sender.TOKEN:return self.reply({'error':'Invalid session'},403)
        try:
            n=int(self.headers.get('Content-Length',0))
            if not 0<n<=8192:raise ValueError('请求长度错误')
            req=json.loads(self.rfile.read(n))
            if self.path=='/api/screen':
                result=screenshot()
            elif self.path=='/api/waveforms/list':
                result=list_waveform_files()
            elif self.path=='/api/waveforms/download':
                send_waveform_file(self,req.get('id'))
                return
            elif self.path=='/api/scene/apply':
                result=scene_apply(req)
            elif self.path=='/api/mode/apply':
                result=mode_apply(req)
            elif self.path=='/api/scene/quiet':
                result=scene_quiet(req)
            else:
                result=generator(req,self.path.endswith('/apply'))
            self.reply(result)
        except Exception as e:
            self.reply({'error':friendly_error(e)},400)

def friendly_error(e):
    """把底层报错翻成人能看懂的话——拍摄现场没时间查 VISA 状态码。"""
    msg=str(e)
    if 'VISA status -0x4000ffef' in msg or 'VISA status' in msg and '找不到' not in msg:
        return ('信号发生器未连接电脑（USB 已断开）。'
                '请把发生器的 USB 线插回电脑，或按「手动操作卡」在仪器面板上直接设参数。')
    if 'adb' in msg.lower() or 'device' in msg.lower() and 'not found' in msg.lower():
        return ('屏端未连接电脑（USB 已断开）。抓屏功能不可用，但发生器控制不受影响。')
    if '发生器正在执行任务' in msg:
        return msg+'（图片发送进行中，等它结束或点「停止」）'
    return msg
if __name__=='__main__':
    # 默认只供本机浏览器使用；需要局域网访问时由用户显式设置绑定地址。
    bind=os.environ.get('V32_BIND','127.0.0.1')
    port=int(os.environ.get('V32_PORT','8770'))
    url=f'http://127.0.0.1:{port}/'
    http=ThreadingHTTPServer((bind,port),Handler)
    http.daemon_threads=True
    print('控制台 '+url,flush=True)
    # 只有用户显式配置外部监听时，才显示局域网访问地址。
    if bind not in ('127.0.0.1','localhost','::1'):
        try:
            s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.connect(('8.8.8.8',80));lan=s.getsockname()[0];s.close()
            print(f'局域网访问 http://{lan}:{port}/',flush=True)
        except Exception:
            pass
    webbrowser.open(url)
    try:http.serve_forever()
    except KeyboardInterrupt:sender.STOP.set()
    finally:http.server_close()
