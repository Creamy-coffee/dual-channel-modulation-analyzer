"""Local unified control; reuses image sender lease, protocol and restoration."""
import base64,io,json,math,os,socket,subprocess,time,threading,webbrowser
from pathlib import Path
from http.server import ThreadingHTTPServer
import server as sender
from instrument import Visa

ADB=Path('D:/软件/微信/all_users/head_imgs/0/微信web开发者工具/bin/adb-win/adb.exe')
SCREEN_LOCK=threading.Lock()
def adb(*args):
    r=subprocess.run([str(ADB),'-s','6ed3ab1135df1ef9',*args],capture_output=True,timeout=15)
    if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace') or r.stdout.decode(errors='replace'))
    return r.stdout
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

def screenshot():
    from PIL import Image
    with SCREEN_LOCK:
        pid=adb('shell','pidof waveform_demo').decode().split()
        if len(pid)!=1 or not pid[0].isdigit():raise RuntimeError('屏幕程序必须只有一个实例')
        adb('shell',f'rm -f /tmp/wave_frame.ppm; kill -USR1 {pid[0]}')
        for _ in range(30):
            if adb('shell','test ! -f /tmp/wave_frame.ppm || wc -c < /tmp/wave_frame.ppm').strip()==b'1843216':break
            time.sleep(.1)
        else:raise RuntimeError('屏幕截图超时，请检查屏端程序')
        data=adb('exec-out','cat /tmp/wave_frame.ppm')
        im=Image.open(io.BytesIO(data));im.load();out=io.BytesIO();im.save(out,format='PNG')
        return dict(image='data:image/png;base64,'+base64.b64encode(out.getvalue()).decode())
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
                             '/api/scene/apply','/api/mode/apply','/api/scene/quiet'):
            return super().do_POST()
        if self.headers.get('X-Session-Token')!=sender.TOKEN:return self.reply({'error':'Invalid session'},403)
        try:
            n=int(self.headers.get('Content-Length',0))
            if not 0<n<=8192:raise ValueError('请求长度错误')
            req=json.loads(self.rfile.read(n))
            if self.path=='/api/screen':
                result=screenshot()
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
    if 'adb' in msg or 'device' in msg and 'not found' in msg:
        return ('屏端未连接电脑（USB 已断开）。抓屏功能不可用，但发生器控制不受影响。')
    if '发生器正在执行任务' in msg:
        return msg+'（图片发送进行中，等它结束或点「停止」）'
    return msg
if __name__=='__main__':
    # 监听地址默认 0.0.0.0，手机通过热点 IP 也能访问；
    # 用 V32_BIND=127.0.0.1 可退回「仅本机」。
    bind=os.environ.get('V32_BIND','0.0.0.0')
    port=int(os.environ.get('V32_PORT','8770'))
    url=f'http://127.0.0.1:{port}/'
    http=ThreadingHTTPServer((bind,port),Handler)
    http.daemon_threads=True
    print('控制台 '+url,flush=True)
    # 列出本机可用的局域网/热点地址，手机直接输入其中之一
    try:
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        s.connect(('8.8.8.8',80));lan=s.getsockname()[0];s.close()
        print(f'手机访问 http://{lan}:{port}/',flush=True)
    except Exception:
        pass
    webbrowser.open(url)
    try:http.serve_forever()
    except KeyboardInterrupt:sender.STOP.set()
    finally:http.server_close()
