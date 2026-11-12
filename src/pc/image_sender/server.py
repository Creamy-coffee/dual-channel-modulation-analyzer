"""Local UTG2062X image sender. Python standard library only, Windows VISA64."""
import argparse,base64,datetime,hashlib,json,math,os,secrets,threading,time,webbrowser,zlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from protocol import make_packets,symbols_for
from instrument import Visa,FIELDS,bsv,write_raw,arb_select,restore

HOME=Path(__file__).resolve().parent
TOKEN=secrets.token_urlsafe(24)
VERSION='20261104-slotfix'
LOCK=threading.Lock();STOP=threading.Event()
STATE={'busy':False,'phase':'空闲','progress':0,'error':None,'restored':None}

def update(**kwargs):
    with LOCK:STATE.update(kwargs)

def validate(req):
    mode=req.get('mode');size=req.get('size');rounds=req.get('rounds',1)
    if mode not in ('ook','fsk','bpsk') or type(size)!=int or size not in (32,64,128,256):raise ValueError('制式或尺寸错误')
    height=req.get('height',size)
    if type(height)!=int or height not in (size,size//2,size*2) or height>256 or size*height>32768:raise ValueError('尺寸上限为128×256或256×128，支持1:1、2:1、1:2')
    if type(rounds)!=int or not 1<=rounds<=3:raise ValueError('轮数应为1..3')
    pixels=base64.b64decode(req.get('pixels',''),validate=True)
    if len(pixels)!=size*height//8:raise ValueError('图片字节数错误')
    return mode,size,rounds,pixels

def pick_image_id(req):
    """Optional fixed image id so repeated sends accumulate on one image.

    The receiver keeps a per-image bitmask of received chunks and resets it
    whenever the image id changes.  With a random id every send starts the
    image from scratch, so a lossy link can never finish a whole image.
    Passing image_id lets a retry fill in only the chunks that are still
    missing, and the receiver then reaches its whole-image CRC.
    """
    raw=req.get('image_id')
    if raw is None:return secrets.randbits(32)
    if isinstance(raw,str):
        try:raw=int(raw,16)
        except ValueError:raise ValueError('image_id 应为十六进制字符串')
    if type(raw)!=int or not 0<=raw<=0xFFFFFFFF:raise ValueError('image_id 超出 32 位范围')
    return raw

def transmit(req,visa_factory=Visa):
    v=None;original=extra=None;log=[];out=None;restore_ok=None
    try:
        mode,size,rounds,pixels=validate(req)
        image_id=pick_image_id(req)
        height=req.get('height',size)
        packets=make_packets(pixels,size,height,image_id)
        out=HOME/'runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');out.mkdir(parents=True)
        (out/'image.pbm').write_bytes(f'P4\n{size} {height}\n'.encode()+pixels)
        manifest={'mode':mode,'size':size,'height':height,'rounds':rounds,'image_id':f'{image_id:08x}',
                  'crc32':f'{zlib.crc32(pixels):08x}','sha256':hashlib.sha256(pixels).hexdigest(),
                  'packets':len(packets),'baud':1000,'carrier_hz':1000000,'hardware_channel':2}
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        update(phase='连接发生器并保存原设置',report=str(out),**manifest)
        p=':CHANnel2:';v=visa_factory(log)
        original={f:v.query(p+f+'?') for f in FIELDS}
        extra={f:v.query(p+f+'?') for f in ('MODulate:ARB','BASE:ARB','ARB:MODe','ARB:FILTer','BASE:ARB:SAMPling','PM:PHASe:DEV')}
        (out/'original.json').write_text(json.dumps({'fields':original,'extra':extra},indent=2))
        if original['AMPLitude:UNIT'].upper()!='VPP' or not 0<float(original['BASE:AMPLitude'])<=.5:
            raise ValueError('当前幅度单位需为Vpp且幅度须在0..0.5V范围；未开始发送')
        if STOP.is_set():return
        v.write(p+'OUTPut OFF')
        slot_path=HOME/'slots.json'
        names=json.loads(slot_path.read_text()) if slot_path.exists() else [f'v32img{i:02d}.bsv' for i in range(4)]
        if len(names)!=4 or len(set(names))!=4 or any(not isinstance(n,str) or not n.endswith('.bsv') or not n.replace('.','').replace('_','').isalnum() for n in names):raise ValueError('slots.json需要四个不同的测试波形文件名')
        # 上一轮发送结束时 restore() 会把 MODulate:ARB 还原成某个测试槽，于是
        # 下一次发送必然命中“原设置正在使用发送槽”而永久卡死。这里改成先解除占用：
        # 把对应的波形源切回内部正弦（发送流程稍后自己会设回 ARB），再覆盖测试槽。
        # 此刻 OUTPut 已关，且这些槽本来就是本工具的测试位，不会毁掉别人的波形。
        busy_slots={f for f in ('MODulate:ARB','BASE:ARB')
                    for n in names
                    if extra[f].strip('"').upper().startswith('USER,') and extra[f].strip('"').endswith(n)}
        if 'MODulate:ARB' in busy_slots:v.write(p+'MODulate:WAVe SINe')
        if 'BASE:ARB' in busy_slots:v.write(p+'BASE:WAVe SINe')
        if busy_slots:(out/'slot_reuse.txt').write_text('已切回内部正弦后复用测试槽：'+','.join(sorted(busy_slots))+'\n')
        waveforms=[]
        for i,packet in enumerate(packets):
            if STOP.is_set():return
            symbols=symbols_for([packet])+bytes([0,1,1,0])*112
            assert len(symbols)==2048
            data=bsv([32767 if b else -32767 for b in symbols for _ in range(2)])
            (out/f'packet{i:02d}.bsv').write_bytes(data);waveforms.append(data)
        for f,val in [('BASE:WAVe','SINe'),('AMPLitude:UNIT','Vpp'),('BASE:FREQuency','1000000'),
                      ('BASE:OFFSet','0'),('BASE:AMPLitude','0.1666667' if mode=='ook' else '0.3333333'),
                      ('MODulate:TYPe',{'ook':'AM','fsk':'FM','bpsk':'PM'}[mode]),('MODulate:SOURce','INTernal'),
                      ('MODulate:WAVe','ARB'),('MODulate:FREQuency','0.48828125'),('MODE','MODulation')]:v.write(p+f+' '+val)
        v.write(p+{'ook':'MODulate:DEPTh 100','fsk':'FM:FREQuency:DEV 1000','bpsk':'PM:PHASe:DEV 90'}[mode])
        events=[];total=len(packets)*rounds
        for turn in range(rounds):
            for start in range(0,len(packets),4):
                if STOP.is_set():return
                v.write(p+'OUTPut OFF')
                batch=waveforms[start:start+4]
                for slot,data in enumerate(batch):
                    if STOP.is_set():return
                    name=names[slot]
                    update(phase=f'上传第 {turn+1}/{rounds} 轮 · 包 {start+slot+1}/{len(packets)}',progress=round(100*(turn*len(packets)+start)/total,1))
                    v.write(':WARB2:MODulate "'+name+'"');write_raw(v,data)
                    arb_select(v,p,'MODulate:ARB','USER,'+name)
                    actual=v.query(p+'MODulate:ARB?')
                    if actual.strip().upper()!=('USER,'+name).upper():raise RuntimeError(f'波形未选中：期望 USER,{name}，实际 {actual}。可能用户波形存储已满；请使用可覆盖的测试槽')
                for slot in range(len(batch)):
                    if STOP.is_set():return
                    name=names[slot];arb_select(v,p,'MODulate:ARB','USER,'+name)
                    check={f:v.query(p+f+'?') for f in ('BASE:FREQuency','BASE:AMPLitude','MODulate:ARB','MODulate:FREQuency','MODulate:WAVe','MODE','MODulate:TYPe')}
                    events.append({'round':turn+1,'packet':start+slot,'readback':check});(out/'source_readback.json').write_text(json.dumps(events,indent=2))
                    expected_type={'ook':'AM','fsk':'FM','bpsk':'PM'}[mode]
                    if float(check['BASE:FREQuency'])!=1000000 or abs(float(check['BASE:AMPLitude'])-(.1666667 if mode=='ook' else .3333333))>1e-6 or check['MODulate:ARB'].upper()!=('USER,'+name).upper() or abs(float(check['MODulate:FREQuency'])-.48828125)>1e-7 or check['MODulate:WAVe'].upper()!='ARB' or check['MODulate:TYPe'].upper()!=expected_type or check['MODE'].upper()!='MODULATION':raise RuntimeError('发生器参数回读不匹配：'+json.dumps(check,ensure_ascii=False))
                    v.write(p+'OUTPut ON')
                    update(phase=f'发送第 {turn+1}/{rounds} 轮 · 包 {start+slot+1}/{len(packets)}',progress=round(100*(turn*len(packets)+start+slot)/total,1))
                    if STOP.wait(6.144):return
        update(phase='停止中' if STOP.is_set() else '发送完成，正在恢复设置')
    except Exception as exc:
        update(error=str(exc),phase='发送失败，正在恢复设置')
    finally:
        try:
            if v and original and extra:
                result=restore(v,original,extra);restore_ok=result['verified']
                if out:(out/'restored.json').write_text(json.dumps(result,indent=2))
                if not restore_ok:raise RuntimeError('原设置恢复回读不一致，请检查发生器')
        except Exception as exc:
            restore_ok=False;update(error='恢复失败：'+str(exc))
            if v:
                try:v.write(':CHANnel2:OUTPut OFF')
                except Exception:pass
        finally:
            if v:
                try:v.close()
                except Exception as exc:update(error='关闭VISA失败：'+str(exc))
            if out:(out/'visa_log.json').write_text(json.dumps(log,indent=2))
            with LOCK:
                STATE.update(busy=False,restored=restore_ok,phase='失败，请查看错误' if STATE.get('error') else '已停止并恢复' if STOP.is_set() else '已发送并恢复',progress=STATE['progress'] if STOP.is_set() or STATE.get('error') else 100)

def friendly_error(exc):
    """把底层报错翻成人话——拍摄现场没时间查 VISA 状态码。"""
    msg=str(exc)
    if 'VISA status' in msg:
        return ('信号发生器未连接电脑（USB 已断开）。'
                '请把发生器的 USB 线插回电脑后重试。')
    return msg

class Handler(BaseHTTPRequestHandler):
    def reply(self,obj,status=200):
        data=json.dumps(obj,ensure_ascii=False).encode();self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
    def do_GET(self):
        if self.path=='/':
            data=(HOME/'index.html').read_text(encoding='utf-8').replace('__TOKEN__',TOKEN).replace('__VERSION__',VERSION).encode()
            self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        elif self.path=='/api/status':
            with LOCK:s=dict(STATE)
            s['version']=VERSION
            s['pid']=os.getpid()
            self.reply(s)
        else:self.reply({'error':'Not found'},404)
    def do_POST(self):
        if self.headers.get('X-Session-Token')!=TOKEN:return self.reply({'error':'Invalid session'},403)
        try:
            length=int(self.headers.get('Content-Length',0))
            if not 0<length<=20000:raise ValueError('请求过大或为空')
            req=json.loads(self.rfile.read(length))
            if self.path=='/api/start':
                validate(req)
                with LOCK:
                    if STATE['busy']:return self.reply({'error':'已有发送任务，请先停止'},409)
                    STATE.clear();STATE.update(busy=True,phase='准备',progress=0,error=None,restored=None)
                    STOP.clear()
                threading.Thread(target=transmit,args=(req,),daemon=False).start();self.reply({'ok':True})
            elif self.path=='/api/stop':STOP.set();self.reply({'ok':True})
            else:self.reply({'error':'Not found'},404)
        except Exception as exc:self.reply({'error':friendly_error(exc)},400)
    def log_message(self,*args):pass

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--port',type=int,default=8768);ap.add_argument('--bind',default=os.environ.get('V32_BIND','127.0.0.1'));ap.add_argument('--no-browser',action='store_true');args=ap.parse_args()
    server=ThreadingHTTPServer((args.bind,args.port),Handler)
    print(f'图片发送工具 http://{args.bind}:{args.port} · Ctrl+C停止并恢复',flush=True)
    if not args.no_browser:webbrowser.open(f'http://127.0.0.1:{args.port}')
    try:server.serve_forever()
    except KeyboardInterrupt:STOP.set();print('正在停止发送并恢复发生器，请勿强制关闭窗口。',flush=True)
    finally:server.server_close()
