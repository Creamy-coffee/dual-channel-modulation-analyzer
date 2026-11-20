"""Forward live scope records to VOFA+ or manage an offline CSV file."""
import argparse
import csv
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
import socket
import select
import struct
import subprocess
import zlib


def read_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        block = sock.recv(size-len(data))
        if not block:
            raise ConnectionError('screen disconnected')
        data.extend(block)
    return data


def firewater_sample(ch1, ch2, scale=None):
    """Format one FireWater sample: one numeric value per channel."""
    if scale:
        return f'{-ch1*scale[0]:.8g},{-ch2*scale[1]:.8g}\n'
    return f'{-ch1},{-ch2}\n'


def forward_live(client, upstream_port, scale):
    frames = 0
    with socket.create_connection(('127.0.0.1', upstream_port), timeout=1) as source:
        while True:
            ready, _, _ = select.select([client, source], [], [], 1)
            if client in ready and not client.recv(1024):
                return
            if source not in ready:
                continue
            header = read_exact(source,40)
            if header[:8]!=b'V32CAP3!' or struct.unpack_from('<I',header,8)[0]!=3:
                raise ValueError('invalid CAP3 header')
            points = struct.unpack_from('<I',header,24)[0]
            if points not in (256,512,1024,2048,4096):
                raise ValueError('invalid CAP3 point count')
            payload = read_exact(source,4*points+4)
            if zlib.crc32(header+payload[:-4])!=struct.unpack_from('<I',payload,4*points)[0]:
                raise ValueError('CAP3 checksum mismatch')
            lines = ['nan,nan\n'] if frames else []
            for a,b in struct.iter_unpack('<hh',payload[:-4]):
                lines.append(firewater_sample(a, b, scale))
            client.sendall(''.join(lines).encode('ascii'))
            frames += 1
            if frames==1 or frames%100==0:
                print(f'LIVE: {frames} validated records forwarded',flush=True)


def serve_live(adb, port, scale):
    with socket.create_server(('127.0.0.1',port)) as server:
        upstream=int(subprocess.check_output([*adb,'forward','tcp:0','localabstract:scope_raw_live'],text=True,timeout=5).strip())
        print(f'LIVE VOFA+ TCP client: 127.0.0.1:{port}, FireWater',flush=True)
        try:
            while True:
                client,_=server.accept()
                with client:
                    client.settimeout(2)
                    try:
                        forward_live(client,upstream,scale)
                    except (OSError,ValueError) as exc:
                        print(f'LIVE connection ended: {exc}',flush=True)
        finally:
            subprocess.run([*adb,'forward','--remove',f'tcp:{upstream}'],check=False,timeout=5)


def firewater(path, scale=None):
    with path.open(newline='', encoding='utf-8') as file:
        rows = csv.DictReader(line for line in file if not line.startswith('#'))
        for index, row in enumerate(rows):
            if index >= 4096 or int(row['sample_index']) != index:
                raise ValueError('CSV does not contain one ordered CAP3 record')
            a, b = int(row['ch1_adc_code']), int(row['ch2_adc_code'])
            if not all(-2048 <= code <= 2047 for code in (a,b)):
                raise ValueError('ADC code out of range')
            # Display in the same input polarity as the 2026-11-17 screen.
            yield firewater_sample(a, b, scale).encode('ascii')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, nargs='?', help='offline CSV for calibration or destination for --pull')
    parser.add_argument('--live', action='store_true', help='forward new records from the screen continuously over ADB (default)')
    parser.add_argument('--pull', action='store_true', help='fetch the last CSV exported from the screen')
    parser.add_argument('--adb', default='adb', help='ADB executable path')
    parser.add_argument('--serial', help='ADB device serial number')
    parser.add_argument('--port', type=int, default=8771)
    parser.add_argument('--probe', nargs=2, type=float, metavar=('CH1','CH2'),
                        help='apply these probe factors and send volts using the nominal scale')
    parser.add_argument('--record', type=Path, help='write a calibration observation JSON instead of serving TCP')
    parser.add_argument('--reference-mvpp',type=float)
    parser.add_argument('--measured-mvpp',type=float)
    parser.add_argument('--channel',type=int,choices=(1,2),default=1)
    args = parser.parse_args()
    if args.probe and not all(math.isfinite(x) and 1 <= x <= 100 for x in args.probe):
        parser.error('probe factors must be between 1 and 100')
    factors=tuple(x*.002441 for x in args.probe) if args.probe else None
    adb=[args.adb]+(['-s',args.serial] if args.serial else [])
    if args.live and (args.csv or args.pull or args.record):
        parser.error('--live cannot be combined with a file, --pull or --record')
    if not (args.csv or args.pull or args.record):
        serve_live(adb,args.port,factors)
        return
    if args.csv is None:
        parser.error('--pull and --record require a CSV path')
    if not (args.pull or args.record):
        parser.error('TCP only sends live records; use --pull to copy an exported CSV or --record for calibration')
    if args.pull:
        subprocess.run([*adb,'pull','/tmp/scope_history.csv',str(args.csv)], check=True)
        if not args.record:
            return
    frames = list(firewater(args.csv,factors))
    if not frames:
        parser.error('CSV contains no samples')
    if args.record:
        ref,meas=args.reference_mvpp,args.measured_mvpp
        if ref is None or meas is None or not all(math.isfinite(x) and x>0 for x in (ref,meas)):
            parser.error('--record needs positive --reference-mvpp and --measured-mvpp')
        raw=args.csv.read_bytes()
        entry=dict(timestamp=datetime.now().astimezone().isoformat(),capture=str(args.csv.resolve()),
                   sha256=hashlib.sha256(raw).hexdigest(),header=raw.splitlines()[1].decode('ascii'),
                   channel=args.channel,probe_factors=args.probe or [1,1],reference_mvpp=ref,
                   measured_mvpp=meas,relative_difference_pct=(meas/ref-1)*100)
        with args.record.open('x',encoding='utf-8') as file:
            json.dump(entry,file,indent=2)


if __name__ == '__main__':
    main()
