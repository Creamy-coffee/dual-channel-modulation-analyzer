"""Open the existing image sender or run it. Opening the page never starts RF."""
import json
import runpy
import sys
import urllib.error
import urllib.request
import webbrowser
from pathlib import Path

URL = 'http://127.0.0.1:8768/'

def main():
    try:
        with urllib.request.urlopen(URL + 'api/status', timeout=2) as response:
            status = json.load(response)
        with urllib.request.urlopen(URL, timeout=2) as response:
            page = response.read().decode('utf-8')
    except urllib.error.HTTPError:
        print('Port 8768 is used by another service. Nothing was changed.')
        return 1
    except (urllib.error.URLError, TimeoutError, OSError):
        sys.argv = [str(Path(__file__).with_name('server.py')), '--port', '8768']
        try:
            runpy.run_path(sys.argv[0], run_name='__main__')
        except OSError as exc:
            print('Unable to start image sender:', exc)
            return 1
        return 0
    except (ValueError, UnicodeError):
        print('Unexpected response from port 8768. Nothing was changed.')
        return 1
    if not isinstance(status, dict) or 'busy' not in status or '载波图片发送' not in page:
        print('Port 8768 is not the image sender. Nothing was changed.')
        return 1
    print('Image sender is already running. Opening ' + URL)
    webbrowser.open(URL)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
