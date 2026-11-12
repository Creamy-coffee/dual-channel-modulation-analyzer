import argparse
from pathlib import Path
import zlib

def check(payload, expected):
    actual = zlib.crc32(payload) & 0xffffffff
    print(f"CRC32={actual:08x} expected={expected:08x}")
    return actual == expected

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("payload", type=Path)
    parser.add_argument("expected", type=lambda s: int(s, 16))
    args = parser.parse_args()
    raise SystemExit(0 if check(args.payload.read_bytes(), args.expected) else 1)
