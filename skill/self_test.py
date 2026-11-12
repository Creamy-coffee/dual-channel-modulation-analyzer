from pathlib import Path
import subprocess
import sys
import tempfile

here = Path(__file__).resolve().parent
smiley = bytes.fromhex((here / "fixtures" / "smiley_payload.hex").read_text())
if len(smiley) != 256:
    raise SystemExit("Expected 256 image bytes")
mutated = bytes([smiley[0] ^ 1]) + smiley[1:]
cases = [("standard", b"123456789", "cbf43926", 0),
         ("smiley", smiley, "32f70bed", 0),
         ("corrupted", mutated, "32f70bed", 1)]
with tempfile.TemporaryDirectory(dir=here.parent) as tmp:
    payload = Path(tmp) / "payload.bin"
    for name, data, expected, exit_code in cases:
        payload.write_bytes(data)
        run = subprocess.run([sys.executable, str(here / "check_crc32.py"),
                              str(payload), expected], check=False)
        if run.returncode != exit_code:
            raise SystemExit(f"{name}: expected exit {exit_code}, got {run.returncode}")
        print(f"{name}: PASS")
