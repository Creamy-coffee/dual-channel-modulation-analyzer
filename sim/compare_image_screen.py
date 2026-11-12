import json
from pathlib import Path
from PIL import Image

root = Path(__file__).resolve().parents[1]
reference = Image.open(root / "data/image/reference.pbm").convert("L")
screen = Image.open(root / "board/screenshots/ook_smiley_20261112.png").convert("L")
errors = [(x, y) for y in range(reference.height) for x in range(reference.width)
          if (reference.getpixel((x, y)) >= 128)
          != (screen.getpixel((289 + 6*x, 64 + 6*y)) >= 128)]
print(json.dumps({"pixels_checked": reference.width * reference.height, "mismatches": errors}))
raise SystemExit(bool(errors))
