from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
linux = root / "src/linux"
if not (linux / "lvgl-release-v9.1/lvgl.h").is_file():
    subprocess.run(["cmake", "-E", "tar", "xzf",
                    str(linux / "third_party/lvgl-9.1.tar.gz")],
                   cwd=linux, check=True)
shutil.copyfile(root / "build/linux/cmakelists.in", linux / "CMakeLists.txt")
