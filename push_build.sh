#!/bin/bash
# push_build.sh —— 一键「推源码到构建机 -> 交叉编译 -> 把产物拉回 out/」
#
# 为什么需要它：
#   本机（Windows）没有交叉工具链，也刻意不放 LVGL 源码（10MB+），
#   编译在一台 Linux 构建机上做（地址见 BUILD_HOST）。
#   手工三件事（sftp_put / ssh build / sftp_get）很容易漏掉某一步，
#   导致「改了代码但跑的还是旧二进制」——所以固化成脚本。
#
# 用法（waveform_demo/ 目录下）：
#   ./push_build.sh            增量编译
#   CLEAN=1 ./push_build.sh    全量重编（LVGL 从头编，约 1 分钟）
#
# 产物：out/waveform_demo
set -e
set -o pipefail   # 管道里 build.sh 失败也要让整个脚本失败，别再把旧产物当成功拉回来
cd "$(dirname "$0")"

# 构建机地址、账号、口令一律从环境变量取，脚本里不留任何默认值。
#   export BUILD_HOST=192.0.2.10  BUILD_USER=you  BUILD_PASS=******
#   export REMOTE_DIR=/home/you/waveform_demo/project
: "${BUILD_HOST:?请先 export BUILD_HOST=<构建机地址>}"
: "${BUILD_USER:?请先 export BUILD_USER=<构建机账号>}"
: "${BUILD_PASS:?请先 export BUILD_PASS=<构建机口令，或改用 SSH 公钥>}"
REMOTE_DIR=${REMOTE_DIR:-/tmp/waveform_demo_project}

# 需要一个装了 paramiko 的 python；PYBIN 指路径，或直接落到 PATH 里的 python
PYBIN=${PYBIN:-python}
command -v "$PYBIN" >/dev/null 2>&1 || { echo "找不到 python，请设 PYBIN=<python 路径>" >&2; exit 127; }
"$PYBIN" -c "import paramiko" 2>/dev/null \
  || { echo "该 python 没有 paramiko：pip install paramiko，或用 PYBIN 指到装好的解释器" >&2; exit 1; }

export SSH_HOST="$BUILD_HOST" SSH_USER="$BUILD_USER" SSH_PASS="$BUILD_PASS"
# ⚠️ 远程路径以 / 开头，不关掉 MSYS 路径转换会被改写成
#    "E:/Program Files/.../remote/dir" 这种鬼东西（踩过）
export MSYS_NO_PATHCONV=1

# build.sh 把产物写到源码树的上一级 out/，所以远端产物在 $(dirname REMOTE_DIR)/out
REMOTE_OUT=${REMOTE_OUT:-$(dirname "$REMOTE_DIR")/out}

echo "==> 1/3 推源码 -> $BUILD_USER@$BUILD_HOST:$REMOTE_DIR"
# 本地 tools/ 里的生成脚本和字体也一起推，保证两边可复现
"$PYBIN" ../scripts/sftp_put.py . "$REMOTE_DIR" | tail -3

echo
echo "==> 2/3 交叉编译"
CLEAN_ENV=""
[ "x$CLEAN" = "x1" ] && CLEAN_ENV="CLEAN=1"
"$PYBIN" ../scripts/ssh_run.py - 900 <<EOF
cd $REMOTE_DIR
$CLEAN_ENV ./build.sh 2>&1 | tail -40
EOF

echo
echo "==> 3/3 拉回产物"
"$PYBIN" ../scripts/sftp_get.py "$REMOTE_OUT/waveform_demo" out/waveform_demo

echo
echo "================ 本地产物 ================"
ls -la out/waveform_demo
md5sum out/waveform_demo
