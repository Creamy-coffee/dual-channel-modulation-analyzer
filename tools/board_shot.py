#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""board_shot.py —— 板子逐页截图（换页 -> SIGUSR1 -> 拉回 PPM -> 转 PNG）

原理：
  1) 换页不用真手指，直接跑 `./waveform_demo --tap X Y 150` 往 /dev/input/event0
     注入一次触摸（`--tap` 路径在 main.c 里是 lv_init() 之前就返回的，
     不碰 DRM，所以可以在主实例跑着的时候安全地执行）。
     底部导航 7 个按钮的坐标见 NAV_TAP。
  2) 截图是给主实例发 SIGUSR1，它会调 drm_display_snapshot_ppm() 写 /tmp/wave_frame.ppm。
  3) PPM(P6, 8bit RGB) 用 Pillow 转 PNG。drm_disp.c 里已经做过 BGR->RGB 交换，
     所以这里不需要再动字节序。

用法：
    python tools/board_shot.py out/shot_scope.png scope
    python tools/board_shot.py out/shot_fft.png   fft
    python tools/board_shot.py out/x.png          # 只截图，不换页
"""
import os
import sys

import paramiko

# 地址与凭据只从环境变量取，代码里不留默认值（仓库公开，写死等于泄露）
HOST = os.environ.get("BOARD_HOST")
PORT = int(os.environ.get("BOARD_PORT", "22"))
USER = os.environ.get("BOARD_USER")
PASS = os.environ.get("BOARD_PASS")


def _need(name, value):
    if not value:
        sys.exit(f"缺少环境变量 {name}：export {name}=... 后再运行")
    return value

REMOTE_PPM = "/tmp/wave_frame.ppm"

# 底部导航 7 个按钮的中心点（1024 宽七等分，每格 146.3px）。
# 页序与 src/ui.h 的 ui_page_t 一致：
#   示波器 / 频谱 / 锁相 / 重建 / 调制 / 系统 / 图片
NAV_TAP = {
    "scope":  (73,  572),
    "wave":   (73,  572),
    "fft":    (219, 572),
    "lock":   (366, 572),
    "ai":     (512, 572),      # 重建页（UI_PAGE_AI）
    "recon":  (512, 572),
    "mod":    (658, 572),      # 调制分析页（UI_PAGE_MOD）
    "system": (805, 572),
    "image":  (951, 572),      # 图片页（UI_PAGE_IMAGE）
}


def sh(cli, cmd, timeout=30):
    _, out, err = cli.exec_command(cmd, timeout=timeout)
    rc = out.channel.recv_exit_status()
    o = out.read().decode("utf-8", "replace")
    e = err.read().decode("utf-8", "replace")
    return rc, o, e


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    local_png = sys.argv[1]
    page = sys.argv[2].lower() if len(sys.argv) > 2 else None

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(_need("BOARD_HOST", HOST), port=PORT,
                username=_need("BOARD_USER", USER),
                password=_need("BOARD_PASS", PASS),
                timeout=20, look_for_keys=False, allow_agent=False)

    rc, o, _ = sh(cli, "pidof waveform_demo")
    pid = o.strip().split()
    if not pid:
        cli.close()
        sys.exit("板上没有 waveform_demo 在跑，先 ./deploy_net.sh")
    print("pid =", " ".join(pid))

    if page:
        if page not in NAV_TAP:
            cli.close()
            sys.exit("未知页面 %r，可选: %s" % (page, "/".join(NAV_TAP)))
        x, y = NAV_TAP[page]
        # 注入点击（--tap 不碰 DRM，可以和主实例并存）
        rc, o, e = sh(cli, "cd /tmp && ./waveform_demo --tap %d %d 150" % (x, y))
        print("tap ->", (o + e).strip().replace("\n", " | "))
        import time
        time.sleep(1.2)          # 等 LVGL 重绘并翻页

    sh(cli, "rm -f %s" % REMOTE_PPM)
    sh(cli, "kill -USR1 %s" % pid[0])
    import time
    time.sleep(1.0)              # 等 SIGUSR1 被处理 + 落盘

    rc, o, e = sh(cli, "ls -la %s && md5sum %s" % (REMOTE_PPM, REMOTE_PPM))
    print((o + e).strip())

    os.makedirs(os.path.dirname(os.path.abspath(local_png)), exist_ok=True)
    sftp = cli.open_sftp()
    sftp.get(REMOTE_PPM, local_png)
    sftp.close()
    cli.close()

    try:
        from PIL import Image
        im = Image.open(local_png).convert("RGB")
        png = os.path.splitext(local_png)[0] + ".png"
        im.save(png)
        if os.path.abspath(png) != os.path.abspath(local_png):
            os.remove(local_png)
        print("PNG -> %s  %s" % (png, im.size))
    except ImportError:
        print("没装 Pillow，保留 PPM: %s" % local_png)


if __name__ == "__main__":
    main()
