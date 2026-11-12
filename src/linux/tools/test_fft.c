/*
 * test_fft.c —— 在 PC 上单独验证 FFT + 假数据源（不依赖 LVGL）
 *
 * 编译（x86 主机）：
 *   gcc -O2 -Isrc -o test_fft tools/test_fft.c src/fft.c src/wave_source.c -lm
 *
 * 帧长与采样率由数据源在 init 后给出（wave_source_t::frame_len / sample_rate），
 * 不再是编译期宏；WAVE_MAX_FRAME 只作为数组上界。早期版本用的是
 * WAVE_FRAME_LEN / WAVE_SAMPLE_RATE 两个宏，接口改名后本文件没有跟着改，
 * 已按当前头文件修正。
 */
#include <stdio.h>
#include <math.h>

#include "wave_source.h"
#include "fft.h"

int main(void)
{
    float frame[WAVE_MAX_FRAME];
    float mag[WAVE_MAX_FRAME / 2];

    const wave_source_t *src = wave_source_get();
    if (!src || !src->init || !src->read_frame) {
        printf("no source\n");
        return 1;
    }
    src->init();

    const int n  = src->frame_len;
    const int fs = src->sample_rate;
    if (n <= 0 || n > WAVE_MAX_FRAME || (n & (n - 1)) != 0) {
        printf("bad frame_len %d\n", n);
        return 1;
    }
    const float df = (float)fs / (float)n;

    printf("source = %s, n = %d, fs = %d Hz, df = %.3f Hz, span = %.1f ms\n\n",
           src->name, n, fs, df, 1000.0f * n / fs);

    /* 跑 5 帧，看看三次谐波的幅度是不是在变（说明调制生效） */
    for (int f = 0; f < 5; f++) {
        const int got = src->read_frame(frame, n);
        if (got != n) { printf("read_frame returned %d\n", got); return 1; }

        fft_magnitude_spectrum(frame, n, mag);

        /* 找几个主要峰（跳过 DC 附近） */
        printf("frame %d:", f);
        int found = 0;
        for (int i = 3; i < n / 2 - 1 && found < 4; i++) {
            if (mag[i] > mag[i - 1] && mag[i] > mag[i + 1] && mag[i] > 0.08f) {
                printf("   %6.1f Hz = %.3f", (float)i * df, mag[i]);
                found++;
            }
        }
        printf("\n");
    }

    /* 峰值定位检查 */
    fft_magnitude_spectrum(frame, n, mag);
    int pk = 2;
    for (int i = 3; i < n / 2; i++) if (mag[i] > mag[pk]) pk = i;
    printf("\ntop peak: bin %d -> %.1f Hz  (mag %.3f)\n", pk, (float)pk * df, mag[pk]);

    return 0;
}
