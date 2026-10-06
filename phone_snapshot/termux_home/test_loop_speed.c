#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

int main() {
    uint8_t* yuv = malloc(640 * 640 * 3 / 2);
    float* r = malloc(640 * 640 * sizeof(float));
    float* g = malloc(640 * 640 * sizeof(float));
    float* b = malloc(640 * 640 * sizeof(float));

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int iter = 0; iter < 100; iter++) {
        const uint8_t* y_plane = yuv;
        const uint8_t* u_plane = yuv + 640*640;
        const uint8_t* v_plane = yuv + 640*640 + 640*640/4;
        for (int y = 0; y < 640; y++) {
            int uv_row = (y >> 1) * 320;
            int y_row = y * 640;
            for (int x = 0; x < 640; x++) {
                int idx = y_row + x;
                float Y = (float)y_plane[idx];
                int uv_idx = uv_row + (x >> 1);
                float U = (float)u_plane[uv_idx] - 128.0f;
                float V = (float)v_plane[uv_idx] - 128.0f;
                r[idx] = (Y + 1.402f * V) * (1.0f / 255.0f);
                g[idx] = (Y - 0.344136f * U - 0.714136f * V) * (1.0f / 255.0f);
                b[idx] = (Y + 1.772f * U) * (1.0f / 255.0f);
            }
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double ms = ((t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0) / 100.0;
    printf("Real YUV420 to float RGB conversion time in C: %.2f ms (sample=%.2f)\n", ms, r[100]+g[100]+b[100]);
    return 0;
}
