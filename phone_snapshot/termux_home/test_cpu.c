#include <stdio.h>
#include <unistd.h>

int main() {
    for (int i = 0; i < 3; i++) {
        FILE* fp = fopen("/proc/stat", "r");
        if (!fp) { printf("cannot open /proc/stat\n"); return 1; }
        unsigned long long u, n, s, id, io, ir, so, st;
        fscanf(fp, "cpu %llu %llu %llu %llu %llu %llu %llu %llu", &u, &n, &s, &id, &io, &ir, &so, &st);
        fclose(fp);
        unsigned long long total = u + n + s + id + io + ir + so + st;
        printf("Sample %d: idle=%llu, total=%llu\n", i, id, total);
        usleep(300000);
    }
    return 0;
}
