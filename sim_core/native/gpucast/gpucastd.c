/* gpucastd — GPU 射线求交服务 (OpenCL)
 *
 * 用途: 手机上仿真进程跑在 proot (glibc) 里，加载不了厂商的 OpenCL 驱动 (bionic)。这个小服务用系统自带的
 *       C 库编译 (Termux: clang)，独占 GPU，仿真进程通过本机 TCP 把「几何体 + 射线」发过来，取回距离/几何体号/法向。
 *       求交算法与 sim_core/native/simcore.c 的 sc_cast_prims 相同 (有向长方体 + 圆柱 + 解析地面/屋顶)，只是 GPU 上用 float。
 *
 * 编译: cc -O2 -o gpucastd gpucastd.c -ldl -lm -lpthread        (不需要 OpenCL 头文件，运行时 dlopen libOpenCL)
 * 运行: LD_LIBRARY_PATH=/vendor/lib64 ./gpucastd [端口=8068]      (Android: 厂商库要在 LD_LIBRARY_PATH 里才允许加载)
 *       GPUCAST_LIB=/path/libOpenCL.so 指定库；GPUCAST_ANY_DEVICE=1 没有 GPU 时用任意 OpenCL 设备 (测试用，如 pocl)
 *       只监听 127.0.0.1。
 *
 * 协议 (小端)。请求头 32 字节: u32 magic 'GPC1', u32 op, u32 n, u32 flags, 4×f32 保留
 *   op=1 场景: 头后跟 i32 floor_gid, i32 ceil_gid, f64 ceil_h, n×18 f64 (与 sc_cast_prims 的 prims 相同)   → 应答头
 *   op=2 求交: 头后跟 3×f64 原点, f64 量程, n×3 f32 单位方向; flags bit0 = 要法向
 *              → 应答头, n f32 距离 (inf = 无回波), n i32 几何体号 (-1), [n×3 f32 法向]
 *   op=3 探测: → 应答头, 64 字节设备名
 *   op=5 着色数据: 头后跟 i32 ngeom, th, tw, 0; f32 x0, y0, res, 0; ngeom×3 f32 几何体颜色; th×tw×3 u8 地面贴图   → 应答头
 *   op=4 整帧相机: 头后跟 f64 原点[3], R[9] (行主序, 相机→世界), fx, fy, cx, cy, 量程, 噪声 sigma; u32 W, H, seed, 0
 *              → 应答头, W×H×3 u8 (方向生成 + 求交 + 着色 + 噪声全部在 GPU 上，着色公式同 simcore.c 的 sc_cam_shade)
 * 应答头 16 字节: u32 magic, i32 status (0 = 成功), u32 n, u32 GPU 耗时 (微秒)
 * 每个连接有自己的场景；GPU 资源全进程共用，求交串行执行。
 */
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define MAGIC 0x31435047u
#define PRIM_IN 18          /* 请求里每个几何体的 double 数 */
#define PRIM_GPU 24         /* GPU 上每个几何体的 float 数 */
#define MAX_RAYS (8u << 20)
#define MAX_PRIMS 200000u

/* ---------------------------------------------------------------- 最小 OpenCL 声明 (运行时 dlsym) */
typedef int32_t cl_int;
typedef uint32_t cl_uint;
typedef uint64_t cl_ulong;
typedef void *cl_h;
#define CL_DEVICE_TYPE_GPU 4ull
#define CL_DEVICE_TYPE_ALL 0xFFFFFFFFull
#define CL_DEVICE_NAME 0x102B
#define CL_MEM_READ_WRITE 1ull
#define CL_PROGRAM_BUILD_LOG 0x1183

static cl_int (*clGetPlatformIDs)(cl_uint, cl_h *, cl_uint *);
static cl_int (*clGetDeviceIDs)(cl_h, cl_ulong, cl_uint, cl_h *, cl_uint *);
static cl_int (*clGetDeviceInfo)(cl_h, cl_uint, size_t, void *, size_t *);
static cl_h (*clCreateContext)(const intptr_t *, cl_uint, const cl_h *, void *, void *, cl_int *);
static cl_h (*clCreateCommandQueue)(cl_h, cl_h, cl_ulong, cl_int *);
static cl_h (*clCreateProgramWithSource)(cl_h, cl_uint, const char **, const size_t *, cl_int *);
static cl_int (*clBuildProgram)(cl_h, cl_uint, const cl_h *, const char *, void *, void *);
static cl_int (*clGetProgramBuildInfo)(cl_h, cl_h, cl_uint, size_t, void *, size_t *);
static cl_h (*clCreateKernel)(cl_h, const char *, cl_int *);
static cl_h (*clCreateBuffer)(cl_h, cl_ulong, size_t, void *, cl_int *);
static cl_int (*clSetKernelArg)(cl_h, cl_uint, size_t, const void *);
static cl_int (*clEnqueueWriteBuffer)(cl_h, cl_h, cl_uint, size_t, size_t, const void *, cl_uint, const void *, void *);
static cl_int (*clEnqueueReadBuffer)(cl_h, cl_h, cl_uint, size_t, size_t, void *, cl_uint, const void *, void *);
static cl_int (*clEnqueueNDRangeKernel)(cl_h, cl_h, cl_uint, const size_t *, const size_t *, const size_t *, cl_uint, const void *, void *);
static cl_int (*clFinish)(cl_h);
static cl_int (*clReleaseMemObject)(cl_h);

static const char *KSRC =
"#define PN 24\n"
"static int hit_box(__global const float *p, float3 ol, float3 d, float *t_out, float3 *n_out) {\n"
"  float tn = -1e30f, tf = 1e30f, sn = 0.0f; int an = -1;\n"
"  for (int a = 0; a < 3; a++) {\n"
"    float3 ax = (float3)(p[3 + a], p[6 + a], p[9 + a]);\n"
"    float o = a == 0 ? ol.x : (a == 1 ? ol.y : ol.z), h = p[12 + a], dl = dot(d, ax);\n"
"    if (fabs(dl) < 1e-7f) { if (fabs(o) > h) return 0; continue; }\n"
"    float t1 = (-h - o) / dl, t2 = (h - o) / dl, s = -1.0f;\n"
"    if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; s = 1.0f; }\n"
"    if (t1 > tn) { tn = t1; an = a; sn = s; }\n"
"    if (t2 < tf) tf = t2;\n"
"    if (tn > tf) return 0;\n"
"  }\n"
"  if (tf < 0.0f) return 0;\n"
"  float t = tn;\n"
"  if (tn < 0.0f) {\n"
"    t = tf; an = -1; float best = 1e30f;\n"
"    for (int a = 0; a < 3; a++) {\n"
"      float3 ax = (float3)(p[3 + a], p[6 + a], p[9 + a]);\n"
"      float o = a == 0 ? ol.x : (a == 1 ? ol.y : ol.z), pl = o + dot(d, ax) * t, e = fabs(fabs(pl) - p[12 + a]);\n"
"      if (e < best) { best = e; an = a; sn = pl > 0.0f ? 1.0f : -1.0f; }\n"
"    }\n"
"  }\n"
"  *t_out = t;\n"
"  if (an >= 0) *n_out = sn * (float3)(p[3 + an], p[6 + an], p[9 + an]);\n"
"  return 1;\n"
"}\n"
"static int hit_cyl(__global const float *p, float3 ol, float3 d, float *t_out, float3 *n_out) {\n"
"  float r = p[12], hz = p[14];\n"
"  float3 c0 = (float3)(p[3], p[6], p[9]), c1 = (float3)(p[4], p[7], p[10]), c2 = (float3)(p[5], p[8], p[11]);\n"
"  float3 dl = (float3)(dot(d, c0), dot(d, c1), dot(d, c2));\n"
"  float best = 1e30f; float3 nl = (float3)(0.0f);\n"
"  float A = dl.x * dl.x + dl.y * dl.y, B = ol.x * dl.x + ol.y * dl.y, C = ol.x * ol.x + ol.y * ol.y - r * r;\n"
"  if (A > 1e-12f) {\n"
"    float disc = B * B - A * C;\n"
"    if (disc >= 0.0f) {\n"
"      float sq = sqrt(disc);\n"
"      for (int k = 0; k < 2; k++) {\n"
"        float t = (k == 0 ? (-B - sq) : (-B + sq)) / A;\n"
"        if (t < 0.0f || t >= best) continue;\n"
"        float z = ol.z + dl.z * t;\n"
"        if (fabs(z) > hz) continue;\n"
"        best = t; nl = (float3)((ol.x + dl.x * t) / r, (ol.y + dl.y * t) / r, 0.0f);\n"
"      }\n"
"    }\n"
"  }\n"
"  if (fabs(dl.z) > 1e-7f) {\n"
"    for (int k = 0; k < 2; k++) {\n"
"      float zc = k == 0 ? -hz : hz, t = (zc - ol.z) / dl.z;\n"
"      if (t < 0.0f || t >= best) continue;\n"
"      float x = ol.x + dl.x * t, y = ol.y + dl.y * t;\n"
"      if (x * x + y * y > r * r) continue;\n"
"      best = t; nl = (float3)(0.0f, 0.0f, k == 0 ? -1.0f : 1.0f);\n"
"    }\n"
"  }\n"
"  if (best >= 1e29f) return 0;\n"
"  *t_out = best;\n"
"  *n_out = c0 * nl.x + c1 * nl.y + c2 * nl.z;\n"
"  return 1;\n"
"}\n"
"static void trace(float3 d, __global const float *prims, const int np, const float max_range, const float oz, const float ceil_h,\n"
"                  const int floor_gid, const int ceil_gid, float *dist_o, int *gid_o, float3 *n_o) {\n"
"  float best = max_range; int g = -1; float3 n = (float3)(0.0f), nb = (float3)(0.0f);\n"
"  for (int k = 0; k < np; k++) {\n"
"    __global const float *p = prims + PN * k;\n"
"    float r = p[17], b = p[0] * d.x + p[1] * d.y + p[2] * d.z, c2 = p[21];\n"
"    if (b + r < 0.0f || b - r > best) continue;\n"
"    if (c2 - b * b > r * r * 1.01f + 1e-4f * c2) continue;\n"
"    float3 ol = (float3)(p[18], p[19], p[20]);\n"
"    float t;\n"
"    int ok = p[15] < 0.5f ? hit_box(p, ol, d, &t, &nb) : hit_cyl(p, ol, d, &t, &nb);\n"
"    if (ok && t < best) { best = t; g = (int)p[16]; n = nb; }\n"
"  }\n"
"  float dd = g >= 0 ? best : INFINITY;\n"
"  if (d.z < -1e-9f) { float tf = -oz / d.z; if (tf < dd) { dd = tf; g = floor_gid; n = (float3)(0.0f, 0.0f, 1.0f); } }\n"
"  if (d.z > 1e-9f) { float tc = (ceil_h - oz) / d.z; if (tc < dd) { dd = tc; g = ceil_gid; n = (float3)(0.0f, 0.0f, -1.0f); } }\n"
"  if (dd > max_range) dd = INFINITY;\n"
"  *dist_o = dd; *gid_o = g; *n_o = n;\n"
"}\n"
"__kernel void cast(__global const float *dirs, __global const float *prims, const int np, const float max_range,\n"
"                   const float oz, const float ceil_h, const int floor_gid, const int ceil_gid, const int want_n,\n"
"                   __global float *dist, __global int *gid, __global float *nrm) {\n"
"  int i = get_global_id(0);\n"
"  float dd; int g; float3 n;\n"
"  trace(vload3(i, dirs), prims, np, max_range, oz, ceil_h, floor_gid, ceil_gid, &dd, &g, &n);\n"
"  dist[i] = dd; gid[i] = g;\n"
"  if (want_n) vstore3(n, i, nrm);\n"
"}\n"
"static uint hash32(uint x) { x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16; return x; }\n"
"/* 整帧相机: 像素 → 方向 → 求交 → 着色 (同 simcore.c sc_cam_shade) → 噪声 → u8 RGB。cam: o(3) R(9 行主序) fx fy cx cy */\n"
"__kernel void render(__global const float *cam, __global const float *prims, const int np, const float max_range,\n"
"                     const float ceil_h, const int floor_gid, const int ceil_gid, const int W,\n"
"                     __global const float *geom_rgb, const int ngeom, __global const uchar *tex, const int th, const int tw,\n"
"                     const float tx0, const float ty0, const float tres, const float sigma, const uint seed,\n"
"                     __global uchar *out) {\n"
"  int i = get_global_id(0);\n"
"  float3 o = vload3(0, cam);\n"
"  float xo = ((float)(i % W) - cam[14]) / cam[12], yo = ((float)(i / W) - cam[15]) / cam[13];\n"
"  float3 dl = normalize((float3)(1.0f, -xo, -yo));\n"
"  float3 d = (float3)(dot(vload3(1, cam), dl), dot(vload3(2, cam), dl), dot(vload3(3, cam), dl));\n"
"  float dd; int g; float3 n;\n"
"  trace(d, prims, np, max_range, o.z, ceil_h, floor_gid, ceil_gid, &dd, &g, &n);\n"
"  float3 col = (float3)(0.55f);\n"
"  if (isfinite(dd)) {\n"
"    float3 base = (float3)(0.0f);\n"
"    if (g >= 0 && g < ngeom) base = vload3(g, geom_rgb);\n"
"    if (g == floor_gid && th > 0 && tw > 0) {\n"
"      int ix = (int)((o.x + d.x * dd - tx0) / tres), iy = (int)((o.y + d.y * dd - ty0) / tres);\n"
"      ix = clamp(ix, 0, tw - 1); iy = clamp(iy, 0, th - 1);\n"
"      base = convert_float3(vload3(iy * tw + ix, tex)) / 255.0f;\n"
"    }\n"
"    const float3 L = (float3)(0.35f, 0.25f, 0.9f) / 0.99750f;\n"
"    float lam = min(fabs(dot(n, L)), 1.0f), view = min(fabs(dot(n, d)), 1.0f);\n"
"    float k = 0.35f + 0.45f * lam + 0.2f * view, fog = exp(-dd / 45.0f);\n"
"    col = base * (k * fog) + 0.55f * (1.0f - fog);\n"
"  }\n"
"  col = clamp(col, 0.0f, 1.0f) * 255.0f;\n"
"  if (sigma > 0.0f) {\n"
"    uint h = hash32(seed ^ hash32((uint)i * 2654435761u + 1u));\n"
"    float u1 = ((float)(h >> 8) + 1.0f) / 16777217.0f; h = hash32(h + 0x9e3779b9u);\n"
"    float u2 = (float)(h >> 8) / 16777216.0f; h = hash32(h + 0x9e3779b9u);\n"
"    float u3 = ((float)(h >> 8) + 1.0f) / 16777217.0f; h = hash32(h + 0x9e3779b9u);\n"
"    float u4 = (float)(h >> 8) / 16777216.0f;\n"
"    float r1 = sqrt(-2.0f * log(u1)), r2 = sqrt(-2.0f * log(u3));\n"
"    col += sigma * (float3)(r1 * cos(6.2831853f * u2), r1 * sin(6.2831853f * u2), r2 * cos(6.2831853f * u4));\n"
"    col = clamp(col, 0.0f, 255.0f);\n"
"  }\n"
"  vstore3(convert_uchar3(col), i, out);\n"
"}\n";

/* ---------------------------------------------------------------- GPU 状态 (全进程一份，mu 保护) */
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static cl_h ctx, queue, kernel, k_render;
static cl_h b_dirs, b_prims, b_dist, b_gid, b_nrm, b_cam, b_img;
static size_t cap_rays, cap_prims, cap_img;
static char dev_name[64];
static int verbose;

static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

#define SYM(n) do { *(void **)&n = dlsym(h, #n); if (!n) { fprintf(stderr, "[gpucastd] 缺少符号 %s\n", #n); return -1; } } while (0)

static int cl_init(void) {
    const char *cands[] = {getenv("GPUCAST_LIB"), "libOpenCL.so", "/vendor/lib64/libOpenCL.so", "libOpenCL.so.1", NULL};
    void *h = NULL;
    for (int i = 0; i < 5 && !h; i++) if (cands[i]) h = dlopen(cands[i], RTLD_NOW | RTLD_GLOBAL);
    if (!h) { fprintf(stderr, "[gpucastd] 加载不了 OpenCL 库: %s\n", dlerror()); return -1; }
    SYM(clGetPlatformIDs); SYM(clGetDeviceIDs); SYM(clGetDeviceInfo); SYM(clCreateContext); SYM(clCreateCommandQueue);
    SYM(clCreateProgramWithSource); SYM(clBuildProgram); SYM(clGetProgramBuildInfo); SYM(clCreateKernel); SYM(clCreateBuffer);
    SYM(clSetKernelArg); SYM(clEnqueueWriteBuffer); SYM(clEnqueueReadBuffer); SYM(clEnqueueNDRangeKernel); SYM(clFinish);
    SYM(clReleaseMemObject);
    cl_h plats[8]; cl_uint npl = 0;
    if (clGetPlatformIDs(8, plats, &npl) != 0 || npl == 0) { fprintf(stderr, "[gpucastd] 没有 OpenCL 平台\n"); return -1; }
    cl_h dev = NULL; cl_uint nd = 0;
    for (cl_uint i = 0; i < npl && !dev; i++)
        if (clGetDeviceIDs(plats[i], CL_DEVICE_TYPE_GPU, 1, &dev, &nd) != 0 || nd == 0) dev = NULL;
    if (!dev && getenv("GPUCAST_ANY_DEVICE"))
        for (cl_uint i = 0; i < npl && !dev; i++)
            if (clGetDeviceIDs(plats[i], CL_DEVICE_TYPE_ALL, 1, &dev, &nd) != 0 || nd == 0) dev = NULL;
    if (!dev) { fprintf(stderr, "[gpucastd] 没有 GPU 设备\n"); return -1; }
    clGetDeviceInfo(dev, CL_DEVICE_NAME, sizeof(dev_name) - 1, dev_name, NULL);
    cl_int err = 0;
    ctx = clCreateContext(NULL, 1, &dev, NULL, NULL, &err);
    if (!ctx || err) { fprintf(stderr, "[gpucastd] clCreateContext %d\n", err); return -1; }
    queue = clCreateCommandQueue(ctx, dev, 0, &err);
    if (!queue || err) { fprintf(stderr, "[gpucastd] clCreateCommandQueue %d\n", err); return -1; }
    cl_h prog = clCreateProgramWithSource(ctx, 1, &KSRC, NULL, &err);
    if (!prog || err) { fprintf(stderr, "[gpucastd] clCreateProgramWithSource %d\n", err); return -1; }
    err = clBuildProgram(prog, 1, &dev, "", NULL, NULL);
    if (err) {
        static char logbuf[16384];
        clGetProgramBuildInfo(prog, dev, CL_PROGRAM_BUILD_LOG, sizeof(logbuf) - 1, logbuf, NULL);
        fprintf(stderr, "[gpucastd] 着色程序编译失败 %d:\n%s\n", err, logbuf);
        return -1;
    }
    kernel = clCreateKernel(prog, "cast", &err);
    if (!kernel || err) { fprintf(stderr, "[gpucastd] clCreateKernel %d\n", err); return -1; }
    k_render = clCreateKernel(prog, "render", &err);
    if (!k_render || err) { fprintf(stderr, "[gpucastd] clCreateKernel(render) %d\n", err); return -1; }
    b_cam = clCreateBuffer(ctx, CL_MEM_READ_WRITE, 16 * 4, NULL, &err);
    if (err) return -1;
    return 0;
}

static int ensure_bufs(size_t n, size_t np) {
    cl_int e = 0;
    if (n > cap_rays) {
        size_t c = n < 65536 ? 65536 : n + n / 4;
        if (b_dirs) { clReleaseMemObject(b_dirs); clReleaseMemObject(b_dist); clReleaseMemObject(b_gid); clReleaseMemObject(b_nrm); }
        b_dirs = clCreateBuffer(ctx, CL_MEM_READ_WRITE, c * 12, NULL, &e); if (e) return e;
        b_dist = clCreateBuffer(ctx, CL_MEM_READ_WRITE, c * 4, NULL, &e); if (e) return e;
        b_gid = clCreateBuffer(ctx, CL_MEM_READ_WRITE, c * 4, NULL, &e); if (e) return e;
        b_nrm = clCreateBuffer(ctx, CL_MEM_READ_WRITE, c * 12, NULL, &e); if (e) return e;
        cap_rays = c;
    }
    if (np > cap_prims || !b_prims) {
        size_t c = np < 256 ? 256 : np + np / 4;
        if (b_prims) clReleaseMemObject(b_prims);
        b_prims = clCreateBuffer(ctx, CL_MEM_READ_WRITE, c * PRIM_GPU * 4, NULL, &e); if (e) return e;
        cap_prims = c;
    }
    return 0;
}

/* ---------------------------------------------------------------- 连接 */
typedef struct { uint32_t magic, op, n, flags; float a[4]; } req_hdr;
typedef struct { uint32_t magic; int32_t status; uint32_t n, us; } rsp_hdr;

static int rd(int fd, void *buf, size_t len) {
    char *p = buf;
    while (len) { ssize_t k = recv(fd, p, len, 0); if (k <= 0) { if (k < 0 && errno == EINTR) continue; return -1; } p += k; len -= (size_t)k; }
    return 0;
}
static int wr(int fd, const void *buf, size_t len) {
    const char *p = buf;
    while (len) { ssize_t k = send(fd, p, len, MSG_NOSIGNAL); if (k <= 0) { if (k < 0 && errno == EINTR) continue; return -1; } p += k; len -= (size_t)k; }
    return 0;
}
static int reply(int fd, int status, uint32_t n, uint32_t us) { rsp_hdr r = {MAGIC, status, n, us}; return wr(fd, &r, sizeof(r)); }

typedef struct {
    double *prims; uint32_t np; int32_t floor_gid, ceil_gid; double ceil_h;
    float *gp; float *dirs; float *dist; int32_t *gid; float *nrm; size_t cap;
    cl_h b_rgb, b_tex; int32_t ngeom, th, tw; float tx0, ty0, tres;      /* 着色数据 (op=5)，每个连接自己的 GPU 缓冲 */
    unsigned char *img; size_t img_cap;
} conn_state;

/* 原点变换到各几何体局部系 (double)，剔除整体超出量程的几何体 → s->gp；返回保留的几何体数 */
static uint32_t pack_prims(conn_state *s, const double *o, double maxr) {
    uint32_t m = 0;
    for (uint32_t k = 0; k < s->np; k++) {
        const double *p = s->prims + PRIM_IN * k, *R = p + 3;
        double oc[3] = {o[0] - p[0], o[1] - p[1], o[2] - p[2]};
        double c2 = oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2];
        if (sqrt(c2) - p[17] > maxr) continue;
        float *g = s->gp + PRIM_GPU * m++;
        g[0] = (float)-oc[0]; g[1] = (float)-oc[1]; g[2] = (float)-oc[2];           /* 中心 - 原点 */
        for (int a = 3; a < PRIM_IN; a++) g[a] = (float)p[a];
        for (int a = 0; a < 3; a++) g[18 + a] = (float)(oc[0] * R[a] + oc[1] * R[3 + a] + oc[2] * R[6 + a]);
        g[21] = (float)c2; g[22] = g[23] = 0.0f;
    }
    return m;
}

static int do_cast(int fd, conn_state *s, uint32_t n, int want_n) {
    double om[4];
    if (rd(fd, om, sizeof(om))) return -1;
    if (n > s->cap) {
        size_t c = n + n / 4;
        free(s->dirs); free(s->dist); free(s->gid); free(s->nrm);
        s->dirs = malloc(c * 12); s->dist = malloc(c * 4); s->gid = malloc(c * 4); s->nrm = malloc(c * 12);
        s->cap = (s->dirs && s->dist && s->gid && s->nrm) ? c : 0;
        if (!s->cap) return -1;
    }
    if (rd(fd, s->dirs, (size_t)n * 12)) return -1;
    double t0 = now_s();
    double maxr = om[3];
    uint32_t m = pack_prims(s, om, maxr);
    int st = 0;
    pthread_mutex_lock(&mu);
    st = ensure_bufs(n, m);
    if (!st) {
        cl_int np_i = (cl_int)m, fg = s->floor_gid, cg = s->ceil_gid, wn = want_n;
        float mr = (float)maxr, oz = (float)om[2], ch = (float)s->ceil_h;
        size_t gs = n;
        st |= clEnqueueWriteBuffer(queue, b_dirs, 0, 0, (size_t)n * 12, s->dirs, 0, NULL, NULL);
        if (m) st |= clEnqueueWriteBuffer(queue, b_prims, 0, 0, (size_t)m * PRIM_GPU * 4, s->gp, 0, NULL, NULL);
        st |= clSetKernelArg(kernel, 0, sizeof(cl_h), &b_dirs);
        st |= clSetKernelArg(kernel, 1, sizeof(cl_h), &b_prims);
        st |= clSetKernelArg(kernel, 2, sizeof(cl_int), &np_i);
        st |= clSetKernelArg(kernel, 3, sizeof(float), &mr);
        st |= clSetKernelArg(kernel, 4, sizeof(float), &oz);
        st |= clSetKernelArg(kernel, 5, sizeof(float), &ch);
        st |= clSetKernelArg(kernel, 6, sizeof(cl_int), &fg);
        st |= clSetKernelArg(kernel, 7, sizeof(cl_int), &cg);
        st |= clSetKernelArg(kernel, 8, sizeof(cl_int), &wn);
        st |= clSetKernelArg(kernel, 9, sizeof(cl_h), &b_dist);
        st |= clSetKernelArg(kernel, 10, sizeof(cl_h), &b_gid);
        st |= clSetKernelArg(kernel, 11, sizeof(cl_h), &b_nrm);
        if (!st) st = clEnqueueNDRangeKernel(queue, kernel, 1, NULL, &gs, NULL, 0, NULL, NULL);
        if (!st) st = clEnqueueReadBuffer(queue, b_dist, 0, 0, (size_t)n * 4, s->dist, 0, NULL, NULL);
        if (!st) st = clEnqueueReadBuffer(queue, b_gid, want_n ? 0 : 1, 0, (size_t)n * 4, s->gid, 0, NULL, NULL);
        if (!st && want_n) st = clEnqueueReadBuffer(queue, b_nrm, 1, 0, (size_t)n * 12, s->nrm, 0, NULL, NULL);
        if (st) clFinish(queue);
    }
    pthread_mutex_unlock(&mu);
    uint32_t us = (uint32_t)((now_s() - t0) * 1e6);
    if (verbose) fprintf(stderr, "[gpucastd] cast n=%u prims=%u/%u %u us st=%d\n", n, m, s->np, us, st);
    if (st) return reply(fd, st, 0, us);
    if (reply(fd, 0, n, us) || wr(fd, s->dist, (size_t)n * 4) || wr(fd, s->gid, (size_t)n * 4)) return -1;
    if (want_n && wr(fd, s->nrm, (size_t)n * 12)) return -1;
    return 0;
}

typedef struct { double o[3], R[9], fx, fy, cx, cy, max_range, sigma; uint32_t W, H, seed, pad; } render_req;

/* op=5: 着色数据。头后跟 i32 ngeom, i32 th, i32 tw, i32 0, f32 x0, y0, res, 0, 然后 ngeom×3 f32 颜色, th×tw×3 u8 地面贴图 */
static int do_shade_data(int fd, conn_state *s) {
    struct { int32_t ngeom, th, tw, z; float x0, y0, res, z2; } m;
    if (rd(fd, &m, sizeof(m))) return -1;
    if (m.ngeom < 0 || m.ngeom > 1000000 || m.th < 0 || m.tw < 0 || (uint64_t)m.th * (uint64_t)m.tw > (64u << 20)) return -1;
    size_t nrgb = (size_t)m.ngeom * 12, ntex = (size_t)m.th * (size_t)m.tw * 3;
    float *rgb = malloc(nrgb ? nrgb : 4); unsigned char *tex = malloc(ntex ? ntex : 4);
    int st = -1;
    if (rgb && tex && rd(fd, rgb, nrgb) == 0 && rd(fd, tex, ntex) == 0) {
        cl_int e = 0;
        pthread_mutex_lock(&mu);
        if (s->b_rgb) clReleaseMemObject(s->b_rgb);
        if (s->b_tex) clReleaseMemObject(s->b_tex);
        s->b_rgb = clCreateBuffer(ctx, CL_MEM_READ_WRITE, nrgb ? nrgb : 12, NULL, &e);
        st = e;
        if (!st) { s->b_tex = clCreateBuffer(ctx, CL_MEM_READ_WRITE, ntex ? ntex : 4, NULL, &e); st = e; }
        if (!st && nrgb) st = clEnqueueWriteBuffer(queue, s->b_rgb, 1, 0, nrgb, rgb, 0, NULL, NULL);
        if (!st && ntex) st = clEnqueueWriteBuffer(queue, s->b_tex, 1, 0, ntex, tex, 0, NULL, NULL);
        pthread_mutex_unlock(&mu);
        s->ngeom = m.ngeom; s->th = m.th; s->tw = m.tw; s->tx0 = m.x0; s->ty0 = m.y0; s->tres = m.res;
        st = reply(fd, st, (uint32_t)m.ngeom, 0) ? -1 : 0;
    }
    free(rgb); free(tex);
    return st;
}

/* op=4: 整帧相机 → W×H×3 u8 */
static int do_render(int fd, conn_state *s) {
    render_req q;
    if (rd(fd, &q, sizeof(q))) return -1;
    size_t n = (size_t)q.W * (size_t)q.H;
    if (n == 0 || n > MAX_RAYS || !s->gp || !s->b_rgb || !s->b_tex) return -1;
    if (n * 3 > s->img_cap) { free(s->img); s->img = malloc(n * 3); s->img_cap = s->img ? n * 3 : 0; if (!s->img) return -1; }
    double t0 = now_s();
    uint32_t m = pack_prims(s, q.o, q.max_range);
    float cam[16];
    for (int a = 0; a < 3; a++) cam[a] = (float)q.o[a];
    for (int a = 0; a < 9; a++) cam[3 + a] = (float)q.R[a];
    cam[12] = (float)q.fx; cam[13] = (float)q.fy; cam[14] = (float)q.cx; cam[15] = (float)q.cy;
    int st = 0;
    pthread_mutex_lock(&mu);
    st = ensure_bufs(1, m);
    if (!st && n * 3 > cap_img) {
        cl_int e = 0;
        if (b_img) clReleaseMemObject(b_img);
        b_img = clCreateBuffer(ctx, CL_MEM_READ_WRITE, n * 3 + n * 3 / 4, NULL, &e);
        cap_img = e ? 0 : n * 3 + n * 3 / 4; st = e;
    }
    if (!st) {
        cl_int np_i = (cl_int)m, fg = s->floor_gid, cg = s->ceil_gid, W = (cl_int)q.W, ng = s->ngeom, th = s->th, tw = s->tw;
        float mr = (float)q.max_range, ch = (float)s->ceil_h, sg = (float)q.sigma;
        size_t gs = n;
        st |= clEnqueueWriteBuffer(queue, b_cam, 0, 0, sizeof(cam), cam, 0, NULL, NULL);
        if (m) st |= clEnqueueWriteBuffer(queue, b_prims, 0, 0, (size_t)m * PRIM_GPU * 4, s->gp, 0, NULL, NULL);
        st |= clSetKernelArg(k_render, 0, sizeof(cl_h), &b_cam);
        st |= clSetKernelArg(k_render, 1, sizeof(cl_h), &b_prims);
        st |= clSetKernelArg(k_render, 2, sizeof(cl_int), &np_i);
        st |= clSetKernelArg(k_render, 3, sizeof(float), &mr);
        st |= clSetKernelArg(k_render, 4, sizeof(float), &ch);
        st |= clSetKernelArg(k_render, 5, sizeof(cl_int), &fg);
        st |= clSetKernelArg(k_render, 6, sizeof(cl_int), &cg);
        st |= clSetKernelArg(k_render, 7, sizeof(cl_int), &W);
        st |= clSetKernelArg(k_render, 8, sizeof(cl_h), &s->b_rgb);
        st |= clSetKernelArg(k_render, 9, sizeof(cl_int), &ng);
        st |= clSetKernelArg(k_render, 10, sizeof(cl_h), &s->b_tex);
        st |= clSetKernelArg(k_render, 11, sizeof(cl_int), &th);
        st |= clSetKernelArg(k_render, 12, sizeof(cl_int), &tw);
        st |= clSetKernelArg(k_render, 13, sizeof(float), &s->tx0);
        st |= clSetKernelArg(k_render, 14, sizeof(float), &s->ty0);
        st |= clSetKernelArg(k_render, 15, sizeof(float), &s->tres);
        st |= clSetKernelArg(k_render, 16, sizeof(float), &sg);
        st |= clSetKernelArg(k_render, 17, sizeof(cl_uint), &q.seed);
        st |= clSetKernelArg(k_render, 18, sizeof(cl_h), &b_img);
        if (!st) st = clEnqueueNDRangeKernel(queue, k_render, 1, NULL, &gs, NULL, 0, NULL, NULL);
        if (!st) st = clEnqueueReadBuffer(queue, b_img, 1, 0, n * 3, s->img, 0, NULL, NULL);
        if (st) clFinish(queue);
    }
    pthread_mutex_unlock(&mu);
    uint32_t us = (uint32_t)((now_s() - t0) * 1e6);
    if (verbose) fprintf(stderr, "[gpucastd] render %ux%u prims=%u/%u %u us st=%d\n", q.W, q.H, m, s->np, us, st);
    if (st) return reply(fd, st, 0, us);
    if (reply(fd, 0, (uint32_t)n, us) || wr(fd, s->img, n * 3)) return -1;
    return 0;
}

static void *serve(void *arg) {
    int fd = (int)(intptr_t)arg, one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    conn_state s; memset(&s, 0, sizeof(s));
    req_hdr h;
    while (rd(fd, &h, sizeof(h)) == 0) {
        if (h.magic != MAGIC) break;
        if (h.op == 3) {
            if (reply(fd, 0, 0, 0) || wr(fd, dev_name, sizeof(dev_name))) break;
        } else if (h.op == 1) {
            struct { int32_t fg, cg; double ch; } m;
            if (h.n > MAX_PRIMS || rd(fd, &m, sizeof(m))) break;
            free(s.prims); free(s.gp);
            s.prims = malloc(sizeof(double) * PRIM_IN * (h.n ? h.n : 1));
            s.gp = malloc(sizeof(float) * PRIM_GPU * (h.n ? h.n : 1));
            if (!s.prims || !s.gp || rd(fd, s.prims, sizeof(double) * PRIM_IN * h.n)) break;
            s.np = h.n; s.floor_gid = m.fg; s.ceil_gid = m.cg; s.ceil_h = m.ch;
            if (reply(fd, 0, h.n, 0)) break;
        } else if (h.op == 2) {
            if (h.n == 0 || h.n > MAX_RAYS || !s.gp) break;
            if (do_cast(fd, &s, h.n, (int)(h.flags & 1))) break;
        } else if (h.op == 5) {
            if (do_shade_data(fd, &s)) break;
        } else if (h.op == 4) {
            if (do_render(fd, &s)) break;
        } else break;
    }
    close(fd);
    pthread_mutex_lock(&mu);
    if (s.b_rgb) clReleaseMemObject(s.b_rgb);
    if (s.b_tex) clReleaseMemObject(s.b_tex);
    pthread_mutex_unlock(&mu);
    free(s.prims); free(s.gp); free(s.dirs); free(s.dist); free(s.gid); free(s.nrm); free(s.img);
    return NULL;
}

int main(int argc, char **argv) {
    int port = argc > 1 ? atoi(argv[1]) : 8068;
    verbose = getenv("GPUCAST_VERBOSE") != NULL;
    signal(SIGPIPE, SIG_IGN);
    if (cl_init()) return 2;
    int ls = socket(AF_INET, SOCK_STREAM, 0), one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a; memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(ls, (struct sockaddr *)&a, sizeof(a)) || listen(ls, 16)) { perror("[gpucastd] bind/listen"); return 3; }
    fprintf(stderr, "[gpucastd] 设备 %s，监听 127.0.0.1:%d\n", dev_name, port);
    for (;;) {
        int fd = accept(ls, NULL, NULL);
        if (fd < 0) { if (errno == EINTR) continue; perror("[gpucastd] accept"); break; }
        pthread_t th; pthread_attr_t at;
        pthread_attr_init(&at); pthread_attr_setdetachstate(&at, PTHREAD_CREATE_DETACHED);
        if (pthread_create(&th, &at, serve, (void *)(intptr_t)fd)) close(fd);
        pthread_attr_destroy(&at);
    }
    return 0;
}
