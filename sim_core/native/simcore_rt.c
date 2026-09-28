/*
 * libsimcore 实时部分 —— 仿真固定步长循环在原生线程里运行 (不持有 Python GIL)
 *
 *   物理/接触/射线全部由 MuJoCo 自己的 C 函数完成 (mj_step / mj_multiRay，经 dlopen 取 wheel 自带的 libmujoco)，
 *   本文件只把原先 Python 里围绕 MuJoCo 的"壳"下沉到 C:
 *     定步长累加器调度 → 看门狗/急停/触边运动封锁 → 轮系运动学 (sc_kin_step) → 轮地打滑 → 写控制量 → mj_step →
 *     读位姿/接触 → 触边 (车体多边形 vs 场景线段，项目自有模型) / 光电 (mj_multiRay) → 编码器里程计 → IMU →
 *     2D 激光按频率扫描 (mj_multiRay + 噪声/丢点) 与 360° 融合，双缓冲输出
 *   未安装 MuJoCo (兜底 kinematic 后端) 时: SE2 积分 + 线段碰撞/射线 (与原 numpy 兜底实现一致)。
 *   3D 激光 (Livox 类非重复扫描) 同样在这里扫描 + 高度带切片并入融合扫描。
 *   Python (sim_core/rt.py) 负责配置、读写状态快照、行人移动、IO/事件、相机。
 *
 *   SC_WITH_MUJOCO: 编译时有 MuJoCo 头文件 (wheel 自带 include/)；运行期 mj_version() 必须与头文件版本一致才启用。
 */
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef SC_WITH_MUJOCO
#include <dlfcn.h>
#include <mujoco/mujoco.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- simcore.c 中的函数与结构 */
typedef struct {
    int kind, _pad;
    double x, y, radius, steer_min, steer_max, steer_rate, max_speed, drive_tau, gear_ratio;
    double steer, steer_target, speed, speed_target, cmd_speed, angle, motor_rpm, current_a, torque_nm, steer_current_a;
} sc_wheel;
typedef struct {
    double max_speed, max_accel, max_decel, max_ang_speed, max_ang_accel, max_ang_decel, mass;
    double axle_x, kt, rolling_coeff, efficiency;
    int holonomic, _pad;
    double shaped[3];
    double vx, vy, wz, slip_residual, saturation;
} sc_chassis;
void sc_kin_step(sc_chassis *ch, sc_wheel *w, int n, double cvx, double cvy, double cwz, double dt, int brake, double *out);
void sc_se2_integrate(double x, double y, double th, double vx, double vy, double wz, double dt, double *out);
int sc_collides(const double *fp, int np, double x, double y, double th, const double *segs, int ns, double *out_pt);
void sc_raycast2d(const double *segs, int ns, const double *circles, int nc, double zmin, double ox, double oy,
                  const double *angles, int n, double max_range, double *out);
void sc_odom_update(const sc_wheel *w, int n, int holonomic, double axle_x, const double *radius_err, const double *steer_bias,
                    double steer_sigma, double cpr, double dt, uint64_t *rng, double *pose);
void sc_imu_sample(double *st, const double *par, double vx, double vy, double wz, double dt, uint64_t *rng, double *out);
void sc_slip(double *prev, double *v, double dt, uint64_t *rng);
void sc_merge_add(double *bins, int nb, const double *r, int n, double mx, double my, double yaw, double sign, double a0, double inc);
void sc_merge_finish(double *bins, int nb, double rmax);
void sc_lidar_post(double *r, int n, double rmin, int noise, double std, double prop, double dropout, uint64_t *rng);
double sc_wrap(double a);

/* ================================================================== MuJoCo 绑定 */
#ifdef SC_WITH_MUJOCO
typedef void (*fn_step)(const mjModel *, mjData *);
typedef void (*fn_multiray)(const mjModel *, mjData *, const mjtNum *, const mjtNum *, const mjtByte *, mjtBool, int, int *,
                            mjtNum *, mjtNum *, int, mjtNum);
typedef int (*fn_version)(void);
static fn_step p_mj_step = NULL;
static fn_multiray p_mj_multiray = NULL;
#endif

/* 返回 0 = 成功；1 = 编译时无 MuJoCo；2 = dlopen 失败；3 = 版本不一致 */
int sc_mj_bind(const char *libpath) {
#ifdef SC_WITH_MUJOCO
    void *h = dlopen(libpath, RTLD_NOW | RTLD_NOLOAD);
    if (!h) h = dlopen(libpath, RTLD_NOW);
    if (!h) return 2;
    fn_version v = (fn_version)dlsym(h, "mj_version");
    p_mj_step = (fn_step)dlsym(h, "mj_step");
    p_mj_multiray = (fn_multiray)dlsym(h, "mj_multiRay");
    if (!v || !p_mj_step || !p_mj_multiray) return 2;
    if (v() != mjVERSION_HEADER) {
        p_mj_step = NULL;
        p_mj_multiray = NULL;
        return 3;
    }
    return 0;
#else
    (void)libpath;
    return 1;
#endif
}

int sc_mj_header_version(void) {
#ifdef SC_WITH_MUJOCO
    return mjVERSION_HEADER;
#else
    return 0;
#endif
}

/* 水平射线 (MuJoCo): 原点 (ox,oy,z)，世界系角度 angles[n]；未命中/超量程 = inf。scratch 需 ≥ 4n 个 double + n 个 int */
static void mj_rays2d(void *m, void *d, const unsigned char *group, int bodyexclude, double ox, double oy, double z,
                      const double *angles, int n, double maxr, double *vec, int *gid, double *out) {
#ifdef SC_WITH_MUJOCO
    for (int i = 0; i < n; i++) {
        vec[3 * i] = cos(angles[i]);
        vec[3 * i + 1] = sin(angles[i]);
        vec[3 * i + 2] = 0.0;
    }
    double pnt[3] = {ox, oy, z};
    p_mj_multiray((const mjModel *)m, (mjData *)d, pnt, vec, group, 1, bodyexclude, gid, out, NULL, n, maxr);
    for (int i = 0; i < n; i++)
        if (gid[i] < 0 || out[i] < 0 || out[i] > maxr) out[i] = INFINITY;   /* 与 MuJoCoBackend.cast 一致 */
#else
    (void)m; (void)d; (void)group; (void)bodyexclude; (void)ox; (void)oy; (void)z; (void)angles; (void)maxr; (void)vec; (void)gid;
    for (int i = 0; i < n; i++) out[i] = INFINITY;
#endif
}

/* 供 Python 线程直接调用 (m/d 为 MjModel._address / MjData._address，d 须为调用线程独占) */
int sc_mj_raycast2d(void *m, void *d, const unsigned char *group, int bodyexclude, double ox, double oy, double z,
                    const double *angles, int n, double maxr, double *out) {
#ifdef SC_WITH_MUJOCO
    if (!p_mj_multiray) return -1;
    double *vec = (double *)malloc(sizeof(double) * 3 * (size_t)n);
    int *gid = (int *)malloc(sizeof(int) * (size_t)n);
    if (!vec || !gid) { free(vec); free(gid); return -1; }
    mj_rays2d(m, d, group, bodyexclude, ox, oy, z, angles, n, maxr, vec, gid, out);
    free(vec);
    free(gid);
    return 0;
#else
    (void)m; (void)d; (void)group; (void)bodyexclude; (void)ox; (void)oy; (void)z; (void)angles; (void)n; (void)maxr; (void)out;
    return -1;
#endif
}

/* ================================================================== 实时循环 */
#define RT_MAXW 32
#define RT_MAXP 16
#define RT_MAXB 8
#define RT_MAXL 8

typedef struct {   /* 与 sim_core/rt.py 的 RtState 一致 */
    double t, x, y, th, vx, vy, wz;
    double odom[6];
    double imu[5];
    double imu_st[4];
    double slip_prev[3];
    double cmd[3];
    double cmd_time;
    double last_contact[2];
    double step_ms, lidar_ms, mj_step_ms, rtf, rtf_target;
    double bumper_contact_t[RT_MAXB];
    double photo_dist[RT_MAXP];
    uint32_t collisions, overruns, steps, discrete_n;
    int32_t has_contact, paused, brake, _pad;
    int32_t bumper_pressed[RT_MAXB];
    int32_t bumper_count[RT_MAXB];
    int32_t photo_detected[RT_MAXP];
} sc_rt_state;

typedef struct {   /* 与 sim_core/rt.py 的 RtConfig 一致 */
    double dt, cmd_timeout, head, tail, steer_sigma, cpr, merged_rmax, merged_period;
    double imu_par[3];
    int32_t max_substeps, noise, use_mj, nbins;
    int32_t robot_body, lidars_on, _pad0, _pad1;
    unsigned char group[8];
} sc_rt_config;

typedef struct {
    double mx, my, mz, yaw, sign, a0, inc, rmax, rmin, std, prop, dropout, period;
    int32_t n, _pad;
} sc_rt_lidar_cfg;

typedef struct {
    sc_rt_lidar_cfg c;
    double next;
    double *buf[2];
    int front;
    uint32_t seq;
    double t, pose[3];
} rt_lidar;

/* 3D 激光 (Livox 类非重复扫描，与 sim_core/sensors.py Lidar3DSensor 一致)；与 rt.py 的 RtLidar3D 一致 */
typedef struct {
    double mx, my, mz, R[9], vmin, vmax, rmin, rmax, std, ang_noise, period, ceiling, zmin, zmax, frame;
    int32_t n, lines;
} sc_rt_l3d_cfg;

typedef struct {
    sc_rt_l3d_cfg c;
    double next, frame;
    float *xyz[2], *inten[2];
    unsigned char *line[2];
    double *ot[2], *slice[2];
    int cnt[2], front;
    uint32_t seq;
    double t, pose[3];
} rt_l3d;

#define RT_MAXL3 4

typedef struct {
    pthread_mutex_t mu;
    pthread_t th;
    int started, stop;
    sc_rt_config cfg;
    sc_rt_state s;
    double acc, last_wall, rtf_t0, rtf_sim;
    /* 外部内存 (Python 持有) */
    sc_chassis *ch;
    sc_wheel *w;
    int nw;
    const double *radius_err, *steer_bias;
    const double *segs;
    int ns;
    const double *fp;
    int nfp;
    void *m, *d;
    const int *robot_geom;     /* [ngeom] 1 = 车体 geom */
    int ngeom;
    /* 开关量 */
    int nphoto;
    double photo[RT_MAXP][8];  /* x y z yaw half range trigger hyst */
    int nbump;
    double bpoly[RT_MAXB][32];
    int bnp[RT_MAXB], bside[RT_MAXB];
    double bhold[RT_MAXB];
    /* 激光 */
    int nlidar;
    rt_lidar lid[RT_MAXL];
    double *merged[2];
    int mfront;
    uint32_t mseq;
    double mt, mpose[3], mnext;
    double *scratch;
    int *iscratch;
    int scratch_n;
    int nl3d, nbins;
    rt_l3d l3d[RT_MAXL3];
    double *s3;                /* 3D 射线暂存: 方向 3n (世界) + 3n (传感器) + 距离 n */
    int *is3;
    int s3_n;
    uint64_t rng[6];
} sc_rt;

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

sc_rt *sc_rt_create(uint64_t seed) {
    sc_rt *rt = (sc_rt *)calloc(1, sizeof(sc_rt));
    if (!rt) return NULL;
    pthread_mutex_init(&rt->mu, NULL);
    extern void sc_rng_seed(uint64_t *, uint64_t);
    sc_rng_seed(rt->rng, seed);
    rt->s.rtf = 1.0;
    rt->s.rtf_target = 1.0;
    return rt;
}

int sc_rt_sizeof_state(void) { return (int)sizeof(sc_rt_state); }
int sc_rt_sizeof_config(void) { return (int)sizeof(sc_rt_config); }
int sc_rt_sizeof_lidar(void) { return (int)sizeof(sc_rt_lidar_cfg); }
int sc_rt_sizeof_l3d(void) { return (int)sizeof(sc_rt_l3d_cfg); }

void sc_rt_lock(sc_rt *rt) { pthread_mutex_lock(&rt->mu); }
void sc_rt_unlock(sc_rt *rt) { pthread_mutex_unlock(&rt->mu); }

/* 以下 set/get 须在 sc_rt_lock 之内调用 (Python 侧 hold()) */
void sc_rt_set_config(sc_rt *rt, const sc_rt_config *c) { rt->cfg = *c; }
void sc_rt_get_state(sc_rt *rt, sc_rt_state *s) { *s = rt->s; }
void sc_rt_set_state(sc_rt *rt, const sc_rt_state *s) { rt->s = *s; }

void sc_rt_set_kin(sc_rt *rt, sc_chassis *ch, sc_wheel *w, int nw, const double *radius_err, const double *steer_bias) {
    rt->ch = ch; rt->w = w; rt->nw = nw; rt->radius_err = radius_err; rt->steer_bias = steer_bias;
}

void sc_rt_set_geom(sc_rt *rt, const double *segs, int ns, const double *fp, int nfp) {
    rt->segs = segs; rt->ns = ns; rt->fp = fp; rt->nfp = nfp;
}

void sc_rt_set_mj(sc_rt *rt, void *m, void *d, const int *robot_geom, int ngeom) {
    rt->m = m; rt->d = d; rt->robot_geom = robot_geom; rt->ngeom = ngeom;
}

void sc_rt_set_photos(sc_rt *rt, const double *P, int n) {
    rt->nphoto = n > RT_MAXP ? RT_MAXP : n;
    for (int i = 0; i < rt->nphoto; i++) memcpy(rt->photo[i], P + 8 * i, sizeof(double) * 8);
}

/* polys: 各条带顶点拼接；counts/sides(0 前 1 后 2 左 3 右)/holds */
void sc_rt_set_bumpers(sc_rt *rt, const double *polys, const int *counts, const int *sides, const double *holds, int n) {
    rt->nbump = n > RT_MAXB ? RT_MAXB : n;
    for (int i = 0; i < rt->nbump; i++) {
        int k = counts[i] > 16 ? 16 : counts[i];
        memcpy(rt->bpoly[i], polys, sizeof(double) * 2 * k);
        rt->bnp[i] = k;
        rt->bside[i] = sides[i];
        rt->bhold[i] = holds[i];
        polys += 2 * counts[i];
    }
}

void sc_rt_set_lidars(sc_rt *rt, const sc_rt_lidar_cfg *L, int n, int nbins) {
    for (int i = 0; i < rt->nlidar; i++) { free(rt->lid[i].buf[0]); free(rt->lid[i].buf[1]); }
    free(rt->merged[0]); free(rt->merged[1]);
    free(rt->scratch); free(rt->iscratch);
    rt->nlidar = n > RT_MAXL ? RT_MAXL : n;
    int maxn = 16;
    for (int i = 0; i < rt->nlidar; i++) {
        rt_lidar *l = &rt->lid[i];
        uint32_t seq = l->seq;
        memset(l, 0, sizeof(*l));
        l->c = L[i];
        l->seq = seq;
        l->buf[0] = (double *)calloc((size_t)L[i].n, sizeof(double));
        l->buf[1] = (double *)calloc((size_t)L[i].n, sizeof(double));
        if (L[i].n > maxn) maxn = L[i].n;
    }
    if (nbins < 1) nbins = 1;
    rt->merged[0] = (double *)calloc((size_t)nbins, sizeof(double));
    rt->merged[1] = (double *)calloc((size_t)nbins, sizeof(double));
    rt->scratch = (double *)malloc(sizeof(double) * 5 * (size_t)maxn);
    rt->iscratch = (int *)malloc(sizeof(int) * (size_t)maxn);
    rt->scratch_n = maxn;
    rt->mnext = 0.0;
    rt->nbins = nbins;
}

static void l3d_free(sc_rt *rt) {
    for (int i = 0; i < rt->nl3d; i++)
        for (int b = 0; b < 2; b++) {
            rt_l3d *l = &rt->l3d[i];
            free(l->xyz[b]); free(l->inten[b]); free(l->line[b]); free(l->ot[b]); free(l->slice[b]);
        }
    free(rt->s3); free(rt->is3);
    rt->s3 = NULL; rt->is3 = NULL;
    rt->nl3d = 0;
}

/* 须在 sc_rt_set_lidars 之后调用 (融合分箱数) */
void sc_rt_set_lidars3d(sc_rt *rt, const sc_rt_l3d_cfg *L, int n) {
    uint32_t seqs[RT_MAXL3] = {0};
    for (int i = 0; i < rt->nl3d; i++) seqs[i] = rt->l3d[i].seq;
    l3d_free(rt);
    rt->nl3d = n > RT_MAXL3 ? RT_MAXL3 : n;
    int maxn = 16;
    for (int i = 0; i < rt->nl3d; i++) {
        rt_l3d *l = &rt->l3d[i];
        memset(l, 0, sizeof(*l));
        l->c = L[i];
        l->seq = seqs[i];
        l->frame = L[i].frame;
        for (int b = 0; b < 2; b++) {
            l->xyz[b] = (float *)calloc(3 * (size_t)L[i].n, sizeof(float));
            l->inten[b] = (float *)calloc((size_t)L[i].n, sizeof(float));
            l->line[b] = (unsigned char *)calloc((size_t)L[i].n, 1);
            l->ot[b] = (double *)calloc((size_t)L[i].n, sizeof(double));
            l->slice[b] = (double *)calloc((size_t)(rt->nbins > 0 ? rt->nbins : 1), sizeof(double));
            for (int k = 0; k < rt->nbins; k++) l->slice[b][k] = INFINITY;
        }
        if (L[i].n > maxn) maxn = L[i].n;
    }
    rt->s3 = (double *)malloc(sizeof(double) * 7 * (size_t)maxn);
    rt->is3 = (int *)malloc(sizeof(int) * (size_t)maxn);
    rt->s3_n = maxn;
}

/* 读 3D 激光: 点数写 *cnt (缓冲按 cap 截断)；seq ≤ after 时返回 0。frame 返回已扫描帧数 (Python 重建模型时续上) */
int sc_rt_read_lidar3d(sc_rt *rt, int i, uint32_t after, float *xyz, float *inten, unsigned char *line, double *ot, double *slice,
                       int cap, int nbins, int *cnt, uint32_t *seq, double *t, double *pose, double *frame) {
    if (i < 0 || i >= rt->nl3d) return 0;
    rt_l3d *l = &rt->l3d[i];
    *frame = l->frame;
    if (l->seq <= after) return 0;
    int f = l->front, m = l->cnt[f] < cap ? l->cnt[f] : cap;
    memcpy(xyz, l->xyz[f], sizeof(float) * 3 * (size_t)m);
    memcpy(inten, l->inten[f], sizeof(float) * (size_t)m);
    memcpy(line, l->line[f], (size_t)m);
    memcpy(ot, l->ot[f], sizeof(double) * (size_t)m);
    memcpy(slice, l->slice[f], sizeof(double) * (size_t)(nbins < rt->nbins ? nbins : rt->nbins));
    *cnt = m;
    *seq = l->seq; *t = l->t; memcpy(pose, l->pose, sizeof(double) * 3);
    return 1;
}

/* 读激光: i = -1 表示融合扫描。seq ≤ after 时返回 0 */
int sc_rt_read_lidar(sc_rt *rt, int i, uint32_t after, double *out, int n, uint32_t *seq, double *t, double *pose) {
    double *src;
    int len;
    if (i < 0) {
        if (rt->mseq <= after || !rt->merged[rt->mfront]) return 0;
        src = rt->merged[rt->mfront];
        len = rt->cfg.nbins;
        *seq = rt->mseq; *t = rt->mt; memcpy(pose, rt->mpose, sizeof(double) * 3);
    } else {
        if (i >= rt->nlidar) return 0;
        rt_lidar *l = &rt->lid[i];
        if (l->seq <= after) return 0;
        src = l->buf[l->front];
        len = l->c.n;
        *seq = l->seq; *t = l->t; memcpy(pose, l->pose, sizeof(double) * 3);
    }
    memcpy(out, src, sizeof(double) * (size_t)(len < n ? len : n));
    return 1;
}

/* ---------------------------------------------------------------- 单步 (SimCore.step 的逐项移植) */
static void bumper_apply(sc_rt *rt, int i, int hit, int forced) {
    sc_rt_state *s = &rt->s;
    if (hit || forced) {
        if (!s->bumper_pressed[i]) s->bumper_count[i]++;
        s->bumper_pressed[i] = 1;
        s->bumper_contact_t[i] = s->t;
    } else if (s->bumper_pressed[i] && s->t - s->bumper_contact_t[i] > rt->bhold[i]) {
        s->bumper_pressed[i] = 0;
    }
}

static int any_pressed(sc_rt *rt) {
    for (int i = 0; i < rt->nbump; i++)
        if (rt->s.bumper_pressed[i]) return 1;
    return 0;
}

static void on_collision(sc_rt *rt, double px, double py) {
    sc_rt_state *s = &rt->s;
    if (!any_pressed(rt)) s->collisions++;
    double c = cos(s->th), sn = sin(s->th);
    double bx = c * (px - s->x) + sn * (py - s->y);
    int front = bx >= (rt->cfg.head - rt->cfg.tail) / 2.0;
    s->last_contact[0] = px;
    s->last_contact[1] = py;
    s->has_contact = 1;
    int side = front ? 0 : 1;
    for (int i = 0; i < rt->nbump; i++)
        if (rt->bside[i] == side) bumper_apply(rt, i, 1, 1);
}

static void kin_stop(sc_rt *rt) {
    sc_chassis *ch = rt->ch;
    ch->shaped[0] = ch->shaped[1] = ch->shaped[2] = 0.0;
    ch->vx = ch->vy = ch->wz = 0.0;
    for (int i = 0; i < rt->nw; i++) rt->w[i].speed = rt->w[i].speed_target = rt->w[i].cmd_speed = 0.0;
}

static void photos_update(sc_rt *rt) {
    sc_rt_state *s = &rt->s;
    double c = cos(s->th), sn = sin(s->th);
    for (int i = 0; i < rt->nphoto; i++) {
        const double *p = rt->photo[i];
        double ox = s->x + c * p[0] - sn * p[1], oy = s->y + sn * p[0] + c * p[1];
        double a = s->th + p[3];
        double ang[3] = {a - p[4], a, a + p[4]}, d[3];
        if (rt->cfg.use_mj) {
            double vec[9];
            int gid[3];
            mj_rays2d(rt->m, rt->d, rt->cfg.group, rt->cfg.robot_body, ox, oy, fmax(0.005, p[2]), ang, 3, p[5], vec, gid, d);
        } else {
            sc_raycast2d(rt->segs, rt->ns, NULL, 0, p[2], ox, oy, ang, 3, p[5], d);
        }
        double dist = fmin(d[0], fmin(d[1], d[2]));
        s->photo_dist[i] = dist;
        double thr = p[6] + (s->photo_detected[i] ? p[7] : 0.0);
        s->photo_detected[i] = dist <= thr;
    }
}

static void discrete_update(sc_rt *rt) {
    sc_rt_state *s = &rt->s;
    double pt[2];
    for (int i = 0; i < rt->nbump; i++) {
        int was = s->bumper_pressed[i];
        int hit = rt->ns > 0 ? sc_collides(rt->bpoly[i], rt->bnp[i], s->x, s->y, s->th, rt->segs, rt->ns, pt) : 0;
        bumper_apply(rt, i, hit, 0);
        if (s->bumper_pressed[i] && !was) {
            s->collisions++;
            s->last_contact[0] = s->x;
            s->last_contact[1] = s->y;
            s->has_contact = 1;
        }
    }
    s->discrete_n++;
    if (s->discrete_n % 2 == 0) photos_update(rt);
}

static void rt_step(sc_rt *rt) {
    sc_rt_state *s = &rt->s;
    sc_rt_config *cf = &rt->cfg;
    double dt = cf->dt, t0 = now_s();
    int watchdog = (s->t - s->cmd_time) > cf->cmd_timeout;
    int brake = s->brake;
    double cvx = 0, cvy = 0, cwz = 0;
    if (!(watchdog || brake)) { cvx = s->cmd[0]; cvy = s->cmd[1]; cwz = s->cmd[2]; }
    /* 触边运动封锁 (允许反向脱困) */
    for (int i = 0; i < rt->nbump; i++) {
        if (!s->bumper_pressed[i]) continue;
        int sd = rt->bside[i];
        if (sd == 0 && cvx > 0) cvx = 0;
        else if (sd == 1 && cvx < 0) cvx = 0;
        else if (sd == 2 && cvy > 0) cvy = 0;
        else if (sd == 3 && cvy < 0) cvy = 0;
    }
    if (any_pressed(rt)) cwz = 0;

    double kv[3];
    sc_kin_step(rt->ch, rt->w, rt->nw, cvx, cvy, cwz, dt, brake, kv);
    double v[3] = {rt->ch->vx, rt->ch->vy, rt->ch->wz};
    if (cf->noise) sc_slip(s->slip_prev, v, dt, rt->rng);

    if (cf->use_mj) {
#ifdef SC_WITH_MUJOCO
        mjModel *m = (mjModel *)rt->m;
        mjData *d = (mjData *)rt->d;
        double th = d->qpos[2], c = cos(th), sn = sin(th);
        d->ctrl[0] = c * v[0] - sn * v[1];
        d->ctrl[1] = sn * v[0] + c * v[1];
        d->ctrl[2] = v[2];
        double tm = now_s();
        p_mj_step(m, d);
        s->mj_step_ms = 0.9 * s->mj_step_ms + 0.1 * (now_s() - tm) * 1000.0;
        double x = d->qpos[0], y = d->qpos[1];
        th = d->qpos[2];
        double vxw = d->qvel[0], vyw = d->qvel[1], w = d->qvel[2];
        c = cos(th); sn = sin(th);
        int hit = 0;
        double hx = 0, hy = 0;
        for (int i = 0; i < d->ncon; i++) {
            const mjContact *con = &d->contact[i];
            int g1 = con->geom[0], g2 = con->geom[1];
            int r1 = g1 >= 0 && g1 < rt->ngeom && rt->robot_geom[g1];
            int r2 = g2 >= 0 && g2 < rt->ngeom && rt->robot_geom[g2];
            if (r1 != r2 && con->dist < 0.002) {
                hit = 1; hx = con->pos[0]; hy = con->pos[1];
                break;
            }
        }
        if (hit) on_collision(rt, hx, hy);
        s->x = x; s->y = y; s->th = th;
        s->vx = c * vxw + sn * vyw;
        s->vy = -sn * vxw + c * vyw;
        s->wz = w;
#endif
    } else {
        double n[3], pt[2];
        sc_se2_integrate(s->x, s->y, s->th, v[0], v[1], v[2], dt, n);
        int hit = rt->ns > 0 && rt->nfp > 0 ? sc_collides(rt->fp, rt->nfp, n[0], n[1], n[2], rt->segs, rt->ns, pt) : 0;
        if (hit) {
            on_collision(rt, pt[0], pt[1]);
            kin_stop(rt);
            v[0] = v[1] = v[2] = 0.0;
        } else {
            s->x = n[0]; s->y = n[1]; s->th = n[2];
        }
        s->vx = v[0]; s->vy = v[1]; s->wz = v[2];
    }

    discrete_update(rt);
    /* 编码器里程计 */
    if (cf->noise) {
        double pose[6] = {s->odom[0], s->odom[1], s->odom[2], 0, 0, 0};
        sc_odom_update(rt->w, rt->nw, rt->ch->holonomic, rt->ch->axle_x, rt->radius_err, rt->steer_bias, cf->steer_sigma, cf->cpr,
                       dt, rt->rng, pose);
        memcpy(s->odom, pose, sizeof(pose));
    } else {
        double o[3];
        sc_se2_integrate(s->odom[0], s->odom[1], s->odom[2], rt->ch->vx, rt->ch->vy, rt->ch->wz, dt, o);
        s->odom[0] = o[0]; s->odom[1] = o[1]; s->odom[2] = o[2];
        s->odom[3] = rt->ch->vx; s->odom[4] = rt->ch->vy; s->odom[5] = rt->ch->wz;
    }
    sc_imu_sample(s->imu_st, cf->imu_par, s->vx, s->vy, s->wz, dt, rt->rng, s->imu);
    s->t += dt;
    s->steps++;
    s->step_ms = 0.9 * s->step_ms + 0.1 * (now_s() - t0) * 1000.0;
}

/* ---------------------------------------------------------------- 激光 */
static void scan_one(sc_rt *rt, rt_lidar *l, double *out) {
    sc_rt_state *s = &rt->s;
    const sc_rt_lidar_cfg *c = &l->c;
    double cth = cos(s->th), sth = sin(s->th);
    double ox = s->x + cth * c->mx - sth * c->my, oy = s->y + sth * c->mx + cth * c->my;
    double *ang = rt->scratch + 3 * rt->scratch_n;   /* [n] 角度；前 3n 给方向向量 */
    double base = s->th + c->yaw;
    for (int i = 0; i < c->n; i++) ang[i] = base + c->sign * (c->a0 + i * c->inc);
    if (rt->cfg.use_mj)
        mj_rays2d(rt->m, rt->d, rt->cfg.group, rt->cfg.robot_body, ox, oy, fmax(0.005, c->mz), ang, c->n, c->rmax,
                  rt->scratch, rt->iscratch, out);
    else
        sc_raycast2d(rt->segs, rt->ns, NULL, 0, c->mz, ox, oy, ang, c->n, c->rmax, out);
    sc_lidar_post(out, c->n, c->rmin, rt->cfg.noise, c->std, c->prop, c->dropout, rt->rng);
}

/* 一帧 3D 激光 → 后缓冲 (点云 + 高度带切片)；与 Lidar3DSensor.scan + slice_to_scan 逐步对应，噪声用 C 随机数 */
static void scan3d_one(sc_rt *rt, rt_l3d *l) {
    sc_rt_state *s = &rt->s;
    const sc_rt_l3d_cfg *c = &l->c;
    extern double sc_rng_gauss(uint64_t *);
    const int n = c->n, b = 1 - l->front, noise = rt->cfg.noise;
    const double PHI1 = 0.6180339887498949, PHI2 = 0.7548776662466927;
    double *dw = rt->s3, *ds = rt->s3 + 3 * (size_t)rt->s3_n, *dist = rt->s3 + 6 * (size_t)rt->s3_n;
    const double th = s->th, cth = cos(th), sth = sin(th);
    double Rw[9];                               /* Rz(th) · R */
    for (int k = 0; k < 3; k++) {
        Rw[k] = cth * c->R[k] - sth * c->R[3 + k];
        Rw[3 + k] = sth * c->R[k] + cth * c->R[3 + k];
        Rw[6 + k] = c->R[6 + k];
    }
    const double s0 = sin(c->vmin), s1 = sin(c->vmax);
    for (int i = 0; i < n; i++) {
        const double ii = (double)i + l->frame * n;
        double az = 2 * M_PI * fmod(ii * PHI1, 1.0);
        double el = asin(s0 + (s1 - s0) * fmod(ii * PHI2, 1.0));
        if (noise && c->ang_noise > 0) {
            az += sc_rng_gauss(rt->rng) * c->ang_noise;
            el += sc_rng_gauss(rt->rng) * c->ang_noise;
        }
        const double ce = cos(el), x = ce * cos(az), y = ce * sin(az), z = sin(el);
        ds[3 * i] = x; ds[3 * i + 1] = y; ds[3 * i + 2] = z;
        dw[3 * i] = Rw[0] * x + Rw[1] * y + Rw[2] * z;
        dw[3 * i + 1] = Rw[3] * x + Rw[4] * y + Rw[5] * z;
        dw[3 * i + 2] = Rw[6] * x + Rw[7] * y + Rw[8] * z;
    }
    l->frame += 1;
    const double o[3] = {s->x + cth * c->mx - sth * c->my, s->y + sth * c->mx + cth * c->my, c->mz};
#ifdef SC_WITH_MUJOCO
    if (rt->cfg.use_mj && p_mj_multiray) {
        p_mj_multiray((const mjModel *)rt->m, (mjData *)rt->d, o, dw, rt->cfg.group, 1, rt->cfg.robot_body, rt->is3, dist, NULL, n,
                      c->rmax);
        for (int i = 0; i < n; i++)
            if (rt->is3[i] < 0 || dist[i] < 0) dist[i] = INFINITY;
    } else
#endif
    {
        for (int i = 0; i < n; i++) dist[i] = INFINITY;
    }
    float *xyz = l->xyz[b], *inten = l->inten[b];
    unsigned char *line = l->line[b];
    double *ot = l->ot[b], *sl = l->slice[b];
    const int nb = rt->nbins;
    const double binw = 2 * M_PI / (nb > 0 ? nb : 1), dt_pt = 1.0 / fmax(1.0, 1.0 / c->period) / n;
    for (int k = 0; k < nb; k++) sl[k] = INFINITY;
    int m = 0;
    for (int i = 0; i < n; i++) {
        double r = dist[i];
        const double dz = dw[3 * i + 2];
        const double tf = dz < -1e-9 ? -o[2] / dz : INFINITY, tc = dz > 1e-9 ? (c->ceiling - o[2]) / dz : INFINITY;
        if (tf < r) r = tf;
        if (tc < r) r = tc;
        if (r > c->rmax || r < c->rmin || !isfinite(r)) continue;
        if (noise) r += sc_rng_gauss(rt->rng) * c->std;
        const double px = ds[3 * i] * r, py = ds[3 * i + 1] * r, pz = ds[3 * i + 2] * r;
        const float fx = (float)px, fy = (float)py, fz = (float)pz;
        xyz[3 * m] = fx; xyz[3 * m + 1] = fy; xyz[3 * m + 2] = fz;
        line[m] = (unsigned char)(i % (c->lines > 0 ? c->lines : 1));
        double it = 180.0 - 3.0 * r + (noise ? sc_rng_gauss(rt->rng) * 8.0 : 0.0);
        inten[m] = (float)(it < 5 ? 5 : (it > 255 ? 255 : it));
        ot[m] = i * dt_pt;
        m++;
        /* 高度带切片 → 机体系 360° 分箱 (与 slice_to_scan 一致: 用 float32 点) */
        const double bx = c->R[0] * fx + c->R[1] * fy + c->R[2] * fz + c->mx;
        const double by = c->R[3] * fx + c->R[4] * fy + c->R[5] * fz + c->my;
        const double bz = c->R[6] * fx + c->R[7] * fy + c->R[8] * fz + c->mz;
        if (nb <= 0 || bz < c->zmin || bz > c->zmax) continue;
        long k = (long)((atan2(by, bx) + M_PI) / binw);
        if (k < 0) k = 0;
        if (k > nb - 1) k = nb - 1;
        const double d = hypot(bx, by);
        if (d < sl[k]) sl[k] = d;
    }
    for (int k = 0; k < nb; k++)
        if (sl[k] > rt->cfg.merged_rmax) sl[k] = INFINITY;
    l->cnt[b] = m;
}

static void lidars_update(sc_rt *rt) {
    sc_rt_state *s = &rt->s;
    if (!rt->cfg.lidars_on || (rt->nlidar == 0 && rt->nl3d == 0)) return;
    int any = 0, mdue = s->t >= rt->mnext;
    for (int i = 0; i < rt->nlidar; i++)
        if (s->t >= rt->lid[i].next) any = 1;
    for (int i = 0; i < rt->nl3d; i++)
        if (s->t >= rt->l3d[i].next) any = 1;
    if (!any && !mdue) return;
    double t0 = now_s();
    /* 与 Python _sensor_loop 一致: 任一到期即全部 2D 扫描；到期的激光更新缓冲，融合按自己的周期更新。
       3D 激光只在自己到期时扫描 (点多)，融合使用各自最新一帧的高度带切片 */
    double *mb = rt->merged[1 - rt->mfront];
    for (int k = 0; k < rt->cfg.nbins; k++) mb[k] = INFINITY;
    for (int i = 0; i < rt->nlidar; i++) {
        rt_lidar *l = &rt->lid[i];
        double *b = l->buf[1 - l->front];
        scan_one(rt, l, b);
        sc_merge_add(mb, rt->cfg.nbins, b, l->c.n, l->c.mx, l->c.my, l->c.yaw, l->c.sign, l->c.a0, l->c.inc);
        if (s->t >= l->next) {
            l->next = s->t + l->c.period;
            l->front = 1 - l->front;
            l->seq++;
            l->t = s->t;
            l->pose[0] = s->x; l->pose[1] = s->y; l->pose[2] = s->th;
        }
    }
    for (int i = 0; i < rt->nl3d; i++) {
        rt_l3d *l = &rt->l3d[i];
        if (s->t >= l->next) {
            scan3d_one(rt, l);
            l->next = s->t + l->c.period;
            l->front = 1 - l->front;
            l->seq++;
            l->t = s->t;
            l->pose[0] = s->x; l->pose[1] = s->y; l->pose[2] = s->th;
        }
        if (l->seq) {
            const double *sl = l->slice[l->front];
            for (int k = 0; k < rt->cfg.nbins && k < rt->nbins; k++)
                if (sl[k] < mb[k]) mb[k] = sl[k];
        }
    }
    if (mdue) {
        sc_merge_finish(mb, rt->cfg.nbins, rt->cfg.merged_rmax);
        rt->mnext = s->t + rt->cfg.merged_period;
        rt->mfront = 1 - rt->mfront;
        rt->mseq++;
        rt->mt = s->t;
        rt->mpose[0] = s->x; rt->mpose[1] = s->y; rt->mpose[2] = s->th;
    }
    s->lidar_ms = 0.8 * s->lidar_ms + 0.2 * (now_s() - t0) * 1000.0;
}

/* ---------------------------------------------------------------- 线程 */
static void *rt_main(void *arg) {
    sc_rt *rt = (sc_rt *)arg;
    pthread_mutex_lock(&rt->mu);
    rt->last_wall = now_s();
    rt->rtf_t0 = rt->last_wall;
    pthread_mutex_unlock(&rt->mu);
    while (1) {
        pthread_mutex_lock(&rt->mu);
        int stop = rt->stop;
        double dt = rt->cfg.dt > 0 ? rt->cfg.dt : 0.01;
        pthread_mutex_unlock(&rt->mu);
        if (stop) break;
        struct timespec ts = {0, (long)(dt * 0.5 * 1e9)};
        nanosleep(&ts, NULL);
        pthread_mutex_lock(&rt->mu);
        sc_rt_state *s = &rt->s;
        double now = now_s();
        double real = now - rt->last_wall;
        if (real > 0.25) real = 0.25;
        if (real < 0) real = 0;
        rt->last_wall = now;
        int n = 0;
        if (rt->ch && rt->w && (!rt->cfg.use_mj || rt->d)) {
            rt->acc += real * s->rtf_target;
            while (rt->acc >= dt && n < rt->cfg.max_substeps) {
                if (!s->paused) rt_step(rt);
                rt->acc -= dt;
                n++;
            }
            if (rt->acc >= dt) {
                s->overruns++;
                rt->acc = 0.0;
            }
            if (n && !s->paused) lidars_update(rt);
        }
        rt->rtf_sim += s->paused ? 0.0 : n * dt;
        if (now - rt->rtf_t0 >= 1.0) {
            s->rtf = rt->rtf_sim / (now - rt->rtf_t0);
            rt->rtf_t0 = now;
            rt->rtf_sim = 0.0;
        }
        pthread_mutex_unlock(&rt->mu);
    }
    return NULL;
}

int sc_rt_start(sc_rt *rt) {
    if (rt->started) return 0;
    rt->stop = 0;
    if (pthread_create(&rt->th, NULL, rt_main, rt) != 0) return -1;
    rt->started = 1;
    return 0;
}

void sc_rt_stop(sc_rt *rt) {
    if (!rt->started) return;
    pthread_mutex_lock(&rt->mu);
    rt->stop = 1;
    pthread_mutex_unlock(&rt->mu);
    pthread_join(rt->th, NULL);
    rt->started = 0;
}

/* 立即写入速度指令 (cmd_time = 当前仿真时间，看门狗计时起点) —— 自带加锁 */
void sc_rt_set_cmd(sc_rt *rt, double vx, double vy, double wz) {
    pthread_mutex_lock(&rt->mu);
    rt->s.cmd[0] = vx; rt->s.cmd[1] = vy; rt->s.cmd[2] = wz;
    rt->s.cmd_time = rt->s.t;
    pthread_mutex_unlock(&rt->mu);
}

void sc_rt_destroy(sc_rt *rt) {
    if (!rt) return;
    sc_rt_stop(rt);
    for (int i = 0; i < rt->nlidar; i++) { free(rt->lid[i].buf[0]); free(rt->lid[i].buf[1]); }
    free(rt->merged[0]); free(rt->merged[1]); free(rt->scratch); free(rt->iscratch);
    l3d_free(rt);
    pthread_mutex_destroy(&rt->mu);
    free(rt);
}

/* 测试/离线用: 不启动线程，直接推进 n 步 (含激光到期扫描)。调用方负责加锁 */
void sc_rt_step_n(sc_rt *rt, int n) {
    for (int i = 0; i < n; i++) {
        if (!rt->s.paused) rt_step(rt);
        if (!rt->s.paused) lidars_update(rt);
    }
}

/* ================================================================== UDP 速度指令通道 (执行进程 → 仿真，替代 PUT /api/v1/control/cmd_vel)
 * 报文: "AGVC" u32 seq, f64 vx vy wz, char source[16]  (小端，48 字节)。收到即写入指令 (cmd_time = 当前仿真时间) */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct {
    sc_rt *rt;
    int fd;
    volatile int stop;
    pthread_t th;
    uint32_t count;
    double wall;
    char source[17];
    double cmd[3];
} rt_udp;

static rt_udp g_udp = {0};

static double wall_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void *udp_main(void *arg) {
    rt_udp *u = (rt_udp *)arg;
    unsigned char b[128];
    while (!u->stop) {
        ssize_t n = recv(u->fd, b, sizeof(b), 0);
        if (n < 48 || memcmp(b, "AGVC", 4) != 0) continue;
        double v[3];
        memcpy(v, b + 8, sizeof(v));
        if (!isfinite(v[0]) || !isfinite(v[1]) || !isfinite(v[2])) continue;
        sc_rt *rt = u->rt;
        pthread_mutex_lock(&rt->mu);
        rt->s.cmd[0] = v[0]; rt->s.cmd[1] = v[1]; rt->s.cmd[2] = v[2];
        rt->s.cmd_time = rt->s.t;
        u->count++;
        u->wall = wall_s();
        memcpy(u->source, b + 32, 16);
        u->source[16] = 0;
        memcpy(u->cmd, v, sizeof(v));
        pthread_mutex_unlock(&rt->mu);
    }
    return NULL;
}

/* 绑定 host:port (host 为空 = 0.0.0.0)。返回 0 成功 */
int sc_rt_udp_start(sc_rt *rt, const char *host, int port) {
    if (g_udp.fd > 0) return 0;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct timeval tv = {0, 200000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    a.sin_addr.s_addr = (host && *host) ? inet_addr(host) : htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)) != 0) { close(fd); return -2; }
    g_udp.rt = rt; g_udp.fd = fd; g_udp.stop = 0;
    if (pthread_create(&g_udp.th, NULL, udp_main, &g_udp) != 0) { close(fd); g_udp.fd = 0; return -3; }
    return 0;
}

/* 模型重建换了 sc_rt 实例时改指向 (须持新实例的锁以外调用) */
void sc_rt_udp_retarget(sc_rt *rt) {
    if (g_udp.fd <= 0) return;
    pthread_mutex_t *old = &g_udp.rt->mu;
    pthread_mutex_lock(old);
    g_udp.rt = rt;
    pthread_mutex_unlock(old);
}

/* 最近一条 UDP 指令: out[5] = count wall vx vy wz；source 至少 17 字节 */
void sc_rt_udp_meta(double *out, char *source) {
    out[0] = g_udp.count; out[1] = g_udp.wall;
    out[2] = g_udp.cmd[0]; out[3] = g_udp.cmd[1]; out[4] = g_udp.cmd[2];
    memcpy(source, g_udp.source, 17);
}
