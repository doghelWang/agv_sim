/*
 * libsimcore —— 仿真内核热点的纯 C 实现 (ctypes 加载，不依赖 Python 头文件)
 *
 *   sc_collides        车身多边形 vs 竖直线段: 边相交 + 线段端点在多边形内 (与 World.collides 逐项一致)
 *   sc_raycast2d       水平射线 vs 线段 (+ 圆柱)，min_seg_height 过滤 (World.raycast / 光电)
 *   sc_forward         轮系正运动学: 加权最小二乘 (单边 Jacobi SVD，与 numpy.linalg.lstsq 同解/同秩判定)
 *   sc_se2_integrate   SE(2) 精确积分
 *   sc_kin_step        ChassisKinematics.step 整体下沉 (指令整形 → 逆解 → 执行器 → 正解 → 电气量)
 *
 * 构建: sim_core/native/build.sh  (gcc -O2 -shared -fPIC -o libsimcore.so simcore.c -lm)
 * 结构体布局必须与 sim_core/native/__init__.py 中的 ctypes 定义保持一致 (SC_ABI 版本号)。
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SC_ABI 3

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int sc_abi(void) { return SC_ABI; }

/* Python 的 float % (结果与除数同号) */
static double pymod(double a, double b) {
    double m = fmod(a, b);
    if (m != 0.0) {
        if ((b < 0) != (m < 0)) m += b;
    } else {
        m = copysign(0.0, b);
    }
    return m;
}

static double wrap(double a) { return pymod(a + M_PI, 2.0 * M_PI) - M_PI; }

double sc_wrap(double a) { return wrap(a); }

/* ------------------------------------------------------------------ SE(2) */
void sc_se2_integrate(double x, double y, double th, double vx, double vy, double wz, double dt, double *out) {
    double dth = wz * dt, dxb, dyb;
    if (fabs(dth) < 1e-9) {
        dxb = vx * dt;
        dyb = vy * dt;
    } else {
        double s = sin(dth), c = cos(dth);
        double a = s / wz, b = (1.0 - c) / wz;
        dxb = a * vx - b * vy;
        dyb = b * vx + a * vy;
    }
    double ct = cos(th), st = sin(th);
    out[0] = x + ct * dxb - st * dyb;
    out[1] = y + st * dxb + ct * dyb;
    out[2] = wrap(th + dth);
}

/* ------------------------------------------------------------------ 碰撞 */
/* segs: [ns][5] = x0 y0 x1 y1 z_top ; fp: [np][2] 车体系多边形 ; 返回 1 = 碰撞, out_pt = 接触点 */
#define SC_MAX_POLY 64

static int points_in_poly(double px, double py, const double *P, int np) {
    int cross = 0;
    for (int k = 0; k < np; k++) {
        double x0 = P[2 * k], y0 = P[2 * k + 1];
        int k1 = (k + 1) % np;
        double x1 = P[2 * k1], y1 = P[2 * k1 + 1];
        if ((y0 > py) != (y1 > py)) {
            double xint = (x1 - x0) * (py - y0) / (y1 - y0) + x0;
            if (px < xint) cross++;
        }
    }
    return cross % 2 == 1;
}

int sc_collides(const double *fp, int np, double x, double y, double th, const double *segs, int ns, double *out_pt) {
    if (ns <= 0 || np <= 0 || np > SC_MAX_POLY) return 0;
    double P[2 * SC_MAX_POLY];
    double c = cos(th), s = sin(th);
    double mnx = INFINITY, mny = INFINITY, mxx = -INFINITY, mxy = -INFINITY;
    for (int k = 0; k < np; k++) {
        double fx = fp[2 * k], fy = fp[2 * k + 1];
        double px = fx * c + fy * (-s) + x;
        double py = fx * s + fy * c + y;
        P[2 * k] = px;
        P[2 * k + 1] = py;
        if (px < mnx) mnx = px;
        if (px > mxx) mxx = px;
        if (py < mny) mny = py;
        if (py > mxy) mxy = py;
    }
    mnx -= 0.01; mny -= 0.01; mxx += 0.01; mxy += 0.01;
    int any = 0;
    /* 1) 多边形边 × 线段 (numpy argwhere 行优先: 边 i 外层, 线段 j 内层) */
    for (int i = 0; i < np; i++) {
        int i1 = (i + 1) % np;
        double px = P[2 * i], py = P[2 * i + 1];
        double rx = P[2 * i1] - px, ry = P[2 * i1 + 1] - py;
        for (int j = 0; j < ns; j++) {
            const double *S = segs + 5 * j;
            double sx0 = fmin(S[0], S[2]), sx1 = fmax(S[0], S[2]);
            double sy0 = fmin(S[1], S[3]), sy1 = fmax(S[1], S[3]);
            if (!((sx1 >= mnx) && (sx0 <= mxx) && (sy1 >= mny) && (sy0 <= mxy))) continue;
            any = 1;
            double qx = S[0], qy = S[1];
            double sx = S[2] - S[0], sy = S[3] - S[1];
            double rxs = rx * sy - ry * sx;
            if (!(fabs(rxs) > 1e-12)) continue;
            double qpx = qx - px, qpy = qy - py;
            double t = (qpx * sy - qpy * sx) / rxs;
            double u = (qpx * ry - qpy * rx) / rxs;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                out_pt[0] = px + t * rx;
                out_pt[1] = py + t * ry;
                return 1;
            }
        }
    }
    if (!any) return 0;
    /* 2) 线段端点落在多边形内: 先全部起点，再全部终点 */
    for (int e = 0; e < 2; e++) {
        for (int j = 0; j < ns; j++) {
            const double *S = segs + 5 * j;
            double sx0 = fmin(S[0], S[2]), sx1 = fmax(S[0], S[2]);
            double sy0 = fmin(S[1], S[3]), sy1 = fmax(S[1], S[3]);
            if (!((sx1 >= mnx) && (sx0 <= mxx) && (sy1 >= mny) && (sy0 <= mxy))) continue;
            double qx = S[2 * e], qy = S[2 * e + 1];
            if (points_in_poly(qx, qy, P, np)) {
                out_pt[0] = qx;
                out_pt[1] = qy;
                return 1;
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ 射线 */
/* segs: [ns][5]；circles: [nc][4] = cx cy r z_top (可为 NULL)。只考虑 z_top > zmin 的几何。
 * 输出 out[n]: 命中距离，未命中或 > max_range = INFINITY */
void sc_raycast2d(const double *segs, int ns, const double *circles, int nc, double zmin,
                  double ox, double oy, const double *angles, int n, double max_range, double *out) {
    /* 粗筛 (高度 + 线段到原点最近距离 ≤ 量程)，对所有射线只做一次 */
    int stack_idx[256];
    int *idx = ns <= 256 ? stack_idx : (int *)malloc(sizeof(int) * (size_t)ns);
    int m = 0;
    if (idx == NULL) { idx = stack_idx; ns = ns < 256 ? ns : 256; }
    for (int j = 0; j < ns; j++) {
        const double *S = segs + 5 * j;
        if (!(S[4] > zmin)) continue;
        double abx = S[2] - S[0], aby = S[3] - S[1];
        double l2 = abx * abx + aby * aby;
        if (l2 < 1e-12) l2 = 1e-12;
        double tt = ((ox - S[0]) * abx + (oy - S[1]) * aby) / l2;
        tt = tt < 0 ? 0 : (tt > 1 ? 1 : tt);
        if (hypot(S[0] + abx * tt - ox, S[1] + aby * tt - oy) <= max_range) idx[m++] = j;
    }
    for (int i = 0; i < n; i++) {
        double dx = cos(angles[i]), dy = sin(angles[i]);
        double best = INFINITY;
        for (int jj = 0; jj < m; jj++) {
            const double *S = segs + 5 * idx[jj];
            double abx = S[2] - S[0], aby = S[3] - S[1];
            double qx = S[0] - ox, qy = S[1] - oy;
            double den = dx * aby - dy * abx;
            if (!(fabs(den) > 1e-12)) continue;
            double t = (qx * aby - qy * abx) / den;
            double u = (qx * dy - qy * dx) / den;
            if (t > 1e-6 && u >= 0.0 && u <= 1.0 && t < best) best = t;
        }
        for (int j = 0; j < nc; j++) {
            const double *C = circles + 4 * j;
            if (!(C[3] > zmin)) continue;
            double fx = ox - C[0], fy = oy - C[1];
            double b = fx * dx + fy * dy;
            double cc = fx * fx + fy * fy - C[2] * C[2];
            double disc = b * b - cc;
            if (disc < 0) continue;
            double sq = sqrt(disc);
            double t = -b - sq;
            if (t <= 1e-6) t = -b + sq;
            if (t > 1e-6 && t < best) best = t;
        }
        out[i] = best > max_range ? INFINITY : best;
    }
    if (idx != stack_idx) free(idx);
}

/* ------------------------------------------------------------------ 最小二乘 (m×3) */
/* 单边 Jacobi SVD: A = U Σ Vᵀ。解 x = V Σ⁺ Uᵀ b，σ < eps·max(m,3)·σmax 视为 0 (numpy lstsq rcond=None)。
 * 返回秩；resid = sqrt(mean((A x - b)²)) */
#define SC_MAX_ROWS 64

static int lstsq3(const double *A_in, const double *b, int m, double *xout, double *resid) {
    double U[SC_MAX_ROWS * 3], V[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    memcpy(U, A_in, sizeof(double) * 3 * m);
    for (int sweep = 0; sweep < 60; sweep++) {
        double off = 0.0;
        for (int p = 0; p < 2; p++) {
            for (int q = p + 1; q < 3; q++) {
                double alpha = 0, beta = 0, gamma = 0;
                for (int i = 0; i < m; i++) {
                    double up = U[3 * i + p], uq = U[3 * i + q];
                    alpha += up * up;
                    beta += uq * uq;
                    gamma += up * uq;
                }
                if (gamma == 0.0) continue;
                double conv = fabs(gamma) / sqrt(alpha * beta);
                if (!(conv > 1e-15)) continue;
                if (conv > off) off = conv;
                double zeta = (beta - alpha) / (2.0 * gamma);
                double t = copysign(1.0, zeta) / (fabs(zeta) + sqrt(1.0 + zeta * zeta));
                double cs = 1.0 / sqrt(1.0 + t * t), sn = cs * t;
                for (int i = 0; i < m; i++) {
                    double up = U[3 * i + p], uq = U[3 * i + q];
                    U[3 * i + p] = cs * up - sn * uq;
                    U[3 * i + q] = sn * up + cs * uq;
                }
                for (int i = 0; i < 3; i++) {
                    double vp = V[3 * i + p], vq = V[3 * i + q];
                    V[3 * i + p] = cs * vp - sn * vq;
                    V[3 * i + q] = sn * vp + cs * vq;
                }
            }
        }
        if (off <= 1e-15) break;
    }
    double sig[3], smax = 0.0;
    for (int k = 0; k < 3; k++) {
        double s2 = 0;
        for (int i = 0; i < m; i++) s2 += U[3 * i + k] * U[3 * i + k];
        sig[k] = sqrt(s2);
        if (sig[k] > smax) smax = sig[k];
    }
    double tol = 2.220446049250313e-16 * (m > 3 ? m : 3) * smax;
    int rank = 0;
    double x[3] = {0, 0, 0};
    for (int k = 0; k < 3; k++) {
        if (!(sig[k] > tol)) continue;
        rank++;
        double utb = 0;   /* u_k = U[:,k]/σ_k */
        for (int i = 0; i < m; i++) utb += U[3 * i + k] * b[i];
        double coef = utb / (sig[k] * sig[k]);
        for (int r = 0; r < 3; r++) x[r] += V[3 * r + k] * coef;
    }
    double ss = 0;
    for (int i = 0; i < m; i++) {
        double e = A_in[3 * i] * x[0] + A_in[3 * i + 1] * x[1] + A_in[3 * i + 2] * x[2] - b[i];
        ss += e * e;
    }
    *resid = sqrt(ss / m);
    xout[0] = x[0]; xout[1] = x[1]; xout[2] = x[2];
    return rank;
}

/* ------------------------------------------------------------------ 轮系 */
enum { K_DRIVE = 0, K_STEER = 1, K_FIXED = 2, K_CASTER = 3 };

typedef struct {
    int kind, _pad;
    double x, y, radius, steer_min, steer_max, steer_rate, max_speed, drive_tau, gear_ratio;
    /* 状态 */
    double steer, steer_target, speed, speed_target, cmd_speed, angle, motor_rpm, current_a, torque_nm, steer_current_a;
} sc_wheel;

typedef struct {
    double max_speed, max_accel, max_decel, max_ang_speed, max_ang_accel, max_ang_decel, mass;
    double axle_x, kt, rolling_coeff, efficiency;
    int holonomic, _pad;
    /* 状态 */
    double shaped[3];
    double vx, vy, wz, slip_residual, saturation;
} sc_chassis;

int sc_sizeof_wheel(void) { return (int)sizeof(sc_wheel); }
int sc_sizeof_chassis(void) { return (int)sizeof(sc_chassis); }

/* 正运动学。heading_ovr / speed_ovr 可为 NULL，元素为 NaN 表示不覆盖。out = vx vy wz resid；返回秩 (无约束行返回 -1) */
int sc_forward(const sc_wheel *w, int n, int holonomic, double axle_x,
               const double *heading_ovr, const double *speed_ovr, double *out) {
    double A[SC_MAX_ROWS * 3], b[SC_MAX_ROWS];
    int m = 0;
    for (int i = 0; i < n && m + 2 <= SC_MAX_ROWS; i++) {
        int k = w[i].kind;
        if (!(k == K_DRIVE || k == K_STEER || k == K_FIXED)) continue;
        double a = (k == K_STEER) ? w[i].steer : 0.0;
        if (heading_ovr && !isnan(heading_ovr[i])) a = heading_ovr[i];
        double c = cos(a), s = sin(a);
        if (k == K_DRIVE || k == K_STEER) {
            double spd = w[i].speed;
            if (speed_ovr && !isnan(speed_ovr[i])) spd = speed_ovr[i];
            A[3 * m] = c; A[3 * m + 1] = s; A[3 * m + 2] = -w[i].y * c + w[i].x * s; b[m] = spd; m++;
        }
        A[3 * m] = -s; A[3 * m + 1] = c; A[3 * m + 2] = w[i].y * s + w[i].x * c; b[m] = 0.0; m++;
    }
    if (m == 0) {
        out[0] = out[1] = out[2] = out[3] = 0.0;
        return -1;
    }
    double sol[3], resid;
    int rank = lstsq3(A, b, m, sol, &resid);
    double vx = sol[0], vy = sol[1], wz = sol[2];
    if (!holonomic && rank >= 3) vy = -wz * axle_x;
    out[0] = vx; out[1] = vy; out[2] = wz; out[3] = resid;
    return rank;
}

static double ramp(double cur, double tgt, double acc, double dec, double dt) {
    double lim = (fabs(tgt) > fabs(cur) && (tgt * cur >= 0)) ? acc * dt : dec * dt;
    double d = tgt - cur;
    return fabs(d) <= lim ? tgt : cur + copysign(lim, d);
}

static void best_steer(const sc_wheel *w, double ang, double spd, double *a_out, double *s_out) {
    double ca[3] = {ang, wrap(ang + M_PI), wrap(ang - M_PI)};
    double cs[3] = {spd, -spd, -spd};
    int best = -1;
    double be = 0, ba = 0, bs = 0;
    for (int k = 0; k < 3; k++) {
        double a = ca[k];
        if (!(w->steer_min - 1e-6 <= a && a <= w->steer_max + 1e-6)) continue;
        double e = fabs(a - w->steer);
        /* Python: cands.sort() → (误差, 角度, 速度) 字典序 */
        if (best < 0 || e < be || (e == be && (a < ba || (a == ba && cs[k] < bs)))) {
            best = k; be = e; ba = a; bs = cs[k];
        }
    }
    if (best < 0) {
        double a = fmax(w->steer_min, fmin(w->steer_max, ang));
        *a_out = a;
        *s_out = spd * cos(ang - a);
        return;
    }
    *a_out = ba;
    *s_out = bs;
}

#define STEER_GATE_START (10.0 * M_PI / 180.0)
#define STEER_GATE_STOP (45.0 * M_PI / 180.0)

/* ChassisKinematics.step 的逐项移植。out = vx vy wz */
void sc_kin_step(sc_chassis *ch, sc_wheel *w, int n, double cvx, double cvy, double cwz, double dt, int brake, double *out) {
    if (brake) cvx = cvy = cwz = 0.0;
    /* ---- 指令整形 */
    if (!ch->holonomic) cvy = -cwz * ch->axle_x;
    double v = hypot(cvx, cvy);
    if (v > ch->max_speed) {
        double k = ch->max_speed / v;
        cvx *= k;
        cvy *= k;
    }
    cwz = fmax(-ch->max_ang_speed, fmin(ch->max_ang_speed, cwz));
    double sx = ramp(ch->shaped[0], cvx, ch->max_accel, ch->max_decel, dt);
    double sy = ramp(ch->shaped[1], cvy, ch->max_accel, ch->max_decel, dt);
    double sw = ramp(ch->shaped[2], cwz, ch->max_ang_accel, ch->max_ang_decel, dt);
    if (!ch->holonomic) sy = -sw * ch->axle_x;
    ch->shaped[0] = sx; ch->shaped[1] = sy; ch->shaped[2] = sw;
    double vx_c = sx, vy_c = sy, wz_c = sw;

    /* ---- 逆解 */
    double tang[64], tspd[64];
    int has[64];
    double scale = 1.0;
    if (n > 64) n = 64;
    for (int i = 0; i < n; i++) {
        has[i] = 0;
        int k = w[i].kind;
        if (!(k == K_DRIVE || k == K_STEER)) continue;
        double wx = vx_c - wz_c * w[i].y, wy = vy_c + wz_c * w[i].x;
        double spd, ang;
        if (k == K_DRIVE) {
            spd = wx;
            ang = 0.0;
        } else {
            spd = hypot(wx, wy);
            if (spd < 1e-4) {
                ang = w[i].steer_target;
                spd = 0.0;
            } else {
                best_steer(&w[i], atan2(wy, wx), spd, &ang, &spd);
            }
        }
        if (fabs(spd) > w[i].max_speed) {
            double r = w[i].max_speed / fabs(spd);
            if (r < scale) scale = r;
        }
        tang[i] = ang;
        tspd[i] = spd;
        has[i] = 1;
    }
    if (scale < 1.0)
        for (int i = 0; i < n; i++)
            if (has[i]) tspd[i] *= scale;
    ch->saturation = scale;
    int pure_rot = hypot(vx_c, vy_c) < 0.02 && fabs(wz_c) > 1e-3;

    /* ---- 执行器 */
    double accmax = fmax(ch->max_accel, ch->max_decel);
    for (int i = 0; i < n; i++) {
        if (!has[i]) continue;
        sc_wheel *W = &w[i];
        double ang_t = tang[i], spd_t = tspd[i];
        if (W->kind == K_STEER) {
            W->steer_target = ang_t;
            double err = ang_t - W->steer;
            double max_d = W->steer_rate * dt;
            if (fabs(err) > max_d) {
                W->steer += copysign(max_d, err);
                W->steer_current_a = 4.5;
            } else {
                W->steer = ang_t;
                W->steer_current_a = 0.8;
            }
            W->steer = fmax(W->steer_min, fmin(W->steer_max, W->steer));
            double e = fabs(W->steer_target - W->steer);
            double g0 = pure_rot ? 2.0 * M_PI / 180.0 : STEER_GATE_START;
            double g1 = pure_rot ? 8.0 * M_PI / 180.0 : STEER_GATE_STOP;
            if (e > g0) {
                double g = fmax(0.0, 1.0 - (e - g0) / (g1 - g0));
                spd_t *= g;
            }
        }
        W->speed_target = brake ? 0.0 : spd_t;
        double alpha = dt / (W->drive_tau + dt);
        double dv = (W->speed_target - W->cmd_speed) * alpha;
        double acc_lim = accmax * 2.5 * dt;
        dv = fmax(-acc_lim, fmin(acc_lim, dv));
        W->cmd_speed += dv;
        if (brake && fabs(W->cmd_speed) < 0.01) W->cmd_speed = 0.0;
        W->speed = W->cmd_speed;
    }

    /* ---- 正解 */
    double fo[4];
    sc_forward(w, n, ch->holonomic, ch->axle_x, NULL, NULL, fo);
    double vx = fo[0], vy = fo[1], wz = fo[2];
    ch->slip_residual = fo[3];
    double acc = dt > 0 ? (vx - ch->vx) / dt : 0.0;
    ch->vx = vx; ch->vy = vy; ch->wz = wz;
    int n_drv = 0;
    for (int i = 0; i < n; i++)
        if (w[i].kind == K_DRIVE || w[i].kind == K_STEER) n_drv++;
    if (n_drv < 1) n_drv = 1;
    for (int i = 0; i < n; i++) {
        sc_wheel *W = &w[i];
        double heading = W->kind == K_STEER ? W->steer : 0.0;
        if (W->kind == K_FIXED || W->kind == K_CASTER)
            W->speed = (vx - wz * W->y) * cos(heading) + (vy + wz * W->x) * sin(heading);
        double rad = fmax(W->radius, 1e-3);
        W->angle += (W->speed / rad) * dt;
        if (W->kind == K_DRIVE || W->kind == K_STEER) {
            double omega = W->speed / rad;
            W->motor_rpm = omega * W->gear_ratio * 60.0 / (2 * M_PI);
            double f = ch->mass / n_drv * (fabs(acc) + 9.81 * ch->rolling_coeff * (fabs(W->speed) > 1e-3 ? 1 : 0));
            W->torque_nm = f * W->radius / fmax(W->gear_ratio, 1.0) / ch->efficiency;
            W->current_a = (fabs(W->speed_target) > 1e-3 || fabs(W->speed) > 1e-3) ? (0.3 + W->torque_nm / ch->kt) : 0.05;
        }
    }
    out[0] = vx; out[1] = vy; out[2] = wz;
}

/* ================================================================== 随机数 (xoshiro256**，每个传感器一份状态) */
static inline uint64_t rotl(const uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

static uint64_t rng_next(uint64_t *s) {
    const uint64_t r = rotl(s[1] * 5, 7) * 9;
    const uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3];
    s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return r;
}

static double rng_uniform(uint64_t *s) { return (rng_next(s) >> 11) * 0x1.0p-53; }

/* 标准正态 (Box-Muller，缓存第二个值在 s[4]/s[5]) */
static double rng_gauss(uint64_t *s) {
    if (s[5]) {
        s[5] = 0;
        double v;
        memcpy(&v, &s[4], sizeof(v));
        return v;
    }
    double u1, u2;
    do { u1 = rng_uniform(s); } while (u1 <= 1e-300);
    u2 = rng_uniform(s);
    double r = sqrt(-2.0 * log(u1)), a = 2.0 * M_PI * u2;
    double z1 = r * sin(a);
    memcpy(&s[4], &z1, sizeof(z1));
    s[5] = 1;
    return r * cos(a);
}

void sc_rng_seed(uint64_t *s, uint64_t seed) {
    for (int i = 0; i < 4; i++) {           /* splitmix64 */
        seed += 0x9e3779b97f4a7c15ULL;
        uint64_t z = seed;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        s[i] = z ^ (z >> 31);
    }
    s[4] = s[5] = 0;
}

double sc_rng_gauss(uint64_t *s) { return rng_gauss(s); }

/* ================================================================== 开关量传感器 (批量) */
/* 光电 (兜底后端，线段几何): P[n][6] = x y z yaw half range (车体系)。out[n] = 3 条射线最近距离 */
void sc_photo_batch(const double *segs, int ns, const double *P, int n, double x, double y, double th, double *out) {
    double c = cos(th), s = sin(th);
    for (int i = 0; i < n; i++) {
        const double *p = P + 6 * i;
        double ox = x + c * p[0] - s * p[1], oy = y + s * p[0] + c * p[1];
        double a = th + p[3];
        double ang[3] = {a - p[4], a, a + p[4]}, d[3];
        sc_raycast2d(segs, ns, NULL, 0, p[2], ox, oy, ang, 3, p[5], d);
        out[i] = fmin(d[0], fmin(d[1], d[2]));
    }
}

/* 触边: polys = 各条带多边形顶点依次拼接，counts[k] = 第 k 条的顶点数；hits[k] = 是否接触 */
void sc_collides_batch(const double *polys, const int *counts, int k, double x, double y, double th,
                       const double *segs, int ns, int *hits) {
    double pt[2];
    for (int i = 0; i < k; i++) {
        hits[i] = sc_collides(polys, counts[i], x, y, th, segs, ns, pt);
        polys += 2 * counts[i];
    }
}

/* ================================================================== 编码器里程计 */
/* 驱动轮: 编码器量化 (电机端 cpr·减速比) × 轮径误差；舵轮角: + 零偏 + 噪声。之后正解 + SE2 积分。
 * pose[6]: 入 x y th，出 x y th vx vy wz */
void sc_odom_update(const sc_wheel *w, int n, int holonomic, double axle_x, const double *radius_err, const double *steer_bias,
                    double steer_sigma, double cpr, double dt, uint64_t *rng, double *pose) {
    double hov[64], sov[64], fo[4];
    if (n > 64) n = 64;
    for (int i = 0; i < n; i++) {
        hov[i] = NAN;
        sov[i] = NAN;
        if (!(w[i].kind == K_DRIVE || w[i].kind == K_STEER)) continue;
        double tpm = cpr * w[i].gear_ratio / (2 * M_PI * w[i].radius);
        double q = nearbyint(w[i].speed * dt * tpm) / fmax(tpm * dt, 1e-9);   /* Python round(): 银行家舍入 */
        sov[i] = q * radius_err[i];
        if (w[i].kind == K_STEER) hov[i] = w[i].steer + steer_bias[i] + (steer_sigma > 0 ? rng_gauss(rng) * steer_sigma : 0.0);
    }
    sc_forward(w, n, holonomic, axle_x, hov, sov, fo);
    double o[3];
    sc_se2_integrate(pose[0], pose[1], pose[2], fo[0], fo[1], fo[2], dt, o);
    pose[0] = o[0]; pose[1] = o[1]; pose[2] = o[2];
    pose[3] = fo[0]; pose[4] = fo[1]; pose[5] = fo[2];
}

/* ================================================================== IMU / 轮地打滑 */
/* st[4] = prev_vx prev_vy gyro_bias yaw；par[3] = bias_walk gyro_noise acc_noise (enabled=0 时全 0)；out[5] = wz ax ay az yaw */
void sc_imu_sample(double *st, const double *par, double vx, double vy, double wz, double dt, uint64_t *rng, double *out) {
    double k = fmax(dt, 1e-3);
    double ax = (vx - st[0]) / k - wz * vy;
    double ay = (vy - st[1]) / k + wz * vx;
    st[0] = vx; st[1] = vy;
    if (par[0] > 0) st[2] += rng_gauss(rng) * par[0];
    double gz = wz + st[2] + (par[1] > 0 ? rng_gauss(rng) * par[1] : 0.0);
    st[3] = wrap(st[3] + gz * dt);
    out[0] = gz;
    out[1] = ax + (par[2] > 0 ? rng_gauss(rng) * par[2] : 0.0);
    out[2] = ay + (par[2] > 0 ? rng_gauss(rng) * par[2] : 0.0);
    out[3] = 9.81 + (par[2] > 0 ? rng_gauss(rng) * par[2] : 0.0);
    out[4] = st[3];
}

/* prev[3] = 上一步 vx vy wz；v[3] 入/出 */
void sc_slip(double *prev, double *v, double dt, uint64_t *rng) {
    double acc = hypot(v[0] - prev[0], v[1] - prev[1]) / fmax(dt, 1e-3);
    prev[0] = v[0]; prev[1] = v[1]; prev[2] = v[2];
    double k = 1.0 - fmin(0.03, 0.004 + 0.01 * acc) - rng_gauss(rng) * 0.002;
    double kw = 1.0 - fmin(0.05, 0.01 * fabs(v[2])) - rng_gauss(rng) * 0.002;
    v[0] *= k; v[1] *= k; v[2] *= kw;
}

/* ================================================================== 2D 激光 */
/* 测距后处理: noise → N(0,1)·(std + prop·r)，dropout 概率置 inf；最后 r < rmin → rmin (与 LidarSensor.scan 一致) */
void sc_lidar_post(double *out, int n, double rmin, int noise, double std, double prop, double dropout, uint64_t *rng) {
    if (noise) {
        for (int i = 0; i < n; i++)
            if (isfinite(out[i])) out[i] += rng_gauss(rng) * (std + prop * out[i]);
        if (dropout > 0)
            for (int i = 0; i < n; i++)
                if (rng_uniform(rng) < dropout) out[i] = INFINITY;
    }
    for (int i = 0; i < n; i++)
        if (out[i] < rmin) out[i] = rmin;
}

/* 兜底 (无 MuJoCo) 后端的一次扫描: 世界系射线角 = th + yaw + sign·(a0 + i·inc)，对场景线段求交 */
void sc_lidar_scan(const double *segs, int ns, double z, double ox, double oy, double base_angle, double sign, double a0,
                   double inc, int n, double rmax, double rmin, int noise, double std, double prop, double dropout,
                   uint64_t *rng, double *out) {
    double *ang = (double *)malloc(sizeof(double) * (size_t)n);
    if (!ang) return;
    for (int i = 0; i < n; i++) ang[i] = base_angle + sign * (a0 + i * inc);
    sc_raycast2d(segs, ns, NULL, 0, z, ox, oy, ang, n, rmax, out);
    free(ang);
    sc_lidar_post(out, n, rmin, noise, std, prop, dropout, rng);
}

/* 融合扫描: 激光 (安装 mx my yaw sign，角度 a0 inc) 的有限测距点 → 机体系 → 360° 分箱取最近 (bins 需先置 inf) */
void sc_merge_add(double *bins, int nb, const double *r, int n, double mx, double my, double yaw, double sign, double a0, double inc) {
    double binw = 2 * M_PI / nb;
    for (int i = 0; i < n; i++) {
        if (!isfinite(r[i])) continue;
        double a = yaw + sign * (a0 + i * inc);
        double px = mx + r[i] * cos(a), py = my + r[i] * sin(a);
        double d = hypot(px, py);
        long k = (long)((atan2(py, px) + M_PI) / binw);
        if (k < 0) k = 0;
        if (k > nb - 1) k = nb - 1;
        if (d < bins[k]) bins[k] = d;
    }
}

void sc_merge_finish(double *bins, int nb, double rmax) {
    for (int i = 0; i < nb; i++)
        if (bins[i] > rmax) bins[i] = INFINITY;
}

/* ---------------------------------------------------------------- 相机着色 (MuJoCoBackend.shade + CameraSensor._rgb)
 * 射线命中 (dist/gid/nrm，来自 mj_multiRay + 解析地面/屋顶) → Lambert + 环境光 + 距离雾 (地面按纹理查表) → uint8 RGB。
 * 舍入与 numpy 版一致: 颜色先存 float32 并截到 [0,1]，×255 (float32)，加噪声 (double) 后截到 [0,255] 再截断取整。
 * nrm 可为 NULL (按 (0,0,1))；rng 为 NULL 或 sigma<=0 时不加噪声。 */
void sc_cam_shade(const double *o, const double *dirs, const double *dist, const int *gid, const double *nrm, int n,
                  const float *geom_rgb, int ngeom, int floor_geom, const float *tex, int th, int tw, double x0, double y0, double res,
                  double sigma, uint64_t *rng, unsigned char *out) {
    double L[3] = {0.35, 0.25, 0.9};
    const double ln = sqrt(L[0] * L[0] + L[1] * L[1] + L[2] * L[2]);
    L[0] /= ln; L[1] /= ln; L[2] /= ln;
    for (int i = 0; i < n; i++) {
        float col[3] = {0.55f, 0.55f, 0.55f};
        const double d = dist[i];
        if (isfinite(d)) {
            const int g = gid[i];
            float base[3] = {0.0f, 0.0f, 0.0f};
            if (g >= 0 && g < ngeom) { base[0] = geom_rgb[3 * g]; base[1] = geom_rgb[3 * g + 1]; base[2] = geom_rgb[3 * g + 2]; }
            if (g == floor_geom && tex && th > 0 && tw > 0) {
                const double px = o[0] + dirs[3 * i] * d, py = o[1] + dirs[3 * i + 1] * d;
                int ix = (int)((px - x0) / res), iy = (int)((py - y0) / res);
                ix = ix < 0 ? 0 : (ix > tw - 1 ? tw - 1 : ix);
                iy = iy < 0 ? 0 : (iy > th - 1 ? th - 1 : iy);
                const float *t = tex + 3 * ((size_t)iy * tw + ix);
                base[0] = t[0]; base[1] = t[1]; base[2] = t[2];
            }
            const double nx = nrm ? nrm[3 * i] : 0.0, ny = nrm ? nrm[3 * i + 1] : 0.0, nz = nrm ? nrm[3 * i + 2] : 1.0;
            double lam = fabs(nx * L[0] + ny * L[1] + nz * L[2]);
            lam = lam > 1 ? 1 : lam;
            double view = fabs(nx * dirs[3 * i] + ny * dirs[3 * i + 1] + nz * dirs[3 * i + 2]);
            view = view > 1 ? 1 : view;
            const double k = 0.35 + 0.45 * lam + 0.2 * view, fog = exp(-d / 45.0);
            for (int c = 0; c < 3; c++) col[c] = (float)((double)base[c] * (k * fog) + 0.55 * (1 - fog));
        }
        for (int c = 0; c < 3; c++) {
            float v = col[c] < 0.0f ? 0.0f : (col[c] > 1.0f ? 1.0f : col[c]);
            const float f = v * 255.0f;
            double x = f;
            if (rng && sigma > 0) x = (double)f + rng_gauss(rng) * sigma;
            x = x < 0 ? 0 : (x > 255 ? 255 : x);
            out[3 * i + c] = (unsigned char)x;
        }
    }
}


/* ====================================================================== 相机/3D 射线求交 (不经 MuJoCo)
 * sc_cast_prims: 单原点多射线对 group 0 几何体 (有向长方体 + 圆柱) 求交，外加解析地面 z=0 与屋顶；pthread 多线程。
 * 语义与 MuJoCoBackend.cast (mj_multiRay + numpy 后处理) 一致，结果逐射线相同；场景几何体少 (几十个) 时比 BVH 遍历快数倍。
 * prims 每个 18 个 double: [0..2] 中心  [3..11] 旋转矩阵 (行主序，局部→世界)  [12..14] 半尺寸 (圆柱: r, r, 半高)
 *                          [15] 类型 (0 长方体, 1 圆柱)  [16] 几何体编号  [17] 包围球半径
 */
#include <pthread.h>
#define SC_PRIM_N 18
#define SC_CAST_MAX_THREADS 16

/* ------------------------------------------------------------------ 求交 */
/* ol3: 射线原点在几何体局部系下的坐标 (同一原点的所有射线共用，由调用方预先算好) */
static inline int hit_box(const double *p, const double *ol3, const double *d, double *t_out, double *n_out) {
    const double *R = p + 3, *h = p + 12;
    double tn = -1e300, tf = 1e300;
    int an = -1; double sn = 0.0;
    for (int a = 0; a < 3; a++) {
        /* 局部轴 a = R 的第 a 列 */
        double ax = R[a], ay = R[3 + a], az = R[6 + a];
        double ol = ol3[a];
        double dl = d[0] * ax + d[1] * ay + d[2] * az;
        if (fabs(dl) < 1e-12) {
            if (fabs(ol) > h[a]) return 0;
            continue;
        }
        double t1 = (-h[a] - ol) / dl, t2 = (h[a] - ol) / dl, s = -1.0;
        if (t1 > t2) { double tmp = t1; t1 = t2; t2 = tmp; s = 1.0; }
        if (t1 > tn) { tn = t1; an = a; sn = s; }
        if (t2 < tf) tf = t2;
        if (tn > tf) return 0;
    }
    if (tf < 0.0) return 0;
    double t = tn;
    if (tn < 0.0) {            /* 原点在盒内: 取出射点 */
        t = tf;
        an = -1;
        double best = 1e300;
        for (int a = 0; a < 3; a++) {
            double ax = R[a], ay = R[3 + a], az = R[6 + a];
            double ol = ol3[a], dl = d[0] * ax + d[1] * ay + d[2] * az;
            double pl = ol + dl * t, e = fabs(fabs(pl) - h[a]);
            if (e < best) { best = e; an = a; sn = pl > 0 ? 1.0 : -1.0; }
        }
    }
    *t_out = t;
    if (n_out && an >= 0) { n_out[0] = sn * R[an]; n_out[1] = sn * R[3 + an]; n_out[2] = sn * R[6 + an]; }
    return 1;
}

static inline int hit_cyl(const double *p, const double *ol, const double *d, double *t_out, double *n_out) {
    const double *R = p + 3;
    double r = p[12], hz = p[14];
    double dl[3];
    for (int a = 0; a < 3; a++) dl[a] = d[0] * R[a] + d[1] * R[3 + a] + d[2] * R[6 + a];
    double best = 1e300, nl[3] = {0, 0, 0};
    double A = dl[0] * dl[0] + dl[1] * dl[1], B = ol[0] * dl[0] + ol[1] * dl[1], C = ol[0] * ol[0] + ol[1] * ol[1] - r * r;
    if (A > 1e-18) {
        double disc = B * B - A * C;
        if (disc >= 0.0) {
            double sq = sqrt(disc);
            for (int k = 0; k < 2; k++) {
                double t = (k == 0 ? (-B - sq) : (-B + sq)) / A;
                if (t < 0.0 || t >= best) continue;
                double z = ol[2] + dl[2] * t;
                if (fabs(z) > hz) continue;
                best = t; nl[0] = (ol[0] + dl[0] * t) / r; nl[1] = (ol[1] + dl[1] * t) / r; nl[2] = 0.0;
            }
        }
    }
    if (fabs(dl[2]) > 1e-12) {
        for (int k = 0; k < 2; k++) {
            double zc = k == 0 ? -hz : hz, t = (zc - ol[2]) / dl[2];
            if (t < 0.0 || t >= best) continue;
            double x = ol[0] + dl[0] * t, y = ol[1] + dl[1] * t;
            if (x * x + y * y > r * r) continue;
            best = t; nl[0] = 0.0; nl[1] = 0.0; nl[2] = k == 0 ? -1.0 : 1.0;
        }
    }
    if (best >= 1e299) return 0;
    *t_out = best;
    if (n_out) for (int a = 0; a < 3; a++) n_out[a] = R[3 * a] * nl[0] + R[3 * a + 1] * nl[1] + R[3 * a + 2] * nl[2];
    return 1;
}

/* ------------------------------------------------------------------ sc_cast */
typedef struct {
    const double *o, *dirs, *prims, *ol;      /* ol: np×4 = 局部系原点 xyz + 原点到中心距离平方 */
    int n0, n1, np;
    double max_range, ceil_h;
    int floor_gid, ceil_gid;
    double *dist; int32_t *gid; double *nrm;
} cast_job;

static void *cast_worker(void *arg) {
    cast_job *j = (cast_job *)arg;
    const double *o = j->o;
    for (int i = j->n0; i < j->n1; i++) {
        const double *d = j->dirs + 3 * i;
        double best = j->max_range, n[3] = {0, 0, 0}, nb[3] = {0, 0, 0};
        int g = -1;
        for (int k = 0; k < j->np; k++) {
            const double *p = j->prims + SC_PRIM_N * k;
            const double *ol = j->ol + 4 * k;
            /* 包围球粗筛 */
            double b = (p[0] - o[0]) * d[0] + (p[1] - o[1]) * d[1] + (p[2] - o[2]) * d[2], r = p[17];
            if (b + r < 0.0 || b - r > best) continue;
            if (ol[3] - b * b > r * r) continue;
            double t;
            int ok = p[15] < 0.5 ? hit_box(p, ol, d, &t, nb) : hit_cyl(p, ol, d, &t, nb);
            if (ok && t < best) { best = t; g = (int)p[16]; n[0] = nb[0]; n[1] = nb[1]; n[2] = nb[2]; }
        }
        double dist = g >= 0 ? best : INFINITY;
        /* 解析平面: 地面 z=0 / 屋顶 (与 Python 版一致: 先比较再按量程截断) */
        double dz = d[2], oz = o[2];
        if (dz < -1e-9) { double tf = -oz / dz; if (tf < dist) { dist = tf; g = j->floor_gid; n[0] = 0; n[1] = 0; n[2] = 1; } }
        if (dz > 1e-9) { double tc = (j->ceil_h - oz) / dz; if (tc < dist) { dist = tc; g = j->ceil_gid; n[0] = 0; n[1] = 0; n[2] = -1; } }
        if (dist > j->max_range) dist = INFINITY;
        j->dist[i] = dist;
        j->gid[i] = g;
        if (j->nrm) { j->nrm[3 * i] = n[0]; j->nrm[3 * i + 1] = n[1]; j->nrm[3 * i + 2] = n[2]; }
    }
    return NULL;
}

static void sc_run_jobs(void *(*fn)(void *), void *jobs, size_t sz, int nt) {
    pthread_t th[SC_CAST_MAX_THREADS];
    int started[SC_CAST_MAX_THREADS] = {0};
    for (int t = 1; t < nt; t++) started[t] = pthread_create(&th[t], NULL, fn, (char *)jobs + sz * t) == 0;
    fn(jobs);
    for (int t = 1; t < nt; t++) {
        if (started[t]) pthread_join(th[t], NULL);
        else fn((char *)jobs + sz * t);
    }
}

int sc_cast_prims(const double *origin, const double *dirs, int n, double max_range, const double *prims, int np,
            double ceil_h, int floor_gid, int ceil_gid, double *dist, int32_t *gid, double *nrm, int threads) {
    if (n <= 0) return 0;
    int nt = threads < 1 ? 1 : (threads > SC_CAST_MAX_THREADS ? SC_CAST_MAX_THREADS : threads);
    if (n < 1024) nt = 1;
    /* 原点变换到每个几何体的局部系 (一次)，并剔除整体超出量程的几何体 */
    double *ol = (double *)malloc(sizeof(double) * 4 * (np > 0 ? np : 1));
    double *pr = (double *)malloc(sizeof(double) * SC_PRIM_N * (np > 0 ? np : 1));
    if (!ol || !pr) { free(ol); free(pr); return -1; }
    int m = 0;
    for (int k = 0; k < np; k++) {
        const double *p = prims + SC_PRIM_N * k, *R = p + 3;
        double oc[3] = {origin[0] - p[0], origin[1] - p[1], origin[2] - p[2]};
        double c2 = oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2];
        if (sqrt(c2) - p[17] > max_range) continue;
        memcpy(pr + SC_PRIM_N * m, p, sizeof(double) * SC_PRIM_N);
        for (int a = 0; a < 3; a++) ol[4 * m + a] = oc[0] * R[a] + oc[1] * R[3 + a] + oc[2] * R[6 + a];
        ol[4 * m + 3] = c2;
        m++;
    }
    prims = pr; np = m;
    cast_job jobs[SC_CAST_MAX_THREADS];
    for (int t = 0; t < nt; t++) {
        cast_job *j = &jobs[t];
        j->ol = ol;
        j->o = origin; j->dirs = dirs; j->prims = prims; j->np = np; j->max_range = max_range; j->ceil_h = ceil_h;
        j->floor_gid = floor_gid; j->ceil_gid = ceil_gid; j->dist = dist; j->gid = gid; j->nrm = nrm;
        j->n0 = (int)((long long)n * t / nt); j->n1 = (int)((long long)n * (t + 1) / nt);
    }
    sc_run_jobs(cast_worker, jobs, sizeof(cast_job), nt);
    free(ol); free(pr);
    return n;
}

