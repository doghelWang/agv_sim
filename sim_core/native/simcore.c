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
#include <stdlib.h>
#include <string.h>

#define SC_ABI 1

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
