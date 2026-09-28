/*
 * libagvnav · 内置激光 SLAM 的计算内核 (nav_runtime/slam.py 的 GridMap.insert / build_fields / match)
 *
 *   sl_bounds       一帧光束 (含无回波方向按 free_cap 截断) 的世界包围盒 → 调用方扩展地图
 *   sl_insert       占据栅格插入: 光束穿过的格子 (去重) 降低 log-odds，终点格子提高；终点命中密度双线性分摊
 *   sl_field_bbox   占据掩码 (L > 0) 外扩 2 格、包围盒 + 边距
 *   sl_field_level  匹配场一级: 命中密度 (掩码内) → 下采样求和 → 可分离高斯模糊 → 局部最大值归一化 → [0,1]
 *   sl_match        Gauss-Newton 扫描匹配 (粗→细两级，带里程计先验)，协方差与退化判据
 *
 *   浮点语义与 numpy 版一致: 栅格/匹配场为 float32，逐项按同样的顺序累加 (np.add.at 按偏移分组顺序、
 *   模糊按核下标顺序)；编译时 -ffp-contract=off (不合成 FMA)。tests/test_slamcore.py 对比。
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

void sl_bounds(double x, double y, double th, const double *px, const double *py, const unsigned char *hit,
               const double *sx, const double *sy, int n, double free_cap, double *out) {
  const double c = cos(th), s = sin(th);
  double xmin = x, ymin = y, xmax = x, ymax = y;
  for (int i = 0; i < n; ++i) {
    const double wsx = x + c * sx[i] - s * sy[i], wsy = y + s * sx[i] + c * sy[i];
    const double ex = x + c * px[i] - s * py[i], ey = y + s * px[i] + c * py[i];
    const double dx = ex - wsx, dy = ey - wsy, rng = hypot(dx, dy), m = rng > 1e-9 ? rng : 1e-9;
    const double reach = hit[i] ? rng : (rng < free_cap ? rng : free_cap);
    const double fx = wsx + reach * (dx / m), fy = wsy + reach * (dy / m);
    if (fx < xmin) xmin = fx;
    if (fy < ymin) ymin = fy;
    if (fx > xmax) xmax = fx;
    if (fy > ymax) ymax = fy;
  }
  out[0] = xmin; out[1] = ymin; out[2] = xmax; out[3] = ymax;
}

long sl_insert(float *L, float *Hd, int h, int w, double ox, double oy, double res, double x, double y, double th,
               const double *px, const double *py, const unsigned char *hit, const double *sx, const double *sy, int n,
               double free_cap, float L_FREE, float L_OCC, float L_MIN, float L_MAX, float HIT_CAP) {
  const double c = cos(th), s = sin(th);
  const double step = res * 0.8 > 0.03 ? res * 0.8 : 0.03;
  const double back = 4 * res > 0.10 ? 4 * res : 0.10;
  double *wsx = malloc(sizeof(double) * n), *wsy = malloc(sizeof(double) * n), *ca = malloc(sizeof(double) * n),
         *sa = malloc(sizeof(double) * n), *ex = malloc(sizeof(double) * n), *ey = malloc(sizeof(double) * n);
  long *k = malloc(sizeof(long) * (n ? n : 1));
  long tot = 0;
  for (int i = 0; i < n; ++i) {
    wsx[i] = x + c * sx[i] - s * sy[i];
    wsy[i] = y + s * sx[i] + c * sy[i];
    ex[i] = x + c * px[i] - s * py[i];
    ey[i] = y + s * px[i] + c * py[i];
    const double dx = ex[i] - wsx[i], dy = ey[i] - wsy[i], rng = hypot(dx, dy), m = rng > 1e-9 ? rng : 1e-9;
    ca[i] = dx / m;
    sa[i] = dy / m;
    const double reach = hit[i] ? rng : (rng < free_cap ? rng : free_cap);
    const double rf = hit[i] ? rng - back : reach;
    long ki = (long)(rf / step);
    k[i] = ki > 0 ? ki : 0;
    tot += k[i];
  }
  if (tot > 300000) {                       /* 光束太多: 抽稀做空闲更新 (终点仍全部使用) */
    const long every = (long)ceil((double)tot / 300000.0);
    for (int i = 0; i < n; ++i) if (i % every) k[i] = 0;
  }
  const long cells = (long)h * w;
  unsigned char *mark = calloc(cells, 1);   /* 1 空闲 2 占据 */
  long *freel = malloc(sizeof(long) * 1024), nfree = 0, capf = 1024;
  long *occl = malloc(sizeof(long) * (n ? n : 1)), nocc = 0;
  for (int i = 0; i < n; ++i) {
    for (long j = 0; j < k[i]; ++j) {
      const double d = ((double)j + 0.5) * step;
      const long ix = (long)((wsx[i] + d * ca[i] - ox) / res), iy = (long)((wsy[i] + d * sa[i] - oy) / res);
      if (ix < 0 || iy < 0 || ix >= w || iy >= h) continue;
      const long id = iy * w + ix;
      if (mark[id]) continue;
      mark[id] = 1;
      if (nfree == capf) { capf *= 2; freel = realloc(freel, sizeof(long) * capf); }
      freel[nfree++] = id;
    }
  }
  for (int i = 0; i < n; ++i) {
    if (!hit[i]) continue;
    const long ix = (long)((ex[i] - ox) / res), iy = (long)((ey[i] - oy) / res);
    if (ix < 0 || iy < 0 || ix >= w || iy >= h) continue;
    const long id = iy * w + ix;
    if (mark[id] == 2) continue;
    mark[id] = 2;
    occl[nocc++] = id;
  }
  for (long q = 0; q < nfree; ++q) {
    const long id = freel[q];
    if (mark[id] != 1) continue;             /* free - occ */
    const float v = L[id] + L_FREE;
    L[id] = v > L_MIN ? v : L_MIN;
  }
  for (long q = 0; q < nocc; ++q) {
    const long id = occl[q];
    const float v = L[id] + L_OCC;
    L[id] = v < L_MAX ? v : L_MAX;
  }
  /* 命中密度: 双线性分摊到 4 个格心 (与 np.add.at 相同: 先全部 (0,0)，再 (1,0)、(0,1)、(1,1)) */
  for (int pass = 0; pass < 4; ++pass) {
    const int di = pass & 1, dj = pass >> 1;
    for (int i = 0; i < n; ++i) {
      if (!hit[i]) continue;
      const double gx = (ex[i] - ox) / res - 0.5, gy = (ey[i] - oy) / res - 0.5;
      const double i0 = floor(gx), j0 = floor(gy), fx = gx - i0, fy = gy - j0;
      const long ii = (long)i0 + di, jj = (long)j0 + dj;
      if (ii < 0 || ii >= w || jj < 0 || jj >= h) continue;
      const double wt = (di ? fx : 1 - fx) * (dj ? fy : 1 - fy);
      Hd[jj * w + ii] += (float)wt;
    }
  }
  for (long q = 0; q < nocc; ++q) if (Hd[occl[q]] > HIT_CAP) Hd[occl[q]] = HIT_CAP;
  free(mark); free(freel); free(occl); free(k); free(wsx); free(wsy); free(ca); free(sa); free(ex); free(ey);
  return nocc;
}

/* 占据掩码 (L > 0) 外扩 2 格 (5×5 最大值滤波)；返回是否有占据，bbox = y0, y1, x0, x1 (含边距 margin 格) */
int sl_field_bbox(const float *L, int h, int w, int margin, unsigned char *mask, int *bbox) {
  unsigned char *occ = malloc((size_t)h * w), *tmp = calloc((size_t)h * w, 1);
  for (long i = 0; i < (long)h * w; ++i) occ[i] = L[i] > 0.0f;
  for (int y = 0; y < h; ++y)                /* 沿 y (axis 0)，逐行访问 */
    for (int d = -2; d <= 2; ++d) {
      const int yy = y + d;
      if (yy < 0 || yy >= h) continue;
      const unsigned char *src = occ + (long)yy * w;
      unsigned char *dst = tmp + (long)y * w;
      for (int x = 0; x < w; ++x) dst[x] |= src[x];
    }
  int y0 = h, y1 = -1, x0 = w, x1 = -1;
  unsigned char *pad = calloc((size_t)w + 4, 1);
  for (int y = 0; y < h; ++y) {              /* 沿 x (axis 1)，两侧补 0 的行缓冲 */
    memcpy(pad + 2, tmp + (long)y * w, (size_t)w);
    unsigned char *out = mask + (long)y * w;
    int any = 0;
    for (int x = 0; x < w; ++x) {
      const unsigned char v = pad[x] | pad[x + 1] | pad[x + 2] | pad[x + 3] | pad[x + 4];
      out[x] = v;
      any |= v;
    }
    if (!any) continue;
    if (y < y0) y0 = y;
    y1 = y;
    int a = 0, b = w - 1;
    while (!out[a]) ++a;
    while (!out[b]) --b;
    if (a < x0) x0 = a;
    if (b > x1) x1 = b;
  }
  free(pad);
  free(occ);
  free(tmp);
  if (y1 < 0) return 0;
  bbox[0] = y0 - margin > 0 ? y0 - margin : 0;
  bbox[1] = y1 + margin + 1 < h ? y1 + margin + 1 : h;
  bbox[2] = x0 - margin > 0 ? x0 - margin : 0;
  bbox[3] = x1 + margin + 1 < w ? x1 + margin + 1 : w;
  return 1;
}

/* 逐格按核下标 i 递增累加 (与 numpy 版 acc += k[i] * p[i:i+n] 相同顺序)；axis 0 按行块访问，内层连续 */
static void blur_axis(const float *in, float *out, int h, int w, int axis, const float *kern, int rad) {
  memset(out, 0, sizeof(float) * (size_t)h * w);
  if (axis == 0) {
    for (int y = 0; y < h; ++y) {
      float *o = out + (long)y * w;
      for (int i = 0; i < 2 * rad + 1; ++i) {
        const int src = y + i - rad;
        const float kv = kern[i];
        if (src < 0 || src >= h) { for (int x = 0; x < w; ++x) { const float t = kv * 0.0f; o[x] = o[x] + t; } continue; }
        const float *r = in + (long)src * w;
        for (int x = 0; x < w; ++x) { const float t = kv * r[x]; o[x] = o[x] + t; }
      }
    }
  } else {
    float *pad = calloc((size_t)w + 2 * rad, sizeof(float));
    for (int y = 0; y < h; ++y) {
      memcpy(pad + rad, in + (long)y * w, sizeof(float) * w);
      float *o = out + (long)y * w;
      for (int i = 0; i < 2 * rad + 1; ++i) {
        const float kv = kern[i], *r = pad + i;
        for (int x = 0; x < w; ++x) { const float t = kv * r[x]; o[x] = o[x] + t; }
      }
    }
    free(pad);
  }
}

static void maxf_axis(const float *in, float *out, int h, int w, int axis, int rad) {
  memcpy(out, in, sizeof(float) * (size_t)h * w);
  float *pad = calloc((size_t)w + 2 * rad, sizeof(float));   /* axis 1 用: 两侧补 0 */
  for (int y = 0; y < h; ++y) {
    float *o = out + (long)y * w;
    if (axis == 0) {
      for (int i = -rad; i <= rad; ++i) {
        const int src = y + i;
        if (src < 0 || src >= h) { for (int x = 0; x < w; ++x) if (0.0f > o[x]) o[x] = 0.0f; continue; }
        const float *r = in + (long)src * w;
        for (int x = 0; x < w; ++x) if (r[x] > o[x]) o[x] = r[x];
      }
    } else {
      memcpy(pad + rad, in + (long)y * w, sizeof(float) * w);
      for (int i = 0; i <= 2 * rad; ++i) {
        const float *r = pad + i;
        for (int x = 0; x < w; ++x) if (r[x] > o[x]) o[x] = r[x];
      }
    }
  }
  free(pad);
}

/* 一级匹配场。Hd/mask 为整幅 (宽 w)，bbox 为 sl_field_bbox 的结果；k 下采样倍数；输出 F (尺寸 *oh × *ow) */
void sl_field_level(const float *Hd, const unsigned char *mask, int w, const int *bbox, int k, double sigma_cells, int rad_cells,
                    float *F, int *oh, int *ow) {
  const int h0 = bbox[1] - bbox[0], w0 = bbox[3] - bbox[2];
  const int h1 = k > 1 ? (h0 + k - 1) / k : h0, w1 = k > 1 ? (w0 + k - 1) / k : w0;
  float *H = calloc((size_t)h1 * w1, sizeof(float));
  for (int y = 0; y < h1; ++y)
    for (int x = 0; x < w1; ++x) {
      float acc = 0.0f;
      for (int a = 0; a < (k > 1 ? k : 1); ++a)
        for (int b = 0; b < (k > 1 ? k : 1); ++b) {
          const int yy = y * (k > 1 ? k : 1) + a, xx = x * (k > 1 ? k : 1) + b;
          if (yy >= h0 || xx >= w0) continue;
          const long id = (long)(yy + bbox[0]) * w + (xx + bbox[2]);
          if (mask[id]) acc = acc + Hd[id];
        }
      H[(long)y * w1 + x] = acc;
    }
  const int rad = (int)ceil(3 * sigma_cells) > 1 ? (int)ceil(3 * sigma_cells) : 1;
  float *kern = malloc(sizeof(float) * (2 * rad + 1));
  double ks = 0;
  double *kd = malloc(sizeof(double) * (2 * rad + 1));
  for (int i = -rad; i <= rad; ++i) { kd[i + rad] = exp(-0.5 * (i / sigma_cells) * (i / sigma_cells)); ks += kd[i + rad]; }
  for (int i = 0; i < 2 * rad + 1; ++i) kern[i] = (float)(kd[i] / ks);
  float *T = malloc(sizeof(float) * h1 * w1), *B = malloc(sizeof(float) * h1 * w1), *M = malloc(sizeof(float) * h1 * w1);
  blur_axis(H, T, h1, w1, 0, kern, rad);
  blur_axis(T, B, h1, w1, 1, kern, rad);
  maxf_axis(B, T, h1, w1, 0, rad_cells);
  maxf_axis(T, M, h1, w1, 1, rad_cells);
  float bmax = 0.0f;
  for (long i = 0; i < (long)h1 * w1; ++i) if (B[i] > bmax) bmax = B[i];
  const float floor_v = (float)(1e-3 * (double)bmax + 1e-6);
  for (long i = 0; i < (long)h1 * w1; ++i) {
    const float d = M[i] > floor_v ? M[i] : floor_v;
    float v = B[i] / d;
    F[i] = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
  }
  *oh = h1;
  *ow = w1;
  free(H); free(kern); free(kd); free(T); free(B); free(M);
}

/* ---- 扫描匹配 */
typedef struct { const float *F; int h, w; double ox, oy, res; } Fld;

static void fsample(const Fld *f, double wx, double wy, double *val, double *dx, double *dy) {
  const double gx = (wx - f->ox) / f->res - 0.5, gy = (wy - f->oy) / f->res - 0.5;
  const long i0 = (long)floor(gx), j0 = (long)floor(gy);
  const int mx = f->w - 2, my = f->h - 2;
  if (!(i0 >= 0 && i0 <= mx && j0 >= 0 && j0 <= my)) { *val = *dx = *dy = 0.0; return; }
  const double fx = gx - (double)i0, fy = gy - (double)j0;
  const double v00 = f->F[j0 * f->w + i0], v10 = f->F[j0 * f->w + i0 + 1];
  const double v01 = f->F[(j0 + 1) * f->w + i0], v11 = f->F[(j0 + 1) * f->w + i0 + 1];
  *val = (1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v10 + (1 - fx) * fy * v01 + fx * fy * v11;
  *dx = ((1 - fy) * (v10 - v00) + fy * (v11 - v01)) / f->res;
  *dy = ((1 - fx) * (v01 - v00) + fx * (v11 - v10)) / f->res;
}

static int inv3(const double *A, double *o) {
  const double a = A[0], b = A[1], c = A[2], d = A[3], e = A[4], f = A[5], g = A[6], h = A[7], i = A[8];
  const double C0 = e * i - f * h, C1 = -(d * i - f * g), C2 = d * h - e * g;
  const double det = a * C0 + b * C1 + c * C2;
  if (fabs(det) < 1e-300) return 0;
  o[0] = C0 / det; o[1] = -(b * i - c * h) / det; o[2] = (b * f - c * e) / det;
  o[3] = C1 / det; o[4] = (a * i - c * g) / det; o[5] = -(a * f - c * d) / det;
  o[6] = C2 / det; o[7] = -(a * h - b * g) / det; o[8] = (a * e - b * d) / det;
  return 1;
}

static double wrapa(double a) {                 /* Python: (a + π) % (2π) - π */
  double r = fmod(a + M_PI, 2 * M_PI);
  if (r < 0) r += 2 * M_PI;
  return r - M_PI;
}

/* info: inliers(F>0.3 比例), score(F 均值), eig_min, eig_max (平移信息矩阵特征值)；返回 1 成功 */
int sl_match(const float *Ff, int fh, int fw, double fox, double foy, double fres, const float *Fc, int chh, int cw, double cox,
             double coy, double cres, const double *px, const double *py, int n, const double *init, const double *P0, double sigma,
             int it_coarse, int it_fine, double *pose, double *cov, double *info) {
  const Fld fine = {Ff, fh, fw, fox, foy, fres}, coarse = {Fc, chh, cw, cox, coy, cres};
  double x[3] = {init[0], init[1], init[2]}, x0[3] = {init[0], init[1], init[2]};
  double Pinv[9] = {0};
  if (P0 && !inv3(P0, Pinv)) memset(Pinv, 0, sizeof(Pinv));
  double *wgt = malloc(sizeof(double) * (n ? n : 1)), *Fv = malloc(sizeof(double) * (n ? n : 1));
  double ws = 0;
  for (int i = 0; i < n; ++i) {
    const double rr = hypot(px[i], py[i]), q = 0.015 / (0.015 + 0.002 * rr);
    wgt[i] = q * q;
    ws += wgt[i];
  }
  const double wm = n ? ws / n : 1.0;
  for (int i = 0; i < n; ++i) wgt[i] /= wm;
  double JtJ[9] = {0};
  int have_f = 0;
  for (int lvl = 0; lvl < 2; ++lvl) {
    const Fld *f = lvl == 0 ? &coarse : &fine;
    const int its = lvl == 0 ? it_coarse : it_fine;
    const double s2 = sigma * sigma * (lvl == 0 ? 4.0 : 1.0);
    for (int it = 0; it < its; ++it) {
      const double c = cos(x[2]), s = sin(x[2]);
      double b[3] = {0, 0, 0};
      memset(JtJ, 0, sizeof(JtJ));
      for (int i = 0; i < n; ++i) {
        const double wx = x[0] + c * px[i] - s * py[i], wy = x[1] + s * px[i] + c * py[i];
        double F, gx, gy;
        fsample(f, wx, wy, &F, &gx, &gy);
        Fv[i] = F;
        const double J[3] = {gx, gy, gx * (-s * px[i] - c * py[i]) + gy * (c * px[i] - s * py[i])};
        const double r = 1.0 - F;
        for (int a = 0; a < 3; ++a) {
          b[a] += J[a] * wgt[i] * r;
          for (int bb = 0; bb < 3; ++bb) JtJ[a * 3 + bb] += J[a] * wgt[i] * J[bb];
        }
      }
      have_f = 1;
      double e[3] = {x[0] - x0[0], x[1] - x0[1], wrapa(x[2] - x0[2])};
      double H[9], Hi[9], rhs[3];
      for (int a = 0; a < 9; ++a) H[a] = JtJ[a] / s2 + Pinv[a] + (a % 4 == 0 ? 1e-6 : 0.0);
      for (int a = 0; a < 3; ++a) rhs[a] = b[a] / s2 - (Pinv[a * 3] * e[0] + Pinv[a * 3 + 1] * e[1] + Pinv[a * 3 + 2] * e[2]);
      if (!inv3(H, Hi)) break;
      double dx[3];
      for (int a = 0; a < 3; ++a) dx[a] = Hi[a * 3] * rhs[0] + Hi[a * 3 + 1] * rhs[1] + Hi[a * 3 + 2] * rhs[2];
      for (int a = 0; a < 2; ++a) dx[a] = dx[a] < -0.2 ? -0.2 : (dx[a] > 0.2 ? 0.2 : dx[a]);
      dx[2] = dx[2] < -0.1 ? -0.1 : (dx[2] > 0.1 ? 0.1 : dx[2]);
      x[0] += dx[0]; x[1] += dx[1]; x[2] = wrapa(x[2] + dx[2]);
      if (fabs(dx[0]) < 1e-5 && fabs(dx[1]) < 1e-5 && fabs(dx[2]) < 1e-5) break;
    }
  }
  double Hm[9], A[9];
  for (int a = 0; a < 9; ++a) { Hm[a] = JtJ[a] / (sigma * sigma); A[a] = Hm[a] + Pinv[a] + (a % 4 == 0 ? 1e-9 : 0.0); }
  if (!inv3(A, cov)) {
    if (P0) memcpy(cov, P0, sizeof(double) * 9);
    else { memset(cov, 0, sizeof(double) * 9); cov[0] = cov[4] = cov[8] = 1.0; }
  }
  const double ta = Hm[0], tb = Hm[1], td = Hm[4], mid = (ta + td) / 2, disc = sqrt(((ta - td) / 2) * ((ta - td) / 2) + tb * tb);
  int in = 0;
  double sc = 0;
  for (int i = 0; i < n; ++i) { in += Fv[i] > 0.3; sc += Fv[i]; }
  info[0] = (have_f && n) ? (double)in / n : 0.0;
  info[1] = (have_f && n) ? sc / n : 0.0;
  info[2] = mid - disc;
  info[3] = mid + disc;
  pose[0] = x[0]; pose[1] = x[1]; pose[2] = x[2];
  free(wgt); free(Fv);
  return 1;
}
