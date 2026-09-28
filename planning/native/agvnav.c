/*
 * libagvnav —— 执行进程规划算法的 C 实现 (ctypes 加载，不依赖 Python 头文件)
 *
 *   an_clearance     车体在一组位姿上到线段集合的最小距离 (planning/maneuver.py clearance)
 *   an_plan_corner   拐点过弯方式: 圆弧 (多种半径) / 原地转向 (两个方向) 中车体净空足够者 (maneuver.plan_corner)
 *   an_router_*      拓扑贴合规划 (planning/dijkstra_planner.py DijkstraPlanner.plan_route):
 *                    起终点投影到可达拓扑边接入 → 状态 = (节点, 来向) 的 Dijkstra，代价 = 路程 + 转向 + 拐点停车
 *                    + 拐点转不过去的惩罚；结果点列/节点标签的整理规则与 Python 版一致
 *   与 Python 逐项一致 (tests/test_agvnav.py 随机对比)：包括 heapq 的并列顺序 (代价相同按节点名字典序)，
 *   调用方传入每个节点名的排序名次。
 *
 * 构建: bash planning/native/build.sh   加载: planning/native/__init__.py (AGV_NATIVE_PLAN=0 回退 Python)
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AN_ABI 1
int an_abi(void) { return AN_ABI; }

static double wrapa(double a) { return atan2(sin(a), cos(a)); }

/* ---------------------------------------------------------------- 车体净空 */
static int perimeter(double head, double tail, double hw, double step, double *out /* 2*cap */, int cap) {
  const double E[4][4] = {{-tail, -hw, head, -hw}, {head, -hw, head, hw}, {head, hw, -tail, hw}, {-tail, hw, -tail, -hw}};
  int k = 0;
  for (int e = 0; e < 4; ++e) {
    double ax = E[e][0], ay = E[e][1], bx = E[e][2], by = E[e][3];
    int n = (int)(hypot(bx - ax, by - ay) / step) + 1;
    if (n < 2) n = 2;
    for (int i = 0; i < n && k < cap; ++i, ++k) {
      out[2 * k] = ax + (bx - ax) * i / (n - 1);
      out[2 * k + 1] = ay + (by - ay) * i / (n - 1);
    }
  }
  return k;
}

double an_clearance(const double *segs, int ns, const double *poses, int np, double head, double tail, double hw) {
  if (ns <= 0 || np <= 0) return 9.0;
  double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
  for (int i = 0; i < np; ++i) {
    double x = poses[3 * i], y = poses[3 * i + 1];
    if (x < xmin) xmin = x;
    if (x > xmax) xmax = x;
    if (y < ymin) ymin = y;
    if (y > ymax) ymax = y;
  }
  const double R = hypot(head > tail ? head : tail, hw) + 0.3;
  int *near = (int *)malloc(sizeof(int) * ns);
  int nn = 0;
  for (int j = 0; j < ns; ++j) {
    const double *s = segs + 4 * j;
    double lox = s[0] < s[2] ? s[0] : s[2], hix = s[0] > s[2] ? s[0] : s[2];
    double loy = s[1] < s[3] ? s[1] : s[3], hiy = s[1] > s[3] ? s[1] : s[3];
    if (lox < xmax + R && hix > xmin - R && loy < ymax + R && hiy > ymin - R) near[nn++] = j;
  }
  if (!nn) { free(near); return 9.0; }
  double per[2 * 512];
  const int npp = perimeter(head, tail, hw, 0.08, per, 512);
  double best = 1e300;
  for (int i = 0; i < np; ++i) {
    const double x = poses[3 * i], y = poses[3 * i + 1], c = cos(poses[3 * i + 2]), s = sin(poses[3 * i + 2]);
    for (int q = 0; q < nn; ++q) {          /* 线段端点落入车体 → -1 */
      const double *g = segs + 4 * near[q];
      for (int e = 0; e < 2; ++e) {
        const double ex = g[2 * e], ey = g[2 * e + 1];
        const double lx = (ex - x) * c + (ey - y) * s, ly = -(ex - x) * s + (ey - y) * c;
        if (lx > -tail && lx < head && fabs(ly) < hw) { free(near); return -1.0; }
      }
    }
  }
  for (int i = 0; i < np; ++i) {
    const double x = poses[3 * i], y = poses[3 * i + 1], c = cos(poses[3 * i + 2]), s = sin(poses[3 * i + 2]);
    for (int k = 0; k < npp; ++k) {
      const double qx = x + c * per[2 * k] - s * per[2 * k + 1], qy = y + s * per[2 * k] + c * per[2 * k + 1];
      for (int q = 0; q < nn; ++q) {
        const double *g = segs + 4 * near[q];
        const double dx = g[2] - g[0], dy = g[3] - g[1];
        double L2 = dx * dx + dy * dy;
        if (L2 < 1e-12) L2 = 1e-12;
        double u = ((qx - g[0]) * dx + (qy - g[1]) * dy) / L2;
        u = u < 0 ? 0 : (u > 1 ? 1 : u);
        const double cx = g[0] + u * dx, cy = g[1] + u * dy;
        const double d2 = (qx - cx) * (qx - cx) + (qy - cy) * (qy - cy);
        if (d2 < best) best = d2;
      }
    }
  }
  free(near);
  return sqrt(best);
}

/* ---------------------------------------------------------------- 拐点过弯方式
 * mode: 0 rotate, 1 arc, 2 auto
 * out[8]: kind (1 圆弧 / 2 原地转向), rot_dir, R, d, heading, turn, cx, cy；返回净空 */
static double py_round2(double x) { return nearbyint(x * 100.0) / 100.0; }

double an_plan_corner(const double *segs, int ns, double nx, double ny, double h1, double h2, double len_in, double len_out,
                      double head, double tail, double hw, double r_pref, double clear_min, int mode, double *out) {
  const double turn = wrapa(h2 - h1);
  const double sgn = turn > 0 ? 1.0 : -1.0;
  double rot_clr[2], rot_dir[2], poses[3 * 17];
  for (int k = 0; k < 2; ++k) {
    const double tt = k == 0 ? turn : turn - sgn * 2.0 * M_PI;
    for (int i = 0; i < 17; ++i) { poses[3 * i] = nx; poses[3 * i + 1] = ny; poses[3 * i + 2] = h1 + tt * i / 16.0; }
    rot_clr[k] = an_clearance(segs, ns, poses, 17, head, tail, hw);
    rot_dir[k] = tt > 0 ? 1.0 : -1.0;
  }
#define ROT(k) do { out[0] = 2; out[1] = rot_dir[k]; for (int z = 2; z < 8; ++z) out[z] = 0; return rot_clr[k]; } while (0)
  if (mode == 2) {
    for (int k = 0; k < 2; ++k) if (rot_clr[k] >= clear_min) ROT(k);
  }
  /* 候选 (arcs + rots) 中净空最大者 (并列取先出现的) */
  double cand_clr[8], cand[8][8];
  int nc = 0;
  if ((mode == 1 || mode == 2) && fabs(turn) < 2.6) {
    const double t_half = tan(fabs(turn) / 2.0);
    const double r_lim = 0.45 * (len_in < len_out ? len_in : len_out) / (t_half > 1e-3 ? t_half : 1e-3);
    const double r_max = r_pref < r_lim ? r_pref : r_lim;
    double rs[4] = {r_pref * 1.25 < r_lim ? r_pref * 1.25 : r_lim, r_max, r_max * 0.7, r_max * 0.45};
    /* Python: sorted(set(...), reverse=True) */
    for (int i = 0; i < 4; ++i) for (int j = i + 1; j < 4; ++j) if (rs[j] > rs[i]) { double t = rs[i]; rs[i] = rs[j]; rs[j] = t; }
    double arcs_clr[4], arcs[4][8];
    int na = 0;
    for (int i = 0; i < 4; ++i) {
      if (i > 0 && rs[i] == rs[i - 1]) continue;
      const double R = rs[i];
      if (R < 0.25) continue;
      const double d = R * t_half;
      const double sx = nx - d * cos(h1), sy = ny - d * sin(h1);
      const double cx = sx - sgn * R * sin(h1), cy = sy + sgn * R * cos(h1);
      double ap[3 * 13];
      for (int k = 0; k < 13; ++k) {
        const double th = h1 + turn * k / 12.0;
        ap[3 * k] = cx + sgn * R * sin(th);
        ap[3 * k + 1] = cy - sgn * R * cos(th);
        ap[3 * k + 2] = th;
      }
      arcs_clr[na] = an_clearance(segs, ns, ap, 13, head, tail, hw);
      const double a[8] = {1, 0, R, d, h2, turn, cx, cy};
      memcpy(arcs[na], a, sizeof(a));
      ++na;
    }
    int bi = -1;
    for (int i = 0; i < na; ++i) {
      if (arcs_clr[i] < clear_min) continue;
      if (bi < 0 || py_round2(arcs_clr[i]) > py_round2(arcs_clr[bi]) ||
          (py_round2(arcs_clr[i]) == py_round2(arcs_clr[bi]) && arcs[i][2] > arcs[bi][2])) bi = i;
    }
    if (bi >= 0) { memcpy(out, arcs[bi], sizeof(arcs[bi])); return arcs_clr[bi]; }
    for (int i = 0; i < na; ++i) { cand_clr[nc] = arcs_clr[i]; memcpy(cand[nc], arcs[i], sizeof(arcs[i])); ++nc; }
  }
  for (int k = 0; k < 2; ++k) {
    const double a[8] = {2, rot_dir[k], 0, 0, 0, 0, 0, 0};
    cand_clr[nc] = rot_clr[k];
    memcpy(cand[nc], a, sizeof(a));
    ++nc;
    if (rot_clr[k] >= clear_min) ROT(k);
  }
  int bi = 0;
  for (int i = 1; i < nc; ++i) if (cand_clr[i] > cand_clr[bi]) bi = i;
  memcpy(out, cand[bi], sizeof(cand[bi]));
  return cand_clr[bi];
#undef ROT
}

/* ---------------------------------------------------------------- 拓扑贴合规划 */
#define TURN_COST_PER_RAD 0.8
#define CORNER_STOP_COST 1.5
#define CORNER_BLOCK_COST 40.0
#define OFF_NET_COST 3.0
#define ATTACH_SLACK 0.5

typedef struct { int to; double w; } Arc;
typedef struct { double key[6]; double val; int used; } CacheEnt;

typedef struct {
  int n;                      /* 真实节点数 */
  double *pos;                /* 2*(n+2): 真实节点 + START(n) + GOAL(n+1) */
  int *rank;                  /* n+2: 节点名字典序名次 (heapq 并列顺序) */
  int m;                      /* 无向边数 (连接顺序) */
  int *eu, *ev;
  double *elen;
  int *adj_off, *adj_cnt;     /* 真实节点的邻接 (按 Python edges[u] 的追加顺序) */
  Arc *adj;
  double *segs;
  int ns;
  double half_width, circum;
  int fp_on;
  double fp_head, fp_tail, fp_hw, fp_r, fp_cmin;
  int fp_mode;
  CacheEnt *cache;
  int cache_cap;
} Router;

void *an_router_new(void) { return calloc(1, sizeof(Router)); }

static void router_clear(Router *r) {
  free(r->pos); free(r->rank); free(r->eu); free(r->ev); free(r->elen); free(r->adj_off); free(r->adj_cnt); free(r->adj);
  free(r->segs); free(r->cache);
  r->pos = NULL; r->rank = NULL; r->eu = r->ev = NULL; r->elen = NULL; r->adj_off = r->adj_cnt = NULL; r->adj = NULL;
  r->segs = NULL; r->cache = NULL; r->cache_cap = 0;
}

void an_router_free(void *h) {
  if (!h) return;
  router_clear((Router *)h);
  free(h);
}

/* nodes: 2n；rank: n+2 (含 START/GOAL)；edges: 2m (节点序号，按场景 connections 顺序)；segs: 4*ns 静态墙/货架线段 */
void an_router_set_graph(void *h, const double *nodes, int n, const int *rank, const int *edges, int m, const double *segs, int ns) {
  Router *r = (Router *)h;
  router_clear(r);
  r->n = n;
  r->pos = (double *)calloc(2 * (n + 2), sizeof(double));
  memcpy(r->pos, nodes, sizeof(double) * 2 * n);
  r->rank = (int *)malloc(sizeof(int) * (n + 2));
  memcpy(r->rank, rank, sizeof(int) * (n + 2));
  r->m = m;
  r->eu = (int *)malloc(sizeof(int) * (m ? m : 1));
  r->ev = (int *)malloc(sizeof(int) * (m ? m : 1));
  r->elen = (double *)malloc(sizeof(double) * (m ? m : 1));
  r->adj_cnt = (int *)calloc(n ? n : 1, sizeof(int));
  r->adj_off = (int *)calloc(n + 1, sizeof(int));
  for (int e = 0; e < m; ++e) {
    r->eu[e] = edges[2 * e];
    r->ev[e] = edges[2 * e + 1];
    const double *a = r->pos + 2 * r->eu[e], *b = r->pos + 2 * r->ev[e];
    r->elen[e] = hypot(b[0] - a[0], b[1] - a[1]);
    r->adj_cnt[r->eu[e]]++;
    r->adj_cnt[r->ev[e]]++;
  }
  for (int i = 0; i < n; ++i) r->adj_off[i + 1] = r->adj_off[i] + r->adj_cnt[i];
  r->adj = (Arc *)malloc(sizeof(Arc) * (2 * m + 1));
  int *fill = (int *)calloc(n ? n : 1, sizeof(int));
  for (int e = 0; e < m; ++e) {           /* Python: edges[u].append((v, d)); edges[v].append((u, d)) */
    int u = r->eu[e], v = r->ev[e];
    r->adj[r->adj_off[u] + fill[u]++] = (Arc){v, r->elen[e]};
    r->adj[r->adj_off[v] + fill[v]++] = (Arc){u, r->elen[e]};
  }
  free(fill);
  r->ns = ns;
  r->segs = (double *)malloc(sizeof(double) * 4 * (ns ? ns : 1));
  memcpy(r->segs, segs, sizeof(double) * 4 * ns);
}

void an_router_set_robot(void *h, double half_width, double circum) {
  Router *r = (Router *)h;
  r->half_width = half_width;
  r->circum = circum;
}

void an_router_set_footprint(void *h, int on, double head, double tail, double hw, double rad, double cmin, int mode) {
  Router *r = (Router *)h;
  r->fp_on = on; r->fp_head = head; r->fp_tail = tail; r->fp_hw = hw; r->fp_r = rad; r->fp_cmin = cmin; r->fp_mode = mode;
  free(r->cache);
  r->cache = NULL;
  r->cache_cap = 0;
}

/* 拐点惩罚 (与 Python 同一个缓存键: 坐标保留 2 位小数) */
static double corner_penalty(Router *r, const double *a, const double *b, const double *c) {
  double key[6] = {py_round2(a[0]), py_round2(a[1]), py_round2(b[0]), py_round2(b[1]), py_round2(c[0]), py_round2(c[1])};
  if (!r->cache) { r->cache_cap = 4096; r->cache = (CacheEnt *)calloc(r->cache_cap, sizeof(CacheEnt)); }
  unsigned long hsh = 1469598103934665603ul;
  for (int i = 0; i < 6; ++i) { long long v = llround(key[i] * 100.0); hsh = (hsh ^ (unsigned long)v) * 1099511628211ul; }
  int idx = (int)(hsh % (unsigned long)r->cache_cap);
  for (int probe = 0; probe < r->cache_cap; ++probe) {
    CacheEnt *e = &r->cache[(idx + probe) % r->cache_cap];
    if (!e->used) break;
    if (!memcmp(e->key, key, sizeof(key))) return e->val;
  }
  const double h1 = atan2(b[1] - a[1], b[0] - a[0]), h2 = atan2(c[1] - b[1], c[0] - b[0]);
  double out[8];
  const double clr = an_plan_corner(r->segs, r->ns, b[0], b[1], h1, h2, hypot(b[0] - a[0], b[1] - a[1]), hypot(c[0] - b[0], c[1] - b[1]),
                                    r->fp_head, r->fp_tail, r->fp_hw, r->fp_r, r->fp_cmin, r->fp_mode, out);
  const double val = clr < r->fp_cmin ? CORNER_BLOCK_COST : 0.0;
  for (int probe = 0; probe < r->cache_cap; ++probe) {
    CacheEnt *e = &r->cache[(idx + probe) % r->cache_cap];
    if (e->used) continue;
    memcpy(e->key, key, sizeof(key));
    e->val = val;
    e->used = 1;
    break;
  }
  return val;
}

static int edge_blocked_r(const double *p1, const double *p2, const double *obs, int k, double clearance) {
  for (int i = 0; i < k; ++i) {
    const double cx = obs[3 * i], cy = obs[3 * i + 1], radius = obs[3 * i + 2] + clearance;
    const double dx = p2[0] - p1[0], dy = p2[1] - p1[1], l2 = dx * dx + dy * dy;
    double d;
    if (l2 == 0) d = hypot(cx - p1[0], cy - p1[1]);
    else {
      double t = ((cx - p1[0]) * dx + (cy - p1[1]) * dy) / l2;
      t = t < 0 ? 0 : (t > 1 ? 1 : t);
      d = hypot(cx - (p1[0] + t * dx), cy - (p1[1] + t * dy));
    }
    if (d < radius) return 1;
  }
  return 0;
}

static int node_blocked(const Router *r, const double *p, const double *obs, int k) {
  for (int i = 0; i < k; ++i)
    if (hypot(obs[3 * i] - p[0], obs[3 * i + 1] - p[1]) < obs[3 * i + 2] + r->circum) return 1;
  return 0;
}

static double orient(const double *p, const double *q, const double *s) {
  return (q[0] - p[0]) * (s[1] - p[1]) - (q[1] - p[1]) * (s[0] - p[0]);
}

static int connector_ok(const Router *r, const double *p, const double *q, const double *obs, int k) {
  if (hypot(q[0] - p[0], q[1] - p[1]) < 0.02) return 1;
  for (int j = 0; j < r->ns; ++j) {
    const double *w = r->segs + 4 * j, *c = w, *d = w + 2;
    const double o1 = orient(p, q, c), o2 = orient(p, q, d), o3 = orient(c, d, p), o4 = orient(c, d, q);
    if (o1 * o2 < 0 && o3 * o4 < 0) return 0;
  }
  return !(k && edge_blocked_r(p, q, obs, k, r->half_width));
}

typedef struct { double px, py; int link_n[2]; double link_w[2]; int nlinks; double d; int edge; int order; } Attach;

static int attach_cmp(const void *a, const void *b) {        /* 稳定: 距离相同按遍历顺序 */
  const Attach *x = (const Attach *)a, *y = (const Attach *)b;
  if (x->d < y->d) return -1;
  if (x->d > y->d) return 1;
  return x->order - y->order;
}

/* Python _attach: 按 edges 字典遍历顺序 (节点插入顺序 × 各自邻接追加顺序) 去重 */
static int attach(const Router *r, const double *pt, const double *obs, int k, const char *blocked, Attach *out, int cap) {
  int nc = 0, nf = 0;
  Attach *fb = (Attach *)malloc(sizeof(Attach) * (r->m + 1));
  char *seen = (char *)calloc(r->m + 1, 1);
  int order = 0;
  for (int u = 0; u < r->n; ++u) {
    for (int a = r->adj_off[u]; a < r->adj_off[u] + r->adj_cnt[u]; ++a) {
      const int v = r->adj[a].to;
      const double L = r->adj[a].w;
      /* 找对应的无向边序号 */
      int e = -1;
      for (int q = 0; q < r->m; ++q)
        if ((r->eu[q] == u && r->ev[q] == v) || (r->eu[q] == v && r->ev[q] == u)) { if (!seen[q]) { e = q; break; } }
      if (e < 0 || blocked[e] || L < 1e-6) continue;
      for (int q = 0; q < r->m; ++q)                       /* 同一对节点的重复边一并视为已处理 */
        if ((r->eu[q] == u && r->ev[q] == v) || (r->eu[q] == v && r->ev[q] == u)) seen[q] = 1;
      const double *U = r->pos + 2 * u, *V = r->pos + 2 * v;
      double t = ((pt[0] - U[0]) * (V[0] - U[0]) + (pt[1] - U[1]) * (V[1] - U[1])) / (L * L);
      t = t < 0 ? 0 : (t > 1 ? 1 : t);
      Attach c;
      c.px = U[0] + t * (V[0] - U[0]);
      c.py = U[1] + t * (V[1] - U[1]);
      c.d = hypot(pt[0] - c.px, pt[1] - c.py);
      if (t < 1e-3) { c.nlinks = 1; c.link_n[0] = u; c.link_w[0] = 0.0; }
      else if (t > 1 - 1e-3) { c.nlinks = 1; c.link_n[0] = v; c.link_w[0] = 0.0; }
      else { c.nlinks = 2; c.link_n[0] = u; c.link_w[0] = t * L; c.link_n[1] = v; c.link_w[1] = (1 - t) * L; }
      c.edge = e;
      c.order = order++;
      const double proj[2] = {c.px, c.py};
      if (connector_ok(r, pt, proj, obs, k)) { if (nc < cap) out[nc++] = c; }
      else fb[nf++] = c;
    }
  }
  qsort(out, nc, sizeof(Attach), attach_cmp);
  if (!nc && nf) {
    qsort(fb, nf, sizeof(Attach), attach_cmp);
    if (fb[0].d < 1.0) out[nc++] = fb[0];
  }
  free(fb);
  free(seen);
  /* near(): 与最近接入距离相差 < ATTACH_SLACK，最多 3 个 */
  int kept = 0;
  for (int i = 0; i < nc && kept < 3; ++i) if (out[i].d <= out[0].d + ATTACH_SLACK) out[kept++] = out[i];
  return kept;
}

typedef struct { double c; int n, prv; } HeapEnt;
typedef struct { HeapEnt *a; int size, cap; const int *rank; } Heap;

static int hless(const Heap *h, const HeapEnt *x, const HeapEnt *y) {      /* (c, name(n), name(prv))；prv=None 最小 */
  if (x->c != y->c) return x->c < y->c;
  if (x->n != y->n) return h->rank[x->n] < h->rank[y->n];
  int rx = x->prv < 0 ? -1 : h->rank[x->prv], ry = y->prv < 0 ? -1 : h->rank[y->prv];
  return rx < ry;
}
static void hpush(Heap *h, HeapEnt e) {
  if (h->size == h->cap) { h->cap = h->cap ? 2 * h->cap : 256; h->a = (HeapEnt *)realloc(h->a, sizeof(HeapEnt) * h->cap); }
  int i = h->size++;
  h->a[i] = e;
  while (i > 0) {
    int p = (i - 1) / 2;
    if (!hless(h, &h->a[i], &h->a[p])) break;
    HeapEnt t = h->a[i]; h->a[i] = h->a[p]; h->a[p] = t;
    i = p;
  }
}
static HeapEnt hpop(Heap *h) {
  HeapEnt top = h->a[0];
  h->a[0] = h->a[--h->size];
  int i = 0;
  for (;;) {
    int l = 2 * i + 1, rr = l + 1, s = i;
    if (l < h->size && hless(h, &h->a[l], &h->a[s])) s = l;
    if (rr < h->size && hless(h, &h->a[rr], &h->a[s])) s = rr;
    if (s == i) break;
    HeapEnt t = h->a[i]; h->a[i] = h->a[s]; h->a[s] = t;
    i = s;
  }
  return top;
}

static double turn_ang(const double *a, const double *b, const double *c) {
  if (hypot(b[0] - a[0], b[1] - a[1]) < 1e-6 || hypot(c[0] - b[0], c[1] - b[1]) < 1e-6) return 0.0;
  const double h1 = atan2(b[1] - a[1], b[0] - a[0]), h2 = atan2(c[1] - b[1], c[0] - b[0]);
  return fabs(atan2(sin(h2 - h1), cos(h2 - h1)));
}

/* obs: 3k (中心 x, y, 外接圆半径)；out_pts: 2*max_out，out_labels: 节点序号 (-1 = 非节点)；返回点数 (0 = 无路径) */
int an_router_plan(void *h, double sx, double sy, double gx, double gy, const double *obs, int k,
                   double *out_pts, int *out_labels, int max_out, double *out_length) {
  Router *r = (Router *)h;
  const int n = r->n, S = n, G = n + 1, N = n + 2;
  char *blocked = (char *)calloc(r->m + 1, 1);
  if (k) for (int e = 0; e < r->m; ++e) blocked[e] = (char)edge_blocked_r(r->pos + 2 * r->eu[e], r->pos + 2 * r->ev[e], obs, k, r->half_width + 0.15);
  const double sp[2] = {sx, sy}, gp[2] = {gx, gy};
  Attach sc[3], gc[3];
  Attach *tmp = (Attach *)malloc(sizeof(Attach) * (r->m + 1));
  int ns_ = attach(r, sp, obs, k, blocked, tmp, r->m + 1);
  memcpy(sc, tmp, sizeof(Attach) * ns_);
  int ng_ = attach(r, gp, obs, k, blocked, tmp, r->m + 1);
  memcpy(gc, tmp, sizeof(Attach) * ng_);
  free(tmp);
  *out_length = 0.0;
  if (!ns_ || !ng_) { free(blocked); return 0; }

  /* 状态 (节点, 来向)：来向 -1..N-1 → 下标 (prv+1) */
  const int NS = N * (N + 1);
  double *cost = (double *)malloc(sizeof(double) * NS);
  int *par = (int *)malloc(sizeof(int) * NS);
  /* 扩展邻接: 真实节点 (过滤阻断边) + START/GOAL 接入 */
  Arc *xadj = (Arc *)malloc(sizeof(Arc) * (2 * r->m + 16));
  int *xoff = (int *)malloc(sizeof(int) * (N + 1)), *xcnt = (int *)malloc(sizeof(int) * N);
  double best_total = 0;
  int best_len = 0, have_best = 0;
  int *best_seq = (int *)malloc(sizeof(int) * (N + 2));
  double best_pos_s[2] = {0, 0}, best_pos_g[2] = {0, 0};
  Heap hp = {0};
  hp.rank = r->rank;
  for (int si = 0; si < ns_; ++si) {
    for (int gi = 0; gi < ng_; ++gi) {
      const Attach *A = &sc[si], *B = &gc[gi];
      r->pos[2 * S] = A->px; r->pos[2 * S + 1] = A->py;
      r->pos[2 * G] = B->px; r->pos[2 * G + 1] = B->py;
      /* 组装: 真实节点邻接 (保持顺序) + 追加 START/GOAL (按 s_links、g_links 顺序) */
      int w = 0;
      for (int u = 0; u < N; ++u) {
        xoff[u] = w;
        if (u < n) {
          for (int a = r->adj_off[u]; a < r->adj_off[u] + r->adj_cnt[u]; ++a) {
            int v = r->adj[a].to, e = -1;
            for (int q = 0; q < r->m; ++q) if ((r->eu[q] == u && r->ev[q] == v) || (r->eu[q] == v && r->ev[q] == u)) { e = q; if (blocked[q]) break; }
            if (e >= 0 && blocked[e]) continue;
            xadj[w++] = r->adj[a];
          }
          for (int q = 0; q < A->nlinks; ++q) if (A->link_n[q] == u) xadj[w++] = (Arc){S, A->link_w[q]};
          for (int q = 0; q < B->nlinks; ++q) if (B->link_n[q] == u) xadj[w++] = (Arc){G, B->link_w[q]};
        } else if (u == S) {
          for (int q = 0; q < A->nlinks; ++q) xadj[w++] = (Arc){A->link_n[q], A->link_w[q]};
          if (A->edge == B->edge) xadj[w++] = (Arc){G, hypot(A->px - B->px, A->py - B->py)};
        }
        xcnt[u] = w - xoff[u];
      }
      for (int i = 0; i < NS; ++i) { cost[i] = INFINITY; par[i] = -2; }
      hp.size = 0;
      const double c0 = A->d * OFF_NET_COST;
      cost[S * (N + 1) + 0] = c0;
      hpush(&hp, (HeapEnt){c0, S, -1});
      int done_n = -1, done_p = -1;
      double done_c = 0;
      while (hp.size) {
        HeapEnt t = hpop(&hp);
        const int si2 = t.n * (N + 1) + (t.prv + 1);
        if (t.c > cost[si2] + 1e-9) continue;
        if (t.n == G) { done_n = t.n; done_p = t.prv; done_c = t.c; break; }
        for (int a = xoff[t.n]; a < xoff[t.n] + xcnt[t.n]; ++a) {
          const int mm = xadj[a].to;
          if (mm == t.prv || mm == S) continue;
          if (k && mm != G && mm < n && node_blocked(r, r->pos + 2 * mm, obs, k)) continue;
          double tc = 0.0;
          if (t.prv >= 0) {
            const double ang = turn_ang(r->pos + 2 * t.prv, r->pos + 2 * t.n, r->pos + 2 * mm);
            tc = ang * TURN_COST_PER_RAD + (ang > 0.02 ? CORNER_STOP_COST : 0.0);
            if (ang > 0.02 && r->fp_on && t.n < n) tc += corner_penalty(r, r->pos + 2 * t.prv, r->pos + 2 * t.n, r->pos + 2 * mm);
          }
          const double nc = t.c + xadj[a].w + tc;
          const int sm = mm * (N + 1) + (t.n + 1);
          if (nc < cost[sm] - 1e-9) {
            cost[sm] = nc;
            par[sm] = si2;
            hpush(&hp, (HeapEnt){nc, mm, t.n});
          }
        }
      }
      if (done_n < 0) continue;
      const double total = done_c + B->d * OFF_NET_COST;
      if (!have_best || total < best_total) {
        int len = 0, st = done_n * (N + 1) + (done_p + 1);
        int seq_rev[4096];
        while (par[st] != -2 && len < 4095) { seq_rev[len++] = st / (N + 1); st = par[st]; }
        seq_rev[len++] = S;
        for (int i = 0; i < len; ++i) best_seq[i] = seq_rev[len - 1 - i];
        best_len = len;
        best_total = total;
        have_best = 1;
        best_pos_s[0] = A->px; best_pos_s[1] = A->py;
        best_pos_g[0] = B->px; best_pos_g[1] = B->py;
      }
    }
  }
  free(hp.a); free(cost); free(par); free(xadj); free(xoff); free(xcnt); free(blocked);
  if (!have_best) { free(best_seq); return 0; }
  /* 点列与标签整理 (与 Python 一致) */
  int cnt = 0;
  double *P = out_pts;
  int *Lb = out_labels;
  P[0] = sx; P[1] = sy; Lb[0] = -1; cnt = 1;
  for (int i = 0; i < best_len && cnt < max_out; ++i) {
    const int nd = best_seq[i];
    const double *p = nd == S ? best_pos_s : (nd == G ? best_pos_g : r->pos + 2 * nd);
    const int lab = (nd == S || nd == G) ? -1 : nd;
    if (hypot(p[0] - P[2 * (cnt - 1)], p[1] - P[2 * (cnt - 1) + 1]) > 0.08) {
      P[2 * cnt] = p[0]; P[2 * cnt + 1] = p[1]; Lb[cnt] = lab; ++cnt;
    } else if (lab >= 0 && Lb[cnt - 1] < 0) {
      Lb[cnt - 1] = lab;
    }
  }
  free(best_seq);
  if (cnt < max_out && hypot(gx - P[2 * (cnt - 1)], gy - P[2 * (cnt - 1) + 1]) > 0.08) {
    P[2 * cnt] = gx; P[2 * cnt + 1] = gy; Lb[cnt] = -1; ++cnt;
  }
  if (cnt > 2 && hypot(P[2] - P[0], P[3] - P[1]) < 0.35) {
    memmove(P, P + 2, sizeof(double) * 2 * (cnt - 1));
    memmove(Lb, Lb + 1, sizeof(int) * (cnt - 1));
    --cnt;
  }
  double length = 0;
  for (int i = 1; i < cnt; ++i) length += hypot(P[2 * i] - P[2 * i - 2], P[2 * i + 1] - P[2 * i - 1]);
  *out_length = nearbyint(length * 100.0) / 100.0;
  return cnt;
}
