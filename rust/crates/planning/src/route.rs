//! Topological road-network planner (port of planning/dijkstra_planner.py: plan_route, plan_route_dir).
use crate::maneuver::{self, CornerMode};
use crate::scenario::{Library, Scenario};
use crate::{wrap, Pt, Seg};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

const NONE: usize = usize::MAX;

#[derive(Clone, Copy, Debug)]
pub struct Obs { pub cx: f64, pub cy: f64, pub r: f64 }
impl Obs {
    /// dict obstacle {x,y,w,h}: radius = hypot(w,h)/2
    pub fn from_value(v: &serde_json::Value) -> Obs {
        let g = |k: &str, d: f64| v.get(k).and_then(|x| x.as_f64()).unwrap_or(d);
        Obs { cx: g("x", 0.0), cy: g("y", 0.0), r: g("w", 0.8).hypot(g("h", 0.8)) / 2.0 }
    }
    pub fn from_rect(x1: f64, y1: f64, x2: f64, y2: f64) -> Obs { Obs { cx: (x1 + x2) / 2.0, cy: (y1 + y2) / 2.0, r: 0.3 } }
}

#[derive(Clone, Debug, Default)]
pub struct Route {
    pub points: Vec<Pt>,
    pub labels: Vec<Option<String>>,
    pub length: f64,
    pub reverse: Vec<bool>,
    pub blocked: Vec<(f64, f64, String)>,
    pub dir: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Footprint { pub head: f64, pub tail: f64, pub hw: f64, pub r: f64, pub clear_min: f64, pub mode: CornerMode }

struct Attach { proj: Pt, links: Vec<(usize, f64)>, d: f64, edge: (usize, usize) }

#[derive(PartialEq)]
struct Qe { c: f64, n: String, p: String, ni: usize, pi: usize }
impl Eq for Qe {}
impl PartialOrd for Qe { fn partial_cmp(&self, o: &Self) -> Option<Ordering> { Some(self.cmp(o)) } }
impl Ord for Qe {
    // min-heap on (cost, node name, prev name) like Python's heapq on tuples
    fn cmp(&self, o: &Self) -> Ordering { o.c.total_cmp(&self.c).then_with(|| o.n.cmp(&self.n)).then_with(|| o.p.cmp(&self.p)) }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct St { n: usize, prv: usize, rev: bool, h: i64 }

#[derive(PartialEq)]
struct Qd { c: f64, tie: u64, st: St }
impl Eq for Qd {}
impl PartialOrd for Qd { fn partial_cmp(&self, o: &Self) -> Option<Ordering> { Some(self.cmp(o)) } }
impl Ord for Qd { fn cmp(&self, o: &Self) -> Ordering { o.c.total_cmp(&self.c).then_with(|| o.tie.cmp(&self.tie)) } }

pub struct Planner {
    pub sc: Scenario,
    pub names: Vec<String>,
    pub pos: Vec<Pt>,
    pub edges: Vec<Vec<(usize, f64)>>,
    statics: Vec<Seg>,
    pub robot_half_width: f64,
    pub robot_circum_radius: f64,
    pub footprint: Option<Footprint>,
    pub allow_reverse: bool,
    pub rot_blocked: Vec<Pt>,
    pub last_route: Route,
    ccache: HashMap<[i64; 6], f64>,
    rcache: HashMap<[i64; 5], f64>,
}

pub const TURN_COST_PER_RAD: f64 = 0.8;
pub const CORNER_STOP_COST: f64 = 1.5;
pub const CORNER_BLOCK_COST: f64 = 40.0;
pub const OFF_NET_COST: f64 = 3.0;
pub const ATTACH_SLACK: f64 = 0.5;
pub const REVERSE_COST: f64 = 3.0;
pub const DIR_SWITCH_COST: f64 = 1.0;

fn k100(x: f64) -> i64 { (x * 100.0).round_ties_even() as i64 }
fn dist(a: Pt, b: Pt) -> f64 { (b.0 - a.0).hypot(b.1 - a.1) }

impl Planner {
    pub fn new(lib: &Library, default_scenario: &str) -> Planner {
        let mut p = Planner {
            sc: lib.get("grid_9_square").or(lib.list.first()).cloned().expect("no scenarios"),
            names: vec![], pos: vec![], edges: vec![], statics: vec![],
            robot_half_width: 0.35, robot_circum_radius: 0.6, footprint: None, allow_reverse: true,
            rot_blocked: vec![], last_route: Route::default(), ccache: HashMap::new(), rcache: HashMap::new(),
        };
        p.set_scenario(lib, default_scenario);
        p
    }

    pub fn set_scenario(&mut self, lib: &Library, id: &str) {
        let sc = lib.get(id).or_else(|| lib.get("grid_9_square")).cloned().unwrap_or_else(|| self.sc.clone());
        self.ccache.clear();
        self.rcache.clear();
        self.names = sc.nodes.iter().map(|(k, _)| k.clone()).collect();
        self.pos = sc.nodes.iter().map(|(_, p)| *p).collect();
        self.edges = vec![vec![]; self.names.len()];
        let idx: HashMap<&str, usize> = self.names.iter().enumerate().map(|(i, n)| (n.as_str(), i)).collect();
        for (u, v) in &sc.connections {
            if let (Some(&a), Some(&b)) = (idx.get(u.as_str()), idx.get(v.as_str())) {
                let d = dist(self.pos[a], self.pos[b]);
                self.edges[a].push((b, d));
                self.edges[b].push((a, d));
            }
        }
        self.statics = sc.static_segments();
        self.sc = sc;
    }

    pub fn static_segments(&self) -> &[Seg] { &self.statics }
    pub fn walls(&self) -> Vec<Seg> { self.sc.walls.clone() }
    pub fn scenario_id(&self) -> &str { &self.sc.id }

    pub fn set_footprint(&mut self, head: f64, tail: f64, hw: f64, corner_radius: f64, clear_min: f64, mode: CornerMode) {
        let r3 = |x: f64| (x * 1000.0).round_ties_even() / 1000.0;
        let fp = Footprint { head: r3(head), tail: r3(tail), hw: r3(hw), r: r3(corner_radius), clear_min: r3(clear_min), mode };
        if self.footprint != Some(fp) { self.footprint = Some(fp); self.ccache.clear(); self.rcache.clear(); }
    }

    pub fn mark_rot_blocked(&mut self, x: f64, y: f64) { self.rot_blocked.push((x, y)); }
    fn rot_marked(&self, p: Pt) -> bool { self.rot_blocked.iter().any(|q| dist(p, *q) < 0.6) }

    // ---------------- blocking checks
    fn is_node_blocked(&self, p: Pt, obs: &[Obs]) -> bool {
        obs.iter().any(|o| (o.cx - p.0).hypot(o.cy - p.1) < o.r + self.robot_circum_radius)
    }
    fn edge_blocked_r(p1: Pt, p2: Pt, obs: &[Obs], clearance: f64) -> bool {
        for o in obs {
            let radius = o.r + clearance;
            let (dx, dy) = (p2.0 - p1.0, p2.1 - p1.1);
            let l2 = dx * dx + dy * dy;
            let d = if l2 == 0.0 { (o.cx - p1.0).hypot(o.cy - p1.1) } else {
                let t = (((o.cx - p1.0) * dx + (o.cy - p1.1) * dy) / l2).clamp(0.0, 1.0);
                (o.cx - (p1.0 + t * dx)).hypot(o.cy - (p1.1 + t * dy))
            };
            if d < radius { return true; }
        }
        false
    }
    fn is_edge_blocked(&self, p1: Pt, p2: Pt, obs: &[Obs]) -> bool { Self::edge_blocked_r(p1, p2, obs, self.robot_half_width + 0.15) }

    fn seg_intersect(a: Pt, b: Pt, c: Pt, d: Pt) -> bool {
        let orient = |p: Pt, q: Pt, r: Pt| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
        let (o1, o2, o3, o4) = (orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b));
        o1 * o2 < 0.0 && o3 * o4 < 0.0
    }
    fn connector_ok(&self, p: Pt, q: Pt, obs: &[Obs]) -> bool {
        if dist(p, q) < 0.02 { return true; }
        for w in &self.statics { if Self::seg_intersect(p, q, (w[0], w[1]), (w[2], w[3])) { return false; } }
        !(!obs.is_empty() && Self::edge_blocked_r(p, q, obs, self.robot_half_width))
    }

    fn blocked_set(&self, obs: &[Obs]) -> Vec<(usize, usize)> {
        let mut b = vec![];
        if !obs.is_empty() {
            for (u, nb) in self.edges.iter().enumerate() {
                for &(v, _) in nb {
                    if self.is_edge_blocked(self.pos[u], self.pos[v], obs) { b.push((u.min(v), u.max(v))); }
                }
            }
        }
        b
    }

    fn attach(&self, pt: Pt, obs: &[Obs], blocked: &[(usize, usize)]) -> Vec<Attach> {
        let mut cands: Vec<Attach> = vec![];
        let mut fallback: Vec<Attach> = vec![];
        let mut seen: Vec<(usize, usize)> = vec![];
        for (u, nb) in self.edges.iter().enumerate() {
            for &(v, l) in nb {
                let key = (u.min(v), u.max(v));
                if seen.contains(&key) || blocked.contains(&key) || l < 1e-6 { continue; }
                seen.push(key);
                let ((ux, uy), (vx, vy)) = (self.pos[u], self.pos[v]);
                let t = (((pt.0 - ux) * (vx - ux) + (pt.1 - uy) * (vy - uy)) / (l * l)).clamp(0.0, 1.0);
                let proj = (ux + t * (vx - ux), uy + t * (vy - uy));
                let d = dist(pt, proj);
                let links = if t < 1e-3 { vec![(u, 0.0)] } else if t > 1.0 - 1e-3 { vec![(v, 0.0)] } else { vec![(u, t * l), (v, (1.0 - t) * l)] };
                let c = Attach { proj, links, d, edge: key };
                if self.connector_ok(pt, proj, obs) { cands.push(c) } else { fallback.push(c) }
            }
        }
        cands.sort_by(|a, b| a.d.total_cmp(&b.d));
        if cands.is_empty() && !fallback.is_empty() {
            fallback.sort_by(|a, b| a.d.total_cmp(&b.d));
            return fallback.into_iter().filter(|c| c.d < 1.0).take(1).collect();
        }
        cands
    }

    fn near(c: Vec<Attach>) -> Vec<Attach> {
        if c.is_empty() { return c; }
        let lim = c[0].d + ATTACH_SLACK;
        c.into_iter().filter(|x| x.d <= lim).take(3).collect()
    }

    pub fn turn(a: Pt, b: Pt, c: Pt) -> f64 {
        if dist(a, b) < 1e-6 || dist(b, c) < 1e-6 { return 0.0; }
        let h1 = (b.1 - a.1).atan2(b.0 - a.0);
        let h2 = (c.1 - b.1).atan2(c.0 - b.0);
        wrap(h2 - h1).abs()
    }

    pub fn corner_penalty(&mut self, a: Pt, b: Pt, c: Pt) -> f64 {
        let key = [k100(a.0), k100(a.1), k100(b.0), k100(b.1), k100(c.0), k100(c.1)];
        if let Some(&v) = self.ccache.get(&key) { return v; }
        let fp = self.footprint.unwrap();
        let h1 = (b.1 - a.1).atan2(b.0 - a.0);
        let h2 = (c.1 - b.1).atan2(c.0 - b.0);
        let (_, clr) = maneuver::plan_corner(&self.statics, b, h1, h2, dist(a, b), dist(b, c), fp.head, fp.tail, fp.hw, fp.r, fp.clear_min, fp.mode);
        let v = if clr < fp.clear_min { CORNER_BLOCK_COST } else { 0.0 };
        self.ccache.insert(key, v);
        v
    }

    fn rot_penalty(&mut self, p: Pt, h_from: f64, h_to: f64) -> f64 {
        let key = [k100(p.0), k100(p.1), (h_from * 1000.0).round_ties_even() as i64, (h_to * 1000.0).round_ties_even() as i64, 0];
        if let Some(&v) = self.rcache.get(&key) { return v; }
        let fp = self.footprint.unwrap();
        let (_, clr) = maneuver::plan_corner(&self.statics, p, h_from, h_to, 1.0, 1.0, fp.head, fp.tail, fp.hw, fp.r, fp.clear_min, CornerMode::Rotate);
        let v = if clr < fp.clear_min { CORNER_BLOCK_COST } else { 0.0 };
        self.rcache.insert(key, v);
        v
    }

    pub fn plan(&mut self, start: Pt, goal: Pt, obs: &[Obs], start_yaw: Option<f64>, goal_yaw: Option<f64>) -> Vec<Pt> {
        let r = if start_yaw.is_some() && self.footprint.is_some() && self.allow_reverse {
            self.plan_route_dir(start, goal, obs, start_yaw.unwrap(), goal_yaw)
        } else { self.plan_route(start, goal, obs) };
        self.last_route = r.clone();
        r.points
    }

    fn name_of(&self, i: usize) -> String {
        let n = self.names.len();
        if i == n { "__S__".into() } else if i == n + 1 { "__G__".into() } else if i == NONE { String::new() } else { self.names[i].clone() }
    }

    pub fn plan_route(&mut self, start: Pt, goal: Pt, obs: &[Obs]) -> Route {
        let blocked = self.blocked_set(obs);
        let sc = Self::near(self.attach(start, obs, &blocked));
        let gc = Self::near(self.attach(goal, obs, &blocked));
        let empty = Route::default();
        if sc.is_empty() || gc.is_empty() { return empty; }
        let n = self.names.len();
        let (si, gi) = (n, n + 1);
        let mut best: Option<(f64, Vec<usize>, Vec<Pt>)> = None;
        for s in &sc {
            for g in &gc {
                let mut pos = self.pos.clone();
                pos.push(s.proj); pos.push(g.proj);
                let mut adj: Vec<Vec<(usize, f64)>> = self.edges.iter().enumerate().map(|(k, vs)| {
                    vs.iter().filter(|(v, _)| !blocked.contains(&(k.min(*v), k.max(*v)))).cloned().collect()
                }).collect();
                adj.push(vec![]); adj.push(vec![]);
                for &(nn, w) in &s.links { adj[si].push((nn, w)); }
                for &(nn, w) in &g.links { adj[nn].push((gi, w)); adj[gi].push((nn, w)); }
                if s.edge == g.edge { adj[si].push((gi, dist(s.proj, g.proj))); }
                let mut pq = BinaryHeap::new();
                let mut cost: HashMap<(usize, usize), f64> = HashMap::new();
                let mut par: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
                cost.insert((si, NONE), s.d * OFF_NET_COST);
                pq.push(Qe { c: s.d * OFF_NET_COST, n: self.name_of(si), p: String::new(), ni: si, pi: NONE });
                let mut done: Option<(usize, usize, f64)> = None;
                while let Some(Qe { c, ni, pi, .. }) = pq.pop() {
                    if c > cost.get(&(ni, pi)).copied().unwrap_or(f64::INFINITY) + 1e-9 { continue; }
                    if ni == gi { done = Some((ni, pi, c)); break; }
                    for &(m, w) in &adj[ni] {
                        if m == pi || m == si { continue; }
                        if !obs.is_empty() && m != gi && m < n && self.is_node_blocked(self.pos[m], obs) { continue; }
                        let mut tc = 0.0;
                        if pi != NONE {
                            let ang = Self::turn(pos[pi], pos[ni], pos[m]);
                            tc = ang * TURN_COST_PER_RAD + if ang > 0.02 { CORNER_STOP_COST } else { 0.0 };
                            if ang > 0.02 && self.footprint.is_some() && ni < n { tc += self.corner_penalty(pos[pi], pos[ni], pos[m]); }
                        }
                        let nc = c + w + tc;
                        if nc < cost.get(&(m, ni)).copied().unwrap_or(f64::INFINITY) - 1e-9 {
                            cost.insert((m, ni), nc);
                            par.insert((m, ni), (ni, pi));
                            pq.push(Qe { c: nc, n: self.name_of(m), p: self.name_of(ni), ni: m, pi: ni });
                        }
                    }
                }
                let Some(done) = done else { continue };
                let total = done.2 + g.d * OFF_NET_COST;
                if best.as_ref().map(|b| total < b.0).unwrap_or(true) {
                    let mut seq = vec![];
                    let mut st = (done.0, done.1);
                    while let Some(&p) = par.get(&st) { seq.push(st.0); st = p; }
                    seq.push(si);
                    seq.reverse();
                    best = Some((total, seq, pos));
                }
            }
        }
        let Some((_, seq, pos)) = best else { return empty };
        let mut pts = vec![start];
        let mut labels: Vec<Option<String>> = vec![None];
        for &nn in &seq {
            let p = pos[nn];
            let lab = if nn >= n { None } else { Some(self.names[nn].clone()) };
            if dist(p, *pts.last().unwrap()) > 0.08 { pts.push(p); labels.push(lab); }
            else if lab.is_some() && labels.last().unwrap().is_none() { *labels.last_mut().unwrap() = lab; }
        }
        if dist(goal, *pts.last().unwrap()) > 0.08 { pts.push(goal); labels.push(None); }
        if pts.len() > 2 && dist(pts[1], pts[0]) < 0.35 { pts.remove(0); labels.remove(0); }
        let length: f64 = pts.windows(2).map(|w| dist(w[0], w[1])).sum();
        Route { points: pts, labels, length: (length * 100.0).round_ties_even() / 100.0, reverse: vec![], blocked: vec![], dir: false }
    }

    pub fn plan_route_dir(&mut self, start: Pt, goal: Pt, obs: &[Obs], start_yaw: f64, goal_yaw: Option<f64>) -> Route {
        let blocked = self.blocked_set(obs);
        let sc = Self::near(self.attach(start, obs, &blocked));
        let gc = Self::near(self.attach(goal, obs, &blocked));
        if sc.is_empty() || gc.is_empty() { return Route { dir: true, ..Route::default() }; }
        let n = self.names.len();
        let (si, gi) = (n, n + 1);
        type Why = Option<(f64, f64, &'static str)>;
        let mut best: Option<(f64, Vec<St>, Vec<Pt>, Vec<(f64, f64, &'static str)>)> = None;
        for s in &sc {
            for g in &gc {
                let mut pos = self.pos.clone();
                pos.push(s.proj); pos.push(g.proj);
                let mut adj: Vec<Vec<(usize, f64)>> = self.edges.iter().enumerate().map(|(k, vs)| {
                    vs.iter().filter(|(v, _)| !blocked.contains(&(k.min(*v), k.max(*v)))).cloned().collect()
                }).collect();
                adj.push(vec![]); adj.push(vec![]);
                for &(nn, w) in &s.links { adj[si].push((nn, w)); }
                for &(nn, w) in &g.links { adj[nn].push((gi, w)); }
                if s.edge == g.edge { adj[si].push((gi, dist(s.proj, g.proj))); }
                let s0 = St { n: si, prv: NONE, rev: false, h: k100(start_yaw) };
                let mut cost: HashMap<St, f64> = HashMap::new();
                let mut hb: HashMap<St, f64> = HashMap::new();
                let mut par: HashMap<St, St> = HashMap::new();
                let mut note: HashMap<St, Why> = HashMap::new();
                cost.insert(s0, s.d * OFF_NET_COST);
                hb.insert(s0, start_yaw);
                let mut pq = BinaryHeap::new();
                pq.push(Qd { c: s.d * OFF_NET_COST, tie: 0, st: s0 });
                let mut tie = 1u64;
                let mut fin: Option<(f64, St, Why)> = None;
                while let Some(Qd { c, st, .. }) = pq.pop() {
                    if let Some(f) = &fin { if c >= f.0 { break; } }
                    if c > cost.get(&st).copied().unwrap_or(f64::INFINITY) + 1e-9 { continue; }
                    let (nn, prv, rev) = (st.n, st.prv, st.rev);
                    let h_body = hb[&st];
                    if nn == gi {
                        let (mut tc, mut why): (f64, Why) = (0.0, None);
                        if let Some(gy) = goal_yaw {
                            let d = wrap(gy - h_body).abs();
                            if d > 0.05 {
                                let pen = if self.rot_marked(goal) { CORNER_BLOCK_COST } else { self.rot_penalty(goal, h_body, gy) };
                                tc = d * TURN_COST_PER_RAD + CORNER_STOP_COST + pen;
                                why = if pen != 0.0 { Some((goal.0, goal.1, "终点对位转向")) } else { None };
                            }
                        }
                        if fin.as_ref().map(|f| c + tc < f.0).unwrap_or(true) { fin = Some((c + tc, st, why)); }
                        continue;
                    }
                    let adj_n = adj[nn].clone();
                    for (m, w) in adj_n {
                        if m == si { continue; }
                        if !obs.is_empty() && m != gi && m < n && self.is_node_blocked(self.pos[m], obs) { continue; }
                        let zero = w < 1e-6;
                        let h_seg = if zero { 0.0 } else { (pos[m].1 - pos[nn].1).atan2(pos[m].0 - pos[nn].0) };
                        let revs: &[bool] = if zero { if rev { &[true] } else { &[false] } } else { &[false, true] };
                        for &rev2 in revs {
                            let h2 = if zero { h_body } else { wrap(h_seg + if rev2 { std::f64::consts::PI } else { 0.0 }) };
                            let d = wrap(h2 - h_body).abs();
                            let (mut tc, mut why): (f64, Why) = (0.0, None);
                            if d > 0.02 {
                                tc = d * TURN_COST_PER_RAD + CORNER_STOP_COST;
                                let geo = prv != NONE && dist(pos[nn], pos[prv]) > 1e-6
                                    && wrap((pos[nn].1 - pos[prv].1).atan2(pos[nn].0 - pos[prv].0) - h_body).abs() < 0.02;
                                let mut pen = if geo && !rev && !rev2 && nn < n && prv != m {
                                    self.corner_penalty(pos[prv], pos[nn], pos[m])
                                } else {
                                    let at = if prv == NONE { start } else { pos[nn] };
                                    self.rot_penalty(at, h_body, h2)
                                };
                                let at = if prv == NONE { start } else { pos[nn] };
                                if self.rot_marked(at) { pen = CORNER_BLOCK_COST; }
                                if pen != 0.0 {
                                    tc += pen;
                                    why = Some((at.0, at.1, if prv == NONE { "起步转向" } else { "拐点转向" }));
                                }
                            } else if prv != NONE && rev2 != rev {
                                tc = DIR_SWITCH_COST;
                            } else if prv != NONE && m == prv {
                                continue;
                            }
                            let nc = c + w * if rev2 { REVERSE_COST } else { 1.0 } + tc;
                            let st2 = St { n: m, prv: nn, rev: rev2, h: k100(h2) };
                            if nc < cost.get(&st2).copied().unwrap_or(f64::INFINITY) - 1e-9 {
                                cost.insert(st2, nc); hb.insert(st2, h2); par.insert(st2, st); note.insert(st2, why);
                                pq.push(Qd { c: nc, tie, st: st2 });
                                tie += 1;
                            }
                        }
                    }
                }
                let Some(fin) = fin else { continue };
                let total = fin.0 + g.d * OFF_NET_COST;
                if best.as_ref().map(|b| total < b.0).unwrap_or(true) {
                    let mut chain = vec![];
                    let mut st = fin.1;
                    while let Some(&p) = par.get(&st) { chain.push(st); st = p; }
                    chain.reverse();
                    let mut blk: Vec<(f64, f64, &'static str)> = chain.iter().filter_map(|x| note.get(x).copied().flatten()).collect();
                    if let Some(w) = fin.2 { blk.push(w); }
                    best = Some((total, chain, pos, blk));
                }
            }
        }
        let Some((_, chain, pos, blk)) = best else { return Route { dir: true, ..Route::default() } };
        let mut pts = vec![start];
        let mut labels: Vec<Option<String>> = vec![None];
        let mut revs: Vec<bool> = vec![];
        let first_rev = chain.first().map(|c| c.rev).unwrap_or(false);
        if !chain.is_empty() && dist(pos[si], pts[0]) > 0.08 { pts.push(pos[si]); labels.push(None); revs.push(first_rev); }
        for st in &chain {
            let p = pos[st.n];
            let lab = if st.n == gi { None } else { Some(self.names.get(st.n).cloned().unwrap_or_default()) };
            let lab = lab.filter(|_| st.n < n);
            if dist(p, *pts.last().unwrap()) > 0.08 { pts.push(p); labels.push(lab); revs.push(st.rev); }
            else if lab.is_some() && labels.last().unwrap().is_none() { *labels.last_mut().unwrap() = lab; }
        }
        if dist(goal, *pts.last().unwrap()) > 0.08 { pts.push(goal); labels.push(None); revs.push(revs.last().copied().unwrap_or(false)); }
        if pts.len() > 2 && dist(pts[1], pts[0]) < 0.35 && revs[0] == revs[1] { pts.remove(0); labels.remove(0); revs.remove(0); }
        let length: f64 = pts.windows(2).map(|w| dist(w[0], w[1])).sum();
        let r2 = |x: f64| (x * 100.0).round_ties_even() / 100.0;
        Route { dir: true, points: pts, labels, length: r2(length), reverse: revs,
                blocked: blk.into_iter().map(|(x, y, w)| (r2(x), r2(y), w.to_string())).collect() }
    }
}
