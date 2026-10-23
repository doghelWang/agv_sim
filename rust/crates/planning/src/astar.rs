//! A* grid planner with inflation + line-of-sight smoothing (port of a_star_planner.py).
use crate::Pt;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

pub struct AStar { pub resolution: f64, pub inflation_radius: f64, inflation_cells: i32 }

#[derive(PartialEq)]
struct K(f64, (i32, i32));
impl Eq for K {}
impl PartialOrd for K { fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
impl Ord for K { fn cmp(&self, o: &Self) -> std::cmp::Ordering { self.0.total_cmp(&o.0).then(self.1.cmp(&o.1)) } }

impl AStar {
    pub fn new(resolution: f64, inflation_radius: f64) -> AStar {
        AStar { resolution, inflation_radius, inflation_cells: (inflation_radius / resolution).ceil() as i32 }
    }
    fn w2g(&self, x: f64, y: f64) -> (i32, i32) { ((x / self.resolution).round_ties_even() as i32, (y / self.resolution).round_ties_even() as i32) }
    fn g2w(&self, gx: i32, gy: i32) -> Pt { (((gx as f64 * self.resolution) * 1000.0).round_ties_even() / 1000.0, ((gy as f64 * self.resolution) * 1000.0).round_ties_even() / 1000.0) }

    fn grid(&self, walls: &[[f64; 4]]) -> HashSet<(i32, i32)> {
        let mut occ = HashSet::new();
        let ic = self.inflation_cells;
        for w in walls {
            let dist = (w[2] - w[0]).hypot(w[3] - w[1]);
            let steps = 1.max((dist / (self.resolution * 0.5)) as i32);
            for s in 0..=steps {
                let t = s as f64 / steps as f64;
                let (cx, cy) = self.w2g(w[0] + (w[2] - w[0]) * t, w[1] + (w[3] - w[1]) * t);
                for dx in -ic..=ic { for dy in -ic..=ic {
                    if (dx as f64).hypot(dy as f64) <= ic as f64 { occ.insert((cx + dx, cy + dy)); }
                } }
            }
        }
        occ
    }

    fn heur(a: (i32, i32), b: (i32, i32)) -> f64 {
        let (dx, dy) = ((a.0 - b.0).abs() as f64, (a.1 - b.1).abs() as f64);
        (dx + dy) + (2f64.sqrt() - 2.0) * dx.min(dy)
    }

    pub fn plan(&self, start: Pt, goal: Pt, walls: &[[f64; 4]]) -> Vec<Pt> {
        let (s, g) = (self.w2g(start.0, start.1), self.w2g(goal.0, goal.1));
        let obs = self.grid(walls);
        let r2 = 2f64.sqrt();
        let motions = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, r2), (-1, 1, r2), (1, -1, r2), (-1, -1, r2)];
        let mut open = BinaryHeap::new();
        open.push(Reverse(K(0.0, s)));
        let mut came: HashMap<(i32, i32), (i32, i32)> = HashMap::new();
        let mut gs: HashMap<(i32, i32), f64> = HashMap::new();
        gs.insert(s, 0.0);
        while let Some(Reverse(K(_, cur))) = open.pop() {
            if cur == g {
                let mut path = vec![];
                let mut c = cur;
                while let Some(&p) = came.get(&c) { path.push(self.g2w(c.0, c.1)); c = p; }
                path.push(self.g2w(s.0, s.1));
                path.reverse();
                return self.smooth(path, &obs);
            }
            for &(dx, dy, cost) in &motions {
                let nb = (cur.0 + dx, cur.1 + dy);
                if obs.contains(&nb) && nb != g { continue; }
                let tg = gs[&cur] + cost;
                if gs.get(&nb).map(|&v| tg < v).unwrap_or(true) {
                    came.insert(nb, cur);
                    gs.insert(nb, tg);
                    open.push(Reverse(K(tg + Self::heur(nb, g), nb)));
                }
            }
        }
        vec![start, goal]
    }

    fn smooth(&self, path: Vec<Pt>, obs: &HashSet<(i32, i32)>) -> Vec<Pt> {
        if path.len() <= 2 { return path; }
        let mut out = vec![path[0]];
        let mut ci = 0;
        while ci < path.len() - 1 {
            let mut best = ci + 1;
            for t in (ci + 1..path.len()).rev() {
                if self.clear(path[ci], path[t], obs) { best = t; break; }
            }
            out.push(path[best]);
            ci = best;
        }
        out
    }

    fn clear(&self, p0: Pt, p1: Pt, obs: &HashSet<(i32, i32)>) -> bool {
        let dist = (p1.0 - p0.0).hypot(p1.1 - p0.1);
        let steps = 1.max((dist / (self.resolution * 0.5)) as i32);
        for s in 0..=steps {
            let t = s as f64 / steps as f64;
            if obs.contains(&self.w2g(p0.0 + (p1.0 - p0.0) * t, p0.1 + (p1.1 - p0.1) * t)) { return false; }
        }
        true
    }
}
