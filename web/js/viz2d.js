// ============================================================================
// 2D 可视化: 场景缩略图、车辆俯视 SVG、交互式地图画布 (SLAM 栅格 + 拓扑 + 任务动线 + 实时位姿)
// ============================================================================
import { esc } from './core.js';

// ---------------------------------------------------------------- PGM 解码 → ImageBitmap 源 (canvas)
export function decodePGM(buf) {
  const u8 = new Uint8Array(buf);
  let i = 0; const tok = [];
  const ws = (c) => c === 9 || c === 10 || c === 13 || c === 32;
  const magic = String.fromCharCode(u8[0], u8[1]); i = 2;
  while (tok.length < 3) {
    while (ws(u8[i])) i++;
    if (u8[i] === 35) { while (u8[i] !== 10 && i < u8.length) i++; continue; }
    let s = '';
    while (!ws(u8[i]) && i < u8.length) s += String.fromCharCode(u8[i++]);
    tok.push(parseInt(s, 10));
  }
  i++;
  const [W, H] = tok;
  if (magic !== 'P5') throw new Error('仅支持 P5 PGM');
  const cv = document.createElement('canvas'); cv.width = W; cv.height = H;
  const cx = cv.getContext('2d'); const img = cx.createImageData(W, H);
  for (let k = 0; k < W * H; k++) {
    const v = u8[i + k];
    // 白底风格: 占据=深灰, 空闲=白, 未知=浅灰
    const c = v < 100 ? 71 : (v > 230 ? 255 : 226);
    img.data[k * 4] = c; img.data[k * 4 + 1] = c; img.data[k * 4 + 2] = v < 100 ? 85 : c; img.data[k * 4 + 3] = 255;
  }
  cx.putImageData(img, 0, 0);
  return { canvas: cv, W, H };
}
export function parseMapYaml(t) {
  const r = /resolution:\s*([\d.eE+-]+)/.exec(t), o = /origin:\s*\[([^\]]*)\]/.exec(t);
  return { res: r ? parseFloat(r[1]) : 0.05, origin: o ? o[1].split(',').map(parseFloat) : [0, 0, 0] };
}

// ---------------------------------------------------------------- 场景几何工具
export function sceneBounds(sc) {
  const w = sc.walls || [];
  if (!w.length) return { x0: -10, y0: -10, x1: 10, y1: 10 };
  const xs = w.flatMap(a => [a[0], a[2]]), ys = w.flatMap(a => [a[1], a[3]]);
  return { x0: Math.min(...xs), y0: Math.min(...ys), x1: Math.max(...xs), y1: Math.max(...ys) };
}
export function topoFromScene(sc) {
  const nodes = {};
  Object.entries(sc.nodes || {}).forEach(([k, v]) => { nodes[k] = { x: v[0], y: v[1], name: k }; });
  const edges = (sc.connections || []).filter(([a, b]) => nodes[a] && nodes[b]).map(([a, b]) => ({ from: a, to: b, p1: nodes[a], p2: nodes[b] }));
  return { nodes, edges };
}

// ---------------------------------------------------------------- 场景缩略图 (深色，同原型)
export function drawSceneThumb(cv, sc) {
  const W = cv.width = cv.clientWidth * 2 || 600, H = cv.height = cv.clientHeight * 2 || 300;
  const g = cv.getContext('2d');
  g.fillStyle = '#0f172a'; g.fillRect(0, 0, W, H);
  const b = sceneBounds(sc);
  const s = Math.min((W - 40) / (b.x1 - b.x0 || 1), (H - 40) / (b.y1 - b.y0 || 1));
  const ox = W / 2 - (b.x0 + b.x1) / 2 * s, oy = H / 2 + (b.y0 + b.y1) / 2 * s;
  const P = (x, y) => [ox + x * s, oy - y * s];
  g.strokeStyle = 'rgba(148,163,184,.55)'; g.lineWidth = 2;
  (sc.walls || []).forEach((w, i) => {
    g.setLineDash(i < 4 ? [6, 5] : []);
    const [a, bb] = [P(w[0], w[1]), P(w[2], w[3])];
    g.beginPath(); g.moveTo(...a); g.lineTo(...bb); g.stroke();
  });
  g.setLineDash([]);
  (sc.shelves || []).forEach(sh => {
    const [x0, y0] = P(Math.min(sh.x1, sh.x2), Math.max(sh.y1, sh.y2)), [x1, y1] = P(Math.max(sh.x1, sh.x2), Math.min(sh.y1, sh.y2));
    g.fillStyle = '#334155'; g.fillRect(x0, y0, x1 - x0, y1 - y0);
    g.strokeStyle = '#475569'; g.strokeRect(x0, y0, x1 - x0, y1 - y0);
  });
  const t = topoFromScene(sc);
  g.strokeStyle = '#22c55e'; g.lineWidth = 2.5;
  t.edges.forEach(e => { g.beginPath(); g.moveTo(...P(e.p1.x, e.p1.y)); g.lineTo(...P(e.p2.x, e.p2.y)); g.stroke(); });
  (sc.stations || []).forEach(st => { g.fillStyle = '#38bdf8'; g.beginPath(); g.arc(...P(st.x, st.y), 5, 0, 7); g.fill(); });
  (sc.reflectors || []).forEach(r => { g.fillStyle = '#e2e8f0'; g.fillRect(...P(r.x, r.y), 3, 3); });
}

// ---------------------------------------------------------------- 车辆俯视 SVG (模型卡片)
export function modelTopSVG(sm, opts = {}) {
  const fp = sm?.footprint || [[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]];
  const xs = fp.map(p => p[0]), ys = fp.map(p => p[1]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const L = x1 - x0, Wd = y1 - y0;
  const VW = 400, VH = opts.h || 160;
  const s = Math.min((VW - 110) / L, (VH - 56) / Wd);
  const cx = VW / 2 - (x0 + x1) / 2 * s, cy = VH / 2 + (y0 + y1) / 2 * s;
  const P = (x, y) => [cx + x * s, cy - y * s];
  const [rx, ry] = P(x0, y1);
  let out = `<svg viewBox="0 0 ${VW} ${VH}" class="w-full h-full">`;
  out += `<rect x="${rx}" y="${ry}" width="${L * s}" height="${Wd * s}" rx="8" fill="#e2e8f0" stroke="#334155" stroke-width="2"/>`;
  const [hx, hy] = P(x1, 0);
  out += `<path d="M${hx + 2} ${hy - 6} L${hx + 12} ${hy} L${hx + 2} ${hy + 6}Z" fill="#ef4444"/>`;
  const colors = ['#7c3aed', '#2563eb', '#0891b2', '#db2777'];
  (sm?.wheels || []).forEach((w, i) => {
    const [px, py] = P(w.x, w.y);
    const r = Math.max(6, (w.r || 0.1) * s);
    if (w.kind === 'steer') {
      out += `<circle cx="${px}" cy="${py}" r="${r}" fill="${colors[i % 4]}" stroke="#0f172a" stroke-width="1.5"/>`;
      out += `<line x1="${px - r * .7}" y1="${py}" x2="${px + r * .7}" y2="${py}" stroke="#fff" stroke-width="2"/>`;
    } else if (w.kind === 'drive') {
      out += `<rect x="${px - r}" y="${py - 4}" width="${2 * r}" height="8" rx="2" fill="#0f172a"/>`;
    } else {
      out += `<circle cx="${px}" cy="${py}" r="${Math.max(4, r * .6)}" fill="#94a3b8"/>`;
    }
    if (w.kind === 'steer' || w.kind === 'drive') {
      out += `<text x="${px}" y="${py + (w.y > 0 ? -r - 5 : r + 12)}" font-size="9" text-anchor="middle" fill="#475569" font-family="monospace">${esc(w.name)}(${Math.round(w.x * 1000)},${Math.round(w.y * 1000)})</text>`;
    }
  });
  (sm?.lidars || []).forEach(l => {
    const [px, py] = P(l.x, l.y);
    out += `<circle cx="${px}" cy="${py}" r="5" fill="${l.type === '3d' ? '#7c3aed' : '#10b981'}" stroke="#fff" stroke-width="1.5"/>`;
  });
  (sm?.cameras || []).forEach(c => {
    const [px, py] = P(c.x || 0, c.y || 0);
    out += `<rect x="${px - 4}" y="${py - 4}" width="8" height="8" fill="#db2777"/>`;
  });
  out += `<text x="${VW / 2}" y="${VH - 6}" font-size="10" text-anchor="middle" fill="#2563eb">${esc(sm?.chassis_label || '')} · ${(L * 1000).toFixed(0)}×${(Wd * 1000).toFixed(0)} mm</text>`;
  return out + '</svg>';
}

export function topoNodeName(l) {
  if (!l) return '';
  return String(l).replace(/^N_/, '').replace(/^(FMS|GRID|WH|DEMO)_/i, '').replace(/^STATION_/, '站 ').replace(/_/g, ' ');
}

// ---------------------------------------------------------------- 交互地图画布
export class MapCanvas {
  constructor(el, opts = {}) {
    this.el = el; this.opts = opts;
    this.cv = document.createElement('canvas');
    this.cv.className = 'absolute inset-0 w-full h-full';
    el.appendChild(this.cv);
    this.g = this.cv.getContext('2d');
    this.layers = { grid: true, topo: true, labels: true, path: true };
    this.scene = null; this.topo = null; this.grid = null; this.gridMeta = null;
    this.robot = null; this.footprint = null; this.plan = []; this.taskPath = []; this.obstacles = []; this.trail = []; this.ghost = [];
    this.highlight = new Set(); this.pick = null;
    this.view = { s: 30, ox: 0, oy: 0 }; this._fit = true;
    this._bind();
    this._ro = new ResizeObserver(() => { this._resize(); });
    this._ro.observe(el);
    this._resize();
  }
  destroy() { this._ro.disconnect(); this.cv.remove(); }
  _resize() {
    const r = this.el.getBoundingClientRect();
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    this.cv.width = Math.max(10, r.width * dpr); this.cv.height = Math.max(10, r.height * dpr);
    this.dpr = dpr; this.W = r.width; this.H = r.height;
    if (this._fit) this.fit();
    this.draw();
  }
  setScene(sc, topo) { this.scene = sc; this.topo = topo || topoFromScene(sc); this._fit = true; this.fit(); this.draw(); }
  setGrid(dec, meta) { this.grid = dec; this.gridMeta = meta; this.draw(); }
  fit() {
    if (!this.scene || !this.W) return;
    const b = sceneBounds(this.scene);
    const s = Math.min((this.W - 60) / (b.x1 - b.x0 || 1), (this.H - 60) / (b.y1 - b.y0 || 1));
    this.view = { s, ox: this.W / 2 - (b.x0 + b.x1) / 2 * s, oy: this.H / 2 + (b.y0 + b.y1) / 2 * s };
  }
  zoom(k, cx, cy) {
    cx = cx ?? this.W / 2; cy = cy ?? this.H / 2;
    const v = this.view; const wx = (cx - v.ox) / v.s, wy = (v.oy - cy) / v.s;
    v.s = Math.max(2, Math.min(400, v.s * k));
    v.ox = cx - wx * v.s; v.oy = cy + wy * v.s; this._fit = false; this.draw();
  }
  reset() { this._fit = true; this.fit(); this.draw(); }
  w2s(x, y) { return [this.view.ox + x * this.view.s, this.view.oy - y * this.view.s]; }
  s2w(px, py) { return [(px - this.view.ox) / this.view.s, (this.view.oy - py) / this.view.s]; }
  _bind() {
    let drag = null, moved = false;
    this.cv.addEventListener('wheel', (e) => { e.preventDefault(); const r = this.cv.getBoundingClientRect(); this.zoom(e.deltaY < 0 ? 1.15 : 1 / 1.15, e.clientX - r.left, e.clientY - r.top); }, { passive: false });
    this.cv.addEventListener('mousedown', (e) => { drag = { x: e.clientX, y: e.clientY, ox: this.view.ox, oy: this.view.oy }; moved = false; });
    window.addEventListener('mousemove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) moved = true;
      if (moved) { this.view.ox = drag.ox + dx; this.view.oy = drag.oy + dy; this._fit = false; this.draw(); }
    });
    window.addEventListener('mouseup', (e) => {
      if (drag && !moved) {
        const r = this.cv.getBoundingClientRect();
        const px = e.clientX - r.left, py = e.clientY - r.top;
        const [wx, wy] = this.s2w(px, py);
        const hit = this.hitTest(px, py);
        this.opts.onClick && this.opts.onClick({ x: wx, y: wy, hit });
      }
      drag = null;
    });
    this.cv.addEventListener('mousemove', (e) => {
      const r = this.cv.getBoundingClientRect();
      const hit = this.hitTest(e.clientX - r.left, e.clientY - r.top);
      this.cv.style.cursor = hit ? 'pointer' : (this.pick ? 'crosshair' : 'grab');
    });
  }
  hitTest(px, py) {
    const pts = [];
    (this.scene?.stations || []).forEach(s => pts.push({ kind: 'station', id: s.id, name: s.name, x: s.x, y: s.y }));
    Object.entries(this.topo?.nodes || {}).forEach(([k, n]) => pts.push({ kind: 'node', id: k, name: k, x: n.x, y: n.y }));
    let best = null, bd = 12;
    pts.forEach(p => { const [sx, sy] = this.w2s(p.x, p.y); const d = Math.hypot(sx - px, sy - py); if (d < bd) { bd = d; best = p; } });
    return best;
  }
  _drawPlan(g) {
    const P = this.plan, n = P.length, idx = Math.max(1, Math.min(n - 1, this.planIndex || 1));
    const line = (pts, color, w, dash) => {
      if (pts.length < 2) return;
      g.strokeStyle = color; g.lineWidth = w; g.lineJoin = 'round'; g.lineCap = 'round'; g.setLineDash(dash || []); g.beginPath();
      pts.forEach((p, i) => { const [x, y] = this.w2s(p.x, p.y); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke(); g.setLineDash([]);
    };
    const a = P[idx - 1], b = P[idx], r = this.robot;
    let proj = a;
    if (r) {
      const dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy || 1, u = Math.max(0, Math.min(1, ((r.x - a.x) * dx + (r.y - a.y) * dy) / L2));
      proj = { x: a.x + dx * u, y: a.y + dy * u };
    }
    line(P.slice(0, idx).concat([proj]), 'rgba(148,163,184,.8)', 3);
    g.shadowColor = '#22d3ee'; g.shadowBlur = 6; line(this.planCurve && this.planCurve.length > 1 ? this.planCurve : P.slice(idx), '#06b6d4', 3); g.shadowBlur = 0;
    g.shadowColor = '#fb923c'; g.shadowBlur = 10; line([proj, b], '#f97316', 5); g.shadowBlur = 0;
    // 当前与下一路段的方向箭头
    const arrows = (p, q, color) => {
      const [x1, y1] = this.w2s(p.x, p.y), [x2, y2] = this.w2s(q.x, q.y), L = Math.hypot(x2 - x1, y2 - y1); if (L < 12) return;
      const ang = Math.atan2(y2 - y1, x2 - x1), step = 26;
      g.fillStyle = color;
      for (let d = step / 2; d < L - 4; d += step) {
        const x = x1 + (x2 - x1) * d / L, y = y1 + (y2 - y1) * d / L;
        g.save(); g.translate(x, y); g.rotate(ang); g.beginPath(); g.moveTo(5, 0); g.lineTo(-4, -4.5); g.lineTo(-2, 0); g.lineTo(-4, 4.5); g.closePath(); g.fill(); g.restore();
      }
    };
    arrows(proj, b, '#fff');
    if (P[idx + 1]) arrows(b, P[idx + 1], 'rgba(8,145,178,.9)');
    // 途经节点
    P.forEach((p, i) => {
      if (!i) return;
      const [x, y] = this.w2s(p.x, p.y);
      g.fillStyle = i < idx ? '#94a3b8' : (i === n - 1 ? '#16a34a' : '#0891b2'); g.beginPath(); g.arc(x, y, i === n - 1 ? 5 : 3.5, 0, 7); g.fill();
    });
    // 下一航点
    const [bx, by] = this.w2s(b.x, b.y);
    g.strokeStyle = '#f97316'; g.lineWidth = 2.5; g.beginPath(); g.arc(bx, by, 9, 0, 7); g.stroke();
    const lab = this.planLabels && this.planLabels[idx];
    this._planLabel = { bx, by, txt: '下一点 ' + (lab ? topoNodeName(lab) : `(${b.x.toFixed(1)}, ${b.y.toFixed(1)})`) };
  }
  _drawPlanLabel(g) {       // 画在车体之上，避免被遮挡
    const L = this._planLabel; if (!L) return;
    g.font = 'bold 11px sans-serif'; const tw = g.measureText(L.txt).width;
    g.fillStyle = 'rgba(15,23,42,.88)'; g.fillRect(L.bx + 11, L.by - 22, tw + 10, 17);
    g.fillStyle = '#fdba74'; g.fillText(L.txt, L.bx + 16, L.by - 9);
  }
  draw() {
    const g = this.g; if (!g) return;
    const d = this.dpr || 1;
    g.setTransform(d, 0, 0, d, 0, 0);
    g.fillStyle = '#f8fafc'; g.fillRect(0, 0, this.W, this.H);
    // 背景网格 (1 m)
    const v = this.view;
    if (v.s > 8) {
      g.strokeStyle = '#eef2f7'; g.lineWidth = 1; g.beginPath();
      const [wx0, wy1] = this.s2w(0, 0), [wx1, wy0] = this.s2w(this.W, this.H);
      for (let x = Math.floor(wx0); x <= wx1; x++) { const [sx] = this.w2s(x, 0); g.moveTo(sx, 0); g.lineTo(sx, this.H); }
      for (let y = Math.floor(wy0); y <= wy1; y++) { const [, sy] = this.w2s(0, y); g.moveTo(0, sy); g.lineTo(this.W, sy); }
      g.stroke();
    }
    if (!this.scene) return;
    // SLAM 栅格
    if (this.layers.grid && this.grid && this.gridMeta) {
      const { res, origin } = this.gridMeta;
      const [sx, sy] = this.w2s(origin[0], origin[1] + this.grid.H * res);
      g.imageSmoothingEnabled = false; g.globalAlpha = 0.9;
      g.drawImage(this.grid.canvas, sx, sy, this.grid.W * res * v.s, this.grid.H * res * v.s);
      g.globalAlpha = 1;
    } else {
      g.strokeStyle = '#94a3b8'; g.lineWidth = 3;
      (this.scene.walls || []).forEach((w, i) => { g.lineWidth = i < 4 ? 4 : 2; g.beginPath(); g.moveTo(...this.w2s(w[0], w[1])); g.lineTo(...this.w2s(w[2], w[3])); g.stroke(); });
    }
    // 货架/设备岛
    (this.scene.shelves || []).forEach(sh => {
      const [x0, y0] = this.w2s(Math.min(sh.x1, sh.x2), Math.max(sh.y1, sh.y2)), [x1, y1] = this.w2s(Math.max(sh.x1, sh.x2), Math.min(sh.y1, sh.y2));
      g.fillStyle = 'rgba(254,243,199,.85)'; g.fillRect(x0, y0, x1 - x0, y1 - y0);
      g.strokeStyle = '#d97706'; g.lineWidth = 1.5; g.strokeRect(x0, y0, x1 - x0, y1 - y0);
      if (this.layers.labels && v.s > 12) { g.fillStyle = '#b45309'; g.font = '11px sans-serif'; g.fillText(sh.name || '', x0 + 4, y0 + 13); }
    });
    // 拓扑
    if (this.layers.topo && this.topo) {
      g.strokeStyle = '#94a3b8'; g.lineWidth = 1.5; g.setLineDash([5, 4]);
      this.topo.edges.forEach(e => { g.beginPath(); g.moveTo(...this.w2s(e.p1.x, e.p1.y)); g.lineTo(...this.w2s(e.p2.x, e.p2.y)); g.stroke(); });
      g.setLineDash([]);
      Object.entries(this.topo.nodes).forEach(([k, n]) => {
        const [x, y] = this.w2s(n.x, n.y);
        g.fillStyle = this.highlight.has(k) ? '#2563eb' : '#cbd5e1'; g.beginPath(); g.arc(x, y, 3.5, 0, 7); g.fill();
      });
    }
    // 任务动线 (任务流工步路线)
    if (this.layers.path && this.taskPath.length > 1) {
      g.strokeStyle = 'rgba(6,182,212,.85)'; g.lineWidth = 5; g.lineJoin = 'round'; g.beginPath();
      this.taskPath.forEach((p, i) => { const [x, y] = this.w2s(p.x, p.y); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke();
    }
    // 回放幽灵路径
    if (this.ghost.length > 1) {
      g.strokeStyle = 'rgba(37,99,235,.35)'; g.lineWidth = 2; g.setLineDash([6, 5]); g.beginPath();
      this.ghost.forEach((p, i) => { const [x, y] = this.w2s(p.x, p.y); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke(); g.setLineDash([]);
    }
    // 实时规划路径: 已走过 (灰) / 当前路段 (橙 + 箭头) / 后续 (青) / 下一航点
    this._planLabel = null;
    if (this.plan.length > 1) this._drawPlan(g);
    if (this.trail.length > 1) {
      g.strokeStyle = 'rgba(37,99,235,.5)'; g.lineWidth = 2; g.beginPath();
      this.trail.forEach((p, i) => { const [x, y] = this.w2s(p.x, p.y); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke();
    }
    // 工位
    (this.scene.stations || []).forEach(s => {
      const [x, y] = this.w2s(s.x, s.y);
      const hl = this.highlight.has(s.id);
      g.fillStyle = hl ? '#2563eb' : '#3b82f6'; g.strokeStyle = '#dbeafe'; g.lineWidth = 3;
      g.beginPath(); g.arc(x, y, hl ? 8 : 6.5, 0, 7); g.fill(); g.stroke();
      if (this.layers.labels) {
        g.font = 'bold 11px sans-serif'; g.fillStyle = '#1e293b'; g.fillText(`${s.id}${s.name && s.name !== s.id ? ': ' + s.name : ''}`, x + 10, y + 4);
        g.font = '9px monospace'; g.fillStyle = '#94a3b8'; g.fillText(`(${(+s.x).toFixed(1)}, ${(+s.y).toFixed(1)})`, x + 10, y + 15);
      }
    });
    // 障碍物/注入元素
    this.obstacles.forEach(o => {
      const [x, y] = this.w2s(o.x, o.y);
      const c = { person: '#e11d48', pallet: '#b45309', shelf: '#2563eb', box: '#d97706' }[o.type] || '#f59e0b';
      g.save(); g.translate(x, y); g.rotate(-(o.yaw || 0));
      g.fillStyle = c + 'cc'; g.strokeStyle = c; g.lineWidth = 1.5;
      if (o.type === 'person') { g.beginPath(); g.arc(0, 0, Math.max(4, (o.w || .5) / 2 * v.s), 0, 7); g.fill(); }
      else { const w = (o.w || .8) * v.s, h = (o.h || .8) * v.s; g.fillRect(-w / 2, -h / 2, w, h); g.strokeRect(-w / 2, -h / 2, w, h); }
      g.restore();
      if (o.motion) { g.strokeStyle = '#e11d48'; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(...this.w2s(o.motion.ax, o.motion.ay)); g.lineTo(...this.w2s(o.motion.bx, o.motion.by)); g.stroke(); g.setLineDash([]); }
    });
    // 保护空间 (机体系多边形随车体变换)
    if (this.robot && this.protection && this.protection.polygons) {
      const r = this.robot, c = Math.cos(r.yaw), s2 = Math.sin(r.yaw), pv = this.protection, Pg = pv.polygons;
      const col = { stop: '#dc2626', slow: '#f59e0b', warn: '#f59e0b' }[pv.zone] || '#16a34a';
      const path = (pts) => { g.beginPath(); pts.forEach((p, i) => { const [x, y] = this.w2s(r.x + c * p[0] - s2 * p[1], r.y + s2 * p[0] + c * p[1]); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.closePath(); };
      path(Pg.stop); g.fillStyle = col + (pv.zone === 'stop' ? '44' : '1a'); g.fill(); g.strokeStyle = col; g.lineWidth = 1.5; g.stroke();
      g.setLineDash([5, 4]); path(Pg.slow); g.strokeStyle = col + 'aa'; g.lineWidth = 1; g.stroke(); g.setLineDash([]);
      if (pv.loaded && Pg.payload) { path(Pg.payload); g.strokeStyle = '#7c3aed'; g.lineWidth = 2; g.stroke(); }
    }
    // 车体
    if (this.robot) {
      const r = this.robot, fp = this.footprint || [[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]];
      const c = Math.cos(r.yaw), s2 = Math.sin(r.yaw);
      g.fillStyle = 'rgba(37,99,235,.9)'; g.strokeStyle = '#1e3a8a'; g.lineWidth = 1.5; g.beginPath();
      fp.forEach((p, i) => { const [x, y] = this.w2s(r.x + c * p[0] - s2 * p[1], r.y + s2 * p[0] + c * p[1]); i ? g.lineTo(x, y) : g.moveTo(x, y); });
      g.closePath(); g.fill(); g.stroke();
      const hx = Math.max(...fp.map(p => p[0]));
      const [ax, ay] = this.w2s(r.x + c * hx, r.y + s2 * hx), [bx, by] = this.w2s(r.x + c * (hx + .35), r.y + s2 * (hx + .35));
      g.strokeStyle = '#ef4444'; g.lineWidth = 3; g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by); g.stroke();
    }
    this._drawPlanLabel(g);
    if (this.pickPoint) {
      const [x, y] = this.w2s(this.pickPoint.x, this.pickPoint.y);
      g.strokeStyle = '#e11d48'; g.lineWidth = 2; g.beginPath(); g.arc(x, y, 9, 0, 7); g.moveTo(x - 13, y); g.lineTo(x + 13, y); g.moveTo(x, y - 13); g.lineTo(x, y + 13); g.stroke();
    }
  }
}

// ---------------------------------------------------------------- 拓扑路线 (与执行进程 DijkstraPlanner.plan_route 一致)
// 起点/终点投影到最近的拓扑边上 (不再跳到最近节点)，状态 = (节点, 来向)，代价 = 距离 + 转弯惩罚
const TURN_COST_PER_RAD = 0.8, CORNER_STOP_COST = 1.5, OFF_NET_COST = 3.0, ATTACH_SLACK = 0.5;
function attachTopo(nodes, edges, pt) {
  const out = [];
  edges.forEach(([a, b]) => {
    const A = nodes[a], B = nodes[b], dx = B.x - A.x, dy = B.y - A.y, L2 = dx * dx + dy * dy;
    if (L2 < 1e-9) return;
    const u = Math.max(0, Math.min(1, ((pt.x - A.x) * dx + (pt.y - A.y) * dy) / L2));
    const q = { x: A.x + dx * u, y: A.y + dy * u }, L = Math.sqrt(L2);
    out.push({ proj: q, d: Math.hypot(pt.x - q.x, pt.y - q.y), links: [[a, u * L], [b, (1 - u) * L]], edge: a + '|' + b });
  });
  return out.sort((p, q) => p.d - q.d);
}
function turnCost(a, b, c) {
  if (Math.hypot(b.x - a.x, b.y - a.y) < 1e-6 || Math.hypot(c.x - b.x, c.y - b.y) < 1e-6) return 0;
  const h1 = Math.atan2(b.y - a.y, b.x - a.x), h2 = Math.atan2(c.y - b.y, c.x - b.x);
  let d = Math.abs(h2 - h1) % (2 * Math.PI); if (d > Math.PI) d = 2 * Math.PI - d;
  return d * TURN_COST_PER_RAD + (d > 0.02 ? CORNER_STOP_COST : 0);
}
export function topoRoute(topo, fromXY, toXY) {
  const nodes = Object.assign({}, topo.nodes);
  const edges = topo.edges.filter(e => nodes[e.from] && nodes[e.to]).map(e => [e.from, e.to]);
  if (!edges.length) return { points: [fromXY, toXY], labels: [null, null] };
  const near = c => c.filter(x => x.d <= c[0].d + ATTACH_SLACK).slice(0, 3);
  const S = near(attachTopo(nodes, edges, fromXY)), G = near(attachTopo(nodes, edges, toXY));
  const adj = {};
  const link = (a, b, d) => { (adj[a] = adj[a] || []).push([b, d]); (adj[b] = adj[b] || []).push([a, d]); };
  edges.forEach(([a, b]) => link(a, b, Math.hypot(nodes[a].x - nodes[b].x, nodes[a].y - nodes[b].y)));
  let best = null;
  for (const s of S) for (const g of G) {
    const P = Object.assign({}, nodes, { __S: s.proj, __G: g.proj });
    const A = {}; for (const k in adj) A[k] = adj[k].slice();
    const add = (a, b, d) => { (A[a] = A[a] || []).push([b, d]); (A[b] = A[b] || []).push([a, d]); };
    s.links.forEach(([n, d]) => add('__S', n, d)); g.links.forEach(([n, d]) => add('__G', n, d));
    if (s.edge === g.edge) add('__S', '__G', Math.hypot(s.proj.x - g.proj.x, s.proj.y - g.proj.y));
    // Dijkstra over (node, prev)
    const key = (n, p) => n + '#' + (p || ''), dist = { [key('__S', null)]: 0 }, prev = {}, done = new Set();
    const open = [[0, '__S', null]];
    let endKey = null;
    while (open.length) {
      open.sort((x, y) => x[0] - y[0]);
      const [c, u, pu] = open.shift(), ku = key(u, pu);
      if (done.has(ku)) continue; done.add(ku);
      if (u === '__G') { endKey = ku; break; }
      (A[u] || []).forEach(([w, d]) => {
        if (w === pu) return;
        const nc = c + d + (pu ? turnCost(P[pu], P[u], P[w]) : 0), kw = key(w, u);
        if (dist[kw] === undefined || nc < dist[kw]) { dist[kw] = nc; prev[kw] = ku; open.push([nc, w, u]); }
      });
    }
    if (!endKey) continue;
    const total = dist[endKey] + (s.d + g.d) * OFF_NET_COST;
    if (best && total >= best.total) continue;
    const seq = []; let k = endKey;
    while (k) { seq.unshift(k.split('#')[0]); k = prev[k]; }
    best = { total, seq, P };
  }
  if (!best) return { points: [fromXY, toXY], labels: [null, null] };
  const pts = [fromXY], labels = [null];
  best.seq.forEach(n => {
    const q = best.P[n], last = pts[pts.length - 1];
    if (Math.hypot(q.x - last.x, q.y - last.y) < 0.08) { if (!n.startsWith('__') && !labels[labels.length - 1]) labels[labels.length - 1] = n; return; }
    pts.push({ x: q.x, y: q.y }); labels.push(n.startsWith('__') ? null : n);
  });
  const last = pts[pts.length - 1];
  if (Math.hypot(toXY.x - last.x, toXY.y - last.y) > 0.02) { pts.push(toXY); labels.push(null); }
  if (pts.length > 2 && Math.hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y) < 0.35) { pts.shift(); labels.shift(); }
  // 去掉共线的中间投影点
  for (let i = pts.length - 2; i >= 1; i--) {
    if (labels[i]) continue;
    const a = pts[i - 1], b = pts[i], c = pts[i + 1];
    if (Math.abs((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) < 1e-4) { pts.splice(i, 1); labels.splice(i, 1); }
  }
  return { points: pts, labels };
}
export function topoPath(topo, sc, fromXY, toId) {
  const st = (sc.stations || []).find(s => s.id === toId);
  const tgt = st ? { x: st.x, y: st.y } : topo.nodes[toId];
  if (!tgt) return [fromXY];
  return topoRoute(topo, fromXY, { x: tgt.x, y: tgt.y }).points;
}
