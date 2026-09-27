// ============================================================================
// 3D 可视化 (three.js r13x，全局 THREE + OrbitControls，vendor 本地)
//   Viewer3D   : 工作台实时视图 / 回放视图 (白底，与原型一致)
//   ModelView  : 车辆模型详情 (深色背景，透视/俯/侧/前视 + 线框)
//   SceneView  : 场景详情 (3D 环境/SLAM 栅格/拓扑 图层 + 全景/2D/巷道视角)
// 机器人与仓储环境复用 vendor/amr_model3d.js (AMR3D.create() 独立实例)
// ============================================================================
import { topoFromScene, sceneBounds } from './viz2d.js';

const T = () => window.THREE;

// 平台/仿真模型 spec → AMR3D 所需 robot_spec (与网关 _apply_model 相同的字段)
export function robotSpecFromModel(spec) {
  const ch = spec.chassis || {};
  return {
    model_file: spec.model_file, native_chassis: ch.type, active_chassis: spec.active_chassis || ch.type, model_rev: spec.model_rev || 1,
    photoelectric: spec.photoelectric || [], lift: spec.lift, footprint: ch.footprint, wheels: spec.active_wheels || spec.wheels || [],
    lidars: spec.lidars || [], cameras: spec.cameras || [], chassis: ch, imu: spec.imu, battery: spec.battery, io: spec.io,
    lidars_full: spec.lidars || [],
  };
}

function baseScene(el, { bg = 0xf1f5f9, fog = true, grid = true } = {}) {
  const THREE = T();
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: false });
  renderer.setPixelRatio(Math.min(1.5, window.devicePixelRatio || 1));
  renderer.domElement.className = 'absolute inset-0 w-full h-full';
  el.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(bg);
  if (fog) scene.fog = new THREE.FogExp2(bg, 0.012);
  const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 400);
  camera.up.set(0, 0, 1);
  camera.position.set(-8, -10, 9);
  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; controls.dampingFactor = 0.08;
  scene.add(new THREE.AmbientLight(0xffffff, 0.75));
  const dl = new THREE.DirectionalLight(0xffffff, 0.65); dl.position.set(10, -12, 20); scene.add(dl);
  let gridObj = null;
  if (grid) {
    gridObj = new THREE.GridHelper(60, 60, 0x94a3b8, bg === 0x0f172a ? 0x1e293b : 0xe2e8f0);
    gridObj.rotation.x = Math.PI / 2; gridObj.position.z = -0.003; scene.add(gridObj);
  }
  const ro = new ResizeObserver(() => {
    const r = el.getBoundingClientRect();
    if (r.width < 2) return;
    camera.aspect = r.width / r.height; camera.updateProjectionMatrix(); renderer.setSize(r.width, r.height, false);
  });
  ro.observe(el);
  return { THREE, renderer, scene, camera, controls, ro, gridObj };
}

// ---------------------------------------------------------------- 障碍物/注入元素 3D (托盘/货架/纸箱/人员)
export function nodeName(l) {
  if (!l) return '';
  return String(l).replace(/^N_/, '').replace(/^(FMS|GRID|WH|DEMO)_/i, '').replace(/^STATION_/, '站 ').replace(/_/g, ' ');
}
function textSprite(text, color) {
  const THREE = T();
  const cv = document.createElement('canvas'); cv.width = 256; cv.height = 64;
  const c = cv.getContext('2d');
  c.fillStyle = 'rgba(15,23,42,.86)'; c.fillRect(6, 8, 244, 48);
  c.fillStyle = color; c.fillRect(6, 8, 8, 48);
  c.font = 'bold 22px "PingFang SC","Microsoft YaHei",sans-serif'; c.fillStyle = '#f8fafc'; c.textAlign = 'center'; c.textBaseline = 'middle';
  c.fillText(text, 132, 33);
  const tex = new THREE.CanvasTexture(cv); tex.minFilter = THREE.LinearFilter;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
  sp.scale.set(1.3, 0.33, 1); sp.renderOrder = 12;
  return sp;
}
const std = (c, r = 0.5) => new (T().MeshLambertMaterial)({ color: c });
export function obstacleMesh(o) {
  const THREE = T();
  const g = new THREE.Group();
  const w = o.w || 0.8, h = o.h || 0.8, type = o.type || 'box';
  if (type === 'person') {
    const ring = new THREE.Mesh(new THREE.RingGeometry(Math.max(w, h) * .65, Math.max(w, h) * .78, 32), new THREE.MeshBasicMaterial({ color: 0xef4444, side: THREE.DoubleSide, transparent: true, opacity: .6 }));
    ring.position.z = .02; g.add(ring);
    [-0.1, 0.1].forEach(x => { const l = new THREE.Mesh(new THREE.CylinderGeometry(.08, .09, .75, 10), std(0x1e3a8a)); l.rotation.x = Math.PI / 2; l.position.set(x, 0, .375); g.add(l); });
    const v = new THREE.Mesh(new THREE.BoxGeometry(.42, .28, .65), std(0xf97316)); v.position.z = 1.05; g.add(v);
    const s = new THREE.Mesh(new THREE.BoxGeometry(.44, .3, .07), new THREE.MeshBasicMaterial({ color: 0xf1f5f9 })); s.position.z = 1.18; g.add(s);
    const hd = new THREE.Mesh(new THREE.SphereGeometry(.12, 14, 14), std(0xfde68a)); hd.position.z = 1.5; g.add(hd);
    const hm = new THREE.Mesh(new THREE.SphereGeometry(.145, 14, 14, 0, Math.PI * 2, 0, Math.PI / 2), std(0xeab308)); hm.position.z = 1.53; g.add(hm);
    const lb = textSprite(`👷 人员 #${o.id ?? ''}`, '#e11d48'); lb.position.z = 2.0; g.add(lb);
  } else if (type === 'pallet') {
    [-w / 2 + .08, 0, w / 2 - .08].forEach(x => { const s = new THREE.Mesh(new THREE.BoxGeometry(.1, h, .09), std(0x92400e)); s.position.set(x, 0, .045); g.add(s); });
    const d = new THREE.Mesh(new THREE.BoxGeometry(w, h, .04), std(0xb45309)); d.position.z = .11; g.add(d);
    const lb = textSprite(`🪵 栈板 #${o.id ?? ''}`, '#d97706'); lb.position.z = .7; g.add(lb);
  } else if (type === 'shelf') {
    const H = o.z || 2.2, lx = w / 2 - .04, ly = h / 2 - .04;
    [[-lx, -ly], [lx, -ly], [lx, ly], [-lx, ly]].forEach(([x, y]) => { const p = new THREE.Mesh(new THREE.BoxGeometry(.06, .06, H), std(0x1d4ed8)); p.position.set(x, y, H / 2); g.add(p); });
    [.9, H - .05].forEach(z => { const d = new THREE.Mesh(new THREE.BoxGeometry(w, h, .04), std(0xea580c)); d.position.z = z; g.add(d); });
    const b = new THREE.Mesh(new THREE.BoxGeometry(w * .8, h * .7, .35), std(0x0284c7)); b.position.z = 1.1; g.add(b);
    const lb = textSprite(`🏗️ 货架 #${o.id ?? ''}`, '#2563eb'); lb.position.z = H + .4; g.add(lb);
  } else {
    const z = o.z || Math.min(w, h) * .9;
    const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, z), std(0xd97706)); b.position.z = z / 2; g.add(b);
    const t = new THREE.Mesh(new THREE.BoxGeometry(w + .004, .1, .006), new THREE.MeshBasicMaterial({ color: 0x78350f })); t.position.z = z + .003; g.add(t);
    const lb = textSprite(`📦 纸箱 #${o.id ?? ''}`, '#b45309'); lb.position.z = z + .35; g.add(lb);
  }
  g.position.set(o.x, o.y, 0); g.rotation.z = o.yaw || 0;
  return g;
}
function disposeTree(o) { o.traverse(c => { c.geometry && c.geometry.dispose(); if (c.material) (Array.isArray(c.material) ? c.material : [c.material]).forEach(m => { m.map && m.map.dispose(); m.dispose(); }); }); }

// ---------------------------------------------------------------- 工作台实时视图
export class Viewer3D {
  constructor(el) {
    Object.assign(this, baseScene(el, { bg: 0xf1f5f9 }));
    this.el = el;
    this.amr = window.AMR3D.create();
    const THREE = this.THREE;
    this.robotGroup = new THREE.Group(); this.scene.add(this.robotGroup);
    this.obsGroup = new THREE.Group(); this.scene.add(this.obsGroup);
    const band = (color, op) => { const m = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: op, side: THREE.DoubleSide, depthWrite: false })); this.scene.add(m); return m; };
    this.pathDone = band(0x94a3b8, .35);        // 已走过
    this.pathNext = band(0x06b6d4, .55);        // 后续路段
    this.pathCur = band(0xf97316, .75);         // 当前路段
    this.pathLead = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0xea580c, dashSize: .18, gapSize: .12 }));
    this.scene.add(this.pathLead);
    this.chevGroup = new THREE.Group(); this.scene.add(this.chevGroup);
    this.nodeGroup = new THREE.Group(); this.scene.add(this.nodeGroup);
    this.wpMarker = new THREE.Group(); this.scene.add(this.wpMarker);
    const ring = new THREE.Mesh(new THREE.RingGeometry(.3, .45, 40), new THREE.MeshBasicMaterial({ color: 0xf97316, transparent: true, opacity: .9, side: THREE.DoubleSide, depthWrite: false }));
    ring.position.z = .09; this.wpMarker.add(ring); this.wpRing = ring;
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(.03, .03, 1.1, 8), new THREE.MeshBasicMaterial({ color: 0xf97316 }));
    pole.rotation.x = Math.PI / 2; pole.position.z = .55; this.wpMarker.add(pole);
    this.wpMarker.visible = false; this._wpLabel = null; this._wpLabelText = '';
    this.protGroup = new THREE.Group(); this.scene.add(this.protGroup); this._protSig = ''; this.showProt = true;
    this.ghost = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0x2563eb, dashSize: .3, gapSize: .2 }));
    this.scene.add(this.ghost);
    // SLAM 地图 (执行进程建图) + 估计位姿轮廓 (车体为真值位置，轮廓为定位结果)
    this.slamMesh = null; this.showSlam = true; this._slamRev = null;
    this.estLine = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xf97316 }));
    this.estLine.renderOrder = 6; this.scene.add(this.estLine); this._estSig = '';
    const N = 16000;
    this.ptPos = new Float32Array(N * 3); this.ptCol = new Float32Array(N * 3);
    const pg = new THREE.BufferGeometry();
    pg.setAttribute('position', new THREE.BufferAttribute(this.ptPos, 3));
    pg.setAttribute('color', new THREE.BufferAttribute(this.ptCol, 3));
    this.points = new THREE.Points(pg, new THREE.PointsMaterial({ size: .07, vertexColors: true }));
    this.scene.add(this.points);
    this.mode = 'free'; this.colorMode = 'height';
    this.tel = {}; this.pose = { x: 0, y: 0, yaw: 0 }; this._obsSig = ''; this._pathSig = ''; this._envSig = '';
    this.running = true;
    const loop = () => { if (!this.running) return; this._frame(); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }
  destroy() { this.running = false; this.ro.disconnect(); this.controls.dispose(); this.renderer.dispose(); this.renderer.domElement.remove(); }
  setMode(m) {
    this.mode = m;
    const p = this.pose;
    if (m === 'top') { this.camera.position.set(p.x, p.y - 0.01, 22); this.controls.target.set(p.x, p.y, 0); }
    if (m === 'free') { this.camera.position.set(p.x - 8, p.y - 10, 9); this.controls.target.set(p.x, p.y, .5); }
  }
  setTelemetry(t) {
    this.tel = t || {};
    if (t && t.x !== undefined) this.target = { x: t.x, y: t.y, yaw: t.yaw };
  }
  setSlamMap(m) {
    const THREE = this.THREE;
    if (this.slamMesh) { this.scene.remove(this.slamMesh); this.slamMesh.geometry.dispose(); this.slamMesh.material.map.dispose(); this.slamMesh.material.dispose(); this.slamMesh = null; }
    if (!m || m.empty || !m.data) return;
    const raw = atob(m.data), n = m.w * m.h, px = new Uint8Array(n * 4);
    for (let i = 0; i < n; i++) {
      const c = raw.charCodeAt(i), k = i * 4;
      if (c === 2) { px[k] = 30; px[k + 1] = 41; px[k + 2] = 59; px[k + 3] = 235; }           // 占据
      else if (c === 1) { px[k] = 147; px[k + 1] = 197; px[k + 2] = 253; px[k + 3] = 60; }    // 空闲
    }
    const tex = new THREE.DataTexture(px, m.w, m.h, THREE.RGBAFormat);
    tex.magFilter = THREE.NearestFilter; tex.minFilter = THREE.LinearFilter; tex.needsUpdate = true;
    const W = m.w * m.res, H = m.h * m.res;
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
    mesh.position.set(m.origin[0] + W / 2, m.origin[1] + H / 2, .006); mesh.renderOrder = 1;
    mesh.visible = this.showSlam;
    this.slamMesh = mesh; this.scene.add(mesh);
  }
  setSlamVisible(v) { this.showSlam = v; if (this.slamMesh) this.slamMesh.visible = v; this.estLine.visible = v; }
  _drawEst(t) {
    const L = t.localization, p = L && L.pose, fp = (t.robot_spec || {}).footprint;
    if (!p || !fp || !this.showSlam) { this.estLine.visible = false; return; }
    this.estLine.visible = true;
    const sig = `${p.x},${p.y},${p.yaw},${fp.length}`;
    if (sig === this._estSig) return;
    this._estSig = sig;
    const c = Math.cos(p.yaw), s = Math.sin(p.yaw), V = this.THREE.Vector3;
    const hx = Math.max(...fp.map(q => q[0]));
    const pts = fp.map(q => new V(p.x + c * q[0] - s * q[1], p.y + s * q[0] + c * q[1], .05));
    pts.push(new V(p.x + c * hx, p.y + s * hx, .05), new V(p.x + c * (hx + .4), p.y + s * (hx + .4), .05), new V(p.x + c * hx, p.y + s * hx, .05));
    this.estLine.geometry.dispose();
    this.estLine.geometry = new this.THREE.BufferGeometry().setFromPoints(pts);
  }
  setGhost(pts) {
    const THREE = this.THREE;
    this.ghost.geometry.dispose();
    this.ghost.geometry = new THREE.BufferGeometry().setFromPoints(pts.map(p => new THREE.Vector3(p.x, p.y, .03)));
    this.ghost.computeLineDistances();
  }
  _band(mesh, pts, w, z) {
    const THREE = this.THREE, tri = [];
    for (let i = 0; i + 1 < pts.length; i++) {
      const a = pts[i], b = pts[i + 1], dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy);
      if (L < 1e-4) continue;
      const nx = -dy / L * w, ny = dx / L * w;
      tri.push(a.x + nx, a.y + ny, z, a.x - nx, a.y - ny, z, b.x + nx, b.y + ny, z, b.x + nx, b.y + ny, z, a.x - nx, a.y - ny, z, b.x - nx, b.y - ny, z);
    }
    mesh.geometry.dispose();
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(tri, 3)); mesh.geometry = g;
  }
  _chevrons(pts, phase) {
    // 沿折线每 0.6 m 放一个 ">" 箭头，phase 使其向前流动
    const THREE = this.THREE, G = this.chevGroup;
    if (!this._chevGeo) {
      const sh = new THREE.Shape(); sh.moveTo(-.12, .14); sh.lineTo(.06, 0); sh.lineTo(-.12, -.14); sh.lineTo(-.05, 0); sh.closePath();
      this._chevGeo = new THREE.ShapeGeometry(sh);
      this._chevMat = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: .95, depthWrite: false });
    }
    let k = 0; const step = .6;
    for (let i = 0; i + 1 < pts.length; i++) {
      const a = pts[i], b = pts[i + 1], L = Math.hypot(b.x - a.x, b.y - a.y); if (L < 1e-3) continue;
      const yaw = Math.atan2(b.y - a.y, b.x - a.x);
      for (let d = (phase * step) % step; d < L; d += step) {
        let m = G.children[k]; if (!m) { m = new THREE.Mesh(this._chevGeo, this._chevMat); m.renderOrder = 5; G.add(m); }
        m.visible = true; m.position.set(a.x + (b.x - a.x) * d / L, a.y + (b.y - a.y) * d / L, .08); m.rotation.z = yaw; k++;
        if (k > 160) break;
      }
    }
    for (let j = k; j < G.children.length; j++) G.children[j].visible = false;
  }
  _drawPath(t) {
    const THREE = this.THREE, path = t.plan_path || [], labels = t.path_labels || [];
    const active = path.length >= 2 && !['ARRIVED', 'CANCELED', 'FAILED', 'NO_PATH', 'IDLE'].includes(t.nav_status);
    let idx = Math.max(1, Math.min(path.length - 1, t.path_index || 1));
    const sig = path.length + ':' + (path[0] ? path[0].x + ',' + path[0].y + ',' + path[path.length - 1].x : '') + ':' + idx + ':' + active;
    const p = this.pose;
    if (!active) {
      if (this._pathSig !== sig) { this._pathSig = sig; [this.pathDone, this.pathNext, this.pathCur].forEach(m => this._band(m, [], .1, 0)); this.chevGroup.children.forEach(c => c.visible = false); }
      this.wpMarker.visible = false; this.pathLead.visible = false;
      this._nodes(path, labels, false);
      return;
    }
    // 投影车辆到当前路段，当前路段从车辆位置画起
    const a = path[idx - 1], b = path[idx];
    const dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy || 1;
    const u = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2));
    const proj = { x: a.x + dx * u, y: a.y + dy * u };
    if (this._pathSig !== sig || !this._lastProj || Math.hypot(proj.x - this._lastProj.x, proj.y - this._lastProj.y) > .03) {
      this._pathSig = sig; this._lastProj = proj;
      this._band(this.pathDone, path.slice(0, idx).concat([proj]), .1, .05);
      this._band(this.pathCur, [proj, b], .3, .07);
      const curve = (t.plan_curve || []).length > 1 ? t.plan_curve : null;
      this._band(this.pathNext, curve ? curve : path.slice(idx), .2, .06);
      this.pathLead.geometry.dispose();
      this.pathLead.geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(p.x, p.y, .06), new THREE.Vector3(b.x, b.y, .06)]);
      this.pathLead.computeLineDistances();
      this._nodes(path, labels, true);
    }
    this.pathLead.visible = true;
    // 流动箭头: 当前路段 + 下一路段
    const ph = (performance.now() / 700) % 1;
    const ahead = [proj, b]; if (path[idx + 1]) ahead.push(path[idx + 1]);
    this._chevrons(ahead, ph);
    this.pathCur.material.opacity = .7 + .2 * Math.sin(performance.now() / 260);
    // 下一航点标记
    this.wpMarker.visible = true; this.wpMarker.position.set(b.x, b.y, 0);
    const sc = 1 + .15 * Math.sin(performance.now() / 200); this.wpRing.scale.set(sc, sc, 1);
    const dist = Math.hypot(b.x - p.x, b.y - p.y);
    const lbl = `下一点 ${nodeName(labels[idx]) || `(${b.x.toFixed(1)}, ${b.y.toFixed(1)})`} · ${dist.toFixed(1)}m`;
    if (lbl !== this._wpLabelText) {
      this._wpLabelText = lbl;
      if (this._wpLabel) { this.wpMarker.remove(this._wpLabel); disposeTree(this._wpLabel); }
      this._wpLabel = textSprite(lbl, '#f97316'); this._wpLabel.position.z = 1.35; this._wpLabel.scale.set(2.6, .65, 1); this.wpMarker.add(this._wpLabel);
    }
  }
  _drawProtection(t) {
    // 保护空间 (机体系): 当前档停车区 (按状态着色) / 预警区 (虚线) / 带载外形
    const THREE = this.THREE, G = this.protGroup, pv = t.protection;
    G.visible = !!(this.showProt && pv && pv.polygons);
    if (!G.visible) return;
    G.position.set(this.pose.x, this.pose.y, 0); G.rotation.z = this.pose.yaw;
    const zone = pv.zone || 'clear';
    const sig = JSON.stringify([pv.polygons, zone, pv.loaded, pv.band]);
    if (sig === this._protSig) return;
    this._protSig = sig;
    while (G.children.length) { const c = G.children[0]; G.remove(c); disposeTree(c); }
    const col = { stop: 0xdc2626, slow: 0xf59e0b, warn: 0xf59e0b, clear: 0x16a34a }[zone] || 0x16a34a;
    const loop = (pts, color, z, dashed, op) => {
      const v = pts.concat([pts[0]]).map(p => new THREE.Vector3(p[0], p[1], z));
      const geo = new THREE.BufferGeometry().setFromPoints(v);
      const mat = dashed ? new THREE.LineDashedMaterial({ color, dashSize: .15, gapSize: .1, transparent: true, opacity: op }) : new THREE.LineBasicMaterial({ color, transparent: true, opacity: op });
      const l = new THREE.Line(geo, mat); if (dashed) l.computeLineDistances(); G.add(l); return l;
    };
    const fill = (pts, color, op, z) => {
      const sh = new THREE.Shape(); pts.forEach((p, i) => i ? sh.lineTo(p[0], p[1]) : sh.moveTo(p[0], p[1])); sh.closePath();
      const m = new THREE.Mesh(new THREE.ShapeGeometry(sh), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: op, depthWrite: false, side: THREE.DoubleSide }));
      m.position.z = z; G.add(m);
    };
    const P = pv.polygons;
    fill(P.stop, col, zone === 'stop' ? .28 : .12, .012);
    loop(P.stop, col, .03, false, .9);
    loop(P.slow, col, .03, true, .6);
    if (pv.loaded && P.payload) loop(P.payload, 0x7c3aed, .05, false, .95);
    if (P.photo && (pv.config?.photo?.mode === 'custom')) loop(P.photo, 0xca8a04, .04, true, .7);   // 光电包络 (field 模式与停车区重合)
  }
  _nodes(path, labels, on) {
    const THREE = this.THREE, G = this.nodeGroup;
    while (G.children.length) { const c = G.children[0]; G.remove(c); disposeTree(c); }
    if (!on) return;
    path.forEach((q, i) => {
      if (i === 0) return;
      const d = new THREE.Mesh(new THREE.CircleGeometry(i === path.length - 1 ? .16 : .1, 20),
        new THREE.MeshBasicMaterial({ color: i === path.length - 1 ? 0x16a34a : (labels[i] ? 0x0891b2 : 0x64748b), transparent: true, opacity: .95, depthWrite: false }));
      d.position.set(q.x, q.y, .09); G.add(d);
    });
  }
  _frame() {
    const THREE = this.THREE, t = this.tel;
    if (this.target) {   // 位姿平滑
      const k = 0.35, p = this.pose;
      p.x += (this.target.x - p.x) * k; p.y += (this.target.y - p.y) * k;
      let dy = this.target.yaw - p.yaw; while (dy > Math.PI) dy -= 2 * Math.PI; while (dy < -Math.PI) dy += 2 * Math.PI; p.yaw += dy * k;
    }
    this.amr.update({ scene: this.scene, robotGroup: this.robotGroup, pointCloud: this.points, telemetry: t, pose: this.pose, cameraMode: this.mode === 'inspect' ? 'inspect' : '3d', camera: this.camera, controls: this.controls, renderer: this.renderer });
    this.robotGroup.position.set(this.pose.x, this.pose.y, 0); this.robotGroup.rotation.z = this.pose.yaw;
    this._drawProtection(t);
    this._drawEst(t);
    // 障碍物
    const obs = t.dynamic_obstacles || t.obstacles || [];
    const sig = JSON.stringify(obs.map(o => [o.id, o.type, o.x, o.y, o.yaw, o.w, o.h]));
    if (sig !== this._obsSig) {
      this._obsSig = sig;
      while (this.obsGroup.children.length) { const c = this.obsGroup.children[0]; this.obsGroup.remove(c); disposeTree(c); }
      obs.forEach(o => this.obsGroup.add(obstacleMesh(o)));
    }
    // 规划路径: 已走过 (灰) / 当前路段 (高亮 + 流动箭头) / 后续路段 (青) / 下一航点标记
    this._drawPath(t);
    // 融合扫描回退点云 (无分激光数据时) + 着色模式
    if (this.points.visible && t.scan_ranges && !t.lidar_scans) {
      const rs = t.scan_ranges, sp = t.scan_pose || this.pose, a0 = t.scan_angle_min ?? -Math.PI, inc = t.scan_angle_inc ?? (2 * Math.PI / rs.length);
      const rmax = t.scan_range_max || 30; let k = 0;
      for (let i = 0; i < rs.length && k < 16000; i++) {
        const r = rs[i]; if (!(r > .05 && r < rmax - .1)) continue;
        const a = sp.yaw + a0 + i * inc;
        this.ptPos[k * 3] = sp.x + r * Math.cos(a); this.ptPos[k * 3 + 1] = sp.y + r * Math.sin(a); this.ptPos[k * 3 + 2] = .3;
        const c = this.colorMode === 'distance' ? Math.min(1, r / 10) : .6;
        this.ptCol[k * 3] = 1 - c; this.ptCol[k * 3 + 1] = .4 + c * .4; this.ptCol[k * 3 + 2] = c; k++;
      }
      this.points.geometry.setDrawRange(0, k);
      this.points.geometry.attributes.position.needsUpdate = true; this.points.geometry.attributes.color.needsUpdate = true;
    }
    // 相机
    const p = this.pose;
    if (this.mode === 'follow') {
      this.controls.target.set(p.x, p.y, .6);
      const cx = p.x - Math.cos(p.yaw) * 7, cy = p.y - Math.sin(p.yaw) * 7;
      this.camera.position.x += (cx - this.camera.position.x) * .08; this.camera.position.y += (cy - this.camera.position.y) * .08;
      this.camera.position.z += (4.5 - this.camera.position.z) * .08;
    } else if (this.mode === 'top') {
      this.controls.target.x += (p.x - this.controls.target.x) * .05; this.controls.target.y += (p.y - this.controls.target.y) * .05;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

// ---------------------------------------------------------------- 回放视图: 由 sim_bundle 构建环境
export class ReplayView extends Viewer3D {
  loadBundle(b) {
    const env = b.environment || {};
    const meta = { id: env.sceneId || env.sceneName || 'replay', name: env.sceneName, walls: env.walls || [], shelves: env.shelves || [],
      stations: env.stations || [], bounds: env.bounds || boundsOf(env) };
    const nodes = {}; (env.topologyNodes || []).forEach(n => { nodes[n.id] = { x: n.x, y: n.y, name: n.id }; });
    const edges = (env.topologyEdges || []).filter(([a, c]) => nodes[a] && nodes[c]).map(([a, c]) => ({ from: a, to: c, p1: nodes[a], p2: nodes[c] }));
    const vm = b.vehicleModel || {};
    const fp = vm.footprint || [[(vm.length || 1.2) / 2, (vm.width || .8) / 2], [(vm.length || 1.2) / 2, -(vm.width || .8) / 2], [-(vm.length || 1.2) / 2, -(vm.width || .8) / 2], [-(vm.length || 1.2) / 2, (vm.width || .8) / 2]];
    const spec = { model_file: vm.modelName, active_chassis: vm.chassis || vm.type, model_rev: Math.random(),
      chassis: { type: vm.type, head_offset_m: Math.max(...fp.map(p => p[0])), tail_offset_m: -Math.min(...fp.map(p => p[0])),
        left_offset_m: Math.max(...fp.map(p => p[1])), right_offset_m: -Math.min(...fp.map(p => p[1])), height_m: .5, footprint: fp },
      wheels: [], lidars_full: [], cameras: [] };
    this.baseTel = { robot_spec: spec, scenario_metadata: meta, topo_graph: { nodes, edges } };
    this.points.visible = false;
    this.setGhost(b.trajectory || []);
    const tr = b.trajectory || [];
    if (tr.length) { this.pose = { x: tr[0].x, y: tr[0].y, yaw: tr[0].yaw }; this.setMode('free'); }
  }
  showFrame(fr, obstacles) {
    this.setTelemetry(Object.assign({}, this.baseTel, { x: fr.x, y: fr.y, yaw: fr.yaw, dynamic_obstacles: obstacles || [] }));
  }
}
function boundsOf(env) {
  const w = env.walls || []; if (!w.length) return { min_x: -10, max_x: 10, min_y: -10, max_y: 10 };
  const xs = w.flatMap(a => [a[0], a[2]]), ys = w.flatMap(a => [a[1], a[3]]);
  return { min_x: Math.min(...xs), max_x: Math.max(...xs), min_y: Math.min(...ys), max_y: Math.max(...ys) };
}

// ---------------------------------------------------------------- 模型详情 3D
export class ModelView {
  constructor(el, spec) {
    Object.assign(this, baseScene(el, { bg: 0x0f172a, fog: false }));
    this.amr = window.AMR3D.create();
    this.group = new this.THREE.Group(); this.scene.add(this.group);
    this.spec = robotSpecFromModel(spec);
    this.amr.buildRobot(this.group, this.spec);
    const ch = spec.chassis || {};
    this.size = Math.max(1.2, (ch.length_m || 1.5));
    this.center = [((ch.head_offset_m || .6) - (ch.tail_offset_m || .6)) / 2, 0, (ch.height_m || .5) / 2];
    this.view('persp');
    this.running = true;
    const loop = () => { if (!this.running) return; this.controls.update(); this.renderer.render(this.scene, this.camera); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }
  view(v) {
    const k = this.size, [cx, cy, cz] = this.center;
    const P = { persp: [cx + 1.6 * k, cy - 1.9 * k, cz + 1.4 * k], top: [cx, cy - .001, cz + 2.8 * k], side: [cx, cy - 2.6 * k, cz], front: [cx + 2.6 * k, cy, cz] }[v];
    this.camera.position.set(...P); this.controls.target.set(cx, cy, cz);
  }
  wireframe(on) { this.group.traverse(c => { if (c.material && 'wireframe' in c.material) c.material.wireframe = on; }); }
  destroy() { this.running = false; this.ro.disconnect(); this.controls.dispose(); this.renderer.dispose(); this.renderer.domElement.remove(); }
}

// ---------------------------------------------------------------- 场景详情 3D
export class SceneView {
  constructor(el, sc, grid) {
    Object.assign(this, baseScene(el, { bg: 0x0b1220, fog: false, grid: false }));
    const THREE = this.THREE;
    this.sc = sc;
    this.layers = { env: new THREE.Group(), grid: new THREE.Group(), topo: new THREE.Group() };
    Object.values(this.layers).forEach(g => this.scene.add(g));
    const b = sceneBounds(sc);
    this.b = b;
    const W = b.x1 - b.x0, H = b.y1 - b.y0, cx = (b.x0 + b.x1) / 2, cy = (b.y0 + b.y1) / 2;
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshLambertMaterial({ color: 0x1e293b }));
    floor.position.set(cx, cy, -0.01); this.scene.add(floor);
    const gh = new THREE.GridHelper(Math.max(W, H), Math.round(Math.max(W, H)), 0x334155, 0x1e293b); gh.rotation.x = Math.PI / 2; gh.position.set(cx, cy, 0); this.scene.add(gh);
    const shelfH = (sc.meta && sc.meta.shelf_height_m) || 2.5;
    (sc.walls || []).forEach((w, i) => {
      const L = Math.hypot(w[2] - w[0], w[3] - w[1]); if (L < .02) return;
      const h = i < 4 ? 1.2 : (sc.meta && sc.meta.walls_from_grid ? 1.0 : shelfH);
      const m = new THREE.Mesh(new THREE.BoxGeometry(L, .1, h), new THREE.MeshLambertMaterial({ color: i < 4 ? 0x475569 : 0x64748b, transparent: i < 4, opacity: i < 4 ? .7 : 1 }));
      m.position.set((w[0] + w[2]) / 2, (w[1] + w[3]) / 2, h / 2); m.rotation.z = Math.atan2(w[3] - w[1], w[2] - w[0]);
      this.layers.env.add(m);
    });
    (sc.shelves || []).forEach(s => {
      const x0 = Math.min(s.x1, s.x2), x1 = Math.max(s.x1, s.x2), y0 = Math.min(s.y1, s.y2), y1 = Math.max(s.y1, s.y2);
      const m = new THREE.Mesh(new THREE.BoxGeometry(x1 - x0, y1 - y0, shelfH), new THREE.MeshLambertMaterial({ color: 0x475569 }));
      m.position.set((x0 + x1) / 2, (y0 + y1) / 2, shelfH / 2); this.layers.env.add(m);
      const e = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: 0x94a3b8 })); e.position.copy(m.position); this.layers.env.add(e);
    });
    (sc.reflectors || []).forEach(r => {
      const m = new THREE.Mesh(new THREE.CylinderGeometry(.06, .06, 2, 10), new THREE.MeshBasicMaterial({ color: 0x38bdf8 }));
      m.rotation.x = Math.PI / 2; m.position.set(r.x, r.y, 1); this.layers.env.add(m);
    });
    if (grid) this.setGrid(grid.dec, grid.meta);
    const topo = topoFromScene(sc);
    const pos = []; topo.edges.forEach(e => pos.push(e.p1.x, e.p1.y, .05, e.p2.x, e.p2.y, .05));
    const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    this.layers.topo.add(new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ color: 0x10b981 })));
    Object.values(topo.nodes).forEach(n => { const m = new THREE.Mesh(new THREE.SphereGeometry(.12, 10, 10), new THREE.MeshBasicMaterial({ color: 0x34d399 })); m.position.set(n.x, n.y, .08); this.layers.topo.add(m); });
    (sc.stations || []).forEach(s => {
      const m = new THREE.Mesh(new THREE.CylinderGeometry(.3, .3, .04, 24), new THREE.MeshBasicMaterial({ color: 0x3b82f6 }));
      m.rotation.x = Math.PI / 2; m.position.set(s.x, s.y, .03); this.layers.topo.add(m);
      const lb = textSprite(s.id + (s.name && s.name !== s.id ? ' ' + s.name : ''), '#3b82f6'); lb.position.set(s.x, s.y, .9); lb.scale.multiplyScalar(Math.max(1.8, Math.max(W, H) / 14)); this.layers.topo.add(lb);
    });
    this.view('persp');
    this.running = true;
    const loop = () => { if (!this.running) return; this.controls.update(); this.renderer.render(this.scene, this.camera); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }
  setGrid(dec, meta) {
    const THREE = this.THREE;
    while (this.layers.grid.children.length) this.layers.grid.remove(this.layers.grid.children[0]);
    const tex = new THREE.CanvasTexture(dec.canvas); tex.magFilter = THREE.NearestFilter;
    const w = dec.W * meta.res, h = dec.H * meta.res;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ map: tex, transparent: true, opacity: .55 }));
    m.position.set(meta.origin[0] + w / 2, meta.origin[1] + h / 2, .015);
    this.layers.grid.add(m);
  }
  toggle(k, on) { this.layers[k].visible = on; }
  view(v) {
    const b = this.b, cx = (b.x0 + b.x1) / 2, cy = (b.y0 + b.y1) / 2, S = Math.max(b.x1 - b.x0, b.y1 - b.y0);
    if (v === 'persp') { this.camera.position.set(cx - S * .55, cy - S * .75, S * .65); this.controls.target.set(cx, cy, 0); }
    if (v === 'top') { this.camera.position.set(cx, cy - .01, S * 1.1); this.controls.target.set(cx, cy, 0); }
    if (v === 'aisle') {
      const st = (this.sc.stations || [])[0] || { x: cx, y: cy };
      this.camera.position.set(st.x - 3, st.y - 3, 1.7); this.controls.target.set(cx, cy, 1.0);
    }
  }
  destroy() { this.running = false; this.ro.disconnect(); this.controls.dispose(); this.renderer.dispose(); this.renderer.domElement.remove(); }
}
