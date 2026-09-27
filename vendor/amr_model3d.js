/*
 * AMR Studio V4 · cmodel 驱动的 3D/2D 机器人与仿真环境渲染
 *
 * 数据来源 (全部来自 /api/telemetry):
 *   robot_spec      ← 仿真节点 /robot_spec (cmodel_parser 解析结果): 车体轮廓/高度、轮组(舵轮/驱动轮/承重轮/万向轮)、
 *                     激光(安装 6DoF、倒装、视场、量程)、相机、IMU、电池、电机
 *   joint_states    ← /joint_states: 舵角、轮子转角 (3D 中实时驱动舵轮转向与车轮转动)
 *   lidar_scans     ← /scan/<name>: 每个物理激光的点云，按其真实安装高度与位置渲染 (?scans=1)
 *   scenario_metadata: 墙体/货架/工位/拓扑，生成 3D 仓储环境
 *
 * 暴露: window.AMR3D.update(ctx) 每帧调用；AMR3D.draw2DRobot(ctx2D, telemetry, scale, dpr)
 */
(function () {
  'use strict';
  const T = () => window.THREE;
  const LIDAR_COLORS = [0x10b981, 0x6366f1, 0xf43f5e, 0xf59e0b, 0x06b6d4];
  const WALL_H = 6.0, SHELF_H = 2.5;   // 外墙到顶 6 m (与仿真世界一致)

  function makeInstance() {
    const st = {
      specSig: '', envSig: '', built: false,
      model: null, joints: {}, lidarClouds: {}, labels: [], fans: [],
      env: null, showLabels: true, showFans: true, showEnv: true, xray: false, labelsEverywhere: false, envLabels: [],
      overlay: null, lastOverlay: 0
    };

    // ------------------------------------------------------------------ utils
    // Lambert 光照 (比 PBR Standard 便宜得多，大面积环境几何的填充开销显著降低)
    function mat(color, opts) {
      const o = Object.assign({ color: color }, opts || {});
      delete o.roughness; delete o.metalness;
      return new (T().MeshLambertMaterial)(o);
    }
    // 自适应宽度的标签 (文字不截断)，世界高度 h 米
    function label(text, color, h) {
      const THREE = T();
      h = h || 0.075;
      const fs = 40, pad = 14;
      const cv = document.createElement('canvas');
      const cx = cv.getContext('2d');
      const font = `bold ${fs}px "PingFang SC","Microsoft YaHei","Noto Sans CJK SC",sans-serif`;
      cx.font = font;
      const tw = Math.ceil(cx.measureText(text).width);
      cv.width = tw + pad * 2; cv.height = fs + pad * 1.4;
      const c2 = cv.getContext('2d');
      c2.fillStyle = 'rgba(15,23,42,0.86)';
      const r = 12, w = cv.width, hh = cv.height;
      c2.beginPath(); c2.moveTo(r, 0); c2.lineTo(w - r, 0); c2.quadraticCurveTo(w, 0, w, r); c2.lineTo(w, hh - r);
      c2.quadraticCurveTo(w, hh, w - r, hh); c2.lineTo(r, hh); c2.quadraticCurveTo(0, hh, 0, hh - r); c2.lineTo(0, r); c2.quadraticCurveTo(0, 0, r, 0); c2.fill();
      c2.fillStyle = color || '#38bdf8'; c2.fillRect(0, 0, 8, hh);
      c2.font = font; c2.fillStyle = '#f8fafc'; c2.textBaseline = 'middle';
      c2.fillText(text, pad, hh / 2 + 2);
      const tex = new THREE.CanvasTexture(cv); tex.minFilter = THREE.LinearFilter;
      const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
      sp.scale.set(h * cv.width / cv.height, h, 1);
      sp.renderOrder = 10;
      return sp;
    }
    function disposeTree(o) {
      o.traverse(c => {
        if (c.geometry) c.geometry.dispose();
        if (c.material) { (Array.isArray(c.material) ? c.material : [c.material]).forEach(m => { if (m.map) m.map.dispose(); m.dispose(); }); }
      });
    }
    function clearGroup(g) { while (g.children.length) { const c = g.children[0]; g.remove(c); disposeTree(c); } }
    function hex(c) { return '#' + c.toString(16).padStart(6, '0'); }

    // ------------------------------------------------------------------ 机器人模型
    function buildRobot(group, spec) {
      const THREE = T();
      clearGroup(group);
      st.joints = {}; st.labels = []; st.fans = []; st.lidarClouds = {};
      const ch = spec.chassis || {};
      const head = ch.head_offset_m || 0.6, tail = ch.tail_offset_m || 0.6;
      const left = ch.left_offset_m || 0.4, right = ch.right_offset_m || 0.4;
      const H = ch.height_m || 0.35;
      const L = head + tail, W = left + right;
      const xc = (head - tail) / 2, yc = (left - right) / 2;
      const wheels = spec.wheels || [];
      const rmin = Math.min.apply(null, wheels.map(w => w.radius_m || 0.1).concat([0.1]));
      const clr = Math.max(0.02, rmin * 0.5);
      const baseH = H > 0.6 ? Math.min(0.35, H) : H;

      // 底盘 (半透明, 可看到内部轮组)
      const chassisMat = mat(0xf59e0b, { transparent: true, opacity: 0.55 });
      const base = new THREE.Mesh(new THREE.BoxGeometry(L, W, baseH), chassisMat);
      base.position.set(xc, yc, clr + baseH / 2);
      base.userData.xray = true;
      group.add(base);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(base.geometry), new THREE.LineBasicMaterial({ color: 0x92400e }));
      edges.position.copy(base.position);
      group.add(edges);

      // 上装 (门架/立柱)：线框 + 极淡填充
      if (H > 0.6) {
        const uh = H - baseH - clr;
        const ug = new THREE.BoxGeometry(L, W, uh);
        const up = new THREE.Mesh(ug, mat(0x94a3b8, { transparent: true, opacity: 0.08, depthWrite: false }));
        up.position.set(xc, yc, clr + baseH + uh / 2);
        group.add(up);
        const ue = new THREE.LineSegments(new THREE.EdgesGeometry(ug), new THREE.LineBasicMaterial({ color: 0x64748b, transparent: true, opacity: 0.6 }));
        ue.position.copy(up.position);
        group.add(ue);
        const lb = label(`上装 H=${H.toFixed(2)}m`, '#64748b');
        lb.position.set(xc - L / 2 + 0.3, yc, H + 0.15);
        group.add(lb); st.labels.push(lb);
      }

      // 运动中心 (base_link 原点) + 车头方向
      const mc = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.02, 24), new THREE.MeshBasicMaterial({ color: 0x111827 }));
      mc.rotation.x = Math.PI / 2; mc.position.set(0, 0, 0.01);
      group.add(mc);
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, clr + baseH + 0.02), Math.max(0.6, head * 0.8), 0xef4444, 0.18, 0.12);
      group.add(arrow);
      const lmc = label('运动中心 base_link', '#111827');
      lmc.position.set(0, 0, 0.05 + clr + baseH + 0.35);
      group.add(lmc); st.labels.push(lmc);

      // 轮组
      wheels.forEach(w => {
        const r = w.radius_m || 0.1, wid = w.width_m || Math.max(0.04, r * 0.6);
        const colors = { steer: 0xdc2626, drive: 0x1f2937, fixed: 0x6b7280, caster: 0xcbd5e1 };
        if (w.kind === 'caster') {
          const s = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), mat(colors.caster));
          s.position.set(w.x, w.y, r);
          group.add(s);
          return;
        }
        const pivot = new THREE.Group();
        pivot.position.set(w.x, w.y, r);
        group.add(pivot);
        const spin = new THREE.Group();
        pivot.add(spin);
        const tire = new THREE.Mesh(new THREE.CylinderGeometry(r, r, wid, 28), mat(colors[w.kind] || 0x1f2937));
        spin.add(tire);                       // Cylinder 轴沿 Y = 车轮轴
        const spoke = new THREE.Mesh(new THREE.BoxGeometry(r * 1.6, wid * 1.05, r * 0.18), mat(0xfbbf24));
        spin.add(spoke);                      // 辐条: 直观看到车轮转动
        if (w.kind === 'steer') {
          const hub = new THREE.Mesh(new THREE.CylinderGeometry(r * 0.8, r * 0.8, r * 0.35, 24), mat(0x475569));
          hub.rotation.x = Math.PI / 2; hub.position.z = r * 1.05;
          pivot.add(hub);
          const dir = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, r * 1.3), r * 2.6, 0xdc2626, r * 0.6, r * 0.4);
          pivot.add(dir);                     // 舵轮朝向指示
        }
        st.joints[w.name] = { w: w, pivot: pivot, spin: spin };
        const dm = w.drive_motor ? ` · ${w.drive_motor.model || w.drive_motor.name}` : '';
        const kindName = { steer: '舵轮', drive: '驱动轮', fixed: '承重轮' }[w.kind] || w.kind;
        const lb = label(`${kindName} ${w.name}${w.source === 'inferred' ? ' (推断)' : ''}`, hex(colors[w.kind] || 0x1f2937));
        lb.position.set(w.x, w.y, r * 2 + 0.35);
        group.add(lb); st.labels.push(lb);
        if (dm) {
          const lb2 = label(dm.slice(3), '#fbbf24');
          lb2.position.set(w.x, w.y, r * 2 + 0.1);
          group.add(lb2); st.labels.push(lb2);
        }
      });

      // 激光
      (spec.lidars_full || spec.lidars || []).forEach((l, i) => {
        const color = LIDAR_COLORS[i % LIDAR_COLORS.length];
        const g = new THREE.Group();
        g.position.set(l.x, l.y, l.z);
        group.add(g);
        const housing = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.1, 24), mat(color, { emissive: color, emissiveIntensity: 0.25 }));
        housing.rotation.x = Math.PI / 2;
        g.add(housing);
        // 安装立柱 (便于看清高度)
        const post = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, Math.max(0.01, l.z - 0.05), 8), new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.5 }));
        post.rotation.x = Math.PI / 2; post.position.z = -(l.z - 0.05) / 2 - 0.05;
        g.add(post);
        // 视场扇区 (传感器系 → 机体系：倒装时扫描方向反向)
        const fov = (l.fov_deg || 270) * Math.PI / 180;
        const sign = l.inverted ? -1 : 1;
        const amin = -fov / 2, amax = fov / 2;
        const thetaStart = (l.yaw || 0) + (sign > 0 ? amin : -amax);
        const is3d = l.type === '3d';
        let fan;
        if (is3d) {
          // 3D 激光垂直视场: 以 0.7 m 半径画出 vfov_min~vfov_max 的环带 (Mid-360S: -7°~+52°)
          const R = 0.7, v0 = (l.vfov_min_deg ?? -7) * Math.PI / 180, v1 = (l.vfov_max_deg ?? 52) * Math.PI / 180;
          const pts2 = [new THREE.Vector2(R * Math.cos(v0), R * Math.sin(v0)), new THREE.Vector2(0.001, 0), new THREE.Vector2(R * Math.cos(v1), R * Math.sin(v1))];
          fan = new THREE.Mesh(new THREE.LatheGeometry(pts2, 48),
            new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.13, side: THREE.DoubleSide, depthWrite: false }));
          fan.rotation.x = Math.PI / 2;     // Lathe 绕 Y 轴 → 转为绕 Z 轴
        } else {
          fan = new THREE.Mesh(new THREE.CircleGeometry(0.8, 64, thetaStart, fov),
            new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }));
        }
        g.add(fan); st.fans.push(fan);
        const fwd = new THREE.ArrowHelper(new THREE.Vector3(Math.cos(l.yaw || 0), Math.sin(l.yaw || 0), 0), new THREE.Vector3(0, 0, 0), 0.4, color, 0.1, 0.06);
        g.add(fwd);
        const lb = label(is3d
          ? `${l.name} ${l.vendor_model || l.model || ''} 3D 360°×${Math.round((l.vfov_max_deg ?? 52) - (l.vfov_min_deg ?? -7))}° z=${(+l.z).toFixed(2)}`
          : `${l.name} ${l.vendor_model || l.model || ''} z=${(+l.z).toFixed(2)}${l.inverted ? ' 倒装' : ''}`, hex(color));
        lb.position.set(0, 0, 0.3);
        g.add(lb); st.labels.push(lb);
        // 点云 (世界系，挂在场景上，由 update 负责)
        const n = is3d ? 4000 : 1440;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
        if (is3d) geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
        geo.setDrawRange(0, 0);
        const pts = new THREE.Points(geo, is3d
          ? new THREE.PointsMaterial({ size: 0.06, vertexColors: true })          // 3D 点云按高度着色
          : new THREE.PointsMaterial({ size: 0.07, color: color, transparent: true, opacity: 0.95 }));
        pts.frustumCulled = false;
        st.lidarClouds[l.name] = { pts: pts, cfg: l, max: n };
      });

      // 相机 / IMU
      const CAMS = { camera: [0xdb2777, '单目相机'], stereo: [0xea580c, '双目相机'], tof: [0x16a34a, 'ToF 相机'], depthCamera: [0x16a34a, 'ToF 相机'] };
      (spec.cameras || []).forEach(c => {
        const kind = CAMS[c.type] || [0x9333ea, '读码相机'];
        const g = new THREE.Group();
        g.position.set(c.x, c.y, Math.max(0.03, c.z));
        g.rotation.order = 'ZYX';
        g.rotation.set(c.roll || 0, c.pitch || 0, c.yaw || 0);
        const w = c.type === 'stereo' ? (c.baseline_m || 0.05) + 0.05 : 0.06;
        g.add(new THREE.Mesh(new THREE.BoxGeometry(0.04, w, 0.04), mat(kind[0])));
        if (CAMS[c.type]) {             // 视锥 (0.6 m)
          const hf = (c.hfov_deg || 70) * Math.PI / 360, vf = Math.atan(Math.tan(hf) * (c.height || 3) / (c.width || 4)), L = 0.6;
          const y = Math.tan(hf) * L, z = Math.tan(vf) * L;
          const P = [[0, 0, 0], [L, y, z], [0, 0, 0], [L, -y, z], [0, 0, 0], [L, y, -z], [0, 0, 0], [L, -y, -z], [L, y, z], [L, -y, z], [L, -y, z], [L, -y, -z], [L, -y, -z], [L, y, -z], [L, y, -z], [L, y, z]];
          const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(P.flat()), 3));
          g.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: kind[0], transparent: true, opacity: 0.7 })));
        }
        group.add(g);
        const lb = label(CAMS[c.type] ? `${kind[1]} ${c.name}` : `读码相机 ${c.name} (${(c.orientation || '').replace('LENS_DIR_', '')})`, '#' + kind[0].toString(16).padStart(6, '0'));
        lb.position.set(c.x, c.y, Math.max(0.03, c.z) + 0.25);
        group.add(lb); st.labels.push(lb);
      });
      if (spec.imu) {
        const m = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.02), mat(0x0ea5e9));
        m.position.set(spec.imu.x || 0, spec.imu.y || 0, clr + baseH + 0.015);
        group.add(m);
      }
      st.built = true;
    }

    // ------------------------------------------------------------------ 环境
    function buildEnv(scene, meta, topo) {
      const THREE = T();
      if (!st.env) { st.env = new THREE.Group(); scene.add(st.env); }
      clearGroup(st.env);
      st.envLabels = [];
      const b = meta.bounds || { min_x: -10, max_x: 10, min_y: -10, max_y: 10 };
      // 地面
      const floor = new THREE.Mesh(new THREE.PlaneGeometry(b.max_x - b.min_x, b.max_y - b.min_y), new THREE.MeshBasicMaterial({ color: 0xeef0f3 }));
      floor.position.set((b.max_x + b.min_x) / 2, (b.max_y + b.min_y) / 2, -0.002);
      st.env.add(floor);
      // 墙 (外墙 3m 半透明，避免遮挡)
      const walls = meta.walls || [];
      walls.slice(0, 4).forEach(w => {
        const [x0, y0, x1, y1] = w;
        const len = Math.hypot(x1 - x0, y1 - y0);
        // 墙体用线框表示 3 m 高度 + 实体踢脚 (大面积半透明墙面是 3D 帧率的主要瓶颈)
        const wg = new THREE.BoxGeometry(len, 0.12, WALL_H);
        const m = new THREE.LineSegments(new THREE.EdgesGeometry(wg), new THREE.LineBasicMaterial({ color: 0x94a3b8 }));
        wg.dispose();
        m.position.set((x0 + x1) / 2, (y0 + y1) / 2, WALL_H / 2);
        m.rotation.z = Math.atan2(y1 - y0, x1 - x0);
        st.env.add(m);
        const kick = new THREE.Mesh(new THREE.BoxGeometry(len, 0.14, 0.5), mat(0xcbd5e1));
        kick.position.set(m.position.x, m.position.y, 0.25); kick.rotation.z = m.rotation.z;
        st.env.add(kick);
      });
      // 货架: 立柱 + 层板 + 半透明体
      (meta.shelves || []).forEach(sh => {
        const xa = Math.min(sh.x1, sh.x2), xb = Math.max(sh.x1, sh.x2), ya = Math.min(sh.y1, sh.y2), yb = Math.max(sh.y1, sh.y2);
        const cx = (xa + xb) / 2, cy = (ya + yb) / 2, sx = xb - xa, sy = yb - ya;
        const g = new THREE.Group(); g.position.set(cx, cy, 0);
        const border = new THREE.Color(sh.border || '#38bdf8');
        const postMat = mat(0x1e3a8a);
        const nx = Math.max(2, Math.round(sx / 1.2) + 1), ny = Math.max(2, Math.round(sy / 1.2) + 1);
        for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
          if (i > 0 && i < nx - 1 && j > 0 && j < ny - 1) continue;
          const p = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.08, SHELF_H), postMat);
          p.position.set(-sx / 2 + i * sx / (nx - 1), -sy / 2 + j * sy / (ny - 1), SHELF_H / 2);
          g.add(p);
        }
        [0.15, 1.2, 2.4].forEach(z => {
          const deck = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, 0.05), mat(border));
          deck.position.z = z; g.add(deck);
        });
        const cargo = new THREE.Mesh(new THREE.BoxGeometry(sx * 0.8, sy * 0.8, 0.55), mat(0xd6a15a));
        cargo.position.z = 0.15 + 0.3; g.add(cargo);
        const lb = label(sh.name, sh.border || '#38bdf8', 0.32); st.envLabels.push(lb);
        lb.position.z = SHELF_H + 0.4; g.add(lb);
        st.env.add(g);
      });
      // 工位 (地面标识 + 停靠方向)
      (meta.stations || []).forEach(s => {
        const c = new THREE.Color(s.color || '#6366f1');
        const pad = new THREE.Mesh(new THREE.RingGeometry(0.28, 0.4, 40), new THREE.MeshBasicMaterial({ color: c, side: THREE.DoubleSide }));
        pad.position.set(s.x, s.y, 0.005); st.env.add(pad);
        const a = new THREE.ArrowHelper(new THREE.Vector3(Math.cos(s.dock_yaw || 0), Math.sin(s.dock_yaw || 0), 0), new THREE.Vector3(s.x, s.y, 0.02), 0.7, c.getHex(), 0.2, 0.14);
        st.env.add(a);
        const lb = label(s.name, s.color || '#6366f1', 0.22); lb.position.set(s.x, s.y, 0.6); st.env.add(lb); st.envLabels.push(lb);
      });
      // 拓扑路网 + 地面二维码
      if (topo) {
        const pos = [];
        (topo.edges || []).forEach(e => { pos.push(e.p1.x, e.p1.y, 0.01, e.p2.x, e.p2.y, 0.01); });
        const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
        st.env.add(new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ color: 0x3b82f6, transparent: true, opacity: 0.6 })));
        Object.values(topo.nodes || {}).forEach(n => {
          const q = new THREE.Mesh(new THREE.PlaneGeometry(0.12, 0.12), new THREE.MeshBasicMaterial({ color: 0x111827 }));
          q.position.set(n.x, n.y, 0.006); st.env.add(q);
        });
      }
    }

    // ------------------------------------------------------------------ 每帧
    function update(c) {
      const THREE = T();
      if (!THREE || !c.scene || !c.robotGroup) return;
      const tel = c.telemetry || {};
      const spec = tel.robot_spec;
      if (spec && spec.chassis) {
        const sig = JSON.stringify([spec.model_file, spec.active_chassis, spec.model_rev, (spec.wheels || []).map(w => [w.name, w.x, w.y, w.kind]), (spec.lidars_full || []).length]);
        if (sig !== st.specSig) {
          st.specSig = sig;
          buildRobot(c.robotGroup, spec);
          Object.values(st.lidarClouds).forEach(lc => c.scene.add(lc.pts));
        }
      }
      const meta = tel.scenario_metadata;
      if (meta && tel.topo_graph) {
        const es = meta.id + ':' + (meta.walls || []).length;
        if (es !== st.envSig) { st.envSig = es; buildEnv(c.scene, meta, tel.topo_graph); }
      }
      if (st.env) st.env.visible = st.showEnv;
      const showRobotLabels = st.showLabels && (c.cameraMode === 'inspect' || st.labelsEverywhere);
      st.labels.forEach(l => { l.visible = showRobotLabels; });
      st.envLabels.forEach(l => { l.visible = c.cameraMode !== 'inspect'; });
      st.fans.forEach(f => { f.visible = st.showFans; });

      // 关节驱动: 舵角 & 车轮转角
      const js = tel.joint_states;
      const now = performance.now();
      const fdt = Math.min(0.1, (now - (st.lastFrame || now)) / 1000);
      st.lastFrame = now;
      adaptResolution(c.renderer, now);
      if (js && js.names) {
        if (js !== st.lastJs) { st.lastJs = js; st.jsTime = now; }
        const age = Math.min(0.3, (now - st.jsTime) / 1000);
        const idx = {}, vel = {};
        js.names.forEach((n, i) => { idx[n] = js.positions[i]; vel[n] = (js.velocities || [])[i] || 0; });
        const k = 1 - Math.exp(-fdt / 0.06);
        Object.values(st.joints).forEach(j => {
          const n = j.w.name;
          const sj = n + '_steer_joint';
          if (idx[sj] !== undefined) j.pivot.rotation.z += (idx[sj] - j.pivot.rotation.z) * k;   // 舵角平滑
          const dj = idx[n + '_drive_joint'] !== undefined ? n + '_drive_joint' : n + '_joint';
          if (idx[dj] !== undefined) j.spin.rotation.y = idx[dj] + vel[dj] * age;             // 轮转角按转速外推
        });
      }

      // 每个物理激光的点云 (真实安装高度)
      const scans = tel.lidar_scans;
      let used = false;
      if (scans) {
        Object.keys(st.lidarClouds).forEach(name => {
          const lc = st.lidarClouds[name], s = scans[name];
          if (!s) { lc.pts.geometry.setDrawRange(0, 0); return; }
          used = true;
          if (lc.lastRef === s) return;          // 同一帧扫描不重复计算
          lc.lastRef = s;
          const l = lc.cfg, arr = lc.pts.geometry.attributes.position.array;
          const P = s.pose, cy = Math.cos(P.yaw), sy = Math.sin(P.yaw);
          if (s.points3d) {
            // 3D 点云 (机体系) → 世界系；高度着色: 蓝(地面)→青→绿→黄→红(屋顶)
            const b = s.points3d, col = lc.pts.geometry.attributes.color.array;
            let k3 = 0;
            for (let i = 0; i + 2 < b.length && k3 < lc.max; i += 3) {
              const bx = b[i], by = b[i + 1], bz = b[i + 2];
              arr[k3 * 3] = P.x + cy * bx - sy * by; arr[k3 * 3 + 1] = P.y + sy * bx + cy * by; arr[k3 * 3 + 2] = bz;
              const h = Math.max(0, Math.min(1, bz / 6.0));
              col[k3 * 3] = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * h - 3)));
              col[k3 * 3 + 1] = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * h - 2)));
              col[k3 * 3 + 2] = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * h - 1)));
              k3++;
            }
            lc.pts.geometry.setDrawRange(0, k3);
            lc.pts.geometry.attributes.position.needsUpdate = true;
            lc.pts.geometry.attributes.color.needsUpdate = true;
            lc.pts.visible = true;
            return;
          }
          const ox = P.x + cy * l.x - sy * l.y, oy = P.y + sy * l.x + cy * l.y;
          const sign = l.inverted ? -1 : 1;
          let k = 0;
          for (let i = 0; i < s.ranges.length && k < lc.max; i++) {
            const r = s.ranges[i];
            if (r <= 0) continue;
            const a = P.yaw + (l.yaw || 0) + sign * (s.angle_min + i * s.angle_inc);
            arr[k * 3] = ox + r * Math.cos(a); arr[k * 3 + 1] = oy + r * Math.sin(a); arr[k * 3 + 2] = l.z;
            k++;
          }
          lc.pts.geometry.setDrawRange(0, k);
          lc.pts.geometry.attributes.position.needsUpdate = true;
          lc.pts.visible = true;
        });
      }
      if (c.pointCloud) c.pointCloud.visible = !used;   // 无分激光数据时回退到融合扫描

      // 构成视角: 围绕车体近距离环视
      if (c.cameraMode === 'inspect' && c.camera && c.controls) {
        const p = c.pose;
        if (!st._inspectInit || st._inspectInit !== st.specSig) {
          st._inspectInit = st.specSig;
          const ch = (spec && spec.chassis) || {};
          const k = Math.max(1.0, (ch.length_m || 1.5) * 0.75);
          const cy = Math.cos(p.yaw), sy = Math.sin(p.yaw);
          const lx = 1.6 * k, ly = -1.9 * k;          // 车体右前方斜上视角
          c.camera.position.set(p.x + cy * lx - sy * ly, p.y + sy * lx + cy * ly, (ch.height_m || 0.5) * 0.6 + 1.6 * k);
        }
        const hc = ((spec && spec.chassis && spec.chassis.head_offset_m) || 0.6) - ((spec && spec.chassis && spec.chassis.tail_offset_m) || 0.6);
        c.controls.target.set(p.x + hc / 2 * Math.cos(p.yaw), p.y + hc / 2 * Math.sin(p.yaw), Math.min(0.8, ((spec && spec.chassis && spec.chassis.height_m) || 0.5) * 0.35));
      } else { st._inspectInit = null; }

      updateOverlay(tel);
    }

    // ------------------------------------------------------------------ 自适应渲染分辨率 (保持帧率)
    function adaptResolution(r, now) {
      if (!r) return;
      st.frames = (st.frames || 0) + 1;
      if (!st.fpsT0) { st.fpsT0 = now; st.pr = r.getPixelRatio(); return; }
      const el = now - st.fpsT0;
      if (el < 2000) return;
      st.fps = st.frames * 1000 / el;
      st.frames = 0; st.fpsT0 = now;
      const cap = Math.min(1.5, window.devicePixelRatio || 1);
      let pr = st.pr;
      if (st.fps < 28 && pr > 0.6) pr = Math.max(0.6, pr - 0.2);
      else if (st.fps > 55 && pr < cap) pr = Math.min(cap, pr + 0.1);
      if (Math.abs(pr - st.pr) > 1e-3) { st.pr = pr; r.setPixelRatio(pr); }
    }

    // ------------------------------------------------------------------ 信息面板
    function updateOverlay(tel) {
      const now = performance.now();
      if (now - st.lastOverlay < 500) return;
      st.lastOverlay = now;
      const el = document.getElementById('amr3d-overlay');
      if (!el) return;
      const sp = tel.robot_spec, ss = tel.sim_status || {}, n2 = tel.nav2 || {}, rg = tel.ros_graph || {};
      const eb = document.getElementById('engine-badge-text');
      if (eb && ss.backend) eb.textContent = `SimCore/${ss.backend} 引擎 · ${Math.round(1 / (ss.dt || 0.01))} Hz 固定步长 · RTF ${ss.rtf}`;
      if (!sp) { el.innerHTML = '<b>等待 /robot_spec …</b>'; return; }
      const ch = sp.chassis || {};
      const tname = { single_steer: '单舵轮', dual_steer: '双舵轮', diff_drive: '差速', multi_steer: '多舵轮' };
      const wl = (sp.wheels || []).filter(w => w.kind !== 'caster').map(w => `${({ steer: '舵轮', drive: '驱动轮', fixed: '承重轮' }[w.kind] || w.kind)} ${w.name}${w.source === 'inferred' ? '*' : ''}`).join('、');
      const ll = (sp.lidars_full || sp.lidars || []).map(l => `${l.name}(${l.vendor_model || l.model || ''}, z=${(+l.z).toFixed(2)}${l.inverted ? ', 倒装' : ''})`).join('、');
      const nodes = (rg.nodes || []);
      el.innerHTML =
        `<div class="font-bold text-slate-800 mb-1">cmodel 机器人构成</div>` +
        `<div>模型文件: <b>${sp.model_file || '-'}</b></div>` +
        `<div>车型: <b>${tname[sp.active_chassis] || sp.active_chassis}</b>${sp.active_chassis !== sp.native_chassis ? ` <span class="text-amber-600">(预设对比, cmodel 原车型 ${tname[sp.native_chassis] || sp.native_chassis})</span>` : ' <span class="text-emerald-600">(cmodel 原车型)</span>'}</div>` +
        `<div>车体: ${(+ch.length_m).toFixed(3)}×${(+ch.width_m).toFixed(3)}×${(+ch.height_m).toFixed(2)} m · 车头 ${ch.head_offset_m} / 车尾 ${ch.tail_offset_m} m</div>` +
        `<div>轮组: ${wl}</div><div>激光: ${ll}</div>` +
        `<div>限速: ${ch.max_speed_mps} m/s · 加速 ${ch.max_accel_mps2} · 减速 ${ch.max_decel_mps2} m/s²</div>` +
        `<div class="mt-1 font-bold text-slate-800">仿真引擎</div>` +
        `<div>3D 帧率 ${st.fps ? st.fps.toFixed(0) : '-'} fps · 渲染分辨率 ×${(st.pr || 1).toFixed(2)}</div>` +
        `<div>SimCore/${ss.backend || '-'} · 步长 ${((ss.dt || 0) * 1000).toFixed(0)} ms · RTF ${ss.rtf} · 物理 ${ss.step_ms} ms · 激光 ${ss.lidar_ms} ms · 碰撞 ${ss.collisions}</div>` +
        `<div>里程计漂移 ${ss.odom_drift_m} m / ${ss.odom_drift_deg}° (真值定位已修正)</div>` +
        `<div class="mt-1 font-bold text-slate-800">ROS 2 (${nodes.length} 个节点, ${rg.topic_count || '-'} 个话题)</div>` +
        `<div>Nav2: ${n2.server_ready ? '<span class="text-emerald-600">就绪</span>' : (n2.process ? '<span class="text-amber-600">启动中</span>' : '未运行')} · 参数 ${n2.chassis || '-'} · 定位 ${n2.localization || '-'}</div>` +
        `<div class="text-slate-500 leading-snug">${nodes.join('  ')}</div>` +
        `<div class="mt-1 flex gap-1 flex-wrap">` +
        ['showLabels:部件标注', 'labelsEverywhere:标注常显', 'showFans:激光视场', 'showEnv:仓储环境'].map(k => {
          const [key, name] = k.split(':');
          return `<button onclick="AMR3D.toggle('${key}')" class="px-1.5 py-0.5 rounded border text-[10px] ${st[key] ? 'bg-blue-50 border-blue-300 text-blue-700' : 'bg-white border-slate-200 text-slate-500'}">${name}</button>`;
        }).join('') + `</div><div class="text-[10px] text-slate-400 mt-1">* 推断部件 (cmodel 未建模)</div>`;
    }

    // ------------------------------------------------------------------ 2D 车体 (与 cmodel 一致)
    function draw2DRobot(ctx, tel, scale, dpr) {
      const sp = tel && tel.robot_spec;
      if (!sp || !sp.footprint) return false;
      const P = (x, y) => [x * scale, -y * scale];
      // 轮廓 (运动中心为原点)
      ctx.fillStyle = 'rgba(245, 158, 11, 0.35)';
      ctx.strokeStyle = '#92400e';
      ctx.lineWidth = 2 * dpr;
      ctx.beginPath();
      sp.footprint.forEach((p, i) => { const q = P(p[0], p[1]); if (i) ctx.lineTo(q[0], q[1]); else ctx.moveTo(q[0], q[1]); });
      ctx.closePath(); ctx.fill(); ctx.stroke();
      // 防撞触边 (碰撞条): 压下为红色
      const bp = tel.bumpers && tel.bumpers.strips;
      if (bp) bp.forEach(b => {
        ctx.fillStyle = b.pressed ? '#ef4444' : '#334155';
        ctx.beginPath();
        b.polygon.forEach((p, i) => { const q = P(p[0], p[1]); if (i) ctx.lineTo(q[0], q[1]); else ctx.moveTo(q[0], q[1]); });
        ctx.closePath(); ctx.fill();
      });
      // 光电: 触发距离光束，检测到为橙色实线 (到检测点)
      (tel.photoelectric || []).forEach(pe => {
        const m = pe.mount, q = P(m.x, m.y);
        const d = pe.detected && pe.distance_m != null ? pe.distance_m : pe.trigger_m;
        const e = P(m.x + Math.cos(m.yaw) * d, m.y + Math.sin(m.yaw) * d);
        ctx.strokeStyle = pe.detected ? '#f97316' : 'rgba(16,185,129,0.55)';
        ctx.lineWidth = (pe.detected ? 2.5 : 1.2) * dpr;
        ctx.setLineDash(pe.detected ? [] : [3 * dpr, 3 * dpr]);
        ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(e[0], e[1]); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = pe.detected ? '#f97316' : '#10b981';
        ctx.beginPath(); ctx.arc(q[0], q[1], 2.5 * dpr, 0, Math.PI * 2); ctx.fill();
      });
      // 轮子
      const idx = {};
      const js = tel.joint_states;
      if (js && js.names) js.names.forEach((n, i) => { idx[n] = js.positions[i]; });
      (sp.wheels || []).forEach(w => {
        const q = P(w.x, w.y), r = (w.radius_m || 0.1) * scale, wd = Math.max(3 * dpr, (w.width_m || 0.05) * scale);
        ctx.save(); ctx.translate(q[0], q[1]);
        if (w.kind === 'caster') {
          ctx.fillStyle = '#94a3b8'; ctx.beginPath(); ctx.arc(0, 0, r * 0.8, 0, Math.PI * 2); ctx.fill();
        } else {
          const a = idx[w.name + '_steer_joint'] || 0;
          ctx.rotate(-a);
          ctx.fillStyle = w.kind === 'steer' ? '#dc2626' : (w.kind === 'drive' ? '#111827' : '#6b7280');
          ctx.fillRect(-r, -wd / 2, 2 * r, wd);
          if (w.kind === 'steer') {
            ctx.strokeStyle = '#dc2626'; ctx.lineWidth = 1.5 * dpr;
            ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(r * 2.2, 0); ctx.stroke();
          }
        }
        ctx.restore();
      });
      // 激光
      (sp.lidars_full || sp.lidars || []).forEach((l, i) => {
        const q = P(l.x, l.y);
        const col = hex(LIDAR_COLORS[i % LIDAR_COLORS.length]);
        ctx.fillStyle = col; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.2 * dpr;
        ctx.beginPath(); ctx.arc(q[0], q[1], 4.5 * dpr, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        ctx.strokeStyle = col; ctx.beginPath(); ctx.moveTo(q[0], q[1]);
        ctx.lineTo(q[0] + Math.cos(l.yaw || 0) * 14 * dpr, q[1] - Math.sin(l.yaw || 0) * 14 * dpr); ctx.stroke();
      });
      // 运动中心 + 航向
      ctx.fillStyle = '#111827'; ctx.beginPath(); ctx.arc(0, 0, 3.5 * dpr, 0, Math.PI * 2); ctx.fill();
      const hx = (sp.chassis && sp.chassis.head_offset_m || 0.6) * scale;
      ctx.fillStyle = '#ef4444'; ctx.beginPath();
      ctx.moveTo(hx + 12 * dpr, 0); ctx.lineTo(hx - 2 * dpr, -8 * dpr); ctx.lineTo(hx - 2 * dpr, 8 * dpr); ctx.closePath(); ctx.fill();
      return true;
    }

    return {
      update: update,
      draw2DRobot: draw2DRobot,
      buildRobot: buildRobot,
      buildEnv: buildEnv,
      state: st,
      toggle: function (k) { st[k] = !st[k]; st.lastOverlay = 0; }
    };
  }
  // 默认实例 (旧调度台) + create() 生成独立实例 (新工作台/模型预览可同时存在)
  window.AMR3D = Object.assign(makeInstance(), { create: makeInstance });
})();
