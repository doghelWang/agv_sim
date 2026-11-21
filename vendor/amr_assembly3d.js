/* ============================================================================
 * AMRAssembly: three.js builder for the simulator's RobotAssembly (GET /api/v1/model/assembly, also robot_spec.assembly)
 *   build(group, asm)            -> { joints: {name: {obj, type, axis, pos0, quat0, mimic}}, links: {name: Object3D} }
 *   apply(joints, js, age)       -> drives joints from joint_states {names, positions, velocities} (prismatic / revolute / continuous)
 *   qrTexture(code)              -> CanvasTexture of a floor QR code ({n, bits:"0101..", size})
 *   prop(o)                      -> Group for warehouse props/cargo (rack / cart / tote / conveyor / charger / pillar / rack_upright / rack_beam / barrier)
 * Conventions follow URDF: z up, cylinder axis = local z, rpy = fixed-axis roll/pitch/yaw (three.js Euler order 'ZYX').
 * ========================================================================== */
(function () {
  const T = () => window.THREE;
  const matCache = new Map();
  function mat(color, opacity) {
    const THREE = T(), k = color + '|' + (opacity ?? 1);
    if (!matCache.has(k)) {
      const m = new THREE.MeshLambertMaterial({ color: new THREE.Color(color) });
      if (opacity !== undefined && opacity < 1) { m.transparent = true; m.opacity = opacity; m.depthWrite = false; }
      matCache.set(k, m);
    }
    return matCache.get(k);
  }
  function geom(v) {
    const THREE = T();
    if (v.shape === 'cylinder') { const g = new THREE.CylinderGeometry(v.radius, v.radius, v.length, 24); g.rotateX(Math.PI / 2); return g; }
    if (v.shape === 'sphere') return new THREE.SphereGeometry(v.radius, 16, 12);
    const s = v.size || [0.1, 0.1, 0.1];
    return new THREE.BoxGeometry(s[0], s[1], s[2]);
  }
  function setRpy(o, rpy) { if (rpy) o.rotation.set(rpy[0] || 0, rpy[1] || 0, rpy[2] || 0, 'ZYX'); }

  function build(group, asm) {
    const THREE = T();
    const out = { joints: {}, links: {} };
    const byParent = {};
    (asm.links || []).forEach(l => { (byParent[l.parent || ''] = byParent[l.parent || ''] || []).push(l); });
    const addVisuals = (obj, l) => (l.visuals || []).forEach(v => {
      const ok = v.shape === 'cylinder' ? v.radius > 1e-4 && v.length > 1e-4 : v.shape === 'sphere' ? v.radius > 1e-4 : (v.size || []).every(x => x > 1e-4);
      if (!ok) return;
      const m = new THREE.Mesh(geom(v), mat(v.color || '#888888', v.opacity));
      m.position.set(...(v.xyz || [0, 0, 0])); setRpy(m, v.rpy);
      m.userData.role = v.role;
      if (v.role === 'upper') m.userData.xray = true;
      obj.add(m);
    });
    const rec = (parentObj, parentName) => (byParent[parentName] || []).forEach(l => {
      const j = l.joint || {};
      const frame = new THREE.Group(); frame.name = l.name + '_frame';
      frame.position.set(...(j.xyz || [0, 0, 0])); setRpy(frame, j.rpy);
      const motion = new THREE.Group(); motion.name = l.name;
      frame.add(motion); parentObj.add(frame);
      addVisuals(motion, l);
      out.links[l.name] = motion;
      if (j.type && j.type !== 'fixed') {
        out.joints[j.name] = { obj: motion, type: j.type, axis: new THREE.Vector3(...(j.axis || [0, 0, 1])).normalize(), lower: j.lower, upper: j.upper, mimic: j.mimic };
      }
      rec(motion, l.name);
    });
    const base = (asm.links || []).find(l => l.name === 'base_link');
    const root = new THREE.Group(); root.name = 'assembly';
    if (base) { addVisuals(root, base); out.links.base_link = root; }
    rec(root, 'base_link');
    group.add(root);
    out.root = root;
    return out;
  }

  const _q = () => new (T().Quaternion)();
  function setJoint(j, v) {
    if (j.type === 'prismatic') { j.obj.position.copy(j.axis).multiplyScalar(v); }
    else { j.obj.quaternion.copy(_q().setFromAxisAngle(j.axis, v)); }
  }
  /** js: {names, positions, velocities}; age: seconds since the sample (continuous joints are extrapolated) */
  function apply(joints, js, age, smooth) {
    if (!js || !js.names) return;
    const pos = {}, vel = {};
    js.names.forEach((n, i) => { pos[n] = js.positions[i]; vel[n] = (js.velocities || [])[i] || 0; });
    Object.entries(joints).forEach(([n, j]) => {
      let v = pos[n];
      if (v === undefined && j.mimic && pos[j.mimic.joint] !== undefined) v = pos[j.mimic.joint] * (j.mimic.multiplier ?? 1);
      if (v === undefined) return;
      if (j.type === 'continuous') v += vel[n] * (age || 0);
      if (smooth && j.type !== 'continuous') { j._v = j._v === undefined ? v : j._v + (v - j._v) * smooth; v = j._v; }
      setJoint(j, v);
    });
  }

  function qrTexture(code) {
    const THREE = T(), n = code.n, px = 4, c = document.createElement('canvas');
    c.width = c.height = n * px;
    const g = c.getContext('2d');
    g.fillStyle = '#f5f5f4'; g.fillRect(0, 0, c.width, c.height);
    g.fillStyle = '#0a0a0b';
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) if (code.bits[j * n + i] === '1') g.fillRect(i * px, j * px, px, px);
    const t = new THREE.CanvasTexture(c);
    t.magFilter = THREE.NearestFilter; t.minFilter = THREE.LinearMipmapLinearFilter;
    return t;
  }

  // ------------------------------------------------------------------ warehouse props / cargo (same decomposition as sim world.rs)
  const C = { wood: 0x9e7340, woodL: 0xcc9e66, card: 0xc29461, blue: 0x2652a0, orange: 0xeb731a, steel: 0x8c949e, dark: 0x292e38, yellow: 0xfacc14, concrete: 0xb3b3ad, tote: 0x2666cc, green: 0x1f9e52, white: 0xebedf2 };
  function box(g, w, h, z, x, y, zc, color, rot) { const THREE = T(); const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, z), mat('#' + color.toString(16).padStart(6, '0'))); m.position.set(x, y, zc); if (rot) m.rotation.z = rot; g.add(m); return m; }
  function cylY(g, r, len, x, y, z, color) { const THREE = T(); const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 16), mat('#' + color.toString(16).padStart(6, '0'))); m.position.set(x, y, z); g.add(m); return m; }
  function cylZ(g, r, len, x, y, z, color) { const m = cylY(g, r, len, x, y, z, color); m.rotation.x = Math.PI / 2; return m; }
  function prop(o) {
    const THREE = T(), g = new THREE.Group();
    const w = o.w || 0.8, h = o.h || 0.8, z = o.z || 1.0, t = o.type, k = o.kind || t;
    if (k === 'rack') {
      const lh = o.leg_h || 0.2, lw = o.leg_w || 0.05;
      [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([sx, sy]) => box(g, lw, lw, z, sx * (w / 2 - lw / 2), sy * (h / 2 - lw / 2), z / 2, C.blue));
      box(g, w, h, 0.03, 0, 0, lh + 0.015, C.orange);
      [0.5, 1.0].forEach(f => box(g, w, h, 0.03, 0, 0, lh + (z - lh) * f - 0.015, C.orange));
      if (z - lh > 0.6) box(g, w * 0.55, h * 0.6, 0.23, 0, 0, lh + (z - lh) * 0.5 + 0.13, C.tote);
      // QR label under the deck (read by the upward code reader)
      const ql = new THREE.Mesh(new THREE.PlaneGeometry(0.08, 0.08), new THREE.MeshBasicMaterial({ color: 0x111111, side: THREE.DoubleSide })); ql.position.z = lh - 0.001; g.add(ql);
    } else if (k === 'cart') {
      const deck = o.deck_z || 0.28, dl = o.drawbar_m || 0.35, hz = o.hitch_z || 0.3;
      box(g, w, h, 0.04, 0, 0, deck, C.steel);
      [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([sx, sy]) => cylY(g, 0.06, 0.05, sx * (w / 2 - 0.1), sy * (h / 2 - 0.08), 0.06, C.dark));
      [-1, 1].forEach(sy => box(g, 0.04, 0.04, z - deck, -w / 2 + 0.03, sy * (h / 2 - 0.03), (deck + z) / 2, C.steel));
      box(g, 0.04, h, 0.04, -w / 2 + 0.03, 0, z - 0.02, C.steel); box(g, w, h, 0.03, 0, 0, z - 0.02, C.steel);
      box(g, dl, 0.05, 0.03, w / 2 + dl / 2, 0, hz, C.yellow);
      cylZ(g, 0.045, 0.03, w / 2 + dl, 0, hz, C.dark);
      box(g, w * 0.7, h * 0.75, 0.4, 0.05, 0, deck + 0.22, C.card);
    } else if (t === 'conveyor') {
      const bed = 0.06;
      [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([sx, sy]) => box(g, 0.05, 0.05, z - bed, sx * (w / 2 - 0.04), sy * (h / 2 - 0.04), (z - bed) / 2, C.dark));
      [1, -1].forEach(sy => box(g, w, 0.04, bed, 0, sy * (h / 2 - 0.02), z - bed / 2 - 0.01, C.steel));
      const n = Math.max(2, Math.floor((h - 0.08) / 0.09));
      for (let i = 0; i < n; i++) { const m = cylY(g, 0.03, w - 0.06, 0, -h / 2 + 0.085 + i * 0.09, z - 0.03, 0xcfd3db); m.rotation.z = Math.PI / 2; }
      box(g, w - 0.08, h - 0.08, 0.02, 0, 0, (z - bed) * 0.3, C.dark);
    } else if (t === 'charger') {
      box(g, w, h, 0.08, 0, 0, 0.04, C.dark); box(g, 0.14, h, z, -w / 2 + 0.07, 0, z / 2, C.white);
      box(g, 0.01, h * 0.5, 0.05, -w / 2 + 0.145, 0, 0.12, 0xd9a626); box(g, 0.01, h * 0.6, 0.06, -w / 2 + 0.145, 0, z - 0.12, C.green);
    } else if (t === 'pillar') {
      box(g, w, h, z - 1, 0, 0, z / 2 + 0.5, C.concrete);
      for (let i = 0; i < 4; i++) box(g, w + 0.02, h + 0.02, 0.125, 0, 0, 0.0625 + i * 0.25, C.yellow), box(g, w + 0.021, h + 0.021, 0.125, 0, 0, 0.1875 + i * 0.25, 0x141414);
    } else if (t === 'rack_upright') { box(g, w, h, z, 0, 0, z / 2, C.blue); }
    else if (t === 'rack_beam') { box(g, w, h, z, 0, 0, z / 2, C.orange); }
    else if (t === 'barrier') {
      [-1, 0, 1].forEach(f => box(g, 0.06, 0.06, z, f * (w / 2 - 0.03), 0, z / 2, C.yellow));
      [0.35, 0.8].forEach(zz => box(g, w, 0.05, 0.08, 0, 0, zz, C.yellow));
    } else if (k === 'tote') { box(g, w, h, z, 0, 0, z / 2, C.tote); box(g, w + 0.01, h + 0.01, 0.03, 0, 0, z - 0.015, 0x1d4ea3); }
    else return null;
    g.position.set(o.x, o.y, o.z0 || 0); g.rotation.z = o.yaw || 0;
    return g;
  }

  window.AMRAssembly = { build, apply, qrTexture, prop };
})();
