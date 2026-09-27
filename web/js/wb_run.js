// ============================================================================
// 3-2 仿真运行与环境: 任务流下发 · 暂停/恢复 · 重置 · 3D/俯瞰/跟随 · HUD · 实时事件流 · 轮组状态
//                     暂停后: 扰动元素注入库 (托盘/货架/纸箱/人员，多种落点)；手动控制抽屉
// ============================================================================
import { $, $$, esc, fmt, toast, guard, icons, TAG_STYLE, http } from './core.js';
import { W, iget, ipost, idel, needLock } from './wbcore.js';
import { Viewer3D, nodeName } from './viz3d.js';
import { MapCanvas } from './viz2d.js';
import { loadFlows } from './wb_taskflow.js';

const ZONE = { clear: ['正常巡航', 'bg-emerald-50 text-emerald-700 border-emerald-200'], slow: ['减速避让', 'bg-amber-50 text-amber-700 border-amber-200'],
  stop: ['停车等待', 'bg-rose-50 text-rose-700 border-rose-200'] };

export function tab32(el) {
  el.innerHTML = `
  <div class="h-12 bg-white border-b border-slate-200 px-4 flex items-center justify-between shrink-0 text-xs gap-3">
    <div class="flex items-center gap-2 min-w-0">
      <span class="font-semibold text-slate-600 whitespace-nowrap">任务流:</span>
      <select data-flow class="inp !w-72 !py-1.5"></select>
      <button class="btn-primary" data-run><i data-lucide="send" class="w-3.5 h-3.5"></i>下发任务</button>
      <button class="btn border bg-amber-50 border-amber-200 text-amber-700 hover:bg-amber-100" data-pause><i data-lucide="pause" class="w-3.5 h-3.5"></i><span>暂停仿真</span></button>
      <button class="btn-ghost" data-reset><i data-lucide="rotate-ccw" class="w-3.5 h-3.5"></i>重置环境</button>
      <button class="btn-ghost" data-stopflow title="终止当前任务流"><i data-lucide="square" class="w-3.5 h-3.5"></i></button>
      <button class="btn-ghost" data-manual><i data-lucide="gamepad-2" class="w-3.5 h-3.5"></i>手动</button>
    </div>
    <div class="flex items-center gap-3 shrink-0">
      <span data-state></span>
      <div class="seg" data-views><button data-view="free" class="on">自由3D</button><button data-view="top">俯瞰2D</button><button data-view="follow">跟随视角</button><button data-view="inspect">车体检视</button></div>
      <label class="flex items-center gap-1 text-slate-600" title="执行进程构建的 SLAM 占据栅格 + 定位结果轮廓 (橙色)"><input type="checkbox" data-slam checked>SLAM 地图</label>
      <span class="text-slate-500">点云:</span><select data-color class="inp !w-32 !py-1.5"><option value="height">高度 (Z-Height)</option><option value="distance">探测距离</option></select>
    </div>
  </div>
  <div class="flex-1 flex min-h-0">
    <div class="flex-1 relative min-w-0" data-vp>
      <div data-banner class="hidden absolute top-4 left-1/2 -translate-x-1/2 z-20 px-4 py-2 rounded-lg bg-amber-500 text-white text-xs font-bold shadow-lg flex items-center gap-2">
        <i data-lucide="pause-circle" class="w-4 h-4"></i>仿真已暂停：请在右侧面板选取并注入特定元素</div>
      <div class="absolute top-4 left-4 z-10 w-72 bg-white/95 border border-slate-200 rounded-xl shadow-lg p-4 text-xs" data-hud></div>
      <div data-drawer class="hidden absolute bottom-4 left-4 z-20 w-80 bg-white border border-slate-200 rounded-xl shadow-xl p-4 text-xs"></div>
    </div>
    <div class="w-80 bg-white border-l border-slate-200 flex flex-col shrink-0">
      <div data-pane-run class="flex-1 flex flex-col min-h-0">
        <div class="px-4 py-3 border-b border-slate-100 flex items-center justify-between"><span class="panel-title"><i data-lucide="radio" class="w-4 h-4 text-blue-600"></i>实时仿真事件流 (Live Events)</span>
          <button class="text-[11px] text-slate-400 hover:text-slate-700" data-clear>清屏</button></div>
        <div data-tags class="px-3 py-2 border-b border-slate-100 flex flex-wrap gap-1"></div>
        <div data-evs class="flex-1 overflow-y-auto p-3 space-y-2"></div>
        <div class="border-t border-slate-200 p-3" data-wheels></div>
      </div>
      <div data-pane-inj class="hidden flex-1 flex flex-col min-h-0 bg-slate-50/40"></div>
    </div>
  </div>`;
  const viewer = new Viewer3D($('[data-vp]', el));
  let paused = false, evFilter = null, clearedAt = 0, catalog = null, sel = { type: 'pallet', placement: 'ahead_2.5', walk: false }, pickMap = null;

  // ---------------- 任务流选择
  async function fillFlows() {
    await loadFlows();
    const s = $('[data-flow]', el);
    s.innerHTML = W.flows.map(f => `<option value="${esc(f.id)}">${esc(f.id)}: ${esc(f.name)}</option>`).join('') || '<option value="">(先在 3-1 配置任务流)</option>';
    if (W.pendingFlow) { s.value = W.pendingFlow; W.pendingFlow = null; } else if (W.flowSel) s.value = W.flowSel;
  }
  $('[data-run]', el).onclick = guard(async () => {
    if (!needLock()) return;
    const f = W.flows.find(x => x.id === $('[data-flow]', el).value);
    if (!f) throw new Error('请选择任务流');
    await ipost('/api/v2/taskflow/run', { flow: f });
    toast(`任务流 ${f.id} 已下发`, 'ok');
  });
  $('[data-stopflow]', el).onclick = guard(async () => { if (!needLock()) return; await ipost('/api/v2/taskflow/stop'); });
  $('[data-pause]', el).onclick = guard(async () => { if (!needLock()) return; await ipost('/api/v2/pause', { paused: !paused }); });
  $('[data-reset]', el).onclick = guard(async () => { if (!needLock()) return; await ipost('/api/v2/reset'); toast('环境已重置: 车辆回到 P0，注入元素已清除', 'ok'); });
  $$('[data-view]', el).forEach(b => b.onclick = () => { viewer.setMode(b.dataset.view); $$('[data-view]', el).forEach(x => x.classList.toggle('on', x === b)); });
  $('[data-color]', el).onchange = (e) => { viewer.colorMode = e.target.value; };
  $('[data-slam]', el).onchange = (e) => { viewer.setSlamVisible(e.target.checked); if (e.target.checked) slamRev = null; };
  // SLAM 地图: 地图版本变化时拉取 (下采样 2 倍，约 5 cm)
  let slamRev = null, slamBusy = false, slamT = 0;
  function pollSlam(t) {
    const L = t.localization; if (!L || !$('[data-slam]', el).checked || slamBusy) return;
    if (L.map_rev === slamRev || Date.now() - slamT < 3000) return;
    slamBusy = true; slamT = Date.now();
    iget('/api/v2/slam/map?step=2').then(m => { slamRev = L.map_rev; viewer.setSlamMap(m); }).catch(() => { }).finally(() => { slamBusy = false; });
  }
  $('[data-clear]', el).onclick = () => { clearedAt = W.evAfter; renderEvents(); };
  $('[data-manual]', el).onclick = () => { const d = $('[data-drawer]', el); d.classList.toggle('hidden'); if (!d.classList.contains('hidden')) renderManual(); };

  // ---------------- HUD / 状态
  function segHTML(t) {
    const ns = t.next_segment, path = t.plan_path || [];
    if (!ns || path.length < 2 || ['ARRIVED', 'CANCELED', 'FAILED', 'NO_PATH', 'IDLE'].includes(t.nav_status)) return '';
    const nm = (l, p) => esc(nodeName(l) || `(${fmt(p.x, 1)}, ${fmt(p.y, 1)})`);
    const from = ns.from_label ? nm(ns.from_label) : '当前位置';
    const labels = t.path_labels || [];
    const rest = path.slice(ns.index + 1).map((p, k) => nm(labels[ns.index + 1 + k], p));
    return `<div class="mt-2 mb-1 border border-orange-200 bg-orange-50/70 rounded px-2 py-1.5">
        <div class="flex justify-between"><span class="text-slate-500">下一路段:</span><span class="font-semibold text-orange-700 truncate ml-2">${from} → ${nm(ns.to_label, ns.to)} · ${fmt(ns.dist, 1)}m</span></div>
        ${ns.next ? `<div class="flex justify-between"><span class="text-slate-500">随后:</span><span class="text-slate-700 truncate ml-2">→ ${nm(ns.next.to_label, ns.next.to)} (${fmt(ns.next.dist, 1)}m)</span></div>` : ''}
        <div class="text-[11px] text-slate-500 truncate" title="${rest.join(' → ')}">剩余 ${ns.remaining_waypoints} 个航点${rest.length ? '：' + rest.slice(0, 4).join(' → ') + (rest.length > 4 ? ' …' : '') : ''}</div>
      </div>`;
  }
  function protHTML(t) {
    const pv = t.protection; if (!pv || !pv.config) return '';
    const f = (pv.config.fields || [])[pv.band || 0] || {};
    const lay = { field_front: '前向防护区', field_rear: '后向防护区', rotate: '转向防护区' }[pv.layer] || '';
    return `<div class="kv"><span>保护空间:</span><span>${esc(f.name || '')}档 前${f.front ?? '-'}m · ${pv.loaded ? '<b class="text-purple-600">带载</b>' : '空载'}${lay ? ' · <b class="text-rose-600">' + lay + '</b>' : ''}</span></div>`;
  }
  function locHTML(t) {
    const L = t.localization; if (!L || !L.mode) return '';
    const name = ({ slam: 'SLAM 建图+定位', localization: 'SLAM 定位', odom: '纯里程计', ground_truth: '真值' }[L.mode] || L.mode) + (L.engine === 'slam_toolbox' ? ' · slam_toolbox' : L.mode === 'slam' || L.mode === 'localization' ? ' · 内置' : '');
    const e = L.err_mm, cls = e == null ? '' : e > 20 ? 'text-rose-600' : e > 10 ? 'text-amber-600' : 'text-emerald-600';
    return `<div class="kv"><span title="${esc(name)}">定位误差:</span><span class="${cls}">${e == null ? '—' : fmt(e, 1) + ' mm / ' + fmt(L.err_yaw_deg, 2) + '°'} ${L.std_xy_mm == null ? '' : ' · σ ' + fmt(L.std_xy_mm, 1) + ' mm'}${L.accepted === false ? ' · <b class="text-amber-600">匹配未采纳</b>' : ''}</span></div>`;
  }
  function renderHUD(t) {
    const tf = t.taskflow || {}, steps = tf.steps || [], i = tf.step_index;
    const st = tf.status === 'running' && steps[i] ? `${String(i + 1).padStart(2, '0')}: ${{ move: '导航前往', lift: '顶升取货', drop: '下降放货', wait: '等待' }[steps[i].type]} ${steps[i].target || (steps[i].seconds ? steps[i].seconds + 's' : '')}`
      : ({ done: '任务流已完成', failed: '任务流失败', stopped: '任务流已终止' }[tf.status] || '待命');
    const sf = t.safety || {}, ob = sf.obstacle || {}, z = ZONE[ob.zone || 'clear'];
    const d = ob.front ?? t.scan_min_dist;
    let pts = 0; Object.values(t.lidar_scans || {}).forEach(s => { pts += s.n_raw || (s.ranges ? s.ranges.length : 0); });
    $('[data-hud]', el).innerHTML = `<div class="flex justify-between items-center mb-2"><span class="font-bold text-slate-800">实时遥测与任务工步</span><span class="px-2 py-0.5 rounded-full text-[11px] font-bold border ${z[1]}">${z[0]}</span></div>
      <div class="flex justify-between bg-blue-50/60 border border-blue-100 rounded px-2 py-1.5 mb-2"><span class="text-slate-500">当前工步:</span><span class="font-semibold text-blue-700 truncate ml-2">${esc(st)}</span></div>
      <div class="kv"><span>车辆位姿 (真值):</span><span>X: ${fmt(t.x)}m Y: ${fmt(t.y)}m θ: ${fmt((t.yaw || 0) * 57.2958, 1)}°</span></div>
      <div class="kv"><span>线速 / 角速度:</span><span class="text-blue-600">${fmt(Math.hypot(t.vx || 0, t.vy || 0))} m/s | ${fmt(t.wz)} rad/s</span></div>
      <div class="kv"><span>导航状态:</span><span>${esc(t.nav_status || '—')} · ${esc(t.planner_type || '')}</span></div>
      ${locHTML(t)}
      ${segHTML(t)}
      ${protHTML(t)}
      <div class="kv"><span>激光点云:</span><span class="text-emerald-600">${pts.toLocaleString()} pts</span></div>
      <div class="kv"><span>最近障碍物:</span><span class="${ob.zone === 'stop' ? 'text-rose-600' : ob.zone === 'slow' ? 'text-amber-600' : ''}">${d == null ? '> 量程 (安全)' : fmt(d) + ' m'}${ob.zone === 'clear' || !ob.zone ? ' (安全)' : ''}</span></div>
      ${tf.status === 'running' ? `<div class="h-1.5 bg-slate-100 rounded mt-2"><div class="h-1.5 bg-blue-600 rounded" style="width:${((i + (tf.step_status === 'done' ? 1 : .5)) / Math.max(1, steps.length) * 100).toFixed(0)}%"></div></div>` : ''}`;
    const running = tf.status === 'running';
    const tag = paused ? ['仿真已暂停', 'bg-amber-50 text-amber-700 border-amber-200', 'bg-amber-500'] : running ? ['仿真进行中', 'bg-emerald-50 text-emerald-700 border-emerald-200', 'bg-emerald-500 pulse-dot'] : ['待命中', 'bg-slate-100 text-slate-600 border-slate-200', 'bg-slate-400'];
    $('[data-state]', el).innerHTML = `<span class="px-2 py-0.5 rounded-full text-[11px] font-bold border ${tag[1]} inline-flex items-center gap-1"><span class="w-2 h-2 rounded-full ${tag[2]}"></span>${tag[0]}</span>`;
  }
  function setPaused(p) {
    if (p === paused) return;
    paused = p;
    const b = $('[data-pause]', el);
    b.className = p ? 'btn border bg-emerald-50 border-emerald-200 text-emerald-700 hover:bg-emerald-100' : 'btn border bg-amber-50 border-amber-200 text-amber-700 hover:bg-amber-100';
    b.innerHTML = p ? '<i data-lucide="play" class="w-3.5 h-3.5"></i><span>恢复仿真</span>' : '<i data-lucide="pause" class="w-3.5 h-3.5"></i><span>暂停仿真</span>';
    $('[data-banner]', el).classList.toggle('hidden', !p);
    $('[data-pane-run]', el).classList.toggle('hidden', p);
    $('[data-pane-inj]', el).classList.toggle('hidden', !p);
    if (p) renderInject();
    icons(el);
  }

  // ---------------- 轮组
  function renderWheels(t) {
    const spec = t.robot_spec || {}, js = t.joint_states || {};
    const idx = {}; (js.names || []).forEach((n, i) => { idx[n] = i; });
    const ws = (spec.wheels || []).filter(w => w.kind === 'steer' || w.kind === 'drive');
    const col = ['text-blue-700', 'text-purple-700', 'text-cyan-700', 'text-pink-700'];
    $('[data-wheels]', el).innerHTML = `<div class="text-xs font-bold text-slate-800 mb-2">${esc(({ single_steer: '单舵轮', dual_steer: '双舵轮', diff_drive: '差速双驱' })[spec.active_chassis] || spec.active_chassis || '')}轮组状态</div>
      <div class="grid grid-cols-2 gap-2">${ws.map((w, k) => {
        const dj = idx[w.name + '_drive_joint'] ?? idx[w.name + '_joint'], sj = idx[w.name + '_steer_joint'];
        const rpm = dj != null ? (js.velocities[dj] || 0) * 60 / (2 * Math.PI) : null;
        const steer = sj != null ? js.positions[sj] * 57.2958 : null;
        return `<div class="border border-slate-200 rounded-lg p-2 text-[11px]"><div class="font-bold ${col[k % 4]} mb-1 truncate">${esc(w.name)}</div>
          <div class="flex justify-between"><span class="text-slate-500">转速:</span><span class="font-mono">${rpm == null ? '—' : rpm.toFixed(0) + ' rpm'}</span></div>
          <div class="flex justify-between"><span class="text-slate-500">舵角:</span><span class="font-mono">${steer == null ? '—' : (steer >= 0 ? '+' : '') + steer.toFixed(1) + '°'}</span></div></div>`;
      }).join('') || '<div class="text-[11px] text-slate-400 col-span-2">无驱动轮数据</div>'}</div>`;
  }

  // ---------------- 事件流
  function renderEvents() {
    const evs = W.events.filter(e => e.id > clearedAt && (!evFilter || e.tag === evFilter)).slice(-120).reverse();
    $('[data-evs]', el).innerHTML = evs.map(e => `<div class="border border-slate-200 rounded-lg p-2.5 ${e.level === 'danger' ? 'bg-rose-50/40' : e.level === 'warning' ? 'bg-amber-50/30' : 'bg-white'}">
      <div class="flex justify-between items-center"><span class="tag ${TAG_STYLE[e.tag] || TAG_STYLE.SYS}">[${e.tag}]</span><span class="text-[10px] font-mono text-slate-400">${esc(e.time_str)}</span></div>
      <div class="text-xs font-semibold text-slate-800 mt-1">${esc((e.title || '').replace(/^\[(执行|仿真)进程\] /, ''))}</div>
      ${e.message && e.message !== e.title ? `<div class="text-[11px] text-slate-500 font-mono mt-0.5 break-all">${esc(e.message).slice(0, 160)}</div>` : ''}</div>`).join('') ||
      '<div class="text-xs text-slate-400 text-center py-6">暂无事件</div>';
    const tags = ['TSK', 'NAV', 'OBS', 'INJ', 'RST', 'DRV', 'SAF'];
    $('[data-tags]', el).innerHTML = `<button data-t="" class="tag ${!evFilter ? 'bg-blue-600 text-white border-blue-600' : 'bg-white text-slate-500 border-slate-200'}">全部</button>` +
      tags.map(t => `<button data-t="${t}" class="tag ${evFilter === t ? 'bg-blue-600 text-white border-blue-600' : TAG_STYLE[t]}">${t} ${W.evTags[t] || 0}</button>`).join('');
    $$('[data-t]', el).forEach(b => b.onclick = () => { evFilter = b.dataset.t || null; renderEvents(); });
  }

  // ---------------- 注入库
  async function renderInject() {
    if (!catalog) catalog = await iget('/api/v2/inject/catalog').catch(() => ({ catalog: [], placements: [] }));
    const P = $('[data-pane-inj]', el);
    const obs = (W.tel.dynamic_obstacles || []);
    P.innerHTML = `<div class="px-4 py-3 border-b border-slate-100"><div class="panel-title"><i data-lucide="package-plus" class="w-4 h-4 text-amber-600"></i>仿真元素扰动注入库</div>
        <div class="text-[11px] text-slate-500 mt-0.5">选取特定元素放入场景，测试避障与路径规划</div></div>
      <div class="flex-1 overflow-y-auto p-4 space-y-4">
        <div><div class="text-xs font-semibold text-slate-700 mb-2">选取要注入的元素类别</div><div class="grid grid-cols-2 gap-2">
          ${catalog.catalog.map(c => `<button data-type="${c.type}" class="p-2.5 rounded-lg border text-left ${sel.type === c.type ? 'border-amber-400 bg-amber-50 ring-1 ring-amber-200' : 'border-slate-200 bg-white hover:border-slate-300'}">
            <div class="text-lg">${c.icon}</div><div class="text-xs font-bold text-slate-800">${esc(c.name)}</div><div class="text-[10px] text-slate-500 font-mono">${esc(c.desc)}</div></button>`).join('')}</div>
          ${sel.type === 'person' ? `<label class="flex items-center gap-2 text-xs mt-2"><input type="checkbox" data-walk ${sel.walk ? 'checked' : ''}>人员沿垂直路径方向往返行走 (0.6 m/s，±1.5 m)</label>` : ''}</div>
        <div><div class="text-xs font-semibold text-slate-700 mb-2">放置注入目标位置</div><div class="space-y-1.5">
          ${catalog.placements.map(p => `<label class="flex items-start gap-2 text-xs p-2 rounded border ${sel.placement === p.id ? 'border-blue-300 bg-blue-50' : 'border-slate-200 bg-white'} cursor-pointer">
            <input type="radio" name="pl" value="${p.id}" ${sel.placement === p.id ? 'checked' : ''} class="mt-0.5">${esc(p.name)}</label>`).join('')}</div>
          <div data-custom class="${sel.placement === 'custom' ? '' : 'hidden'} mt-2"><div class="grid grid-cols-2 gap-2 mb-2"><input class="inp font-mono" data-x placeholder="X (m)" value="${sel.x ?? ''}"><input class="inp font-mono" data-y placeholder="Y (m)" value="${sel.y ?? ''}"></div>
            <div class="h-44 relative border border-slate-200 rounded-lg overflow-hidden" data-pick></div><div class="text-[10px] text-slate-400 mt-1">在小地图上点击选取坐标</div></div></div>
        <button class="btn-primary w-full justify-center !py-2" data-put><i data-lucide="plus-square" class="w-4 h-4"></i>将该元素放入仿真环境中</button>
        <div><div class="flex justify-between text-xs font-semibold text-slate-700 mb-2"><span>已在场景中人工注入的元素:</span><span class="font-mono">${obs.length} 项</span></div>
          <div class="space-y-1.5">${obs.map(o => `<div class="flex justify-between items-center bg-white border border-slate-200 rounded px-2.5 py-1.5 text-xs">
            <span>#${o.id} ${esc(o.name || o.type)} <span class="font-mono text-slate-400">(${fmt(o.x)}, ${fmt(o.y)})${o.motion ? ' · 行走' : ''}</span></span>
            <button class="text-rose-600 hover:underline" data-rmobs="${o.id}">移除</button></div>`).join('') || '<div class="text-[11px] text-slate-400">暂无人工注入的扰动元素</div>'}</div></div>
      </div>
      <div class="p-3 border-t border-slate-200"><button class="btn w-full justify-center !py-2 bg-emerald-600 hover:bg-emerald-700 text-white" data-resume><i data-lucide="play" class="w-4 h-4"></i>恢复仿真运行 (观察避障响应)</button></div>`;
    icons(P);
    $$('[data-type]', P).forEach(b => b.onclick = () => { sel.type = b.dataset.type; renderInject(); });
    const wk = $('[data-walk]', P); if (wk) wk.onchange = () => { sel.walk = wk.checked; };
    $$('input[name=pl]', P).forEach(r => r.onchange = () => { sel.placement = r.value; renderInject(); });
    if (sel.placement === 'custom') {
      if (pickMap) pickMap.destroy();
      pickMap = new MapCanvas($('[data-pick]', P), { onClick: ({ x, y }) => { sel.x = +x.toFixed(2); sel.y = +y.toFixed(2); $('[data-x]', P).value = sel.x; $('[data-y]', P).value = sel.y; pickMap.pickPoint = { x, y }; pickMap.draw(); } });
      pickMap.layers.labels = false;
      if (W.tel.scenario_metadata) pickMap.setScene(W.tel.scenario_metadata, W.tel.topo_graph);
      pickMap.robot = { x: W.tel.x, y: W.tel.y, yaw: W.tel.yaw }; pickMap.footprint = W.tel.robot_spec?.footprint; pickMap.obstacles = obs; pickMap.plan = W.tel.plan_path || []; pickMap.planIndex = W.tel.path_index; pickMap.planLabels = W.tel.path_labels; pickMap.planCurve = W.tel.plan_curve; pickMap.protection = W.tel.protection;
      if (sel.x != null) pickMap.pickPoint = { x: sel.x, y: sel.y };
      pickMap.draw();
    }
    $('[data-put]', P).onclick = guard(async () => {
      if (!needLock()) return;
      const b = { type: sel.type, placement: sel.placement, walk: sel.walk };
      if (sel.placement === 'custom') { b.x = parseFloat($('[data-x]', P).value); b.y = parseFloat($('[data-y]', P).value); if (isNaN(b.x) || isNaN(b.y)) throw new Error('请输入或点选坐标'); }
      const r = await ipost('/api/v2/inject', b);
      toast(`已注入 #${r.id} ${r.name} @ (${fmt(r.x)}, ${fmt(r.y)})`, 'ok');
      setTimeout(renderInject, 600);
    });
    $$('[data-rmobs]', P).forEach(b => b.onclick = guard(async () => { if (!needLock()) return; await idel(`/api/v2/inject/${b.dataset.rmobs}`); setTimeout(renderInject, 500); }));
    $('[data-resume]', P).onclick = guard(async () => { if (!needLock()) return; await ipost('/api/v2/pause', { paused: false }); });
  }

  // ---------------- 手动控制抽屉
  function renderManual() {
    const D = $('[data-drawer]', el), sts = (W.tel.scenario_metadata || {}).stations || [];
    D.innerHTML = `<div class="flex justify-between items-center mb-3"><span class="panel-title"><i data-lucide="gamepad-2" class="w-4 h-4 text-blue-600"></i>手动控制</span><button data-x class="text-slate-400"><i data-lucide="x" class="w-4 h-4"></i></button></div>
      <div class="text-[11px] text-slate-500 mb-2">点动遥控 (按住按钮；键盘 W/A/S/D/Q/E 同样有效)</div>
      <div class="grid grid-cols-3 gap-1.5 w-48 mx-auto mb-4">${[['q', '↺', 0, 0, .6], ['w', '▲', .4, 0, 0], ['e', '↻', 0, 0, -.6], ['a', '◀', 0, .3, 0], ['x', '■', 0, 0, 0], ['d', '▶', 0, -.3, 0], ['', '', 0, 0, 0], ['s', '▼', -.3, 0, 0], ['', '', 0, 0, 0]]
        .map(([k, s, vx, vy, wz]) => k ? `<button class="btn-ghost justify-center !py-2 text-sm" data-jog="${vx},${vy},${wz}" data-key="${k}">${s}</button>` : '<span></span>').join('')}</div>
      <div class="text-[11px] text-slate-500 mb-1.5">单点导航到工位 (不经任务流)</div>
      <div class="flex flex-wrap gap-1 mb-3">${sts.map(s => `<button class="tag bg-blue-50 text-blue-700 border-blue-200" data-goto="${esc(s.id)}">${esc(s.id)}</button>`).join('')}</div>
      <button class="btn-danger w-full justify-center" data-cancel>取消当前导航</button>`;
    icons(D);
    $('[data-x]', D).onclick = () => D.classList.add('hidden');
    let jog = null;
    const send = (v) => http('POST', W.base + '/api/cmd_vel', { vx: v[0], vy: v[1], wz: v[2] }, { 'X-Lock-Token': W.token }).catch(() => { });
    $$('[data-jog]', D).forEach(b => {
      const v = b.dataset.jog.split(',').map(Number);
      const start = () => { if (!needLock()) return; clearInterval(jog); send(v); jog = setInterval(() => send(v), 150); };
      const stop = () => { clearInterval(jog); jog = null; send([0, 0, 0]); };
      b.onmousedown = start; b.onmouseup = stop; b.onmouseleave = () => jog && stop();
    });
    $$('[data-goto]', D).forEach(b => b.onclick = guard(async () => {
      if (!needLock()) return;
      const s = sts.find(x => x.id === b.dataset.goto);
      await http('POST', W.base + '/api/navigate_to_pose', { x: s.x, y: s.y, yaw: s.dock_yaw || 0 }, { 'X-Lock-Token': W.token });
    }));
    $('[data-cancel]', D).onclick = guard(async () => { await http('POST', W.base + '/api/cancel_navigation', {}, { 'X-Lock-Token': W.token }); });
  }
  const keys = { w: [.4, 0, 0], s: [-.3, 0, 0], a: [0, .3, 0], d: [0, -.3, 0], q: [0, 0, .6], e: [0, 0, -.6] };
  const onKey = (e) => {
    if (W.tab !== '32' || $('[data-drawer]', el).classList.contains('hidden') || !W.lock.mine || /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
    const v = keys[e.key.toLowerCase()]; if (!v) return;
    http('POST', W.base + '/api/cmd_vel', e.type === 'keydown' ? { vx: v[0], vy: v[1], wz: v[2] } : { vx: 0, vy: 0, wz: 0 }, { 'X-Lock-Token': W.token }).catch(() => { });
  };
  window.addEventListener('keydown', onKey); window.addEventListener('keyup', onKey);

  fillFlows();
  let lastObsSig = '';
  return {
    onTel(t) {
      viewer.setTelemetry(t);
      renderHUD(t);
      pollSlam(t);
      setPaused(!!t.is_paused);
      renderWheels(t);
      const os = JSON.stringify((t.dynamic_obstacles || []).map(o => o.id));
      if (paused && os !== lastObsSig) { lastObsSig = os; renderInject(); }
    },
    onEvents() { renderEvents(); },
    show() { fillFlows(); renderEvents(); },
    destroy() { viewer.destroy(); pickMap && pickMap.destroy(); window.removeEventListener('keydown', onKey); window.removeEventListener('keyup', onKey); },
  };
}
