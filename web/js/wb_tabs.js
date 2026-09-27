// ============================================================================
// 3-3 仿真记录 · 3-4 仿真回放 · 3-5 传感器与安全 · 3-6 系统与导航
// ============================================================================
import { $, $$, esc, fmt, hub, http, toast, guard, icons, mmss, download, TAG_STYLE, CHASSIS_LABEL } from './core.js';
import { W, iget, ipost, iput, needLock, lockKey } from './wbcore.js';
import { ReplayView } from './viz3d.js';

const RES = { SUCCESS: ['顺利完成', 'bg-emerald-50 text-emerald-700 border-emerald-200'], AVOIDED: ['避障达成', 'bg-amber-50 text-amber-700 border-amber-200'],
  FAILED: ['失败', 'bg-rose-50 text-rose-700 border-rose-200'], COLLIDED: ['发生碰撞', 'bg-rose-50 text-rose-700 border-rose-200'],
  STOPPED: ['已终止', 'bg-slate-100 text-slate-600 border-slate-200'], RESET: ['已重置', 'bg-slate-100 text-slate-600 border-slate-200'], SUPERSEDED: ['被替代', 'bg-slate-100 text-slate-600 border-slate-200'] };
const resBadge = (r) => { const x = RES[r] || [r, RES.STOPPED[1]]; return `<span class="px-2 py-0.5 rounded-full text-[10px] font-bold border ${x[1]}">● ${x[0]}</span>`; };

// ============================================================================ 3-3
export function tab33(el) {
  el.innerHTML = `<div class="flex-1 overflow-y-auto p-6"><div data-stats class="grid grid-cols-4 gap-4 mb-5"></div>
    <div class="card flex flex-col"><div class="px-5 py-3.5 border-b border-slate-100 flex items-center justify-between">
      <span class="panel-title"><i data-lucide="archive" class="w-4 h-4 text-blue-600"></i>环境启动以来的仿真任务记录列表 <span class="text-[11px] font-normal text-slate-400 ml-2">(支持单独打包下载与一键载入回放)</span></span>
      <button class="btn-ghost" data-all><i data-lucide="download" class="w-3.5 h-3.5"></i>打包下载全部记录</button></div>
      <table class="w-full text-xs"><thead class="bg-slate-50 text-slate-500"><tr><th class="text-left px-4 py-3">任务编号</th><th class="text-left px-4">任务流名称</th><th class="text-left px-4">仿真场景</th>
        <th class="text-left px-4">车辆模型</th><th class="text-left px-4">执行时刻</th><th class="text-left px-4">运行耗时</th><th class="text-left px-4">注入扰动</th><th class="text-left px-4">避障</th><th class="text-left px-4">结果状态</th><th class="text-right px-4">操作</th></tr></thead>
        <tbody data-rows></tbody></table></div></div>`;
  $('[data-all]', el).onclick = () => download(`${W.base}/api/v2/records/bundle_all`);
  let sig = '';
  const render = (d) => {
    const s = d.stats || {};
    const card = (ic, c, label, v) => `<div class="card p-5 flex items-center gap-4"><div class="w-10 h-10 rounded-lg bg-${c}-50 text-${c}-600 flex items-center justify-center border border-${c}-100"><i data-lucide="${ic}" class="w-5 h-5"></i></div>
      <div><div class="text-[11px] text-slate-500">${label}</div><div class="text-xl font-bold font-mono text-${c === 'blue' ? 'slate-800' : c + '-600'}">${v}</div></div></div>`;
    $('[data-stats]', el).innerHTML = card('check-circle-2', 'blue', '累计执行仿真任务', `${s.count || 0} 次`) + card('shield-check', 'emerald', '无碰撞成功达成率', s.success_rate == null ? '—' : s.success_rate + '%') +
      card('alert-triangle', 'amber', '扰动元素避障次数', `${s.avoid_count || 0} 次`) + card('clock', 'purple', '平均任务用时', s.avg_duration == null ? '—' : s.avg_duration + ' 秒');
    icons($('[data-stats]', el));
    const k = JSON.stringify(d.records.map(r => r.id));
    if (k !== sig) {
      sig = k;
      $('[data-rows]', el).innerHTML = d.records.map(r => `<tr class="border-t border-slate-100 hover:bg-slate-50">
        <td class="px-4 py-3 font-mono font-bold text-blue-600">${esc(r.taskId)}</td><td class="px-4 font-semibold">${esc(r.taskName)}</td><td class="px-4">${esc(r.scene)}</td>
        <td class="px-4 font-mono">${esc(r.model)}</td><td class="px-4 font-mono text-slate-500">${esc(r.recordedAt)}</td><td class="px-4 font-mono font-bold">${fmt(r.durationSeconds, 1)}s</td>
        <td class="px-4 font-mono text-orange-600">${r.injections} 次</td><td class="px-4 font-mono text-amber-600">${r.avoidCount} 次</td><td class="px-4">${resBadge(r.result)}</td>
        <td class="px-4 text-right whitespace-nowrap"><button class="btn-ghost !py-1" data-dl="${r.id}"><i data-lucide="download" class="w-3.5 h-3.5"></i>下载数据包</button>
          <button class="btn-primary !py-1" data-rp="${r.id}"><i data-lucide="play-circle" class="w-3.5 h-3.5"></i>载入回放</button></td></tr>`).join('') ||
        '<tr><td colspan="10" class="text-center text-slate-400 py-12">暂无记录。在 3-2 下发任务流后自动生成 (顶栏录制开启时)。</td></tr>';
      icons(el);
      $$('[data-dl]', el).forEach(b => b.onclick = () => download(`${W.base}/api/v2/records/${b.dataset.dl}/bundle`));
      $$('[data-rp]', el).forEach(b => b.onclick = guard(async () => {
        const bd = await http('GET', `${W.base}/api/v2/records/${b.dataset.rp}/bundle`);
        W.replayBundle = bd; location.hash = `#/wb/${W.iid}/34`;
      }));
    }
  };
  return { onRecords: render, show() { iget('/api/v2/records').then(render).catch(() => { }); } };
}

// ============================================================================ 3-4
let replayApi = null;
export function openReplayRecord(b) { if (replayApi) replayApi.load(b); }
export function tab34(el) {
  el.innerHTML = `<div class="h-12 bg-white border-b border-slate-200 px-4 flex items-center justify-between text-xs shrink-0">
      <div class="flex items-center gap-3"><label class="btn-soft cursor-pointer"><i data-lucide="file-up" class="w-3.5 h-3.5"></i>导入仿真数据包 (.json)<input type="file" accept=".json" class="hidden" data-file></label>
        <span class="text-slate-500">当前回放:</span><span data-title class="font-bold text-slate-800">未载入</span><span data-tid class="tag bg-blue-50 text-blue-700 border-blue-200 hidden"></span></div>
      <div class="flex items-center gap-3"><span class="text-slate-500">倍速:</span><div class="seg" data-speeds>${[0.5, 1, 2, 4].map(s => `<button data-sp="${s}" class="${s === 1 ? 'on' : ''}">${s.toFixed(1)}x</button>`).join('')}</div>
        <div class="seg"><button data-v="free" class="on">自由3D</button><button data-v="top">俯瞰2D</button><button data-v="follow">跟随</button></div></div></div>
    <div class="flex-1 flex min-h-0"><div class="flex-1 relative min-w-0" data-vp>
        <div class="absolute top-4 left-4 z-10 w-64 bg-white/95 border border-slate-200 rounded-xl shadow-lg p-4 text-xs" data-hud></div>
        <div data-empty class="absolute inset-0 z-10 flex items-center justify-center pointer-events-none"><div class="card px-6 py-4 text-sm text-slate-500">从 3-3 载入记录，或导入 sim_bundle (.json)</div></div></div>
      <div class="w-80 bg-white border-l border-slate-200 flex flex-col shrink-0"><div class="px-4 py-3 border-b border-slate-100 flex justify-between"><span class="panel-title"><i data-lucide="clock" class="w-4 h-4 text-blue-600"></i>回放时间线事件流</span><span data-evn class="text-[11px] text-slate-500"></span></div>
        <div data-evs class="flex-1 overflow-y-auto p-3 space-y-2"></div></div></div>
    <div class="h-16 bg-white border-t border-slate-200 px-6 flex items-center gap-4 shrink-0">
      <button data-play class="w-10 h-10 rounded-full bg-blue-600 hover:bg-blue-700 text-white flex items-center justify-center shadow"><i data-lucide="play" class="w-5 h-5"></i></button>
      <button data-back class="btn-ghost !px-2" title="后退 2 秒"><i data-lucide="rewind" class="w-4 h-4 text-amber-600"></i></button>
      <button data-fwd class="btn-ghost !px-2" title="前进 2 秒"><i data-lucide="fast-forward" class="w-4 h-4 text-amber-600"></i></button>
      <span class="font-mono text-xs"><span data-cur class="text-blue-600 font-bold">00:00.0</span> / <span data-total>00:00.0</span></span>
      <input type="range" data-seek min="0" max="1000" value="0" class="flex-1">
      <button data-reset class="btn-ghost">重置到起点</button></div>`;
  const view = new ReplayView($('[data-vp]', el));
  const R = { b: null, t: 0, T: 0, playing: false, speed: 1, last: 0 };
  function load(b) {
    if (!b || !b.trajectory) { toast('不是有效的 sim_bundle 数据包', 'err'); return; }
    R.b = b; R.t = 0; R.T = b.metadata?.durationSeconds || (b.trajectory.length ? b.trajectory[b.trajectory.length - 1].time : 0);
    view.loadBundle(b);
    $('[data-empty]', el).classList.add('hidden');
    $('[data-title]', el).textContent = b.metadata?.taskName || b.taskFlow?.name || '仿真记录';
    const tid = $('[data-tid]', el); tid.textContent = b.metadata?.taskId || ''; tid.classList.remove('hidden');
    $('[data-total]', el).textContent = mmss(R.T);
    $('[data-evn]', el).textContent = `${(b.events || []).length} 项事件`;
    $('[data-evs]', el).innerHTML = (b.events || []).map((e, i) => `<div data-ev="${i}" class="border border-slate-200 rounded-lg p-2.5 cursor-pointer hover:border-blue-300">
      <div class="flex justify-between"><span class="tag ${TAG_STYLE[e.tag] || TAG_STYLE.SYS}">[${esc(e.tag)}]</span><span class="text-[10px] font-mono text-slate-400">${mmss(e.time)}</span></div>
      <div class="text-xs font-semibold text-slate-800 mt-1">${esc((e.title || '').replace(/^\[(执行|仿真)进程\] /, ''))}</div><div class="text-[11px] text-slate-500 font-mono break-all">${esc(e.detail || '').slice(0, 120)}</div></div>`).join('');
    $$('[data-ev]', el).forEach(d => d.onclick = () => { R.t = b.events[+d.dataset.ev].time; frame(); });
    frame();
  }
  replayApi = { load };
  function frame() {
    const b = R.b; if (!b) return;
    const tr = b.trajectory; if (!tr.length) return;
    let lo = 0, hi = tr.length - 1;
    while (lo < hi) { const m = (lo + hi + 1) >> 1; if (tr[m].time <= R.t) lo = m; else hi = m - 1; }
    const a = tr[lo], c = tr[Math.min(tr.length - 1, lo + 1)];
    const k = c.time > a.time ? Math.min(1, (R.t - a.time) / (c.time - a.time)) : 0;
    const fr = { x: a.x + (c.x - a.x) * k, y: a.y + (c.y - a.y) * k, yaw: a.yaw, vx: a.vx, status: a.status, obsDist: a.obsDist };
    const obs = (b.injectedElements || []).filter(e => e.spawnTime <= R.t && (e.removedAt == null || e.removedAt > R.t))
      .map((e, i) => ({ id: e.id ?? i + 1, type: e.type, x: e.x, y: e.y, w: e.size?.[0], h: e.size?.[1], z: e.size?.[2] }));
    view.showFrame(fr, obs);
    const evs = b.events || [];
    const done = evs.filter(e => e.time <= R.t).length;
    $('[data-hud]', el).innerHTML = `<div class="flex justify-between mb-2"><span class="font-bold text-slate-800">回放遥测数据</span><span class="tag bg-blue-50 text-blue-700 border-blue-200">${R.playing ? '回放中' : '已暂停'}</span></div>
      <div class="kv"><span>坐标:</span><span>X: ${fmt(fr.x)}m Y: ${fmt(fr.y)}m</span></div><div class="kv"><span>线速:</span><span class="text-blue-600">${fmt(fr.vx)} m/s</span></div>
      <div class="kv"><span>状态:</span><span>${esc(fr.status || '')}</span></div><div class="kv"><span>最近障碍:</span><span>${fr.obsDist == null ? '—' : fmt(fr.obsDist) + ' m'}</span></div>
      <div class="kv"><span>事件进度:</span><span>${done} / ${evs.length}</span></div>`;
    $$('[data-ev]', el).forEach((d, i) => { const on = evs[i].time <= R.t && (i === evs.length - 1 || evs[i + 1].time > R.t); d.classList.toggle('ring-2', on); d.classList.toggle('ring-blue-300', on); d.classList.toggle('bg-blue-50', on); d.style.opacity = evs[i].time <= R.t ? 1 : .5; });
    $('[data-cur]', el).textContent = mmss(R.t);
    $('[data-seek]', el).value = R.T ? Math.round(R.t / R.T * 1000) : 0;
  }
  const setPlay = (p) => { R.playing = p; $('[data-play]', el).innerHTML = `<i data-lucide="${p ? 'pause' : 'play'}" class="w-5 h-5"></i>`; icons($('[data-play]', el)); R.last = performance.now(); };
  const tick = () => {
    if (!alive) return;
    if (R.playing && R.b) {
      const now = performance.now(); R.t += (now - R.last) / 1000 * R.speed; R.last = now;
      if (R.t >= R.T) { R.t = R.T; setPlay(false); }
      frame();
    }
    requestAnimationFrame(tick);
  };
  let alive = true; requestAnimationFrame(tick);
  $('[data-play]', el).onclick = () => { if (R.b) { if (R.t >= R.T) R.t = 0; setPlay(!R.playing); } };
  $('[data-back]', el).onclick = () => { R.t = Math.max(0, R.t - 2); frame(); };
  $('[data-fwd]', el).onclick = () => { R.t = Math.min(R.T, R.t + 2); frame(); };
  $('[data-reset]', el).onclick = () => { R.t = 0; setPlay(false); frame(); };
  $('[data-seek]', el).oninput = (e) => { R.t = e.target.value / 1000 * R.T; frame(); };
  $$('[data-sp]', el).forEach(b => b.onclick = () => { R.speed = +b.dataset.sp; $$('[data-sp]', el).forEach(x => x.classList.toggle('on', x === b)); });
  $$('[data-v]', el).forEach(b => b.onclick = () => { view.setMode(b.dataset.v); $$('[data-v]', el).forEach(x => x.classList.toggle('on', x === b)); });
  $('[data-file]', el).onchange = (e) => {
    const f = e.target.files[0]; if (!f) return;
    f.text().then(t => load(JSON.parse(t))).catch(err => toast('解析失败: ' + err.message, 'err'));
  };
  return {
    show() { if (W.replayBundle) { load(W.replayBundle); W.replayBundle = null; } },
    destroy() { alive = false; view.destroy(); replayApi = null; },
  };
}

// ============================================================================ 3-5 传感器与安全
export function tab35(el) {
  el.innerHTML = `<div class="flex-1 overflow-y-auto p-6"><div class="grid grid-cols-3 gap-5">
    <div class="card p-4"><div class="panel-title mb-3"><i data-lucide="shield-alert" class="w-4 h-4 text-rose-600"></i>安全链状态</div><div data-safety></div></div>
    <div class="card p-4"><div class="panel-title mb-3"><i data-lucide="scan-line" class="w-4 h-4 text-amber-600"></i>光电 / 防撞触边</div><div data-photo></div></div>
    <div class="card p-4"><div class="panel-title mb-3"><i data-lucide="toggle-right" class="w-4 h-4 text-blue-600"></i>工业 IO</div><div data-io class="max-h-72 overflow-y-auto"></div></div>
    <div class="card p-4 col-span-3"><div class="flex justify-between items-center mb-3"><span class="panel-title"><i data-lucide="camera" class="w-4 h-4 text-pink-600"></i>相机类传感器 (单目 / 双目 / ToF)</span>
      <a data-editor class="btn-ghost" target="_blank"><i data-lucide="wrench" class="w-3.5 h-3.5"></i>模型补全与传感器安装</a></div><div data-cams class="grid grid-cols-4 gap-4"></div></div>
    <div class="card p-4 col-span-3"><div class="panel-title mb-3"><i data-lucide="radar" class="w-4 h-4 text-emerald-600"></i>激光雷达</div><div data-lidars></div></div>
  </div></div>`;
  let camSig = '', camTimer = null, lidSig = '';
  const setDI = (k, v) => http('POST', W.base + '/api/set_io', { key: k, value: v }, { 'X-Lock-Token': W.token });
  function render(t) {
    const io = t.io_states || {}, di = io.inputs || {}, dout = io.outputs || {}, sf = t.safety || {}, ob = sf.obstacle || {};
    const row = (k, v, ok) => `<div class="kv"><span>${k}</span><span class="${ok ? 'text-emerald-600' : 'text-rose-600 font-bold'}">${v}</span></div>`;
    $('[data-safety]', el).innerHTML = row('急停', io.is_emergency_stop ? '已按下 (抱闸)' : '正常', !io.is_emergency_stop) +
      row('防撞触边', io.bumper_active ? '压下' : '正常', !io.bumper_active) + row('抱闸', dout.do_brake_release === false ? '抱死' : '释放', dout.do_brake_release !== false) +
      row('激光走廊', { clear: '正常巡航', slow: '减速避让', stop: '停车等待' }[ob.zone || 'clear'], (ob.zone || 'clear') === 'clear') +
      `<div class="kv"><span>前 / 后最近障碍</span><span>${ob.front == null ? '—' : fmt(ob.front) + ' m'} / ${ob.rear == null ? '—' : fmt(ob.rear) + ' m'}</span></div>
      <div class="kv"><span>执行进程安全状态</span><span>${esc(t.nav_status || '')}</span></div>
      <div class="grid grid-cols-2 gap-2 mt-3"><button class="btn justify-center bg-rose-600 hover:bg-rose-700 text-white" data-estop><i data-lucide="octagon" class="w-3.5 h-3.5"></i>急停</button>
        <button class="btn-ghost justify-center" data-reset>急停复位</button></div>`;
    $('[data-estop]', el).onclick = guard(async () => { if (needLock()) await setDI('di_estop', true); });
    $('[data-reset]', el).onclick = guard(async () => { if (needLock()) await setDI('di_estop', false); });
    const ph = t.photoelectric || [], bu = (t.bumpers || {}).strips || [];
    $('[data-photo]', el).innerHTML = ph.map(p => `<div class="kv"><span>${p.detected ? '🔴' : '🟢'} 光电 ${esc(p.name)}</span><span>${p.distance_m == null ? '—' : fmt(p.distance_m) + ' m'} / 触发 ${fmt(p.trigger_m)} m</span></div>`).join('') +
      bu.map(b => `<div class="kv"><span>${b.pressed ? '🔴' : '🟢'} 触边 ${esc(b.name)} (${esc(b.side)})</span><span>按压 ${b.press_count} 次</span></div>`).join('') || '<div class="text-xs text-slate-400">无</div>';
    const ioRow = (k, v, kind) => `<div class="flex justify-between items-center py-1 border-b border-slate-100 text-xs"><span class="font-mono text-slate-600">${esc(k)}</span>
      <button data-io="${kind}:${esc(k)}" class="w-9 h-5 rounded-full ${v ? 'bg-emerald-500' : 'bg-slate-300'} relative"><span class="absolute top-0.5 ${v ? 'right-0.5' : 'left-0.5'} w-4 h-4 bg-white rounded-full shadow"></span></button></div>`;
    $('[data-io]', el).innerHTML = `<div class="text-[11px] font-semibold text-slate-500 mb-1">DI 输入 (仿真注入)</div>${Object.entries(di).map(([k, v]) => ioRow(k, v, 'di')).join('')}
      <div class="text-[11px] font-semibold text-slate-500 mt-3 mb-1">DO 输出</div>${Object.entries(dout).map(([k, v]) => ioRow(k, v, 'do')).join('')}
      <div class="text-[11px] text-slate-500 mt-2">顶升高度 ${fmt(io.lift_height_mm, 1)} mm</div>`;
    $$('[data-io]', el).forEach(b => b.onclick = guard(async () => {
      if (!needLock()) return;
      const [kind, k] = b.dataset.io.split(':'); const cur = (kind === 'di' ? di : dout)[k];
      await http('PUT', `${W.base}/sim/api/v1/io/${kind}/${k}`, { value: !cur }, { 'X-Lock-Token': W.token });
    }));
    const cams = t.cameras || [];
    const cs = JSON.stringify(cams.map(c => [c.name, c.streams]));
    if (cs !== camSig) {
      camSig = cs;
      $('[data-cams]', el).innerHTML = cams.map(c => `<div class="border border-slate-200 rounded-lg overflow-hidden"><div class="bg-slate-900 aspect-[4/3] flex items-center justify-center"><img data-cam="${esc(c.name)}" class="max-w-full max-h-full"></div>
        <div class="p-2 flex justify-between items-center text-[11px]"><span class="font-semibold">${esc(c.name)} <span class="text-slate-400">${esc(c.type)} ${c.width}×${c.height}</span></span>
        <select data-stream="${esc(c.name)}" class="border border-slate-200 rounded text-[11px]">${(c.streams || ['rgb']).filter(s => s !== 'points').map(s => `<option>${s}</option>`).join('')}</select></div></div>`).join('') ||
        '<div class="text-xs text-slate-400 col-span-4">当前模型没有相机。可在「模型补全与传感器安装」中添加单目/双目/ToF 相机。</div>';
    }
    const lf = (t.robot_spec || {}).lidars_full || [], ls = JSON.stringify(lf.map(l => l.name));
    if (ls !== lidSig) {
      lidSig = ls; const lc = t.lidar_config || {};
      $('[data-lidars]', el).innerHTML = `<table class="w-full text-xs"><thead class="text-slate-500"><tr><th class="text-left py-1">名称</th><th class="text-left">型号</th><th class="text-left">类型</th><th class="text-left">安装 [x,y,z] m</th><th class="text-left">视场</th><th class="text-left">量程</th><th class="text-left">频率</th></tr></thead>
        <tbody>${lf.map(l => `<tr class="border-t border-slate-100"><td class="py-1.5 font-mono">${esc(l.name)}</td><td>${esc(l.vendor_model || l.model || '')}</td><td>${esc(l.type)}</td><td class="font-mono">[${fmt(l.x)}, ${fmt(l.y)}, ${fmt(l.z)}]</td><td>${fmt(l.fov_deg, 0)}°</td><td>${fmt(l.max_range, 1)} m</td><td>${fmt(l.freq_hz, 0)} Hz</td></tr>`).join('')}</tbody></table>
        <div class="flex items-end gap-3 mt-4 text-xs"><div><label class="lbl">融合扫描点数</label><input class="inp !w-28 font-mono" data-lc="beams" value="${lc.beams || ''}"></div>
          <div><label class="lbl">量程上限 (m)</label><input class="inp !w-28 font-mono" data-lc="range_max" value="${lc.range_max || ''}"></div>
          <div><label class="lbl">频率 (Hz)</label><input class="inp !w-28 font-mono" data-lc="freq_hz" value="${lc.freq_hz || ''}"></div><button class="btn-primary" data-lcs>应用激光配置</button></div>`;
      $('[data-lcs]', el).onclick = guard(async () => {
        if (!needLock()) return;
        const b = {}; $$('[data-lc]', el).forEach(i => i.value && (b[i.dataset.lc] = parseFloat(i.value)));
        await http('POST', W.base + '/api/lidar_config', b, { 'X-Lock-Token': W.token }); toast('激光配置已应用', 'ok');
      });
    }
    icons(el);
  }
  const pollCams = () => {
    $$('[data-cam]', el).forEach(img => {
      if (img.dataset.busy) return;
      const n = img.dataset.cam, s = ($(`[data-stream="${n}"]`, el) || {}).value || 'rgb';
      img.dataset.busy = 1;
      const u = `${W.base}/sim/api/v1/sensors/cameras/${encodeURIComponent(n)}?stream=${s}&format=jpeg&_=${Date.now()}`;
      const im = new Image(); im.onload = () => { img.src = im.src; delete img.dataset.busy; }; im.onerror = () => { delete img.dataset.busy; }; im.src = u;
    });
  };
  return {
    onTel: render,
    show() {
      $('[data-editor]', el).href = `/model-editor?api=${encodeURIComponent(W.base + '/sim/api/v1')}&lockkey=${encodeURIComponent(lockKey())}`;
      clearInterval(camTimer); camTimer = setInterval(() => { if (!el.classList.contains('hidden')) pollCams(); }, 400); W.timers.push(camTimer);
    },
    destroy() { clearInterval(camTimer); },
  };
}

// ============================================================================ 3-6 系统与导航
export function tab36(el) {
  el.innerHTML = `<div class="flex-1 overflow-y-auto p-6"><div class="grid grid-cols-3 gap-5">
    <div class="card p-4"><div class="panel-title mb-3"><i data-lucide="route" class="w-4 h-4 text-blue-600"></i>导航规划器</div>
      <select class="inp mb-2" data-planner>${[['nav2', 'Nav2 (ROS 2 导航栈)'], ['dijkstra', 'Dijkstra 拓扑路网'], ['astar', 'A* 栅格'], ['direct', '直连导引']].map(([k, v]) => `<option value="${k}">${v}</option>`).join('')}</select>
      <button class="btn-primary w-full justify-center" data-setplanner>切换规划器</button><div data-nav2 class="mt-3"></div></div>
    <div class="card p-4"><div class="panel-title mb-3"><i data-lucide="shield" class="w-4 h-4 text-amber-600"></i>车辆保护空间</div>
      <div class="flex items-center gap-3 text-xs mb-2"><label class="flex items-center gap-1"><input type="checkbox" data-pe>启用防护区</label>
        <label class="flex items-center gap-1">过弯 <select class="inp !py-0.5 !w-auto" data-pmode><option value="auto">auto</option><option value="rotate">rotate</option><option value="arc">arc</option></select></label>
        <label class="flex items-center gap-1">净空 <input class="inp font-mono !py-0.5 !w-16" data-pbm></label></div>
      <table class="w-full text-[11px]"><thead><tr class="text-slate-500"><th class="text-left">档位</th><th>≤ m/s</th><th>前</th><th>后</th><th>侧</th></tr></thead><tbody data-pf></tbody></table>
      <div class="text-[11px] text-slate-500 mt-2" data-pinfo></div>
      <div class="grid grid-cols-2 gap-2 mt-3"><button class="btn-soft justify-center" data-papply>应用到本次运行</button><button class="btn-primary justify-center" data-psave>保存到车辆模型</button></div></div>
    <div class="card p-4"><div class="panel-title mb-3"><i data-lucide="repeat" class="w-4 h-4 text-purple-600"></i>热切换 (不重新部署)</div>
      <label class="lbl">底盘运动学</label><div class="flex gap-2 mb-3"><select class="inp" data-chassis>${[['cmodel', '按 cmodel'], ['single_steer', '单舵轮'], ['diff_drive', '差速双驱'], ['dual_steer', '双舵轮']].map(([k, v]) => `<option value="${k}">${v}</option>`).join('')}</select><button class="btn-soft" data-setchassis>切换</button></div>
      <label class="lbl">仿真场景 (平台场景库)</label><div class="flex gap-2 mb-3"><select class="inp" data-scene></select><button class="btn-soft" data-setscene>切换</button></div>
      <label class="lbl">车辆模型 (平台模型库)</label><div class="flex gap-2"><select class="inp" data-model></select><button class="btn-soft" data-setmodel>切换</button></div>
      <div class="text-[11px] text-amber-600 mt-2">热切换会终止当前任务流；实例部署记录中的模型/场景不变，重启实例会恢复部署时的配置。</div></div>
    <div class="card p-4 col-span-3"><div class="panel-title mb-3"><i data-lucide="locate-fixed" class="w-4 h-4 text-orange-600"></i>定位 (激光 SLAM + 里程计 / IMU 融合)
        <span class="ml-auto text-[11px] font-normal text-slate-500">导引使用定位结果；车辆真值只用于误差统计</span></div>
      <div class="grid grid-cols-3 gap-5">
        <div><label class="lbl">定位模式</label>
          <div class="flex gap-2 mb-2"><select class="inp" data-lmode>${[['slam', 'SLAM 边建图边定位'], ['localization', 'SLAM 定位 (已保存地图)'], ['odom', '纯里程计 (演示漂移)'], ['ground_truth', '真值 (对照)']].map(([k, v]) => `<option value="${k}">${v}</option>`).join('')}</select><button class="btn-soft" data-lsetmode>切换</button></div>
          <div class="grid grid-cols-2 gap-2"><button class="btn-primary justify-center" data-lsave>保存地图</button><button class="btn-soft justify-center" data-lreset>清空重建</button>
            <a class="btn-ghost justify-center" data-lpgm>下载 PGM</a><a class="btn-ghost justify-center" data-lyaml>下载 YAML</a></div>
          <div class="text-[11px] text-slate-500 mt-2 leading-relaxed">先在 SLAM 模式下把车开遍作业区域 (执行任务即可)，保存后该场景之后都用「SLAM 定位」模式在固定地图上定位。地图保存在执行进程数据目录 (slam_maps/：slam_toolbox 位姿图 .posegraph/.data + PGM/YAML)。</div></div>
        <div data-lstat></div>
        <div class="relative h-48 bg-slate-50 border border-slate-200 rounded"><canvas data-lmap class="absolute inset-0 w-full h-full"></canvas><span class="absolute top-1 left-2 text-[10px] text-slate-400" data-lmapinfo></span></div>
      </div></div>
    <div class="card p-4 col-span-2"><div class="panel-title mb-3"><i data-lucide="network" class="w-4 h-4 text-blue-600"></i>运行架构 (仿真 ⇄ 执行 · REST)</div><div data-arch></div></div>
    <div class="card p-4"><div class="panel-title mb-3"><i data-lucide="gauge" class="w-4 h-4 text-emerald-600"></i>性能</div><div data-perf></div></div>
    <div class="card p-4 col-span-3 flex items-center justify-between text-xs"><div class="text-slate-600">模型参数补全、传感器安装与相机预览 (作用于当前实例；如需长期保存请在资源平台的车辆模型中编辑)</div>
      <div class="flex gap-2"><a data-editor class="btn-soft" target="_blank"><i data-lucide="wrench" class="w-3.5 h-3.5"></i>打开模型补全</a><a data-legacy class="btn-ghost" target="_blank"><i data-lucide="external-link" class="w-3.5 h-3.5"></i>经典调度台</a></div></div>
  </div></div>`;
  const L = (k, v) => `<div class="kv"><span>${k}</span><span>${v}</span></div>`;
  const on = (ok) => ok ? '<span class="text-emerald-600">● 在线</span>' : '<span class="text-rose-600">● 离线</span>';
  $('[data-setplanner]', el).onclick = guard(async () => { if (!needLock()) return; await iput('/api/v2/planner', { type: $('[data-planner]', el).value }); toast('规划器已切换', 'ok'); });
  let P = null;
  const readProt = () => ({
    enabled: $('[data-pe]', el).checked, corner_mode: $('[data-pmode]', el).value, body_margin: parseFloat($('[data-pbm]', el).value),
    fields: $$('[data-pf] tr', el).map(tr => { const f = {}; $$('input', tr).forEach(i => f[i.dataset.k] = i.dataset.k === 'name' ? i.value : parseFloat(i.value)); return f; }),
  });
  const renderProt = (sf) => {
    P = (sf.protection_view || {}).config || sf.protection || {};
    const pv = sf.protection_view || {};
    $('[data-pe]', el).checked = P.enabled !== false; $('[data-pmode]', el).value = P.corner_mode || 'auto'; $('[data-pbm]', el).value = P.body_margin ?? 0.05;
    $('[data-pf]', el).innerHTML = (P.fields || []).map(f => `<tr><td><input class="inp !py-0.5 !px-1 !w-14" data-k="name" value="${esc(f.name)}"></td>${['v_max', 'front', 'rear', 'side'].map(k => `<td><input class="inp font-mono !py-0.5 !px-1 !w-14" data-k="${k}" value="${f[k]}"></td>`).join('')}</tr>`).join('');
    const o = pv.outline || [];
    $('[data-pinfo]', el).innerHTML = `当前外形 ${pv.loaded ? '<b class="text-purple-600">带载</b>' : '空载'}：头 ${o[0] ?? '-'} / 尾 ${o[1] ?? '-'} / 左 ${o[2] ?? '-'} / 右 ${o[3] ?? '-'} m · 原地转向扫掠 R=${((pv.polygons || {}).rotate_radius ?? '-')} m` +
      (P.fields_auto ? '<br>防护区按车型动力学自动生成' : '') + (pv.runtime_modified ? '<br><span class="text-amber-600">本次运行已临时修改，未保存到模型</span>' : '');
  };
  $('[data-papply]', el).onclick = guard(async () => {
    if (!needLock()) return;
    const r = await iput('/api/v2/safety', { protection: readProt() }); renderProt(r); toast('保护空间已应用到本次运行', 'ok');
  });
  $('[data-psave]', el).onclick = guard(async () => {
    if (!needLock()) return;
    const m = W.info?.model || {};
    if (!m.model_id) return toast('当前实例的车辆模型不在平台模型库中，只能应用到本次运行', 'warn');
    const base = `/models/${m.model_id}/versions/${m.version || 'latest'}/api/v1/model`;
    const ed = await hub.get(base + '/editor');
    const ov = Object.assign({}, ed.overrides || {}, { protection: Object.assign({}, (ed.overrides || {}).protection || {}, readProt()) });
    await hub.put(base + '/overrides', ov);
    await iput('/api/v2/safety', { protection: readProt() }).then(renderProt).catch(() => { });
    toast(`已保存到车辆模型 ${m.name || m.model_id} ${m.version || ''}，之后部署的实例都会使用`, 'ok');
  });
  $('[data-setchassis]', el).onclick = guard(async () => { if (!needLock()) return; await iput('/api/v2/chassis', { type: $('[data-chassis]', el).value }); toast('底盘运动学已切换', 'ok'); });
  $('[data-setscene]', el).onclick = guard(async () => { if (!needLock()) return; await ipost('/api/v2/scene', { hub_scene_id: $('[data-scene]', el).value }); W.sceneId = $('[data-scene]', el).value; toast('场景已切换', 'ok'); });
  $('[data-setmodel]', el).onclick = guard(async () => { if (!needLock()) return; await ipost('/api/v2/model', { model_id: $('[data-model]', el).value }); toast('车辆模型已切换', 'ok'); });
  // ---- 定位
  $('[data-lsetmode]', el).onclick = guard(async () => {
    if (!needLock()) return;
    const r = await ipost('/api/v2/slam/mode', { mode: $('[data-lmode]', el).value });
    toast(r.note || '定位模式已切换', r.note ? 'warn' : 'ok'); renderLoc(r);
  });
  $('[data-lsave]', el).onclick = guard(async () => { if (!needLock()) return; const r = await ipost('/api/v2/slam/save'); renderLoc(r.status); toast('SLAM 地图已保存，之后该场景自动使用定位模式', 'ok'); });
  $('[data-lreset]', el).onclick = guard(async () => {
    if (!needLock()) return;
    if (!confirm('清空当前 SLAM 地图并重新建图？(已保存的地图文件保留)')) return;
    renderLoc(await ipost('/api/v2/slam/reset', {})); lrev = null; toast('已清空，重新建图', 'ok');
  });
  $('[data-lpgm]', el).href = W.base + '/api/v2/slam/map?part=pgm';
  $('[data-lyaml]', el).href = W.base + '/api/v2/slam/map?part=yaml';
  let lrev = null, lt = 0;
  function renderLoc(s) {
    if (!s || !s.mode) { $('[data-lstat]', el).innerHTML = '<div class="text-xs text-slate-400">执行进程未连接</div>'; return; }
    const st = s.stats || {}, li = st.last_info || {}, m = s.map || {};
    const e = s.err_mm, cls = e == null ? '' : e > 20 ? 'text-rose-600' : e > 10 ? 'text-amber-600' : 'text-emerald-600';
    const ex = s.ext || {};
    const eng = s.engine === 'slam_toolbox' ? `slam_toolbox + robot_localization (开源) · ${ex.process ? '<span class="text-emerald-600">运行</span>' : '<span class="text-rose-600">未运行</span>'}${ex.tf_age_s != null ? ' · TF ' + fmt(ex.tf_age_s, 2) + ' s 前' : ''}`
      : (s.ext ? '内置 (odom/真值模式不启动 slam_toolbox)' : '内置 nav_runtime/slam.py (未检测到 ROS 2 slam_toolbox)');
    $('[data-lstat]', el).innerHTML = L('当前模式', esc(s.mode)) + L('定位引擎', eng) + L('定位误差 (相对真值)', `<span class="${cls}">${e == null ? '—' : fmt(e, 1) + ' mm / ' + fmt(s.err_yaw_deg, 3) + '°'}</span>`) +
      L('近 30 s 误差', s.err_rms_30s_mm == null ? '—' : `RMS ${fmt(s.err_rms_30s_mm, 1)} · 最大 ${fmt(s.err_max_30s_mm, 1)} mm`) +
      (s.engine === 'slam_toolbox' ? '' : L('估计标准差', `${fmt(s.std_xy_mm, 1)} mm / ${fmt(s.std_yaw_deg, 3)}°`)) +
      (s.engine === 'slam_toolbox' ? L('定位更新', `${fmt(st.hz, 1)} Hz (TF map→base_footprint)`) :
      L('扫描匹配', `${fmt(st.hz, 1)} Hz · ${fmt(st.match_ms, 1)} ms · 得分 ${li.score == null ? '—' : fmt(li.score, 2)} · 内点 ${li.inliers == null ? '—' : fmt(li.inliers * 100, 0) + '%'}`)) +
      L('匹配采纳 / 拒绝', `${st.matches ?? 0} / ${st.rejects ?? 0}`) +
      L('地图', m.w ? `${m.w}×${m.h} @ ${m.res} m · 更新 ${m.updates} 次 · ${m.saved ? '<span class="text-emerald-600">已保存</span>' : '未保存'}` : '空');
    if (document.activeElement !== $('[data-lmode]', el)) $('[data-lmode]', el).value = s.want_mode || s.mode;
  }
  function drawLocMap(mp) {
    const cv = $('[data-lmap]', el), r = cv.getBoundingClientRect(); if (!r.width) return;
    cv.width = r.width; cv.height = r.height; const g = cv.getContext('2d');
    g.fillStyle = '#f8fafc'; g.fillRect(0, 0, cv.width, cv.height);
    if (!mp || mp.empty) { $('[data-lmapinfo]', el).textContent = '地图为空'; return; }
    const raw = atob(mp.data), img = g.createImageData(mp.w, mp.h);
    for (let i = 0; i < mp.w * mp.h; i++) {
      const c = raw.charCodeAt(i), row = mp.h - 1 - Math.floor(i / mp.w), k = (row * mp.w + (i % mp.w)) * 4;
      const v = c === 2 ? [30, 41, 59] : c === 1 ? [255, 255, 255] : [226, 232, 240];
      img.data[k] = v[0]; img.data[k + 1] = v[1]; img.data[k + 2] = v[2]; img.data[k + 3] = 255;
    }
    const tmp = document.createElement('canvas'); tmp.width = mp.w; tmp.height = mp.h; tmp.getContext('2d').putImageData(img, 0, 0);
    const sc = Math.min(cv.width / mp.w, cv.height / mp.h);
    g.imageSmoothingEnabled = false; g.drawImage(tmp, (cv.width - mp.w * sc) / 2, (cv.height - mp.h * sc) / 2, mp.w * sc, mp.h * sc);
    $('[data-lmapinfo]', el).textContent = `${(mp.w * mp.res).toFixed(1)} × ${(mp.h * mp.res).toFixed(1)} m`;
  }
  function pollLoc() {
    if (Date.now() - lt < 1000) return; lt = Date.now();
    iget('/api/v2/slam').then(s => {
      renderLoc(s);
      const rev = (s.stats && s.map) ? `${s.map.updates}:${s.mode}` : null;
      if (rev !== lrev) { lrev = rev; iget('/api/v2/slam/map?step=4').then(drawLocMap).catch(() => { }); }
    }).catch(() => renderLoc(null));
  }
  async function loadStatic() {
    const [sf, sc, md] = await Promise.all([iget('/api/v2/safety').catch(() => ({})), hub.get('/scenes').catch(() => ({ scenes: [] })), hub.get('/models').catch(() => ({ models: [] }))]);
    renderProt(sf);
    $('[data-scene]', el).innerHTML = sc.scenes.map(s => `<option value="${s.id}" ${s.id === W.sceneId ? 'selected' : ''}>${esc(s.name)}</option>`).join('');
    $('[data-model]', el).innerHTML = md.models.map(m => `<option value="${m.id}" ${m.id === W.info?.model?.model_id ? 'selected' : ''}>${esc(m.name)} ${esc(m.latest)}</option>`).join('');
    $('[data-planner]', el).value = W.tel.planner_type || 'dijkstra';
    $('[data-chassis]', el).value = (W.tel.robot_spec || {}).active_chassis || 'cmodel';
    $('[data-editor]', el).href = `/model-editor?api=${encodeURIComponent(W.base + '/sim/api/v1')}&lockkey=${encodeURIComponent(lockKey())}`;
    // 网关只监听本机 (单手机模式 WEB_BIND=127.0.0.1) 时，经典调度台只能在手机本机浏览器打开
    const wu = W.inst?.web_url || '', lg = $('[data-legacy]', el);
    const loop = /^https?:\/\/(127\.|localhost)/.test(wu), here = /^(127\.|localhost)/.test(location.hostname);
    lg.href = wu && (!loop || here) ? wu : '#';
    if (loop && !here) { lg.title = '网关只对本机开放，请在手机浏览器里打开'; lg.classList.add('opacity-50'); }
  }
  let perf = null, pt = 0;
  function render(t) {
    pollLoc();
    const a = t.arch || {}, lk = a.link || {}, n2 = t.nav2 || {};
    $('[data-arch]', el).innerHTML = `<div class="grid grid-cols-2 gap-x-6">
      ${L('仿真进程 (agv-sim)', `${esc(a.sim_api || '')} ${on(a.sim_online)}`)}${L('执行进程 (agv-nav)', `${esc(a.nav_api || '')} ${on(a.nav_online)}`)}
      ${L('执行→仿真链路', `${lk.online ? '已连接' : '未连接'} · ${esc(lk.sim_url || '')}`)}${L('导航回馈延迟', a.nav_feedback_age_s == null ? '—' : fmt(a.nav_feedback_age_s, 2) + ' s')}
      ${L('状态拉取', lk.state_hz ? fmt(lk.state_hz, 1) + ' Hz' : (lk.state_count ? lk.state_count + ' 帧' : '—'))}${L('控制指令来源', esc((a.control || {}).source || '—'))}
      ${L('仿真引擎', esc((t.sim_status || {}).backend || '—'))}${L('实例', `${esc(W.iid)} · ${esc(W.info?.model?.name || '')} @ ${esc(W.info?.scene?.name || '')}`)}</div>`;
    $('[data-nav2]', el).innerHTML = L('Nav2 进程', n2.process ? '运行' : '未运行') + L('Nav2 就绪', n2.server_ready ? '<span class="text-emerald-600">就绪</span>' : '未就绪') + L('当前规划器', esc(t.planner_type || '—'));
    const s = t.sim_status || {};
    if (Date.now() - pt > 3000) { pt = Date.now(); iget('/api/system_perf').then(p => { perf = p; }).catch(() => { }); }
    const h = (perf || {}).host || {};
    $('[data-perf]', el).innerHTML = L('物理步进', fmt(s.step_ms, 3) + ' ms') + L('激光射线', fmt(s.lidar_ms, 2) + ' ms') + L('实时因子 RTF', fmt(s.rtf, 2)) +
      L('超时步数', s.overruns ?? '—') + L('仿真主机 CPU', h.cpu_total_percent == null ? '—' : fmt(h.cpu_total_percent, 0) + '%') + L('内存', h.memory_percent == null ? '—' : fmt(h.memory_percent, 0) + '%') +
      L('温度', h.cpu_temp_c ? fmt(h.cpu_temp_c, 1) + '°C' : '—');
  }
  return { onTel: render, show: () => loadStatic() };
}
