// ============================================================================
// 设备端仿真工作台: 实例选择 / 顶栏 (控制权·录制·日志) / Tab 3-1 ~ 3-6
// ============================================================================
import { $, $$, esc, hub, http, toast, guard, icons, badge, dur, user, confirmBox, download } from './core.js';
import { W, ia, iget, ipost, iput, saveToken, loadToken, every, lockKey } from './wbcore.js';
import { tab31 } from './wb_taskflow.js';
import { tab32 } from './wb_run.js';
import { tab33, tab34, tab35, tab36, openReplayRecord } from './wb_tabs.js';

const root = () => $('#view-workbench');
const TABS = [['31', 'git-branch', '3-1. 任务流配置'], ['32', 'play-circle', '3-2. 仿真运行与环境'], ['33', 'archive', '3-3. 仿真记录'], ['34', 'film', '3-4. 仿真回放'],
  ['35', 'radar', '3-5. 传感器与安全'], ['36', 'settings-2', '3-6. 系统与导航']];
const BUILDERS = { '31': tab31, '32': tab32, '33': tab33, '34': tab34, '35': tab35, '36': tab36 };

export function lastInstance() { try { return localStorage.getItem('amr.wb.last'); } catch (e) { return null; } }

export function leave() {
  W.gen = (W.gen || 0) + 1;
  W.timers.forEach(clearInterval); W.timers = [];
  Object.values(W.tabs).forEach(t => t && t.destroy && t.destroy());
  W.tabs = {};
  if (W.lock.mine && W.token && W.iid) { http('DELETE', W.base + '/api/v2/lock', undefined, { 'X-Lock-Token': W.token }).catch(() => { }); }
  W.lock = { held: false, mine: false }; W.iid = null; W.events = []; W.evAfter = 0; W.tel = {}; W.inst = null;
}

export async function open(iid, tab) {
  if (iid === 'replay') return openOffline(tab);
  if (!iid) return picker();
  if (W.iid === iid) { switchTab(tab || W.tab); return; }
  leave();
  let inst;
  try { inst = await hub.get(`/instances/${iid}`); } catch (e) { toast(e.message, 'err'); return picker(); }
  W.iid = iid; W.inst = inst; W.base = `/inst/${iid}`; W.offline = false;
  try { localStorage.setItem('amr.wb.last', iid); } catch (e) { }
  root().innerHTML = shell(inst);
  icons(root());
  bindHeader();
  try { W.info = await iget('/api/v2/info'); } catch (e) { W.info = {}; toast('实例网关无响应: ' + e.message, 'err'); }
  W.sceneId = W.info?.scene?.id || inst.scene_id;
  $('#wb-title').textContent = `${inst.name}`;
  await initLock();
  startPolling();
  switchTab(tab || '32');
}

function shell(inst) {
  const host = inst.sim_node_view?.host || (inst.web_url || '').replace(/^https?:\/\//, '');
  return `
  <div class="h-12 bg-white border-b border-slate-200 px-6 flex items-center justify-between shrink-0">
    <div class="flex items-center gap-4 min-w-0">
      <div class="flex items-center gap-2 text-xs min-w-0"><span id="wb-dot" class="w-2.5 h-2.5 rounded-full bg-emerald-500"></span>
        <span class="font-bold text-slate-800 whitespace-nowrap">当前仿真:</span><span id="wb-title" class="font-bold text-slate-800 truncate"></span>
        <span class="font-mono text-slate-500">(${esc(host)})</span>
        ${inst.nav_node && inst.nav_node !== inst.sim_node ? `<span class="text-[10px] px-1.5 py-0.5 rounded bg-blue-50 text-blue-600">执行 ${esc(inst.nav_node_view?.host || '')}</span>` : ''}</div>
      <div class="h-5 w-px bg-slate-200"></div>
      <div id="wb-lock" class="text-xs"></div>
      <div id="wb-safety" class="text-xs"></div>
    </div>
    <div class="flex items-center gap-2 text-xs shrink-0">
      <button id="wb-rec" class="btn border border-rose-200 bg-rose-50 text-rose-600"><span class="w-2 h-2 rounded-full bg-rose-500 pulse-dot"></span><span>录制中</span></button>
      <button id="wb-logs" class="btn-ghost"><i data-lucide="download" class="w-3.5 h-3.5"></i>下载仿真日志 (RCD/EVT)</button>
      <a href="#/platform/overview" class="btn-ghost">返回资源平台</a>
    </div>
  </div>
  <div class="h-11 bg-slate-50 border-b border-slate-200 px-6 flex items-end gap-2 shrink-0" id="wb-tabs">
    ${TABS.map(([k, ic, t]) => `<a href="#/wb/${W.iid}/${k}" data-tab="${k}" class="wb-tab"><i data-lucide="${ic}" class="w-3.5 h-3.5"></i>${t}</a>`).join('')}
  </div>
  <div id="wb-body" class="flex-1 min-h-0 relative"></div>`;
}

function switchTab(t) {
  if (!BUILDERS[t]) t = '32';
  W.tab = t;
  $$('#wb-tabs [data-tab]').forEach(a => a.classList.toggle('active', a.dataset.tab === t));
  const body = $('#wb-body');
  Object.entries(W.tabs).forEach(([k, v]) => v.el.classList.toggle('hidden', k !== t));
  if (!W.tabs[t]) {
    const el = document.createElement('div');
    el.className = 'absolute inset-0 flex flex-col';
    body.appendChild(el);
    W.tabs[t] = BUILDERS[t](el) || { el };
    W.tabs[t].el = el;
    icons(el);
  }
  W.tabs[t].show && W.tabs[t].show();
}
export function gotoTab(t) { location.hash = `#/wb/${W.iid}/${t}`; }

// ---------------------------------------------------------------- 控制权
async function initLock() {
  saveToken(loadToken());
  let st = await iget('/api/v2/lock').catch(() => ({ held: false }));
  if (W.token && st.held) { try { st = await iput('/api/v2/lock'); } catch (e) { saveToken(''); st = await iget('/api/v2/lock'); } }
  if (!st.held || (!st.mine && st.user === user.name)) { try { const r = await ipost('/api/v2/lock', { user: user.name }); saveToken(r.token); st = r; } catch (e) { } }
  W.lock = st; renderLock();
  every(10000, async () => {
    if (!W.lock.mine) return;
    try { W.lock = await iput('/api/v2/lock'); } catch (e) { W.lock = { held: false, mine: false }; saveToken(''); toast('控制权已失效', 'warn'); }
    renderLock();
  });
}
function renderLock() {
  const L = W.lock, el = $('#wb-lock'); if (!el) return;
  if (L.mine) {
    el.innerHTML = `<div class="flex items-center gap-2"><span class="px-2.5 py-1 rounded-md bg-emerald-50 border border-emerald-200 text-emerald-700 flex items-center gap-1.5"><i data-lucide="shield-check" class="w-3.5 h-3.5"></i>控制权状态: <b>${esc(L.user)} (当前客户端)</b> 已获取</span>
      <button class="btn-ghost !py-1" data-rel>释放控制权</button></div>`;
  } else if (L.held) {
    el.innerHTML = `<div class="flex items-center gap-2"><span class="px-2.5 py-1 rounded-md bg-amber-50 border border-amber-200 text-amber-700 flex items-center gap-1.5"><i data-lucide="lock" class="w-3.5 h-3.5"></i>只读: 控制权由 <b>${esc(L.user)}</b> 持有</span>
      <button class="btn-ghost !py-1" data-force>强制获取</button></div>`;
  } else {
    el.innerHTML = `<div class="flex items-center gap-2"><span class="px-2.5 py-1 rounded-md bg-slate-100 border border-slate-200 text-slate-600">控制权空闲</span><button class="btn-soft !py-1" data-get>获取控制权</button></div>`;
  }
  icons(el);
  const acquire = (force) => guard(async () => { const r = await ipost('/api/v2/lock', { user: user.name, force }); saveToken(r.token); W.lock = r; renderLock(); toast('已获取控制权', 'ok'); });
  const g = $('[data-get]', el); if (g) g.onclick = acquire(false);
  const f = $('[data-force]', el); if (f) f.onclick = async () => { if (await confirmBox(`强制获取控制权？${esc(L.user)} 将变为只读。`, '强制获取', false)) acquire(true)(); };
  const r = $('[data-rel]', el); if (r) r.onclick = guard(async () => { await ia('DELETE', '/api/v2/lock'); saveToken(''); W.lock = { held: false, mine: false }; renderLock(); });
}

// ---------------------------------------------------------------- 顶栏: 录制/日志/安全徽标
function bindHeader() {
  $('#wb-rec').onclick = guard(async () => {
    const on = !(W.stat.recording !== false);
    W.stat = await ipost('/api/v2/recording', { on }); renderRec();
    toast(on ? '已开启录制：下发任务流时自动生成仿真记录' : '已暂停录制', 'info');
  });
  $('#wb-logs').onclick = () => download(`${W.base}/api/v2/logs`);
}
function renderRec() {
  const b = $('#wb-rec'); if (!b) return;
  const s = W.stat || {};
  if (s.recording === false) {
    b.className = 'btn-ghost'; b.innerHTML = `<span class="w-2 h-2 rounded-full bg-slate-400"></span><span>录制已暂停</span>`;
  } else {
    b.className = 'btn border border-rose-200 bg-rose-50 text-rose-600';
    b.innerHTML = `<span class="w-2 h-2 rounded-full bg-rose-500 pulse-dot"></span><span>${s.active ? `录制中 (${dur(s.active.elapsed).slice(3)})` : '录制待命'}</span>`;
  }
}
function renderSafety() {
  const t = W.tel, el = $('#wb-safety'); if (!el) return;
  const io = t.io_states || {}, sf = t.safety || {};
  let txt = '安全链正常', cls = 'bg-emerald-50 text-emerald-700 border-emerald-200', ic = 'shield';
  const zone = sf.obstacle?.zone;
  if (io.is_emergency_stop) { txt = '急停中'; cls = 'bg-rose-600 text-white border-rose-600'; ic = 'octagon'; }
  else if (io.bumper_active) { txt = '触边压下'; cls = 'bg-rose-50 text-rose-700 border-rose-200'; ic = 'alert-octagon'; }
  else if (zone === 'stop') { txt = '停车等待'; cls = 'bg-rose-50 text-rose-700 border-rose-200'; ic = 'hand'; }
  else if (zone === 'slow' || (sf.photo && sf.photo.front && sf.photo.front.length)) { txt = '减速避让'; cls = 'bg-amber-50 text-amber-700 border-amber-200'; ic = 'alert-triangle'; }
  const sig = txt;
  if (el.dataset.sig === sig) return;
  el.dataset.sig = sig;
  el.innerHTML = `<a href="#/wb/${W.iid}/35" class="px-2.5 py-1 rounded-md border ${cls} flex items-center gap-1.5 font-semibold"><i data-lucide="${ic}" class="w-3.5 h-3.5"></i>${txt}</a>`;
  icons(el);
}

// ---------------------------------------------------------------- 轮询
function startPolling() {
  let full = null, lastWorld = '';
  const gen = W.gen;
  const pull = async () => {
    if (gen !== W.gen) return;
    const fast = W.tab === '32' || W.tab === '35';
    try {
      if (!full) {
        full = await iget('/api/telemetry?full=1');
        lastWorld = (full.scenario_metadata || {}).id + ':' + ((full.robot_spec || {}).model_rev);
      }
      const t = await iget(`/api/telemetry${W.tab === '32' ? '?scans=1' : ''}`);
      const wsig = (t.active_scenario || '') + ':' + ((t.robot_spec || {}).model_rev);
      if (wsig !== lastWorld) { full = null; }    // 场景/模型变化 → 下次取全量
      W.tel = Object.assign({}, full || {}, t, full ? { scenario_metadata: full.scenario_metadata, topo_graph: full.topo_graph, config: full.config } : {});
      $('#wb-dot') && ($('#wb-dot').className = 'w-2.5 h-2.5 rounded-full bg-emerald-500');
      renderSafety();
      Object.values(W.tabs).forEach(tb => tb.onTel && !tb.el.classList.contains('hidden') && tb.onTel(W.tel));
    } catch (e) { $('#wb-dot') && ($('#wb-dot').className = 'w-2.5 h-2.5 rounded-full bg-rose-500'); }
    setTimeout(pull, fast ? 100 : 500);
  };
  const tid = setTimeout(pull, 0); W.timers.push(tid);
  every(1000, async () => {
    try {
      const r = await iget(`/api/v2/events?after=${W.evAfter}&limit=200`);
      if (r.events.length) {
        W.evAfter = r.latest_id;
        r.events.forEach(e => { W.evTags[e.tag] = (W.evTags[e.tag] || 0) + 1; });
        W.events = W.events.concat(r.events).slice(-400);
        Object.values(W.tabs).forEach(tb => tb.onEvents && tb.onEvents(r.events));
      } else if (r.latest_id < W.evAfter) { W.evAfter = r.latest_id; }
    } catch (e) { }
  });
  every(2000, async () => {
    try {
      const [l, rec] = await Promise.all([iget('/api/v2/lock'), iget('/api/v2/records')]);
      if (!W.lock.mine || !l.mine) { W.lock = l; if (!l.mine && W.token) saveToken(l.mine ? W.token : ''); renderLock(); }
      W.stat = rec.stats; W.records = rec.records; renderRec();
      Object.values(W.tabs).forEach(tb => tb.onRecords && tb.onRecords(rec));
    } catch (e) { }
  });
  window.addEventListener('beforeunload', () => {
    if (W.lock.mine && W.token) navigator.sendBeacon && fetch(W.base + '/api/v2/lock', { method: 'DELETE', headers: { 'X-Lock-Token': W.token }, keepalive: true });
  });
}

// ---------------------------------------------------------------- 实例选择
async function picker() {
  leave();
  const d = await hub.get('/instances?active=1').catch(() => ({ instances: [] }));
  const act = d.instances.filter(i => ['running', 'degraded'].includes(i.status));
  root().innerHTML = `<div class="flex-1 overflow-y-auto p-8"><div class="max-w-5xl mx-auto">
    <h2 class="text-base font-bold mb-1">选择要进入的仿真实例</h2><p class="text-xs text-slate-500 mb-6">工作台操作同一实例的所有浏览器共享状态；控制权同一时刻只属于一位使用人。</p>
    <div class="grid grid-cols-2 gap-4">${act.map(i => `<a href="#/wb/${i.id}" class="card p-5 hover:border-blue-300 block">
      <div class="flex justify-between items-center"><span class="text-sm font-bold">${esc(i.id)} · ${esc(i.name)}</span>${badge(i.status)}</div>
      <div class="text-xs text-slate-500 mt-2">${esc(i.model_name || '')} · ${esc(i.scene_name || '')} · ${esc(i.sim_node_view?.host || i.web_url || '')}</div>
      <div class="text-xs text-slate-400 mt-1">使用人 ${esc(i.operator || '—')}${i.brief?.lock?.held ? ` · 控制中: ${esc(i.brief.lock.user)}` : ''}</div></a>`).join('') ||
    `<div class="col-span-2 card p-10 text-center text-sm text-slate-400">没有运行中的实例。<a href="#/platform/overview" class="text-blue-600">去资源平台新建仿真</a></div>`}</div>
    <div class="mt-8 card p-5 flex items-center justify-between"><div><div class="text-sm font-bold">离线回放</div><div class="text-xs text-slate-500">不需要运行中的实例，直接导入 sim_bundle (.json) 回放</div></div>
      <a href="#/wb/replay" class="btn-primary"><i data-lucide="film" class="w-3.5 h-3.5"></i>打开回放</a></div></div></div>`;
  icons(root());
}

async function openOffline(rid) {
  leave();
  W.iid = 'replay'; W.offline = true; W.base = '';
  root().innerHTML = `<div class="h-12 bg-white border-b border-slate-200 px-6 flex items-center justify-between shrink-0">
      <div class="text-xs font-bold text-slate-800 flex items-center gap-2"><i data-lucide="film" class="w-4 h-4 text-blue-600"></i>离线回放 (不连接仿真实例)</div>
      <a href="#/platform/records" class="btn-ghost">返回记录归档</a></div><div id="wb-body" class="flex-1 min-h-0 relative"></div>`;
  icons(root());
  const el = document.createElement('div'); el.className = 'absolute inset-0 flex flex-col'; $('#wb-body').appendChild(el);
  W.tabs['34'] = tab34(el) || { el }; W.tabs['34'].el = el; icons(el);
  if (rid) {
    try { const b = await fetch(`/api/hub/records/${rid}`).then(r => r.json()); openReplayRecord(b); } catch (e) { toast('读取记录失败: ' + e.message, 'err'); }
  }
}
