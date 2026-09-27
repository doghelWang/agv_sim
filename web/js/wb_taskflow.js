// ============================================================================
// 3-1 任务流配置: 任务流列表 + 工步编排 + 2D 地图 (SLAM 栅格 + 业务拓扑 + 任务动线)
// ============================================================================
import { $, $$, esc, hub, toast, guard, icons, confirmBox, STEP_LABEL } from './core.js';
import { W, ipost, needLock } from './wbcore.js';
import { MapCanvas, decodePGM, parseMapYaml, topoPath } from './viz2d.js';

const LOCAL = () => `amr.flows.${W.sceneId}`;

export async function loadFlows() {
  try {
    const r = await hub.get(`/scenes/${W.sceneId}/taskflows`);
    W.flowsRemote = true;
    W.flows = r.taskflows || [];
  } catch (e) {
    W.flowsRemote = false;
    try { W.flows = JSON.parse(localStorage.getItem(LOCAL()) || '[]'); } catch (x) { W.flows = []; }
  }
  if (!W.flowSel && W.flows[0]) W.flowSel = W.flows[0].id;
  return W.flows;
}
export async function saveFlows() {
  if (W.flowsRemote) await hub.put(`/scenes/${W.sceneId}/taskflows`, { taskflows: W.flows });
  else localStorage.setItem(LOCAL(), JSON.stringify(W.flows));
}
export function flowText(f) {
  const t = f.steps.filter(s => s.type === 'move').map(s => s.target);
  return t.length ? t.join(' -> ') : '无移动工步';
}

export function tab31(el) {
  el.innerHTML = `<div class="flex-1 flex min-h-0">
    <div class="w-[400px] bg-white border-r border-slate-200 flex flex-col shrink-0">
      <div class="px-4 py-3 border-b border-slate-100 flex items-center justify-between">
        <span class="panel-title"><i data-lucide="list-tree" class="w-4 h-4 text-blue-600"></i>任务流列表 <span data-n class="cnt">0</span></span>
        <button class="btn-soft !py-1" data-new><i data-lucide="plus" class="w-3.5 h-3.5"></i>新建任务流</button></div>
      <div data-list class="p-3 space-y-2 overflow-y-auto max-h-[34%] border-b border-slate-100"></div>
      <div data-editor class="flex-1 overflow-y-auto p-4"></div>
      <div class="p-3 border-t border-slate-200 grid grid-cols-2 gap-2 bg-slate-50">
        <button class="btn-ghost justify-center" data-save><i data-lucide="save" class="w-3.5 h-3.5"></i>保存任务流</button>
        <button class="btn-primary justify-center" data-send><i data-lucide="send" class="w-3.5 h-3.5"></i>下发至 3-2 仿真</button></div>
    </div>
    <div class="flex-1 flex flex-col min-w-0">
      <div class="h-11 bg-white border-b border-slate-200 px-4 flex items-center justify-between text-xs shrink-0">
        <div class="flex items-center gap-4"><span class="panel-title"><i data-lucide="map" class="w-4 h-4 text-blue-600"></i>仿真 2D 地图 (SLAM栅格底图 + 业务拓扑网络)</span>
          ${[['grid', '2D SLAM栅格'], ['topo', '业务拓扑路线'], ['labels', '站点标签'], ['path', '当前任务动线高亮']].map(([k, t]) => `<label class="flex items-center gap-1"><input type="checkbox" checked data-layer="${k}">${t}</label>`).join('')}</div>
        <div class="flex gap-1"><button class="btn-ghost !px-2.5" data-z="in">+</button><button class="btn-ghost !px-2.5" data-z="out">-</button><button class="btn-ghost" data-z="fit">居中重置</button></div></div>
      <div class="flex-1 relative" data-map>
        <div class="absolute bottom-4 left-4 z-10 bg-white/95 border border-slate-200 rounded-lg shadow-sm px-3 py-2 text-[11px] text-slate-600 space-y-1">
          <div class="flex items-center gap-2"><span class="w-2.5 h-2.5 rounded-full bg-blue-500"></span>待命与作业站点 (Station) · 点击可追加移动工步</div>
          <div class="flex items-center gap-2"><span class="w-4 border-t-2 border-dashed border-slate-400"></span>拓扑导引路段 (Topology Edge)</div>
          <div class="flex items-center gap-2"><span class="w-4 h-1 bg-cyan-500 rounded"></span>任务流动线 (按拓扑最短路预览)</div>
          <div class="flex items-center gap-2"><span class="w-3 h-2 bg-blue-600 rounded-sm"></span>AMR 实体当前实时位姿</div></div></div>
    </div></div>`;
  const map = new MapCanvas($('[data-map]', el), { onClick: ({ hit }) => { if (hit) addStep({ type: 'move', target: hit.id, speed: 1.0 }); } });
  let sceneSig = '';
  const cur = () => W.flows.find(f => f.id === W.flowSel);

  function renderList() {
    $('[data-n]', el).textContent = W.flows.length;
    $('[data-list]', el).innerHTML = W.flows.map(f => `<div data-f="${esc(f.id)}" class="p-3 rounded-lg border cursor-pointer ${f.id === W.flowSel ? 'border-blue-400 bg-blue-50/60 ring-1 ring-blue-200' : 'border-slate-200 hover:border-slate-300'}">
      <div class="flex justify-between items-center"><span class="text-xs font-bold ${f.id === W.flowSel ? 'text-blue-700' : 'text-slate-800'} truncate">${esc(f.name)}</span><span class="tag bg-slate-100 text-slate-600 border-slate-200">${esc(f.id)}</span></div>
      <div class="text-[11px] text-slate-500 mt-1 truncate">${esc(f.description || flowText(f))}</div>
      <div class="flex justify-between text-[11px] mt-2"><span class="text-slate-400">${f.steps.length} 个工步</span><span class="text-blue-600">${f.loop === 'infinite' ? '无限循环' : '单次'}</span></div></div>`).join('') ||
      '<div class="text-xs text-slate-400 p-3">暂无任务流，点击「新建任务流」</div>';
    $$('[data-f]', el).forEach(d => d.onclick = () => { W.flowSel = d.dataset.f; renderAll(); });
  }
  function targets() {
    const sts = (W.tel.scenario_metadata || {}).stations || [];
    const nodes = Object.keys((W.tel.topo_graph || {}).nodes || {});
    return { sts, nodes };
  }
  function renderEditor() {
    const f = cur(), E = $('[data-editor]', el);
    if (!f) { E.innerHTML = ''; return; }
    const { sts, nodes } = targets();
    const tOpt = (v) => `<optgroup label="工位">${sts.map(s => `<option value="${esc(s.id)}" ${s.id === v ? 'selected' : ''}>${esc(s.id)} (${esc(s.name)})</option>`).join('')}</optgroup>
      <optgroup label="拓扑节点">${nodes.map(n => `<option value="${esc(n)}" ${n === v ? 'selected' : ''}>${esc(n)}</option>`).join('')}</optgroup>`;
    E.innerHTML = `<div class="flex items-center justify-between mb-3"><span class="panel-title"><i data-lucide="pencil" class="w-4 h-4 text-blue-600"></i>任务流编排详情</span>
        <span class="flex gap-1"><span class="tag bg-blue-50 text-blue-700 border-blue-200">${esc(f.id)}</span><button class="text-slate-400 hover:text-rose-600" data-delflow title="删除任务流"><i data-lucide="trash-2" class="w-3.5 h-3.5"></i></button></span></div>
      <label class="lbl">任务流名称</label><input class="inp mb-3" data-k="name" value="${esc(f.name)}">
      <div class="grid grid-cols-2 gap-3 mb-4"><div><label class="lbl">任务ID标识 (TID)</label><input class="inp font-mono" data-k="tid" value="${esc(f.tid || '')}"></div>
        <div><label class="lbl">循环执行模式</label><select class="inp" data-k="loop"><option value="single" ${f.loop !== 'infinite' ? 'selected' : ''}>单次执行 (Single)</option><option value="infinite" ${f.loop === 'infinite' ? 'selected' : ''}>无限循环 (Infinite)</option></select></div></div>
      <div class="flex justify-between items-center mb-2"><span class="text-xs font-bold text-slate-700">工步动作时序序列 (Steps)</span>
        <button class="text-xs text-blue-600 font-semibold flex items-center gap-1" data-add><i data-lucide="plus-circle" class="w-3.5 h-3.5"></i>添加工步</button></div>
      <div class="space-y-2.5">${f.steps.map((s, i) => `<div class="border border-slate-200 rounded-lg p-3 bg-white" data-step="${i}">
        <div class="flex justify-between items-center mb-2"><span class="text-xs font-bold text-slate-800 flex items-center gap-2"><span class="w-5 h-5 rounded-full bg-blue-50 text-blue-600 text-[11px] flex items-center justify-center">${i + 1}</span>${esc(STEP_LABEL[s.type].split(' ')[0])}</span>
          <span class="flex gap-1 text-slate-400"><button data-up="${i}" title="上移"><i data-lucide="chevron-up" class="w-3.5 h-3.5"></i></button><button data-rm="${i}" class="hover:text-rose-600" title="删除"><i data-lucide="x" class="w-3.5 h-3.5"></i></button></span></div>
        <div class="grid grid-cols-2 gap-2"><div><label class="lbl !mb-1">工步类型</label><select class="inp !py-1.5" data-s="type">${Object.entries(STEP_LABEL).map(([k, v]) => `<option value="${k}" ${k === s.type ? 'selected' : ''}>${v}</option>`).join('')}</select></div>
          ${s.type === 'wait' ? `<div><label class="lbl !mb-1">等待时长 (s)</label><input class="inp !py-1.5 font-mono" type="number" step="0.5" data-s="seconds" value="${s.seconds ?? 3}"></div>`
        : `<div><label class="lbl !mb-1">目标点位</label><select class="inp !py-1.5 font-mono" data-s="target"><option value="">${s.type === 'move' ? '— 请选择 —' : '(当前位置)'}</option>${tOpt(s.target)}</select></div>`}</div>
        ${s.type === 'move' ? `<div class="flex items-center gap-2 mt-2 text-xs text-slate-500">设定线速: <input class="inp !w-20 !py-1 font-mono" type="number" step="0.1" min="0.1" data-s="speed" value="${s.speed ?? 1.0}"> m/s</div>` : ''}</div>`).join('')}</div>`;
    icons(E);
    $$('[data-k]', E).forEach(i => i.onchange = () => { f[i.dataset.k] = i.value; renderList(); });
    $$('[data-step]', E).forEach(box => {
      const i = +box.dataset.step;
      $$('[data-s]', box).forEach(inp => inp.onchange = () => {
        const k = inp.dataset.s; f.steps[i][k] = ['speed', 'seconds'].includes(k) ? parseFloat(inp.value) : inp.value;
        if (k === 'type') { if (inp.value === 'wait') delete f.steps[i].target; renderEditor(); }
        renderPath(); renderList();
      });
    });
    $$('[data-rm]', E).forEach(b => b.onclick = () => { f.steps.splice(+b.dataset.rm, 1); renderAll(); });
    $$('[data-up]', E).forEach(b => b.onclick = () => { const i = +b.dataset.up; if (i > 0) { [f.steps[i - 1], f.steps[i]] = [f.steps[i], f.steps[i - 1]]; renderAll(); } });
    $('[data-add]', E).onclick = () => addStep({ type: 'move', target: '', speed: 1.0 });
    $('[data-delflow]', E).onclick = async () => { if (!await confirmBox(`删除任务流 ${f.name}？`)) return; W.flows = W.flows.filter(x => x !== f); W.flowSel = W.flows[0]?.id; await saveFlows(); renderAll(); };
  }
  function addStep(s) {
    const f = cur(); if (!f) return;
    f.steps.push(s); renderAll();
    if (s.target) toast(`已追加工步: 导航移动 → ${s.target}`, 'info');
  }
  function renderPath() {
    const f = cur(), sc = W.tel.scenario_metadata, topo = W.tel.topo_graph;
    if (!f || !sc || !topo) { map.taskPath = []; map.draw(); return; }
    let p = { x: W.tel.x ?? sc.origin?.x ?? 0, y: W.tel.y ?? sc.origin?.y ?? 0 };
    const pts = [p]; map.highlight = new Set();
    const stops = [p];
    f.steps.filter(s => s.type === 'move' && s.target).forEach(s => {
      const seg = topoPath(topo, sc, p, s.target); pts.push(...seg.slice(1)); p = seg[seg.length - 1]; map.highlight.add(s.target);
      stops.push(p);
    });
    map.taskPath = pts; map.draw();
    // 用执行进程的规划器 (含车体过弯可行性) 校正预览；失败则保留前端预览
    const gen = (renderPath.gen = (renderPath.gen || 0) + 1);
    if (stops.length > 1) ipost('/api/v2/plan', { points: stops }).then(r => {
      if (gen !== renderPath.gen || !r || !r.legs || r.legs.length !== stops.length - 1 || r.legs.some(l => l.points.length < 2)) return;
      const out = [stops[0]]; r.legs.forEach(l => out.push(...l.points.slice(1)));
      map.taskPath = out; map.draw();
    }).catch(() => { });
  }
  function renderAll() { renderList(); renderEditor(); renderPath(); }
  function ensureScene() {
    const sc = W.tel.scenario_metadata;
    if (!sc) return;
    const sig = sc.id + ':' + (sc.walls || []).length;
    if (sig === sceneSig) return;
    sceneSig = sig;
    map.setScene(sc, W.tel.topo_graph);
    map.footprint = W.tel.robot_spec?.footprint;
    Promise.all([fetch(`${W.base}/sim/api/v1/world/map?part=pgm`).then(r => r.arrayBuffer()), fetch(`${W.base}/sim/api/v1/world/map?part=yaml`).then(r => r.text())])
      .then(([p, y]) => map.setGrid(decodePGM(p), parseMapYaml(y))).catch(() => { });
    renderAll();
  }

  $('[data-new]', el).onclick = () => {
    const n = W.flows.length + 1;
    const id = `TF-${String(n).padStart(2, '0')}` + (W.flows.some(f => f.id === `TF-${String(n).padStart(2, '0')}`) ? '-' + Date.now() % 1000 : '');
    const f = { id, name: `新建任务流 ${n}`, tid: String(26460 + n), loop: 'single', description: '', steps: [] };
    W.flows.push(f); W.flowSel = id; renderAll();
  };
  $('[data-save]', el).onclick = guard(async () => { const f = cur(); if (f) f.description = flowText(f); await saveFlows(); toast(W.flowsRemote ? '任务流已保存到平台场景库' : '任务流已保存 (本地浏览器)', 'ok'); renderList(); });
  $('[data-send]', el).onclick = guard(async () => {
    const f = cur(); if (!f) return;
    if (f.steps.some(s => s.type === 'move' && !s.target)) throw new Error('有移动工步未选择目标点位');
    f.description = flowText(f); await saveFlows();
    W.pendingFlow = f.id;
    location.hash = `#/wb/${W.iid}/32`;
    toast(`已选中 ${f.id}，在 3-2 点「下发任务」开始执行`, 'info');
  });
  $$('[data-layer]', el).forEach(c => c.onchange = () => { map.layers[c.dataset.layer] = c.checked; map.draw(); });
  $$('[data-z]', el).forEach(b => b.onclick = () => b.dataset.z === 'fit' ? map.reset() : map.zoom(b.dataset.z === 'in' ? 1.25 : 0.8));

  loadFlows().then(renderAll);
  return {
    onTel(t) {
      ensureScene();
      map.robot = { x: t.x, y: t.y, yaw: t.yaw }; map.plan = ['ARRIVED', 'CANCELED', 'FAILED', 'NO_PATH', 'IDLE'].includes(t.nav_status) ? [] : (t.plan_path || []); map.planIndex = t.path_index; map.planLabels = t.path_labels; map.planCurve = t.plan_curve; map.protection = t.protection; map.obstacles = t.dynamic_obstacles || [];
      map.draw();
    },
    show() { ensureScene(); renderAll(); },
    destroy() { map.destroy(); },
  };
}
