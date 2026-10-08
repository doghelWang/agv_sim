// ============================================================================
// 仿真资源管理平台: 总览/实例、计算资源、车辆模型、仿真场景、软件程序包、记录归档 + 部署向导
// ============================================================================
import { $, $$, esc, fmt, hub, http, upload, download, toast, guard, icons, modal, confirmBox, badge, dur, bytes, ago, dt,
  user, CHASSIS_LABEL, sleep } from './core.js';
import { drawSceneThumb, modelTopSVG, decodePGM, parseMapYaml } from './viz2d.js';
import { ModelView, SceneView } from './viz3d.js';
import * as Ops from './ops.js';

const S = { page: null, timer: null, overview: null };
const main = () => $('#platform-main');
export function setOverview(o) { S.overview = o; }
export function leave() { clearInterval(S.timer); S.timer = null; Ops.leave(); }

export async function show(page) {
  leave();
  S.page = page;
  const fn = { overview: pOverview, compute: pCompute, models: pModels, scenes: pScenes, software: pSoftware, records: pRecords, ops: () => Ops.render(main()) }[page] || pOverview;
  await guard(fn)();
  icons(main());
  if (page === 'overview' || page === 'compute') {
    S.timer = setInterval(() => { if (S.page === page) guard(fn)(true).then(() => icons(main())); }, 3000);
  }
}

function header(title, actions = '') {
  return `<div class="flex items-center justify-between pb-4 mb-5 border-b border-slate-200">
    <h2 class="text-base font-bold text-slate-800">${title}</h2><div class="flex items-center gap-2">${actions}</div></div>`;
}
function statCard(label, icon, color, value, unit, sub) {
  return `<div class="card p-5 flex flex-col gap-3">
    <div class="flex items-center justify-between"><span class="text-xs text-slate-500">${label}</span>
      <div class="w-9 h-9 rounded-lg bg-${color}-50 text-${color}-600 flex items-center justify-center"><i data-lucide="${icon}" class="w-4 h-4"></i></div></div>
    <div class="flex items-baseline gap-2"><span class="text-2xl font-bold font-mono text-slate-900">${value}</span><span class="text-xs text-slate-500">${unit}</span></div>
    <div class="text-[11px] text-slate-500">${sub}</div></div>`;
}

// ============================================================================ 总览 / 实例
async function pOverview(refresh) {
  const [o, ins] = await Promise.all([hub.get('/overview'), hub.get('/instances')]);
  const act = ins.instances.filter(i => ['deploying', 'running', 'degraded', 'stopping'].includes(i.status));
  const hist = ins.instances.filter(i => !act.includes(i));
  const kinds = o.nodes.by_kind || {};
  const chs = Object.entries(o.models.by_chassis || {}).map(([k, v]) => `${k}*${v}`).join(' · ') || '—';
  const html = `
    <div class="grid grid-cols-4 gap-4 mb-7">
      ${statCard('计算资源节点', 'server', 'blue', o.nodes.total, '台', `<span class="text-emerald-600">${o.nodes.idle} 空闲</span> · 实体${kinds.controller || 0} / 混合${kinds.hybrid || 0} / 仿真${kinds.sim || 0}`)}
      ${statCard('车辆模型 (cmodel)', 'truck', 'purple', o.models.total, '个', chs)}
      ${statCard('仿真场景', 'map', 'emerald', o.scenes.total, '个可用', esc((o.scenes.names || []).slice(0, 3).join(' · ')))}
      ${statCard('软件程序包', 'package', 'amber', o.packages.total, '个版本', `运行程序 ${o.packages.nav} · 仿真引擎 ${o.packages.sim}`)}
    </div>
    ${header('正在进行的仿真任务', `<button class="btn-primary" data-act="new"><i data-lucide="plus" class="w-3.5 h-3.5"></i>新建仿真</button>`)}
    <div class="grid grid-cols-2 gap-5">${act.map(instCard).join('') || `<div class="col-span-2 card p-10 text-center text-sm text-slate-400">
      暂无运行中的仿真。点击「新建仿真」选择设备、车辆模型、场景与软件版本，一键部署。</div>`}</div>
    ${hist.length ? `<div class="mt-8">${header('历史实例', '')}
      <div class="card overflow-hidden"><table class="w-full text-xs"><thead class="bg-slate-50 text-slate-500"><tr>
        <th class="text-left px-4 py-2.5">实例</th><th class="text-left px-4">车辆模型</th><th class="text-left px-4">场景</th><th class="text-left px-4">节点</th>
        <th class="text-left px-4">使用人</th><th class="text-left px-4">创建</th><th class="text-left px-4">状态</th><th class="text-right px-4">操作</th></tr></thead>
        <tbody>${hist.slice(0, 20).map(i => `<tr class="border-t border-slate-100">
          <td class="px-4 py-2.5 font-semibold">${esc(i.id)} · ${esc(i.name)}</td><td class="px-4">${esc(i.model_name || '—')}</td><td class="px-4">${esc(i.scene_name || '—')}</td>
          <td class="px-4 font-mono">${esc(i.sim_node_view?.name || '外部')}${i.nav_node && i.nav_node !== i.sim_node ? ' / ' + esc(i.nav_node_view?.name) : ''}</td>
          <td class="px-4">${esc(i.operator || '—')}</td><td class="px-4 font-mono text-slate-500">${dt(i.created)}</td>
          <td class="px-4">${badge(i.status)}${i.error ? `<div class="text-[10px] text-rose-600 mt-1 max-w-xs truncate" title="${esc(i.error)}">${esc(i.error)}</div>` : ''}</td>
          <td class="px-4 text-right whitespace-nowrap">${i.external ? '' : `<button class="btn-ghost !py-1" data-redeploy="${i.id}">重新部署</button>`}
            <button class="btn-ghost !py-1" data-steps="${i.id}">详情</button><button class="btn-danger !py-1" data-del="${i.id}">删除</button></td></tr>`).join('')}</tbody></table></div></div>` : ''}`;
  main().innerHTML = html;
  $('[data-act=new]', main()).onclick = () => openDeploy();
  $$('[data-enter]', main()).forEach(b => b.onclick = () => { location.hash = `#/wb/${b.dataset.enter}`; });
  $$('[data-stop]', main()).forEach(b => b.onclick = guard(async () => {
    if (!await confirmBox(`终止仿真实例 ${b.dataset.stop}？两个容器将被删除，仿真记录会先归档到平台。`, '终止仿真')) return;
    await hub.post(`/instances/${b.dataset.stop}/stop`); toast('已终止', 'ok'); show('overview');
  }));
  $$('[data-steps]', main()).forEach(b => b.onclick = () => openProgress(b.dataset.steps));
  $$('[data-restart]', main()).forEach(b => b.onclick = guard(async () => {
    if (!await confirmBox(`重启仿真实例 ${b.dataset.restart}？仿真与执行容器会重新部署，当前任务中断。`, '重启')) return;
    await hub.post(`/instances/${b.dataset.restart}/restart`); toast('正在重新部署', 'ok'); show('overview');
  }));
  $$('[data-redeploy]', main()).forEach(b => b.onclick = guard(async () => { await hub.post(`/instances/${b.dataset.redeploy}/restart`); openProgress(b.dataset.redeploy); }));
  $$('[data-del]', main()).forEach(b => b.onclick = guard(async () => { await hub.del(`/instances/${b.dataset.del}`); show('overview'); }));
}

// 卡片上的状态只分三种: 部署中 / 运行中 / 已停止
const CARD_ST = { deploying: ['部署中', 'bg-blue-50 text-blue-700 border-blue-200', 'bg-blue-500 animate-pulse'],
  running: ['运行中', 'bg-emerald-50 text-emerald-700 border-emerald-200', 'bg-emerald-500'],
  stopped: ['已停止', 'bg-slate-100 text-slate-600 border-slate-200', 'bg-slate-400'] };
function cardState(st) {
  const k = st === 'deploying' || st === 'starting' ? 'deploying' : st === 'running' || st === 'degraded' ? 'running' : 'stopped';
  const [t, c, d] = CARD_ST[k];
  return `<span class="px-2 py-0.5 rounded-full text-[11px] font-bold border ${c} inline-flex items-center gap-1 whitespace-nowrap"><span class="w-1.5 h-1.5 rounded-full ${d}"></span>${t}</span>`;
}

function instCard(i) {
  const run = i.status === 'running' || i.status === 'degraded';
  const sn = i.sim_node_view, nn = i.nav_node_view;
  const split = i.nav_node && i.nav_node !== i.sim_node;
  const kindTag = (v) => v ? `<span class="text-[10px] px-1.5 py-0.5 rounded ${v.kind === 'controller' ? 'bg-slate-100 text-slate-600' : 'bg-purple-50 text-purple-600'}">${esc({ controller: '实体控制器', hybrid: '虚拟机(运行+仿真)', sim: '虚拟机(仿真)' }[v.kind] || '')}</span>` : '';
  return `<div class="card p-5 flex flex-col">
    <div class="flex items-center justify-between pb-3 border-b border-slate-100">
      <div class="flex items-center gap-2 min-w-0"><h3 class="text-sm font-bold text-slate-800 truncate">仿真 ${esc(i.id)}: ${esc(i.name)}</h3>${cardState(i.status)}</div>
      <span class="text-xs text-slate-500 font-mono whitespace-nowrap">时长: ${dur(Date.now() / 1000 - (i.started || i.created))}</span></div>
    <div class="grid grid-cols-2 gap-x-6 gap-y-2 py-4 text-xs">
      <div>• 仿真节点: <span class="font-mono">${esc(sn?.host || '外部')}</span> ${kindTag(sn)}</div>
      <div>• 车辆模型: ${esc(i.model_name || '—')}${i.model_ver ? ` <span class="text-slate-400 font-mono">${esc(i.model_ver)}</span>` : ''}</div>
      <div>• 执行节点: <span class="font-mono">${esc(nn?.host || '—')}</span> ${split ? '<span class="text-[10px] px-1.5 py-0.5 rounded bg-blue-50 text-blue-600">分离部署</span>' : kindTag(nn)}</div>
      <div>• 仿真场景: ${esc(i.scene_name || '—')}</div>
      <div>• 程序包: <span class="font-mono text-[11px]">${esc(i.sim_pkg_view?.version || '—')} / ${esc(i.nav_pkg_view?.version || '—')}</span></div>
      <div>• 使用人员: ${esc(i.operator || '—')}${i.brief?.lock?.held ? ` <span class="text-[10px] text-emerald-600">(控制中: ${esc(i.brief.lock.user)})</span>` : ''}</div>
    </div>
    ${i.health && !(i.health.sim && i.health.nav) ? `<div class="text-[11px] text-amber-700 bg-amber-50 border border-amber-200 rounded px-3 py-1.5 mb-3">健康检查: 仿真 ${i.health.sim ? '正常' : '无响应'} · 执行 ${i.health.nav ? '正常' : '无响应'} · 网关 ${i.health.web ? '正常' : '无响应'}</div>` : ''}
    <div class="mt-auto pt-3 border-t border-slate-100 flex items-center justify-end gap-1.5 flex-wrap">
      ${i.external ? '' : `<a class="btn-ghost" title="下载仿真引擎 (agv-sim) 运行日志文件" href="/api/hub/instances/${encodeURIComponent(i.id)}/logs/file?svc=sim" download><i data-lucide="download" class="w-3.5 h-3.5"></i>仿真引擎日志</a>
        <button class="btn-ghost !px-2" title="重启" data-restart="${i.id}"><i data-lucide="rotate-cw" class="w-3.5 h-3.5"></i></button>`}
      <button class="btn-danger" data-stop="${i.id}"><i data-lucide="square" class="w-3.5 h-3.5"></i>终止仿真</button>
      ${run ? `<button class="btn-primary" data-enter="${i.id}"><i data-lucide="play-circle" class="w-3.5 h-3.5"></i>进入仿真控制界面</button>` : ''}
    </div></div>`;
}

// ============================================================================ 部署向导
export async function openDeploy(pre = {}) {
  const [nodes, models, scenes, pkgs] = await Promise.all([hub.get('/nodes'), hub.get('/models'), hub.get('/scenes'), hub.get('/packages')]).catch(e => { toast(e.message, 'err'); return []; });
  if (!nodes) return;
  const N = nodes.nodes, sel = { split: false, ...pre };
  const nodeOpt = (role) => N.filter(n => n.caps[role]).map(n => `<option value="${n.id}" ${n.online || n.pending ? '' : 'disabled'}>${esc(n.host)} - ${esc(n.name)} (${n.cpu_count || '?'}核/${n.mem_total ? Math.round(n.mem_total / 1073741824) + 'G' : '?'}, ${n.pending ? '🔵 待部署' : n.online ? (n.status === 'idle' ? '🟢 空闲' : '🟡 运行中') : '⚪ 离线'}${n.arch ? ', ' + n.arch : ''})</option>`).join('');
  const pk = (k) => pkgs.packages.filter(p => p.kind === k).map(p => `<option value="${p.id}">${esc(p.version)} (${esc({ upload: '平台仓库', node: '节点镜像', process: '源码' }[p.source])}${p.tag === 'baseline' ? ' · 基准推荐' : ''}${p.note ? ' · ' + esc(p.note).slice(0, 24) : ''})</option>`).join('');
  const step = (n, t, inner) => `<div class="mb-5"><div class="flex items-center gap-2 mb-2"><span class="w-5 h-5 rounded-full bg-blue-50 text-blue-600 text-[11px] font-bold flex items-center justify-center">${n}</span><span class="text-xs font-bold text-slate-800">${t}</span></div>${inner}</div>`;
  const m = modal({
    title: '新建仿真环境部署', icon: 'rocket', size: 'max-w-3xl',
    body: `
      ${step(1, '选取计算设备', `<label class="lbl">仿真节点 (运行 agv-sim 仿真程序包)</label><select class="inp mb-2" data-k="sim_node">${nodeOpt('sim')}</select>
        <label class="flex items-center gap-2 text-xs text-slate-600 my-2"><input type="checkbox" data-k="split"> 运行程序分离部署 (仿真引擎与导航执行程序部署到不同设备)</label>
        <div data-navwrap class="hidden"><label class="lbl">执行节点 (运行 agv-nav 运行程序包)</label><select class="inp" data-k="nav_node">${nodeOpt('nav')}</select></div>`)}
      ${step(2, '选取车辆模型', `<select class="inp" data-k="model_id">${models.models.map(m => `<option value="${m.id}">${esc(m.name)} (${esc(m.vtype || m.summary?.chassis_label || '')}${m.material_no ? ' / ' + esc(m.material_no) : ''}) · ${esc(m.latest)}</option>`).join('')}</select>`)}
      ${step(3, '选取仿真场景', `<select class="inp" data-k="scene_id">${scenes.scenes.map(s => `<option value="${s.id}">${esc(s.name)} (${s.summary?.size?.[0]}m × ${s.summary?.size?.[1]}m · ${s.summary?.stations} 工位)</option>`).join('')}</select>`)}
      <div class="border-t border-slate-100 pt-4">${step(4, '选取软件版本程序包', `<div class="grid grid-cols-2 gap-3">
        <div><label class="lbl">运行程序包 (Controller / Navigation)</label><select class="inp" data-k="nav_pkg">${pk('nav')}</select></div>
        <div><label class="lbl">仿真程序包 (Simulation Engine)</label><select class="inp" data-k="sim_pkg">${pk('sim')}</select></div></div>`)}</div>
      <div data-check class="text-xs"></div>`,
    footer: `<button data-close class="btn-ghost">取消</button><button data-go class="btn-primary !px-5">部署并启动仿真</button>`,
  });
  const get = () => {
    const b = {}; $$('[data-k]', m.el).forEach(e => b[e.dataset.k] = e.type === 'checkbox' ? e.checked : e.value);
    if (!b.split) b.nav_node = b.sim_node;
    return b;
  };
  Object.entries(pre).forEach(([k, v]) => { const e = $(`[data-k=${k}]`, m.el); if (e) e.value = v; });
  const pickPkg = () => {   // 默认: 基准推荐 → 选中节点上已有的 → 第一个
    ['sim', 'nav'].forEach(k => {
      const node = get()[k === 'sim' ? 'sim_node' : 'nav_node'];
      const cands = pkgs.packages.filter(p => p.kind === k);
      const best = cands.find(p => p.tag === 'baseline') || cands.find(p => (p.nodes || []).includes(node)) || cands[0];
      if (best) $(`[data-k=${k}_pkg]`, m.el).value = best.id;
    });
  };
  pickPkg();
  const check = guard(async () => {
    const r = await hub.post('/deployments/check', get());
    $('[data-check]', m.el).innerHTML = (r.issues.map(t => `<div class="flex gap-2 text-rose-600 bg-rose-50 border border-rose-200 rounded px-3 py-1.5 mb-1.5"><i data-lucide="x-circle" class="w-3.5 h-3.5 mt-px shrink-0"></i>${esc(t)}</div>`).join('') +
      r.warnings.map(t => `<div class="flex gap-2 text-amber-700 bg-amber-50 border border-amber-200 rounded px-3 py-1.5 mb-1.5"><i data-lucide="alert-triangle" class="w-3.5 h-3.5 mt-px shrink-0"></i>${esc(t)}</div>`).join('')) ||
      `<div class="flex gap-2 text-emerald-700 bg-emerald-50 border border-emerald-200 rounded px-3 py-1.5"><i data-lucide="check-circle-2" class="w-3.5 h-3.5 mt-px"></i>校验通过${r.split ? '：仿真与执行分离部署' : '：同机部署'}</div>`;
    $('[data-go]', m.el).disabled = !r.ok;
    icons(m.el);
  });
  $$('[data-k]', m.el).forEach(e => e.addEventListener('change', () => {
    if (e.dataset.k === 'split') $('[data-navwrap]', m.el).classList.toggle('hidden', !e.checked);
    if (['sim_node', 'nav_node', 'split'].includes(e.dataset.k)) pickPkg();
    check();
  }));
  check();
  $('[data-go]', m.el).onclick = guard(async () => {
    const r = await hub.post('/instances', get());
    m.close();
    openProgress(r.id, true);
  });
}

async function openProgress(iid, autoEnter = false) {
  const m = modal({ title: `部署进度 · 实例 ${iid}`, icon: 'loader', size: 'max-w-2xl', body: `<div data-p>加载中…</div>`,
    footer: `<span data-st class="text-xs text-slate-500"></span><div class="flex gap-2"><button data-close class="btn-ghost">关闭</button><button data-enter class="btn-primary hidden">进入工作台</button></div>` });
  let alive = true, entered = false;
  const ico = { done: ['check-circle-2', 'text-emerald-600'], running: ['loader', 'text-blue-600 animate-spin'], error: ['x-circle', 'text-rose-600'], skipped: ['minus-circle', 'text-slate-400'], pending: ['circle', 'text-slate-300'] };
  while (alive && document.body.contains(m.el)) {
    try {
      const i = await hub.get(`/instances/${iid}`);
      $('[data-p]', m.el).innerHTML = `
        <div class="text-xs text-slate-500 mb-4">${esc(i.name)} · 仿真 <b>${esc(i.sim_node_view?.name || '外部')}</b>${i.nav_node !== i.sim_node ? ` · 执行 <b>${esc(i.nav_node_view?.name || '')}</b>` : ''}</div>
        <div class="space-y-2">${(i.steps || []).map(s => {
          const [ic, c] = ico[s.status] || ico.pending;
          const t = s.t0 ? ((s.t1 || Date.now() / 1000) - s.t0).toFixed(1) + ' s' : '';
          return `<div class="flex items-start gap-3 p-2.5 rounded-lg ${s.status === 'running' ? 'bg-blue-50' : s.status === 'error' ? 'bg-rose-50' : ''}">
            <i data-lucide="${ic}" class="w-4 h-4 mt-0.5 ${c}"></i><div class="flex-1 min-w-0"><div class="flex justify-between text-xs"><span class="font-semibold text-slate-700">${esc(s.label)}</span><span class="font-mono text-slate-400">${t}</span></div>
            ${s.msg ? `<div class="text-[11px] ${s.status === 'error' ? 'text-rose-600 whitespace-pre-wrap font-mono' : 'text-slate-500'} mt-0.5 break-all">${esc(s.msg)}</div>` : ''}
            ${s.progress != null && s.status === 'running' ? `<div class="h-1.5 bg-blue-100 rounded mt-1.5"><div class="h-1.5 bg-blue-600 rounded" style="width:${s.progress}%"></div></div>` : ''}</div></div>`;
        }).join('') || '<div class="text-xs text-slate-500">外部接入实例，无部署步骤</div>'}</div>
        ${i.warnings?.length ? `<div class="mt-4 text-[11px] text-amber-700">${i.warnings.map(esc).join('<br>')}</div>` : ''}`;
      $('[data-st]', m.el).innerHTML = badge(i.status);
      icons(m.el);
      const ok = i.status === 'running';
      $('[data-enter]', m.el).classList.toggle('hidden', !ok);
      $('[data-enter]', m.el).onclick = () => { m.close(); location.hash = `#/wb/${iid}`; };
      if (ok && autoEnter && !entered) { entered = true; await sleep(1200); if (document.body.contains(m.el)) { m.close(); location.hash = `#/wb/${iid}`; } return; }
      if (['error', 'stopped', 'running'].includes(i.status) && !autoEnter) alive = i.status === 'running' ? false : false;
    } catch (e) { $('[data-st]', m.el).textContent = e.message; }
    await sleep(1200);
  }
}

// ============================================================================ 计算资源
const KINDS = [['controller', 'a、实体运行设备 (控制器)', 'bg-blue-600'], ['hybrid', 'b、虚拟机设备 (运行 + 仿真引擎)', 'bg-purple-600'], ['sim', 'c、虚拟机设备 (仿真引擎)', 'bg-emerald-600']];
async function pCompute() {
  const d = await hub.get('/nodes');
  const card = (n) => {
    const st = n.pending ? `<span class="px-2 py-0.5 rounded-md text-[11px] font-bold bg-blue-50 text-blue-700 border border-blue-200">🔵 待部署</span>`
      : n.status === 'offline' ? badge('offline') : n.status === 'running'
      ? `<span class="px-2 py-0.5 rounded-md text-[11px] font-bold bg-amber-50 text-amber-700 border border-amber-200">🟡 运行中 (${esc(n.instances.map(i => i.operator || i.id).join(','))})</span>`
      : `<span class="px-2 py-0.5 rounded-md text-[11px] font-bold bg-emerald-50 text-emerald-700 border border-emerald-200">🟢 空闲 (CPU ${fmt(n.cpu_percent, 0)}%)</span>`;
    return `<div class="card p-4 hover:border-blue-300 cursor-pointer transition-colors" data-node="${n.id}">
      <div class="flex justify-between items-start"><div class="min-w-0"><div class="text-sm font-bold font-mono text-slate-800">${esc(n.lan_host)}</div>
        <div class="text-xs text-slate-600 mt-0.5">${esc(n.name)} (${n.cpu_count || '?'}核/${n.mem_total ? Math.round(n.mem_total / 1073741824) + 'G' : '?'}) · ${esc(n.arch || '')}</div></div>${st}</div>
      ${n.pending ? `<div class="text-[11px] text-slate-500 mt-2 truncate">SSH ${esc(n.ssh?.user || '')}@${esc(n.ssh?.ip || '')}:${n.ssh?.port || ''} 验证通过 (${ago(n.ssh_check?.at)}) · ${esc(n.ssh_check?.brief || '')}</div>
      <div class="text-[11px] text-blue-600 mt-1">尚未安装运行环境，第一次部署仿真到该节点时自动安装</div></div>` : `
      <div class="text-[11px] text-slate-400 mt-2 truncate">系统: ${esc(n.os || '—')} | ${esc(n.runtime?.runtime || '—')} ${esc(n.runtime?.version || '')} | 代理端口: ${n.api_port}${n.temp_c ? ` | ${n.temp_c}°C` : ''}</div>
      <div class="text-[11px] text-slate-400 mt-1">镜像: ${n.images.map(i => esc(i.ref)).join(', ') || '无'}${n.agent_error ? ` · <span class="text-rose-500">${esc(n.agent_error)}</span>` : ''}</div></div>`}`;
  };
  main().innerHTML = header('计算资源池', `<button class="btn-primary" data-add><i data-lucide="plus" class="w-3.5 h-3.5"></i>添加计算节点</button>`) +
    KINDS.map(([k, t, c]) => {
      const ns = d.nodes.filter(n => n.kind === k);
      return `<div class="mb-6"><div class="flex items-center gap-2 mb-3 text-xs font-bold text-slate-700"><span class="w-2 h-2 rounded-full ${c}"></span>${t} [共 ${ns.length} 台]</div>
        <div class="grid grid-cols-2 gap-4">${ns.map(card).join('') || '<div class="text-xs text-slate-400 col-span-2 pl-4">暂无</div>'}</div></div>`;
    }).join('');
  $('[data-add]', main()).onclick = openAddNode;
  $$('[data-node]', main()).forEach(e => e.onclick = () => openNode(e.dataset.node));
}

function openAddNode() {
  const m = modal({ title: '添加计算资源节点', icon: 'server', size: 'max-w-2xl',
    body: `<div class="grid grid-cols-2 gap-3"><div><label class="lbl">计算资源类型</label><select class="inp" data-f="kind">
        <option value="hybrid">虚拟机/树莓派 (运行 + 仿真引擎)</option><option value="controller">实体运行设备 (控制器)</option><option value="sim">虚拟机设备 (仿真引擎)</option></select></div>
      <div><label class="lbl">备注</label><input class="inp" data-f="note" placeholder="例如: 2 号树莓派"></div>
      <div><label class="lbl">节点 IP</label><input class="inp font-mono" data-f="ip" placeholder="192.168.1.20" autocomplete="off"></div>
      <div><label class="lbl">部署端口 (SSH)</label><input class="inp font-mono" data-f="ssh_port" type="number" min="1" max="65535" value="22"></div>
      <div><label class="lbl">用户名</label><input class="inp font-mono" data-f="username" autocomplete="off"></div>
      <div><label class="lbl">密码</label><input class="inp font-mono" data-f="password" type="password" autocomplete="new-password"></div></div>
      <div class="text-[11px] text-slate-500 leading-relaxed mt-3">只验证 SSH 端口与账号能否登录，验证通过即保存，不在目标设备上安装任何东西。第一次部署仿真到该节点时，平台再用这个账号安装运行环境 (目标设备需要 python3 3.8 以上和 Docker)。密码加密保存在平台本机，不会在页面或接口中显示。</div>
      <div data-res class="mt-3 text-xs"></div>`,
    footer: `<button data-close class="btn-ghost">取消</button><button data-ok class="btn-primary">验证并保存</button>` });
  const res = $('[data-res]', m.el), ok = $('[data-ok]', m.el);
  ok.onclick = async () => {
    const b = {}; $$('[data-f]', m.el).forEach(i => b[i.dataset.f] = i.value.trim());
    b.password = $('[data-f=password]', m.el).value;
    if (!b.ip || !b.username || !b.password) { res.innerHTML = '<div class="text-rose-600">请填写节点 IP、用户名和密码</div>'; return; }
    ok.disabled = true;
    res.innerHTML = `<div class="flex items-center gap-2 text-blue-600"><i data-lucide="loader" class="w-3.5 h-3.5 animate-spin"></i>正在验证 ${esc(b.ip)}:${esc(b.ssh_port || '22')} 的 SSH 登录…</div>`; icons(m.el);
    try {
      const r = await hub.post('/nodes/add', b);
      const n = r.node;
      res.innerHTML = `<div class="flex items-center gap-2 text-emerald-700"><i data-lucide="check-circle-2" class="w-3.5 h-3.5"></i>SSH 验证通过，节点 ${esc(n.name)} 已保存: ${esc(r.check || '')}</div>`;
      ok.textContent = '完成'; ok.disabled = false; ok.onclick = () => m.close();
      show('compute');
    } catch (e) {
      res.innerHTML = `<div class="flex gap-2 text-rose-600 bg-rose-50 border border-rose-200 rounded px-3 py-1.5 whitespace-pre-wrap break-all"><i data-lucide="x-circle" class="w-3.5 h-3.5 mt-px shrink-0"></i>${esc(e.message)}</div>`;
      ok.disabled = false;
    }
    icons(m.el);
  };
}

async function openNode(nid) {
  const n = await hub.get(`/nodes/${nid}`);
  const spark = (idx, color, max = 100) => {
    const h = n.history || []; if (h.length < 2) return '<div class="text-[11px] text-slate-400">采样中…</div>';
    const pts = h.map((r, i) => `${(i / (h.length - 1) * 300).toFixed(1)},${(50 - (r[idx] || 0) / max * 48).toFixed(1)}`).join(' ');
    return `<svg viewBox="0 0 300 52" class="w-full h-14"><polyline points="${pts}" fill="none" stroke="${color}" stroke-width="1.5"/></svg>`;
  };
  const m = modal({ title: `计算节点 · ${esc(n.name)}`, icon: 'server', size: 'max-w-4xl', badge: badge(n.status),
    body: `<div class="grid grid-cols-2 gap-5">
      <div class="space-y-4"><div class="card p-4"><div class="panel-title mb-2">基本信息</div>
        <div class="kv"><span>地址</span><span>${esc(n.lan_host)} (${esc(n.host)}:${n.api_port})</span></div>
        <div class="kv"><span>硬件</span><span>${esc(n.model || '—')}</span></div><div class="kv"><span>架构 / CPU</span><span>${esc(n.arch)} / ${n.cpu_count} 核</span></div>
        <div class="kv"><span>内存</span><span>${bytes(n.mem_total)} (${fmt(n.mem_percent, 0)}%)</span></div><div class="kv"><span>系统</span><span>${esc(n.os)}</span></div>
        <div class="kv"><span>运行时</span><span>${esc(n.runtime?.runtime)} ${esc(n.runtime?.version || '')}</span></div><div class="kv"><span>磁盘剩余</span><span>${bytes(n.disk?.free)}</span></div>
        <div class="kv"><span>最后心跳</span><span>${ago(n.last_seen)}</span></div><div class="kv"><span>IP</span><span>${esc((n.ips || []).join(', '))}</span></div>
        <div class="kv"><span>类型</span><span>${esc(n.kind_label || '—')}</span></div><div class="kv"><span>可同时运行实例数</span><span>${n.max_instances}</span></div>
        ${n.ssh ? `<div class="kv"><span>SSH 部署地址</span><span class="font-mono">${esc(n.ssh.user)}@${esc(n.ssh.ip)}:${n.ssh.port}</span></div>` : ''}
        ${n.ssh_check ? `<div class="kv"><span>SSH 验证</span><span>${dt(n.ssh_check.at)} · ${esc(n.ssh_check.brief || '')}</span></div>` : ''}
        ${n.pending ? '<div class="text-[11px] text-blue-600 mt-2">尚未安装运行环境，第一次部署仿真到该节点时自动安装；之后才有 CPU / 内存等监测数据。</div>' : ''}
        ${n.note ? `<div class="kv"><span>备注</span><span>${esc(n.note)}</span></div>` : ''}</div></div>
      <div class="space-y-4"><div class="card p-4"><div class="panel-title">CPU ${fmt(n.cpu_percent, 0)}%</div>${spark(1, '#2563eb')}
        <div class="panel-title mt-2">内存 ${fmt(n.mem_percent, 0)}%</div>${spark(2, '#7c3aed')}${n.temp_c ? `<div class="panel-title mt-2">温度 ${n.temp_c}°C</div>${spark(3, '#dc2626', 90)}` : ''}</div>
        <div class="card p-4"><div class="panel-title mb-2">仿真镜像</div>${n.images.map(i => `<div class="kv"><span class="font-mono">${esc(i.ref)}</span><span>${esc(i.kind)} · ${bytes(i.size)}</span></div>`).join('') || '<div class="text-xs text-slate-400">无</div>'}</div>
        <div class="card p-4"><div class="panel-title mb-2">托管容器</div>${n.containers.map(c => `<div class="kv"><span class="font-mono">${esc(c.name)}</span><span>${esc(c.state)} ${esc(c.status || '')}</span></div>`).join('') || '<div class="text-xs text-slate-400">无</div>'}</div></div></div>` });
}

// ============================================================================ 车辆模型
async function pModels() {
  const d = await hub.get('/models');
  main().innerHTML = header('车辆模型', `<button class="btn-ghost" data-rdmp><i data-lucide="refresh-cw" class="w-3.5 h-3.5 text-blue-600"></i>从RDMP平台同步模型</button>
    <button class="btn-primary" data-add><i data-lucide="plus" class="w-3.5 h-3.5"></i>添加模型</button>`) +
    `<div class="grid grid-cols-3 gap-5">${d.models.map(m => {
      const s = m.summary || {}, a = m.audit?.by_severity || {};
      return `<div class="card p-4 flex flex-col hover:border-blue-300 transition-colors">
        <div class="h-36 bg-slate-50 border border-slate-200 rounded-lg mb-3 p-2">${modelTopSVG(s)}</div>
        <div class="flex items-center justify-between"><h3 class="text-sm font-bold text-slate-800 truncate">${esc(m.name)}</h3>
          ${a.missing ? `<span class="tag bg-rose-50 text-rose-600 border-rose-200">缺失 ${a.missing}</span>` : a.warn ? `<span class="tag bg-amber-50 text-amber-700 border-amber-200">待确认 ${a.warn}</span>` : `<span class="tag bg-emerald-50 text-emerald-700 border-emerald-200">参数完整</span>`}</div>
        <div class="text-xs mt-2 space-y-1.5">
          <div class="flex"><span class="w-16 text-slate-400 shrink-0">项目：</span><span class="truncate">${esc(m.project || '—')}</span></div>
          <div class="flex"><span class="w-16 text-slate-400 shrink-0">车辆类型：</span><span class="truncate">${esc(m.vtype || s.chassis_label || '—')}</span></div>
          <div class="flex"><span class="w-16 text-slate-400 shrink-0">物料号：</span><span class="font-mono">${esc(m.material_no || '—')}</span></div>
          <div class="flex"><span class="w-16 text-slate-400 shrink-0">模型文件：</span><span class="font-mono text-blue-600 truncate">${esc(m.file)} <span class="text-slate-400">${esc(m.latest)}</span></span></div></div>
        <div class="mt-auto pt-3 border-t border-slate-100 flex items-center justify-between mt-3">
          <span class="text-[11px] font-mono text-slate-500">${s.wheelbase_m ? `轴距 ${Math.round(s.wheelbase_m * 1000)}mm` : esc(s.chassis_label || '')} | ${s.mass_kg ? s.mass_kg + 'kg' : '—'}${s.max_load_kg ? ' / 载 ' + s.max_load_kg + 'kg' : ''}</span>
          <button class="btn-soft !py-1" data-detail="${m.id}">查看详情</button></div></div>`;
    }).join('')}</div>`;
  $('[data-add]', main()).onclick = () => openAddModel();
  $('[data-rdmp]', main()).onclick = openRdmp;
  $$('[data-detail]', main()).forEach(b => b.onclick = () => openModel(b.dataset.detail));
}

function openAddModel(mid) {
  const m = modal({ title: mid ? '上传新版本 cmodel' : '添加车辆模型', icon: 'truck', size: 'max-w-xl',
    body: `<label class="lbl">模型文件 (.cmodel，上传后自动解析轮组/尺寸/传感器，无需手填轮坐标)</label>
      <input type="file" accept=".cmodel" class="inp mb-4" data-file>
      ${mid ? '' : `<div class="grid grid-cols-2 gap-3"><div><label class="lbl">车辆模型标识</label><input class="inp" data-f="name" placeholder="缺省取文件名"></div>
        <div><label class="lbl">物料号</label><input class="inp font-mono" data-f="material_no" placeholder="10023489-01"></div>
        <div><label class="lbl">项目</label><input class="inp" data-f="project"></div><div><label class="lbl">车辆类型</label><input class="inp" data-f="vtype" placeholder="缺省按底盘推断"></div></div>`}
      <div data-prog class="mt-3 text-xs text-slate-500"></div>`,
    footer: `<button data-close class="btn-ghost">取消</button><button data-ok class="btn-primary">保存入库</button>` });
  $('[data-ok]', m.el).onclick = guard(async () => {
    const f = $('[data-file]', m.el).files[0];
    if (!f) throw new Error('请选择 .cmodel 文件');
    const q = new URLSearchParams({ filename: f.name });
    if (mid) q.set('model_id', mid);
    $$('[data-f]', m.el).forEach(i => i.value && q.set(i.dataset.f, i.value));
    $('[data-prog]', m.el).textContent = '上传并解析中…';
    const r = await upload(`/api/hub/models/upload?${q}`, f);
    m.close(); toast(`${r.name} ${r.latest} 已入库`, 'ok'); show('models');
    if (mid) openModel(mid);
  });
}

function openRdmp() {
  const m = modal({ title: '从 RDMP 平台同步模型', icon: 'refresh-cw', size: 'max-w-2xl',
    body: `<div class="text-xs text-slate-600 bg-amber-50 border border-amber-200 rounded-lg p-3 mb-4 leading-relaxed">RDMP 研发物料平台接口尚未配置 (需要接口文档与访问账号)。当前可将从 RDMP 导出的多个 .cmodel 批量导入，文件名作为车辆模型标识；物料号可在导入后于模型详情中补充。</div>
      <input type="file" accept=".cmodel" multiple class="inp" data-files><div data-list class="mt-3 space-y-1 text-xs"></div>`,
    footer: `<button data-close class="btn-ghost">关闭</button><button data-ok class="btn-primary">开始同步</button>` });
  $('[data-ok]', m.el).onclick = guard(async () => {
    const fs = Array.from($('[data-files]', m.el).files);
    const L = $('[data-list]', m.el);
    for (const f of fs) {
      const row = document.createElement('div'); row.textContent = `${f.name} … 解析中`; L.appendChild(row);
      try { const r = await upload(`/api/hub/models/upload?filename=${encodeURIComponent(f.name)}`, f); row.innerHTML = `<span class="text-emerald-600">✔ ${esc(f.name)} → ${esc(r.name)} ${esc(r.latest)}</span>`; }
      catch (e) { row.innerHTML = `<span class="text-rose-600">✘ ${esc(f.name)}: ${esc(e.message)}</span>`; }
    }
    show('models');
  });
}

async function openModel(mid) {
  const d = await hub.get(`/models/${mid}`);
  const sp = d.spec, ch = sp.chassis || {}, s = d.summary || {};
  let view = null;
  const m = modal({ full: true, title: `车辆模型详情 - ${esc(d.name)}`, icon: 'truck',
    badge: `<span class="text-[11px] px-2 py-0.5 rounded bg-blue-50 text-blue-700 border border-blue-200 font-semibold ml-2">${esc(d.vtype || s.chassis_label || '')}</span>`,
    headExtra: `<div class="seg mr-2"><button data-tab="ov" class="on">概览</button><button data-tab="ed" class="hidden">补全与传感器安装</button><button data-tab="ver">版本</button></div>
      <a class="btn-ghost" href="/api/hub/models/${mid}/cmodel"><i data-lucide="download" class="w-3.5 h-3.5"></i>下载 cmodel 模型包</a>`,
    body: `<div data-pane="ov" class="h-full flex">
      <div class="flex-1 relative bg-slate-900 min-w-0" data-3d>
        <div class="absolute top-4 left-4 z-10 flex items-center gap-1 bg-slate-800/80 border border-slate-700 rounded-lg p-1 text-xs text-slate-300">
          ${[['persp', '3D透视'], ['top', '俯视图'], ['side', '侧视图'], ['front', '正前视图']].map(([k, t], i) => `<button data-v="${k}" class="px-2.5 py-1 rounded ${i ? '' : 'bg-blue-600 text-white'}">${t}</button>`).join('')}
          <span class="w-px h-4 bg-slate-600 mx-1"></span><label class="flex items-center gap-1 px-2"><input type="checkbox" data-wire>线框模式</label></div>
        <div class="absolute bottom-4 left-4 z-10 max-w-[70%] text-[11px] font-mono text-slate-300 bg-slate-800/80 border border-slate-700 rounded-lg px-3 py-1.5 flex flex-wrap gap-x-3">
          ${(s.wheels || []).filter(w => w.kind !== 'caster').map((w, i) => `<span class="${['text-blue-400', 'text-purple-400', 'text-cyan-400', 'text-pink-400'][i % 4]}">● ${esc(w.name)}: (${Math.round(w.x * 1000)}, ${Math.round(w.y * 1000)})</span>`).join('')}
          ${(s.lidars || []).slice(0, 2).map(l => `<span class="text-emerald-400">● ${esc(l.name)}: [${fmt(l.x)}, ${fmt(l.y)}, ${fmt(l.z)}]</span>`).join('')}</div>
        <div class="absolute bottom-4 right-4 z-10 text-[11px] text-slate-400 bg-slate-800/80 border border-slate-700 rounded-lg px-3 py-1.5">鼠标旋转 / 滚轮缩放 / 右键平移</div>
      </div>
      <div class="w-[470px] shrink-0 overflow-y-auto p-5 space-y-4 bg-white">
        <div class="card p-4 bg-slate-50/50"><div class="flex justify-between mb-2"><span class="panel-title">物料与项目信息</span><span class="text-xs font-mono text-blue-600">物料号: <input data-m="material_no" value="${esc(d.material_no || '')}" class="w-32 bg-transparent border-b border-dashed border-blue-300 text-right focus:outline-none" placeholder="未填写"></span></div>
          <div class="kv"><span>模型标识</span><span><input data-m="name" value="${esc(d.name)}" class="bg-transparent text-right focus:outline-none border-b border-dashed border-slate-300"></span></div>
          <div class="kv"><span>项目名称</span><span><input data-m="project" value="${esc(d.project || '')}" class="bg-transparent text-right focus:outline-none border-b border-dashed border-slate-300 w-60" placeholder="未填写"></span></div>
          <div class="kv"><span>车辆类型</span><span><input data-m="vtype" value="${esc(d.vtype || '')}" class="bg-transparent text-right focus:outline-none border-b border-dashed border-slate-300 w-60"></span></div>
          <div class="kv"><span>模型文件</span><span>${esc(d.file)} (${esc(d.latest)})</span></div>
          <div class="text-right mt-2"><button data-savemeta class="btn-soft !py-1">保存信息</button></div></div>
        <div class="card p-4"><div class="flex justify-between mb-3"><span class="panel-title">动力轮组安装与闭环配置</span><span class="text-[11px] px-2 py-0.5 rounded bg-blue-50 text-blue-700 font-semibold">${esc(s.chassis_label)}架构</span></div>
          ${(sp.wheels || []).map((w, i) => `<div class="rounded-lg border ${w.kind === 'steer' || w.kind === 'drive' ? 'border-blue-100 bg-blue-50/40' : 'border-slate-100'} p-3 mb-2 text-xs">
            <div class="flex justify-between font-semibold ${['text-blue-700', 'text-purple-700', 'text-cyan-700', 'text-pink-700'][i % 4]}"><span>${esc(w.name)} (${esc({ steer: '舵轮', drive: '驱动轮', fixed: '固定轮', caster: '万向轮' }[w.kind] || w.kind)})</span><span class="font-mono">x = ${Math.round(w.x * 1000)} mm, y = ${Math.round(w.y * 1000)} mm</span></div>
            ${w.drive_motor ? `<div class="kv"><span>行走电机 ${esc(w.drive_motor.model || '')}</span><span>速度闭环 · 减速比 ${fmt(w.drive_motor.gear_ratio)}</span></div>` : ''}
            ${w.steer_motor ? `<div class="kv"><span>转向电机 ${esc(w.steer_motor.model || '')}</span><span>角度位置闭环 · ±${fmt(w.steer_max_rad * 57.3, 0)}°</span></div>` : ''}
            <div class="kv"><span>轮径 / 轮宽</span><span>${fmt((w.radius_m || 0) * 2000, 0)} / ${fmt((w.width_m || 0) * 1000, 0)} mm</span></div></div>`).join('')}</div>
        <div class="card p-4"><div class="panel-title mb-2">尺寸与载荷性能</div>
          <div class="kv"><span>外形尺寸 (长×宽×高)</span><span>${Math.round(ch.length_m * 1000)} × ${Math.round(ch.width_m * 1000)} × ${Math.round((ch.height_m || 0) * 1000)} mm</span></div>
          <div class="kv"><span>轮系轴距 / 轮距</span><span>${Math.round(s.wheelbase_m * 1000)} mm / ${Math.round(s.track_m * 1000)} mm</span></div>
          <div class="kv"><span>整备质量 / 额定载重</span><span>${ch.mass_kg || '—'} kg / ${ch.max_load_kg || '—'} kg</span></div>
          <div class="kv"><span>顶升机构行程</span><span>${sp.lift?.enabled ? Math.round(sp.lift.stroke_m * 1000) + ' mm' : '无'}</span></div>
          <div class="kv"><span>最大线速度 / 角速度</span><span>${fmt(ch.max_speed_mps)} m/s / ${fmt(ch.max_ang_speed_radps)} rad/s</span></div>
          <div class="kv"><span>回转直径</span><span>${fmt(ch.rotate_diameter_m)} m</span></div></div>
        <div class="card p-4"><div class="panel-title mb-2">传感器外参配置</div>
          ${(sp.lidars || []).map(l => `<div class="kv"><span>${l.type === '3d' ? '3D' : '2D'} 激光 ${esc(l.name)} (${esc(l.vendor_model || l.model || '')})</span><span>[${fmt(l.x, 3)}, ${fmt(l.y, 3)}, ${fmt(l.z, 3)}] m (${fmt(l.fov_deg, 0)}°)</span></div>`).join('')}
          ${(sp.cameras || []).map(c => `<div class="kv"><span>相机 ${esc(c.name)} (${esc(c.type)})</span><span>[${fmt(c.x, 3)}, ${fmt(c.y, 3)}, ${fmt(c.z, 3)}] m</span></div>`).join('')}
          <div class="kv"><span>光电 / 触边</span><span>${(sp.photoelectric || []).length || '四角缺省'} / 前后缺省</span></div>
          <div class="kv"><span>完整度</span><span>${Object.entries(d.audit?.by_severity || {}).map(([k, v]) => `${{ ok: '完整', info: '提示', warn: '待确认', missing: '缺失' }[k]} ${v}`).join(' · ')}</span></div></div>
        <div class="text-right"><button data-delmodel class="btn-danger">删除模型</button></div>
      </div></div>
      <div data-pane="ed" class="h-full hidden"><iframe data-iframe class="w-full h-full border-0"></iframe></div>
      <div data-pane="ver" class="h-full hidden p-6 overflow-y-auto"><div class="flex justify-between mb-4"><span class="panel-title">版本历史</span><button data-newver class="btn-primary"><i data-lucide="upload" class="w-3.5 h-3.5"></i>上传新版本</button></div>
        <table class="w-full text-xs card overflow-hidden"><thead class="bg-slate-50 text-slate-500"><tr><th class="text-left px-4 py-2">版本</th><th class="text-left px-4">说明</th><th class="text-right px-4">操作</th></tr></thead>
        <tbody>${(d.versions || []).slice().reverse().map(v => `<tr class="border-t border-slate-100"><td class="px-4 py-2 font-mono font-bold">${esc(v)}${v === d.latest ? ' <span class="tag bg-blue-50 text-blue-700 border-blue-200">当前</span>' : ''}</td>
          <td class="px-4 text-slate-500">新版本上传时继承上一版本的人工补全</td><td class="px-4 text-right"><a class="btn-ghost !py-1" href="/api/hub/models/${mid}/cmodel?ver=${v}">下载</a></td></tr>`).join('')}</tbody></table></div>`,
    onClose: () => view && view.destroy() });
  view = new ModelView($('[data-3d]', m.el), sp);
  $$('[data-v]', m.el).forEach(b => b.onclick = () => { view.view(b.dataset.v); $$('[data-v]', m.el).forEach(x => x.className = 'px-2.5 py-1 rounded' + (x === b ? ' bg-blue-600 text-white' : '')); });
  $('[data-wire]', m.el).onchange = (e) => view.wireframe(e.target.checked);
  $$('[data-tab]', m.el).forEach(b => b.onclick = () => {
    $$('[data-tab]', m.el).forEach(x => x.classList.toggle('on', x === b));
    $$('[data-pane]', m.el).forEach(p => p.classList.toggle('hidden', p.dataset.pane !== b.dataset.tab));
    if (b.dataset.tab === 'ed' && !$('[data-iframe]', m.el).src) $('[data-iframe]', m.el).src = `/model-editor?api=${encodeURIComponent(`/api/hub/models/${mid}/versions/${d.latest}/api/v1`)}&title=${encodeURIComponent(d.name + ' ' + d.latest)}`;
  });
  $('[data-savemeta]', m.el).onclick = guard(async () => {
    const b = {}; $$('[data-m]', m.el).forEach(i => b[i.dataset.m] = i.value.trim());
    await hub.patch(`/models/${mid}`, b); toast('已保存', 'ok'); show('models');
  });
  $('[data-newver]', m.el).onclick = () => { m.close(); openAddModel(mid); };
  $('[data-delmodel]', m.el).onclick = guard(async () => {
    if (!await confirmBox(`删除车辆模型 ${d.name} 及其全部版本与人工补全？`)) return;
    await hub.del(`/models/${mid}`); m.close(); show('models');
  });
}

// ============================================================================ 仿真场景
async function pScenes() {
  const d = await hub.get('/scenes');
  main().innerHTML = header('仿真场景', `<button class="btn-primary" data-add><i data-lucide="plus" class="w-3.5 h-3.5"></i>添加仿真场景</button>`) +
    `<div class="grid grid-cols-3 gap-5">${d.scenes.map(s => {
      const m = s.summary || {};
      return `<div class="card p-4 flex flex-col hover:border-emerald-300 transition-colors">
        <div class="h-44 rounded-lg overflow-hidden relative mb-3"><canvas data-thumb="${s.id}" class="w-full h-full"></canvas>
          <span class="absolute bottom-2 right-2 text-[10px] font-mono px-2 py-0.5 rounded bg-slate-900/80 border border-emerald-500/50 text-emerald-400">${m.map === 'pgm' ? 'SLAM 栅格' : '几何栅格'}${m.nodes ? ' + 拓扑' : ''}${m.mesh ? ' + 3D' : ''}</span></div>
        <h3 class="text-sm font-bold text-slate-800">${esc(s.name)}</h3>
        <div class="text-xs text-slate-600 mt-2 space-y-1">
          <div>物理尺寸: <span class="font-mono">${m.size?.[0]}m × ${m.size?.[1]}m</span> <span class="text-slate-400">(${m.resolution}m 分辨率)</span></div>
          <div>实体构件: ${m.shelves} 个货架/设备岛 · ${m.walls} 段墙体</div>
          <div>业务拓扑: ${m.stations} 工位 · ${m.nodes} 节点 · ${m.edges} 路段${m.reflectors ? ` · ${m.reflectors} 反光柱` : ''}</div></div>
        <div class="mt-auto pt-3 border-t border-slate-100 flex items-center justify-between mt-3">
          <span class="text-[11px] ${s.builtin ? 'text-slate-500' : 'text-emerald-600'}">● ${esc((s.tags || []).join(' · ') || (s.builtin ? '内置' : '自定义'))}</span>
          <button class="btn !py-1 bg-emerald-50 border border-emerald-200 text-emerald-700 hover:bg-emerald-100" data-detail="${s.id}">查看详情</button></div></div>`;
    }).join('')}</div>`;
  $('[data-add]', main()).onclick = openAddScene;
  $$('[data-detail]', main()).forEach(b => b.onclick = () => openScene(b.dataset.detail));
  for (const s of d.scenes) {
    hub.get(`/scenes/${s.id}/definition`).then(sc => { const c = $(`[data-thumb="${s.id}"]`); if (c) drawSceneThumb(c, sc); }).catch(() => { });
  }
}

function openAddScene() {
  const m = modal({ title: '添加仿真场景', icon: 'map', size: 'max-w-xl',
    body: `<label class="lbl">场景包 (.zip: scene.json / map.pgm + map.yaml / topology.json / mesh.obj|glb；或单个 scene.json)</label>
      <input type="file" accept=".zip,.json" class="inp mb-4" data-file>
      <div class="grid grid-cols-2 gap-3"><div><label class="lbl">场景名称</label><input class="inp" data-f="name" placeholder="缺省取 scene.json 名称"></div>
      <div><label class="lbl">标签</label><input class="inp" data-f="tags" placeholder="立体库,标定场"></div></div>
      <label class="lbl mt-3">描述</label><input class="inp" data-f="description">
      <div class="text-[11px] text-slate-500 mt-3 leading-relaxed">只有 PGM 栅格没有 scene.json 墙体时，平台会从栅格轮廓生成碰撞墙体 (精度受分辨率限制)。坐标系与 Nav2 地图一致 (米, map 坐标)。</div>`,
    footer: `<button data-close class="btn-ghost">取消</button><button data-ok class="btn-primary">保存入库</button>` });
  $('[data-ok]', m.el).onclick = guard(async () => {
    const f = $('[data-file]', m.el).files[0]; if (!f) throw new Error('请选择场景包');
    const q = new URLSearchParams({ filename: f.name }); $$('[data-f]', m.el).forEach(i => i.value && q.set(i.dataset.f, i.value));
    const r = await upload(`/api/hub/scenes/upload?${q}`, f); m.close(); toast(`场景 ${r.name} 已入库`, 'ok'); show('scenes');
  });
}

async function openScene(sid) {
  const d = await hub.get(`/scenes/${sid}`);
  const sc = d.scene, s = d.summary || {};
  let view = null;
  const m = modal({ full: true, title: `仿真场景详情 - ${esc(d.name)}`, icon: 'map',
    badge: `<span class="text-[11px] px-2 py-0.5 rounded bg-emerald-50 text-emerald-700 border border-emerald-200 font-mono font-semibold ml-2">${s.size?.[0]}m × ${s.size?.[1]}m</span>`,
    headExtra: `<a class="btn-ghost" href="/api/hub/scenes/${sid}/package"><i data-lucide="download" class="w-3.5 h-3.5"></i>下载仿真场景包</a>`,
    body: `<div class="h-full flex"><div class="flex-1 relative bg-slate-950 min-w-0" data-3d>
        <div class="absolute top-4 left-4 z-10 space-y-2 text-xs text-slate-300">
          <div class="flex items-center gap-3 bg-slate-800/80 border border-slate-700 rounded-lg px-3 py-1.5">图层控制:
            <label class="flex gap-1"><input type="checkbox" checked data-l="env">3D环境模型</label><label class="flex gap-1"><input type="checkbox" checked data-l="grid">SLAM栅格图</label><label class="flex gap-1"><input type="checkbox" checked data-l="topo">拓扑地图</label></div>
          <div class="inline-flex gap-1 bg-slate-800/80 border border-slate-700 rounded-lg p-1">${[['persp', '全景透视'], ['top', '2D俯瞰'], ['aisle', '巷道视角']].map(([k, t], i) => `<button data-v="${k}" class="px-2.5 py-1 rounded ${i ? '' : 'bg-emerald-600/30 text-emerald-300'}">${t}</button>`).join('')}</div></div>
        <div class="absolute bottom-4 right-4 z-10 text-[11px] text-slate-400 bg-slate-800/80 border border-slate-700 rounded-lg px-3 py-1.5">鼠标旋转 / 滚轮缩放 / 右键平移</div></div>
      <div class="w-[430px] shrink-0 overflow-y-auto p-5 space-y-4 bg-white">
        <div class="card p-4 bg-slate-50/50"><div class="panel-title mb-2">场景基本属性</div>
          <div class="kv"><span>场景名称</span><span>${esc(d.name)}</span></div><div class="kv"><span>场景标识</span><span>${esc(sid)}</span></div>
          <div class="kv"><span>物理尺寸跨度 (长×宽)</span><span>${s.size?.[0]}m × ${s.size?.[1]}m</span></div><div class="kv"><span>作业占地面积</span><span>${s.area_m2} ㎡</span></div>
          <div class="kv"><span>净空高度 / 货架高度</span><span>${s.ceiling_m}m / ${s.shelf_height_m}m</span></div><div class="kv"><span>起点 P0</span><span>(${fmt(s.origin?.x)}, ${fmt(s.origin?.y)})</span></div>
          <div class="text-xs text-slate-500 mt-2 leading-relaxed">${esc(d.description || '')}</div></div>
        <div class="card p-4"><div class="flex justify-between mb-2"><span class="panel-title">2D SLAM 栅格参数</span><span class="text-[11px] font-mono px-2 py-0.5 rounded bg-emerald-50 text-emerald-700">${s.resolution}m 分辨率</span></div>
          <div class="kv"><span>栅格网格尺寸</span><span data-gsize>${s.grid?.[0]} × ${s.grid?.[1]} 格点</span></div><div class="kv"><span>原点坐标 Origin</span><span data-gorg>—</span></div>
          <div class="kv"><span>占据判定阈值</span><span>占用 &gt; 0.65, 空闲 &lt; 0.25</span></div><div class="kv"><span>地图来源</span><span>${s.map === 'pgm' ? '场景包 SLAM 栅格' : '由场景几何生成'}</span></div></div>
        <div class="card p-4"><div class="panel-title mb-2">3D 空间实体模型</div>
          <div class="kv"><span>货架/设备岛</span><span>${s.shelves} 个</span></div><div class="kv"><span>墙体线段</span><span>${s.walls} 段${sc.meta?.walls_from_grid ? ' (栅格生成)' : ''}</span></div>
          <div class="kv"><span>反光柱</span><span>${s.reflectors} 根</span></div><div class="kv"><span>外观网格</span><span>${esc(s.mesh || '无 (按几何生成)')}</span></div><div class="kv"><span>物理碰撞</span><span>MuJoCo 刚体 (墙体/货架/注入元素)</span></div></div>
        <div class="card p-4"><div class="flex justify-between mb-2"><span class="panel-title">业务拓扑导引网络</span><span class="text-[11px] font-mono px-2 py-0.5 rounded bg-purple-50 text-purple-700">Topology</span></div>
          <div class="kv"><span>工位站点</span><span>${s.stations} 个</span></div><div class="kv"><span>拓扑节点 / 路段</span><span>${s.nodes} / ${s.edges}</span></div>
          <div class="max-h-40 overflow-y-auto mt-2">${(sc.stations || []).map(st => `<div class="kv"><span>${esc(st.id)} ${esc(st.name)}</span><span>(${fmt(st.x)}, ${fmt(st.y)})</span></div>`).join('')}</div></div>
        ${d.builtin ? '' : '<div class="text-right"><button data-del class="btn-danger">删除场景</button></div>'}</div></div>`,
    onClose: () => view && view.destroy() });
  view = new SceneView($('[data-3d]', m.el), sc);
  Promise.all([fetch(`/api/hub/scenes/${sid}/map?part=pgm`).then(r => r.arrayBuffer()), fetch(`/api/hub/scenes/${sid}/map?part=yaml`).then(r => r.text())]).then(([p, y]) => {
    const dec = decodePGM(p), meta = parseMapYaml(y); view.setGrid(dec, meta);
    $('[data-gsize]', m.el).textContent = `${dec.W} × ${dec.H} 格点`; $('[data-gorg]', m.el).textContent = `[${meta.origin.map(v => (+v).toFixed(3)).join(', ')}] m`;
  }).catch(() => { });
  $$('[data-l]', m.el).forEach(c => c.onchange = () => view.toggle(c.dataset.l, c.checked));
  $$('[data-v]', m.el).forEach(b => b.onclick = () => { view.view(b.dataset.v); $$('[data-v]', m.el).forEach(x => x.className = 'px-2.5 py-1 rounded' + (x === b ? ' bg-emerald-600/30 text-emerald-300' : '')); });
  const del = $('[data-del]', m.el);
  if (del) del.onclick = guard(async () => { if (!await confirmBox(`删除场景 ${d.name}？`)) return; await hub.del(`/scenes/${sid}`); m.close(); show('scenes'); });
}

// ============================================================================ 软件程序包
async function pSoftware() {
  const [d, nodes] = await Promise.all([hub.get('/packages'), hub.get('/nodes')]);
  const nn = Object.fromEntries(nodes.nodes.map(n => [n.id, n.name]));
  const TAGC = { baseline: 'bg-emerald-50 text-emerald-700', test: 'bg-blue-50 text-blue-700', current: 'bg-emerald-50 text-emerald-700', lite: 'bg-slate-100 text-slate-600', node: 'bg-purple-50 text-purple-700' };
  const col = (kind, title, color) => {
    const ps = d.packages.filter(p => p.kind === kind);
    return `<div class="card p-4"><div class="flex justify-between items-center mb-3"><span class="text-sm font-bold text-slate-800">${title}</span>
      <span class="text-[11px] px-2 py-0.5 rounded bg-${color}-50 text-${color}-600 font-bold">${ps.length} 个版本</span></div>
      <div class="space-y-2">${ps.map(p => `<div class="border border-slate-200 rounded-lg p-3 hover:border-${color}-300">
        <div class="flex justify-between items-start gap-2"><div class="min-w-0"><div class="text-xs font-bold font-mono text-slate-800 truncate">${esc(p.version)}</div>
          <div class="text-[11px] text-slate-500 truncate">${esc(p.note || p.image_ref)}</div></div>
          ${p.tag ? `<span class="text-[10px] px-2 py-0.5 rounded font-bold whitespace-nowrap ${TAGC[p.tag] || ''}">${esc(d.tags[p.tag] || p.tag)}</span>` : ''}</div>
        <div class="flex justify-between items-center mt-2 text-[11px] text-slate-400">
          <span class="font-mono truncate">${esc(p.image_ref)} · ${esc(p.arch || '')} · ${bytes(p.size)} · ${p.source === 'upload' ? '平台仓库' : '在 ' + esc((p.nodes || []).map(i => nn[i] || i).join(','))}</span>
          <span class="flex gap-1 shrink-0">${p.source === 'node' ? `<button class="btn-soft !py-0.5 !px-2" data-store="${p.id}">入库</button>` : ''}
            <button class="btn-ghost !py-0.5 !px-2" data-edit="${p.id}">编辑</button>
            ${p.file ? `<a class="btn-ghost !py-0.5 !px-2" href="/api/hub/packages/${p.id}/image">下载</a>` : ''}
            <button class="btn-danger !py-0.5 !px-2" data-rm="${p.id}">删除</button></span></div></div>`).join('') || '<div class="text-xs text-slate-400">暂无</div>'}</div></div>`;
  };
  main().innerHTML = header('软件程序包版本库', `<button class="btn-primary" data-add><i data-lucide="plus" class="w-3.5 h-3.5"></i>添加程序包</button>`) +
    `<div class="grid grid-cols-2 gap-5">${col('nav', '运行程序包 (Controller / Navigation · agv-nav)', 'blue')}${col('sim', '仿真程序包 (Simulation Engine · agv-sim)', 'purple')}</div>
    <div class="text-[11px] text-slate-500 mt-4 leading-relaxed">程序包即 Docker 镜像。节点心跳会自动登记节点本机的 agv-sim / agv-nav 镜像 (节点镜像)；「入库」把节点镜像导出上传到平台，之后部署到其它同架构节点时由平台自动分发导入。</div>`;
  $('[data-add]', main()).onclick = () => openAddPackage(nodes.nodes);
  $$('[data-rm]', main()).forEach(b => b.onclick = guard(async () => { if (!await confirmBox('删除该程序包记录 (及平台上的镜像文件)？节点上已导入的镜像不受影响。')) return; await hub.del(`/packages/${b.dataset.rm}`); show('software'); }));
  $$('[data-edit]', main()).forEach(b => b.onclick = () => openEditPackage(d.packages.find(p => p.id === b.dataset.edit), d.tags));
  $$('[data-store]', main()).forEach(b => b.onclick = guard(async () => {
    const p = d.packages.find(x => x.id === b.dataset.store);
    const r = await hub.post('/packages/from_node', { node_id: p.nodes[0], ref: p.image_ref, kind: p.kind, version: p.version, note: p.note });
    toast('节点开始导出镜像并上传到平台 (可能需要数分钟)…', 'info');
    for (let k = 0; k < 900; k++) {
      await sleep(3000);
      const j = await hub.get(`/nodes/${r.node_id}/jobs/${r.job.id}`);
      b.textContent = `${Math.round(j.bytes / 1e6)} MB`;
      if (j.status !== 'running') { toast(j.status === 'done' ? '入库完成' : '入库失败: ' + j.message, j.status === 'done' ? 'ok' : 'err'); show('software'); break; }
    }
  }));
}

function openEditPackage(p, tags) {
  const m = modal({ title: `程序包 ${esc(p.version)}`, icon: 'package', size: 'max-w-lg',
    body: `<label class="lbl">版本号</label><input class="inp mb-3 font-mono" data-f="version" value="${esc(p.version)}">
      <label class="lbl">说明</label><input class="inp mb-3" data-f="note" value="${esc(p.note || '')}">
      <label class="lbl">标签</label><select class="inp" data-f="tag"><option value="">无</option>${Object.entries(tags).filter(([k]) => k !== 'node').map(([k, v]) => `<option value="${k}" ${p.tag === k ? 'selected' : ''}>${v}</option>`).join('')}</select>`,
    footer: `<button data-close class="btn-ghost">取消</button><button data-ok class="btn-primary">保存</button>` });
  $('[data-ok]', m.el).onclick = guard(async () => {
    const b = {}; $$('[data-f]', m.el).forEach(i => b[i.dataset.f] = i.value);
    await hub.patch(`/packages/${p.id}`, b); m.close(); show('software');
  });
}

function openAddPackage(nodes) {
  const m = modal({ title: '添加程序包', icon: 'package', size: 'max-w-xl',
    body: `<div class="seg mb-4"><button data-m="up" class="on">上传镜像包</button><button data-m="node">从节点入库</button></div>
      <div data-pane="up"><label class="lbl">docker save 导出的镜像包 (.tar / .tar.gz)，例如 <span class="font-mono">./deploy.sh save sim</span></label>
        <input type="file" accept=".tar,.gz,.tgz" class="inp mb-3" data-file>
        <div class="grid grid-cols-2 gap-3"><div><label class="lbl">类型</label><select class="inp" data-f="kind"><option value="">自动识别</option><option value="nav">运行程序包 (agv-nav)</option><option value="sim">仿真程序包 (agv-sim)</option></select></div>
        <div><label class="lbl">版本号</label><input class="inp font-mono" data-f="version" placeholder="缺省取镜像标签"></div></div>
        <label class="lbl mt-3">说明</label><input class="inp" data-f="note"><div class="h-2 bg-slate-100 rounded mt-4 overflow-hidden"><div data-bar class="h-2 bg-blue-600 w-0"></div></div></div>
      <div data-pane="node" class="hidden"><label class="lbl">节点</label><select class="inp mb-3" data-n="node">${nodes.map(n => `<option value="${n.id}">${esc(n.name)} (${esc(n.lan_host)})</option>`).join('')}</select>
        <label class="lbl">镜像</label><select class="inp mb-3" data-n="ref"></select><label class="lbl">说明</label><input class="inp" data-n="note"></div>`,
    footer: `<button data-close class="btn-ghost">取消</button><button data-ok class="btn-primary">保存入库</button>` });
  let mode = 'up';
  const fillRefs = () => { const n = nodes.find(x => x.id === $('[data-n=node]', m.el).value); $('[data-n=ref]', m.el).innerHTML = (n?.images || []).map(i => `<option value="${esc(i.ref)}" data-kind="${i.kind}">${esc(i.ref)} (${i.kind}, ${bytes(i.size)})</option>`).join(''); };
  $('[data-n=node]', m.el).onchange = fillRefs; fillRefs();
  $$('[data-m]', m.el).forEach(b => b.onclick = () => { mode = b.dataset.m; $$('[data-m]', m.el).forEach(x => x.classList.toggle('on', x === b)); $$('[data-pane]', m.el).forEach(p => p.classList.toggle('hidden', p.dataset.pane !== mode)); });
  $('[data-ok]', m.el).onclick = guard(async () => {
    if (mode === 'up') {
      const f = $('[data-file]', m.el).files[0]; if (!f) throw new Error('请选择镜像包');
      const q = new URLSearchParams(); $$('[data-f]', m.el).forEach(i => i.value && q.set(i.dataset.f, i.value));
      await upload(`/api/hub/packages/upload?${q}`, f, (p) => { $('[data-bar]', m.el).style.width = (p * 100).toFixed(0) + '%'; });
      toast('程序包已入库', 'ok');
    } else {
      const sel = $('[data-n=ref]', m.el);
      await hub.post('/packages/from_node', { node_id: $('[data-n=node]', m.el).value, ref: sel.value, kind: sel.selectedOptions[0]?.dataset.kind, note: $('[data-n=note]', m.el).value });
      toast('节点开始导出上传，完成后出现在列表中', 'info');
    }
    m.close(); show('software');
  });
}

// ============================================================================ 记录归档
async function pRecords() {
  const d = await hub.get('/records');
  const R = { SUCCESS: ['顺利完成', 'bg-emerald-50 text-emerald-700 border-emerald-200'], AVOIDED: ['避障达成', 'bg-amber-50 text-amber-700 border-amber-200'],
    FAILED: ['失败', 'bg-rose-50 text-rose-700 border-rose-200'], COLLIDED: ['发生碰撞', 'bg-rose-50 text-rose-700 border-rose-200'], STOPPED: ['已终止', 'bg-slate-100 text-slate-600 border-slate-200'], RESET: ['已重置', 'bg-slate-100 text-slate-600 border-slate-200'] };
  main().innerHTML = header('仿真记录归档', '<span class="text-xs text-slate-500">实例结束任务流时自动归档；终止实例前会补归档一次</span>') +
    `<div class="card overflow-hidden"><table class="w-full text-xs"><thead class="bg-slate-50 text-slate-500"><tr>
      <th class="text-left px-4 py-3">任务编号</th><th class="text-left px-4">任务流名称</th><th class="text-left px-4">实例</th><th class="text-left px-4">仿真场景</th><th class="text-left px-4">车辆模型</th>
      <th class="text-left px-4">执行时刻</th><th class="text-left px-4">耗时</th><th class="text-left px-4">注入</th><th class="text-left px-4">结果</th><th class="text-right px-4">操作</th></tr></thead>
      <tbody>${d.records.map(r => `<tr class="border-t border-slate-100 hover:bg-slate-50">
        <td class="px-4 py-3 font-mono font-bold text-blue-600">${esc(r.task_id)}</td><td class="px-4 font-semibold">${esc(r.task_name)}</td><td class="px-4 font-mono">${esc(r.instance_id)}</td>
        <td class="px-4">${esc(r.scene)}</td><td class="px-4 font-mono">${esc(r.model)}</td><td class="px-4 font-mono text-slate-500">${esc(r.recorded_at)}</td>
        <td class="px-4 font-mono font-bold">${fmt(r.duration, 1)}s</td><td class="px-4 font-mono text-amber-600">${r.injections} 次</td>
        <td class="px-4"><span class="px-2 py-0.5 rounded-full text-[10px] font-bold border ${(R[r.result] || R.STOPPED)[1]}">${(R[r.result] || [r.result])[0]}</span></td>
        <td class="px-4 text-right whitespace-nowrap"><a class="btn-ghost !py-1" href="/api/hub/records/${r.id}"><i data-lucide="download" class="w-3.5 h-3.5"></i>下载数据包</a>
          <button class="btn-primary !py-1" data-replay="${r.id}"><i data-lucide="play-circle" class="w-3.5 h-3.5"></i>载入回放</button></td></tr>`).join('') ||
    '<tr><td colspan="10" class="text-center text-slate-400 py-10">暂无归档记录</td></tr>'}</tbody></table></div>`;
  $$('[data-replay]', main()).forEach(b => b.onclick = () => { location.hash = `#/wb/replay/${b.dataset.replay}`; });
}
