// ============================================================================
// 接口文档: 各服务的 OpenAPI 3.1 文档 (平台聚合 GET /api/hub/apidocs/{svc})，按标签分组浏览、搜索、下载
// 依据的规范见 docs/API.md「约定」一节
// ============================================================================
import { $, $$, esc, hub, guard, icons, download } from './core.js';

const D = { svc: 'hub', q: '', specs: {}, open: new Set() };
const METHOD = { get: 'bg-emerald-50 text-emerald-700 border-emerald-200', post: 'bg-blue-50 text-blue-700 border-blue-200',
  put: 'bg-amber-50 text-amber-700 border-amber-200', patch: 'bg-purple-50 text-purple-700 border-purple-200', delete: 'bg-rose-50 text-rose-700 border-rose-200' };
const STANDARDS = [
  ['OpenAPI 3.1', 'https://spec.openapis.org/oas/v3.1.0', '接口描述格式 (每个服务 GET …/openapi.json)'],
  ['RFC 9457', 'https://www.rfc-editor.org/rfc/rfc9457', '错误响应 Problem Details (application/problem+json)'],
  ['RFC 8631', 'https://www.rfc-editor.org/rfc/rfc8631', '接口入口的 Link: rel="service-desc" / "service-doc"'],
  ['RFC 9727', 'https://www.rfc-editor.org/rfc/rfc9727', 'API 目录 /.well-known/api-catalog'],
  ['Google AIP-185', 'https://google.aip.dev/185', '路径中的主版本号 (/api/v1、/api/v2)'],
  ['RFC 9110', 'https://www.rfc-editor.org/rfc/rfc9110', 'HTTP 方法与状态码语义'],
];

export async function render(el) {
  const cat = await hub.get('/apidocs');
  el.innerHTML = `
    <div class="flex items-center justify-between pb-4 mb-5 border-b border-slate-200">
      <div><h2 class="text-base font-bold text-slate-800">接口文档</h2>
        <div class="text-[11px] text-slate-500 mt-1">由各服务的路由表生成的 OpenAPI 3.1 文档；服务在运行时为实时文档，否则为仓库里的离线快照 (docs/openapi/)。</div></div>
      <div class="flex items-center gap-2"><a class="btn-ghost" href="/.well-known/api-catalog" target="_blank"><i data-lucide="list-tree" class="w-3.5 h-3.5"></i>API 目录 (RFC 9727)</a>
        <button class="btn-primary" data-dl><i data-lucide="download" class="w-3.5 h-3.5"></i>下载 OpenAPI JSON</button></div></div>
    <div class="flex flex-wrap gap-2 mb-5">${STANDARDS.map(([n, u, t]) => `<a href="${u}" target="_blank" rel="noopener" title="${esc(t)}"
      class="text-[11px] px-2 py-1 rounded-md border border-slate-200 bg-white hover:border-blue-300"><b class="text-slate-700">${esc(n)}</b> <span class="text-slate-500">${esc(t)}</span></a>`).join('')}</div>
    <div class="flex gap-5 items-start">
      <div class="w-56 shrink-0 card p-2 sticky top-0">${cat.services.map(s => `<button data-svc="${s.id}" class="w-full text-left px-3 py-2 rounded-lg text-xs ${s.id === D.svc ? 'bg-blue-50 text-blue-700 font-semibold' : 'text-slate-600 hover:bg-slate-50'}">
        ${esc(s.title)}<div class="font-mono text-[10px] text-slate-400">${esc(s.id)}</div></button>`).join('')}</div>
      <div class="flex-1 min-w-0" data-main><div class="text-xs text-slate-400">加载中…</div></div></div>`;
  $$('[data-svc]', el).forEach(b => b.onclick = () => { D.svc = b.dataset.svc; D.open.clear(); render(el); });
  $('[data-dl]', el).onclick = guard(async () => {
    const spec = D.specs[D.svc] || await hub.get(`/apidocs/${D.svc}`);
    const u = URL.createObjectURL(new Blob([JSON.stringify(spec, null, 2)], { type: 'application/json' }));
    download(u, `openapi-${D.svc}.json`); setTimeout(() => URL.revokeObjectURL(u), 3000);
  });
  icons(el);
  await guard(showSpec)($('[data-main]', el));
}

async function showSpec(box) {
  let spec;
  try { spec = D.specs[D.svc] = await hub.get(`/apidocs/${D.svc}`); }
  catch (e) { box.innerHTML = `<div class="card p-6 text-sm text-slate-500">${esc(e.message)}</div>`; return; }
  const src = spec['x-source'] || {};
  const ops = [];
  for (const [path, item] of Object.entries(spec.paths || {}))
    for (const [m, op] of Object.entries(item)) ops.push({ path, m, op, tag: (op.tags || ['其它'])[0] });
  const draw = () => {
    const q = D.q.trim().toLowerCase();
    const hit = ops.filter(o => !q || o.path.toLowerCase().includes(q) || (o.op.summary || '').toLowerCase().includes(q) || o.m.includes(q));
    const tags = [...new Set(hit.map(o => o.tag))];
    $('[data-list]', box).innerHTML = tags.map(t => `<div class="mb-5"><div class="panel-title mb-2">${esc(t)} <span class="text-slate-400 font-normal">(${hit.filter(o => o.tag === t).length})</span></div>
      <div class="card overflow-hidden">${hit.filter(o => o.tag === t).map(o => opRow(o)).join('')}</div></div>`).join('') ||
      '<div class="text-xs text-slate-400">没有匹配的接口</div>';
    $$('[data-op]', box).forEach(r => r.onclick = () => { const k = r.dataset.op; D.open.has(k) ? D.open.delete(k) : D.open.add(k); draw(); });
  };
  box.innerHTML = `
    <div class="card p-4 mb-4"><div class="flex items-center justify-between"><div>
      <div class="text-sm font-bold text-slate-800">${esc(spec.info?.title || D.svc)} <span class="font-mono text-xs text-slate-500">v${esc(spec.info?.version || '')}${spec.info?.['x-build'] ? ' · ' + esc(spec.info['x-build']) : ''}</span></div>
      <div class="text-xs text-slate-600 mt-1">${esc(spec.info?.description || '')}</div></div>
      <span class="text-[11px] px-2 py-0.5 rounded-full border ${src.kind === 'live' ? 'bg-emerald-50 text-emerald-700 border-emerald-200' : 'bg-slate-100 text-slate-600 border-slate-200'} whitespace-nowrap">
        ${src.kind === 'live' ? '实时 · ' + esc(src.from || '') : '离线快照'}</span></div>
      <div class="text-[11px] text-slate-500 mt-2">共 ${ops.length} 个接口 · 错误统一为 <span class="font-mono">application/problem+json</span> (RFC 9457)：<span class="font-mono">{type, title, status, detail, instance, code}</span></div></div>
    <input class="inp mb-4" data-q placeholder="搜索路径、说明或方法 (如 missions、post)" value="${esc(D.q)}">
    <div data-list></div>`;
  $('[data-q]', box).oninput = (e) => { D.q = e.target.value; draw(); };
  draw();
}

function opRow({ path, m, op }) {
  const k = m + ' ' + path, open = D.open.has(k);
  const params = op.parameters || [];
  const body = op.requestBody?.content?.['application/json']?.schema;
  return `<div class="border-t border-slate-100 first:border-0">
    <div data-op="${esc(k)}" class="flex items-center gap-3 px-4 py-2 cursor-pointer hover:bg-slate-50">
      <span class="w-16 text-center text-[10px] font-bold uppercase px-1.5 py-0.5 rounded border ${METHOD[m] || 'bg-slate-100 text-slate-600 border-slate-200'}">${esc(m)}</span>
      <span class="font-mono text-xs text-slate-800 break-all">${esc(path)}</span><span class="text-xs text-slate-500 truncate">${esc(op.summary || '')}</span></div>
    ${open ? `<div class="px-4 pb-3 pl-[92px] text-xs text-slate-600 space-y-2">
      ${op.description ? `<div class="whitespace-pre-wrap">${esc(op.description)}</div>` : ''}
      ${params.length ? `<table class="text-[11px]"><tbody>${params.map(p => `<tr><td class="pr-3 font-mono text-slate-800">${esc(p.name)}</td><td class="pr-3 text-slate-400">${esc(p.in)}${p.required ? ' · 必填' : ''}</td><td>${esc(p.description || '')}</td></tr>`).join('')}</tbody></table>` : ''}
      ${body ? `<div><span class="text-slate-400">请求体 (JSON)：</span>${body.properties ? Object.entries(body.properties).map(([n, v]) => `<span class="font-mono text-slate-800">${esc(n)}</span>${v.description ? `<span class="text-slate-400">: ${esc(v.description)}</span>` : ''}`).join('，') : '<span class="text-slate-400">对象 (字段见说明)</span>'}</div>` : ''}
      <div class="text-slate-400">operationId <span class="font-mono">${esc(op.operationId || '')}</span></div></div>` : ''}</div>`;
}
