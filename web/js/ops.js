// ============================================================================
// 运维管理: 平台更新 (上传更新包 → 预览 → 应用/回滚)、App 管理 (APK 上传与安装、无线调试 adb)、服务与日志、设置
// 后端 hub/admin.py；需要管理员密码 (首次进入时设置)。登录令牌放在 sessionStorage (关掉浏览器标签即失效)
// ============================================================================
import { $, $$, esc, http, toast, guard, icons, modal, confirmBox, bytes, ago, dt, sleep } from './core.js';

const O = { tab: 'upd', timer: null, el: null, alive: false };
const TK = 'amr.admin.token';
const getTok = () => { try { return sessionStorage.getItem(TK) || ''; } catch (e) { return O.mem || ''; } };
const setTok = (t) => { O.mem = t; try { t ? sessionStorage.setItem(TK, t) : sessionStorage.removeItem(TK); } catch (e) { } };
const api = (m, p, b) => http(m, '/api/hub/admin' + p, b, { 'X-Admin-Token': getTok() }).catch(e => {
  if (e.status === 401 && e.code === 'admin_auth') { setTok(''); if (O.alive) render(O.el); }
  throw e;
});

const UPD_ST = { uploaded: ['待应用', 'bg-blue-50 text-blue-700 border-blue-200'], applying: ['应用中', 'bg-amber-50 text-amber-700 border-amber-200'],
  applied: ['已应用', 'bg-emerald-50 text-emerald-700 border-emerald-200'], rolling_back: ['回滚中', 'bg-amber-50 text-amber-700 border-amber-200'],
  rolled_back: ['已回滚', 'bg-slate-100 text-slate-600 border-slate-200'], failed: ['失败', 'bg-rose-50 text-rose-700 border-rose-200'] };
const JOB_ST = { running: ['进行中', 'bg-amber-50 text-amber-700 border-amber-200'], done: ['完成', 'bg-emerald-50 text-emerald-700 border-emerald-200'],
  failed: ['失败', 'bg-rose-50 text-rose-700 border-rose-200'], interrupted: ['中断', 'bg-slate-100 text-slate-600 border-slate-200'] };
const chip = (map, k) => { const [t, c] = map[k] || [k || '—', 'bg-slate-100 text-slate-600 border-slate-200']; return `<span class="px-2 py-0.5 rounded-full text-[11px] font-bold border ${c} whitespace-nowrap">${esc(t)}</span>`; };
const FILE_ST = { new: ['新增', 'text-emerald-700'], changed: ['修改', 'text-blue-700'], same: ['相同', 'text-slate-400'], delete: ['删除', 'text-rose-600'],
  absent: ['已不存在', 'text-slate-400'], protected: ['本地配置，跳过', 'text-amber-700'] };

export function leave() { O.alive = false; clearInterval(O.timer); O.timer = null; }

export async function render(el) {
  O.el = el; O.alive = true; clearInterval(O.timer); O.timer = null;
  const st = await http('GET', '/api/hub/admin/state', undefined, { 'X-Admin-Token': getTok() });
  if (!st.authed) return renderLogin(el, st.configured);
  const head = `<div class="flex items-center justify-between pb-4 mb-5 border-b border-slate-200">
    <div class="flex items-center gap-3"><h2 class="text-base font-bold text-slate-800">运维管理</h2><span data-ver class="text-xs font-mono text-slate-500"></span></div>
    <div class="flex items-center gap-2"><div class="seg">${[['upd', '平台更新'], ['app', 'App 管理'], ['svc', '服务与日志'], ['set', '设置']]
      .map(([k, t]) => `<button data-tab="${k}" class="${O.tab === k ? 'on' : ''}">${t}</button>`).join('')}</div>
      <button class="btn-ghost" data-logout><i data-lucide="log-out" class="w-3.5 h-3.5"></i>退出</button></div></div><div data-body></div>`;
  el.innerHTML = head;
  $$('[data-tab]', el).forEach(b => b.onclick = () => { O.tab = b.dataset.tab; render(el); });
  $('[data-logout]', el).onclick = guard(async () => { await api('POST', '/logout').catch(() => { }); setTok(''); render(el); });
  const body = $('[data-body]', el);
  await guard({ upd: tabUpdates, app: tabApps, svc: tabServices, set: tabSettings }[O.tab])(body);
  icons(el);
}

function renderLogin(el, configured) {
  el.innerHTML = `<div class="max-w-sm mx-auto mt-16 card p-6">
    <div class="flex items-center gap-2 mb-1"><i data-lucide="shield" class="w-4 h-4 text-blue-600"></i><h2 class="text-sm font-bold text-slate-800">运维管理</h2></div>
    <p class="text-xs text-slate-500 mb-4 leading-relaxed">${configured ? '请输入管理员密码。' : '首次使用，请设置管理员密码 (至少 6 位)。运维管理可以更新平台代码、安装 App、重启服务。'}</p>
    <input type="password" class="inp mb-3" data-pw placeholder="${configured ? '管理员密码' : '新密码'}" autocomplete="${configured ? 'current-password' : 'new-password'}">
    ${configured ? '' : '<input type="password" class="inp mb-3" data-pw2 placeholder="再输入一次" autocomplete="new-password">'}
    <button class="btn-primary w-full justify-center" data-go>${configured ? '登录' : '设置并进入'}</button><div data-err class="text-xs text-rose-600 mt-2"></div></div>`;
  icons(el);
  const go = async () => {
    const pw = $('[data-pw]', el).value;
    try {
      if (!configured && pw !== $('[data-pw2]', el).value) throw new Error('两次输入的密码不一致');
      const r = await http('POST', `/api/hub/admin/${configured ? 'login' : 'setup'}`, { password: pw });
      setTok(r.token); render(el);
    } catch (e) { $('[data-err]', el).textContent = e.message; }
  };
  $('[data-go]', el).onclick = go;
  $$('input', el).forEach(i => i.onkeydown = (e) => { if (e.key === 'Enter') go(); });
  $('[data-pw]', el).focus();
}

// ---------------------------------------------------------------------------- 平台更新
async function tabUpdates(body) {
  const [u, j] = await Promise.all([api('GET', '/updates'), api('GET', '/jobs')]);
  $('[data-ver]', O.el).textContent = `当前版本 ${u.current.version}${u.current.title ? ' · ' + u.current.title : ''}`;
  const run = j.jobs.find(x => x.status === 'running');
  body.innerHTML = `
    ${run ? `<div class="card p-3 mb-4 flex items-center justify-between border-amber-200 bg-amber-50/50"><span class="text-xs text-amber-800 flex items-center gap-2"><i data-lucide="loader" class="w-3.5 h-3.5 animate-spin"></i>${esc(run.title)} 进行中</span><button class="btn-soft !py-1" data-job="${run.id}">查看进度</button></div>` : ''}
    <div class="card p-4 mb-5"><div class="panel-title mb-2">上传更新包</div>
      <div class="flex items-center gap-3"><input type="file" accept=".tgz,.tar.gz" class="inp flex-1" data-file><button class="btn-primary" data-up><i data-lucide="upload" class="w-3.5 h-3.5"></i>上传并预览</button></div>
      <div class="text-[11px] text-slate-500 mt-2">更新包由 <span class="font-mono">tools/make_update.py</span> 生成。上传后先预览改动和影响，确认后才应用；每次应用都会备份原文件，可回滚最近一次。设备本地的数据与配置 (data/、records/、robot_config.json 等) 不会被覆盖。</div></div>
    <div class="panel-title mb-2">更新记录</div>
    <div class="card overflow-hidden mb-6"><table class="w-full text-xs"><thead class="bg-slate-50 text-slate-500"><tr>
      <th class="text-left px-4 py-2.5">版本</th><th class="text-left px-4">说明</th><th class="text-left px-4">上传</th><th class="text-left px-4">文件</th><th class="text-left px-4">状态</th><th class="text-right px-4">操作</th></tr></thead>
      <tbody>${u.updates.map(x => `<tr class="border-t border-slate-100">
        <td class="px-4 py-2.5 font-mono font-semibold">${esc(x.version)}</td><td class="px-4 max-w-xs truncate" title="${esc(x.title)}">${esc(x.title || '—')}</td>
        <td class="px-4 text-slate-500">${dt(x.uploaded)}${x.uploaded_by ? ' · ' + esc(x.uploaded_by) : ''}</td>
        <td class="px-4 text-slate-500">${Object.entries(x.counts || {}).filter(([k]) => ['new', 'changed', 'delete'].includes(k)).map(([k, v]) => `${FILE_ST[k][0]} ${v}`).join(' · ') || '无改动'}</td>
        <td class="px-4">${chip(UPD_ST, x.status)}</td>
        <td class="px-4 text-right whitespace-nowrap"><button class="btn-ghost !py-1" data-view="${x.id}">${x.status === 'uploaded' ? '预览 / 应用' : '详情'}</button>
          ${['applying', 'rolling_back'].includes(x.status) ? '' : `<button class="btn-danger !py-1" data-del="${x.id}">删除</button>`}</td></tr>`).join('') ||
        '<tr><td colspan="6" class="px-4 py-6 text-center text-slate-400">还没有上传过更新包</td></tr>'}</tbody></table></div>
    <div class="panel-title mb-2">运维任务</div>
    <div class="card overflow-hidden"><table class="w-full text-xs"><tbody>${j.jobs.slice(0, 10).map(x => `<tr class="border-t border-slate-100 first:border-0">
      <td class="px-4 py-2">${esc(x.title)}</td><td class="px-4 text-slate-500">${dt(x.started)}${x.by ? ' · ' + esc(x.by) : ''}</td><td class="px-4">${chip(JOB_ST, x.status)}</td>
      <td class="px-4 text-right"><button class="btn-ghost !py-1" data-job="${x.id}">日志</button></td></tr>`).join('') || '<tr><td class="px-4 py-4 text-center text-slate-400">暂无</td></tr>'}</tbody></table></div>`;
  $('[data-up]', body).onclick = guard(async () => {
    const f = $('[data-file]', body).files[0];
    if (!f) throw new Error('请选择更新包 (.tgz)');
    const b = $('[data-up]', body); b.disabled = true; b.textContent = '上传中…';
    try { const r = await api('POST', '/updates/upload', f); render(O.el); openUpdate(r.id); }
    finally { b.disabled = false; }
  });
  $$('[data-view]', body).forEach(b => b.onclick = () => openUpdate(b.dataset.view));
  $$('[data-job]', body).forEach(b => b.onclick = () => openJob(b.dataset.job));
  $$('[data-del]', body).forEach(b => b.onclick = guard(async () => {
    if (!await confirmBox('删除这个更新包及其备份？删除后无法用它回滚。')) return;
    await api('DELETE', `/updates/${b.dataset.del}`); render(O.el);
  }));
  if (run) O.timer = setInterval(() => { if (O.alive && O.tab === 'upd') render(O.el); }, 5000);
}

async function openUpdate(uid) {
  const u = await api('GET', `/updates/${uid}`);
  const changed = u.plan.filter(r => r.status !== 'same');
  const m = modal({ title: `更新包 ${esc(u.version)}`, icon: 'package', size: 'max-w-3xl', badge: chip(UPD_ST, u.status),
    body: `<div class="text-xs text-slate-600 mb-3">${esc(u.title || '')}</div>
      <div class="grid grid-cols-3 gap-3 mb-4 text-xs">
        <div class="card p-3"><div class="text-slate-400">当前设备版本</div><div class="font-mono font-semibold mt-1">${esc(u.current)}</div></div>
        <div class="card p-3"><div class="text-slate-400">更新包基于</div><div class="font-mono font-semibold mt-1">${esc(u.base || '—')}</div></div>
        <div class="card p-3"><div class="text-slate-400">改动</div><div class="font-semibold mt-1">${Object.entries(u.counts || {}).map(([k, v]) => `${(FILE_ST[k] || [k])[0]} ${v}`).join(' · ')}</div></div></div>
      ${u.base_mismatch ? `<div class="flex gap-2 text-amber-800 bg-amber-50 border border-amber-200 rounded px-3 py-2 mb-3 text-xs"><i data-lucide="alert-triangle" class="w-3.5 h-3.5 mt-px shrink-0"></i>更新包基于 ${esc(u.base)}，设备当前是 ${esc(u.current)}。版本不连续时只会写入包里的文件，其它文件保持设备上的样子。</div>` : ''}
      <div class="panel-title mb-1">影响</div>
      <ul class="text-xs text-slate-700 list-disc pl-5 mb-1">${u.effects.map(e => `<li>${esc(e)}</li>`).join('') || '<li class="text-slate-400">没有需要重启的服务</li>'}
        ${u.builds.length ? `<li>编译 ROS 插件: <span class="font-mono">${u.builds.map(esc).join(' ')}</span> (手机上约 3~5 分钟，编译失败自动恢复原文件)</li>` : ''}</ul>
      ${u.status === 'uploaded' && u.running_instances.length && u.components.some(c => c === 'instances' || c === 'agent') ? `<label class="flex items-center gap-2 text-xs text-slate-700 mt-2"><input type="checkbox" data-ri checked>
        应用后重新部署运行中的实例 (${u.running_instances.map(esc).join(', ')})</label>` : ''}
      <div class="panel-title mt-4 mb-1">文件 (${changed.length} 个有变化${u.plan.length - changed.length ? `，${u.plan.length - changed.length} 个与设备相同` : ''})</div>
      <div class="max-h-64 overflow-auto border border-slate-100 rounded-lg"><table class="w-full text-[11px]"><tbody>${(changed.length ? changed : u.plan).map(r => `<tr class="border-t border-slate-50 first:border-0">
        <td class="px-3 py-1 font-mono break-all">${esc(r.path)}</td><td class="px-3 whitespace-nowrap ${(FILE_ST[r.status] || ['', ''])[1]}">${esc((FILE_ST[r.status] || [r.status])[0])}</td>
        <td class="px-3 text-right text-slate-400 whitespace-nowrap">${r.size ? bytes(r.size) : ''}</td></tr>`).join('')}</tbody></table></div>`,
    footer: `<span class="text-xs text-slate-500">${u.applied_at ? '应用于 ' + dt(u.applied_at) : '上传于 ' + dt(u.uploaded)}</span><div class="flex gap-2"><button data-close class="btn-ghost">关闭</button>
      ${u.can_rollback ? '<button data-rb class="btn-danger">回滚此更新</button>' : ''}
      ${u.status === 'uploaded' ? `<button data-apply class="btn-primary" ${changed.some(r => ['new', 'changed', 'delete'].includes(r.status)) ? '' : 'disabled'}>应用更新</button>` : ''}</div>` });
  icons(m.el);
  const ri = () => { const c = $('[data-ri]', m.el); return c ? c.checked : true; };
  const a = $('[data-apply]', m.el);
  if (a) a.onclick = guard(async () => {
    if (!await confirmBox(`应用更新 ${u.version}？${u.components.includes('hub') ? '平台会自动重启，约 10 秒不可用。' : ''}`, '应用', false)) return;
    const j = await api('POST', `/updates/${uid}/apply`, { restart_instances: ri() }); m.close(); openJob(j.id);
  });
  const rb = $('[data-rb]', m.el);
  if (rb) rb.onclick = guard(async () => {
    if (!await confirmBox(`回滚更新 ${u.version}，恢复到应用之前的文件？`, '回滚')) return;
    const j = await api('POST', `/updates/${uid}/rollback`, { restart_instances: true }); m.close(); openJob(j.id);
  });
}

async function openJob(jid) {
  const m = modal({ title: '运维任务', icon: 'terminal', size: 'max-w-3xl', body: '<div data-h class="text-xs text-slate-500 mb-2">加载中…</div><div data-log class="text-[11px] font-mono bg-slate-900 text-slate-200 rounded-lg p-3 h-[55vh] overflow-auto whitespace-pre-wrap"></div>',
    footer: '<span data-st class="text-xs"></span><button data-close class="btn-ghost">关闭</button>' });
  const color = { error: 'text-rose-400', warn: 'text-amber-300', ok: 'text-emerald-400', info: 'text-slate-200' };
  let down = 0;
  while (document.body.contains(m.el)) {
    try {
      const j = await api('GET', `/jobs/${jid}`);
      down = 0;
      $('[data-h]', m.el).innerHTML = `<b class="text-slate-800">${esc(j.title)}</b> · 开始 ${dt(j.started)}${j.by ? ' · ' + esc(j.by) : ''}`;
      const L = $('[data-log]', m.el), atEnd = L.scrollTop + L.clientHeight >= L.scrollHeight - 20;
      L.innerHTML = j.log.map(l => `<div class="${color[l.level] || ''}"><span class="text-slate-500">${new Date(l.t * 1000).toLocaleTimeString('zh-CN', { hour12: false })}</span>  ${esc(l.msg)}</div>`).join('');
      if (atEnd) L.scrollTop = L.scrollHeight;
      $('[data-st]', m.el).innerHTML = chip(JOB_ST, j.status);
      if (j.status !== 'running') { if (O.alive) render(O.el); break; }
    } catch (e) {
      if (e.status === 401) { $('[data-st]', m.el).textContent = '平台已重启，请重新登录后查看结果'; break; }
      down++;
      $('[data-st]', m.el).textContent = down > 40 ? '平台 1 分钟内没有恢复：请在手机 Termux 里执行 bash ~/start_agv.sh，或查看 ~/hub.log'
        : down > 2 ? '平台重启中，等待恢复…' : e.message;
    }
    await sleep(1500);
  }
}

// ---------------------------------------------------------------------------- App 管理
async function tabApps(body) {
  const d = await api('GET', '/apps');
  const inst = d.installed || {};
  const devs = (d.adb?.devices || []).filter(x => x.state === 'device');
  body.innerHTML = `
    ${d.helper ? '' : `<div class="flex gap-2 text-amber-800 bg-amber-50 border border-amber-200 rounded-lg px-3 py-2 mb-4 text-xs"><i data-lucide="alert-triangle" class="w-3.5 h-3.5 mt-px shrink-0"></i>
      <div>安卓助手没有运行，不能查询或安装 App。它随 <span class="font-mono">start_agv.sh</span> 启动；也可在 Termux 里执行 <span class="font-mono">nohup python ~/android_helper.py &gt; ~/android_helper.log 2&gt;&amp;1 &amp;</span></div></div>`}
    <div class="grid grid-cols-3 gap-4 mb-5">
      <div class="card p-4 col-span-2"><div class="panel-title mb-2">上传 APK</div>
        <div class="flex items-center gap-3"><input type="file" accept=".apk" class="inp flex-1" data-file><button class="btn-primary" data-up><i data-lucide="upload" class="w-3.5 h-3.5"></i>上传</button></div>
        <div class="text-[11px] text-slate-500 mt-2">上传后自动读出包名和版本号。安装方式：<b>手机确认</b> = 手机上弹出系统安装框，点一次「安装」；<b>静默安装</b> = 通过手机自己的无线调试 adb，不用碰手机 (需先在右侧配对连接)。</div></div>
      <div class="card p-4"><div class="panel-title mb-2">外屏面板</div>
        <div class="text-xs text-slate-600 mb-3">已安装版本号: <b class="font-mono">${inst['com.agvsim.cover']?.installed ? inst['com.agvsim.cover'].versionCode : '未安装'}</b></div>
        <button class="btn-soft" data-panel ${d.helper ? '' : 'disabled'}><i data-lucide="monitor-smartphone" class="w-3.5 h-3.5"></i>打开外屏面板</button></div></div>
    <div class="panel-title mb-2">APK 仓库</div>
    <div class="card overflow-hidden mb-6"><table class="w-full text-xs"><thead class="bg-slate-50 text-slate-500"><tr><th class="text-left px-4 py-2.5">应用</th><th class="text-left px-4">版本</th>
      <th class="text-left px-4">手机上</th><th class="text-left px-4">大小 / 上传</th><th class="text-right px-4">操作</th></tr></thead>
      <tbody>${d.apps.map(a => { const i = inst[a.package]; const same = i?.installed && String(i.versionCode) === String(a.versionCode);
        return `<tr class="border-t border-slate-100"><td class="px-4 py-2.5"><div class="font-semibold">${esc(a.label || a.filename)}</div><div class="font-mono text-[10px] text-slate-400">${esc(a.package)}</div></td>
        <td class="px-4 font-mono">${esc(a.versionName || '—')} <span class="text-slate-400">(${esc(a.versionCode)})</span></td>
        <td class="px-4">${i == null ? '<span class="text-slate-400">—</span>' : i.installed ? `<span class="${same ? 'text-emerald-600' : 'text-slate-600'}">已装 ${esc(i.versionCode)}${same ? ' (就是这个版本)' : ''}</span>` : '<span class="text-slate-400">未安装</span>'}</td>
        <td class="px-4 text-slate-500">${bytes(a.size)} · ${dt(a.uploaded)}</td>
        <td class="px-4 text-right whitespace-nowrap"><button class="btn-soft !py-1" data-ins="${a.id}" data-mode="prompt" ${d.helper ? '' : 'disabled'}>手机确认安装</button>
          <button class="btn-primary !py-1" data-ins="${a.id}" data-mode="adb" ${devs.length ? '' : 'disabled title="先在下方连接无线调试 adb"'}>静默安装</button>
          <button class="btn-danger !py-1" data-del="${a.id}">删除</button></td></tr>`; }).join('') || '<tr><td colspan="5" class="px-4 py-6 text-center text-slate-400">还没有上传 APK</td></tr>'}</tbody></table></div>
    <div class="panel-title mb-2">无线调试 adb (静默安装用)</div>
    <div class="card p-4 text-xs">
      <div class="text-slate-600 leading-relaxed mb-3">手机上: 设置 → 开发者选项 → <b>无线调试</b> 打开 → 「使用配对码配对设备」，把弹窗里的<b>端口</b>和 <b>6 位配对码</b>填到下面配对；配对一次即可。然后把无线调试主页面「IP 地址和端口」里的<b>端口</b>填到连接。手机重启或换网络后端口会变，需要重新连接。</div>
      <div class="flex flex-wrap items-end gap-3 mb-3">
        <div><label class="lbl">配对端口</label><input class="inp w-28 font-mono" data-pport></div><div><label class="lbl">配对码</label><input class="inp w-28 font-mono" data-pcode maxlength="6"></div>
        <button class="btn-soft" data-pair ${d.helper ? '' : 'disabled'}>配对</button><span class="w-px h-8 bg-slate-200 mx-1"></span>
        <div><label class="lbl">连接端口</label><input class="inp w-28 font-mono" data-cport></div><button class="btn-soft" data-conn ${d.helper ? '' : 'disabled'}>连接</button>
        <button class="btn-ghost" data-mdns ${d.helper ? '' : 'disabled'}>自动发现端口</button></div>
      <div>已连接设备: ${(d.adb?.devices || []).map(x => `<span class="font-mono px-1.5 py-0.5 rounded ${x.state === 'device' ? 'bg-emerald-50 text-emerald-700' : 'bg-slate-100 text-slate-500'}">${esc(x.serial)} ${esc(x.state)}</span> <button class="text-rose-600 underline" data-disc="${esc(x.serial)}">断开</button>`).join(' ') || '<span class="text-slate-400">无</span>'}</div>
      <pre data-out class="mt-3 text-[11px] font-mono bg-slate-50 rounded p-2 whitespace-pre-wrap hidden"></pre></div>`;
  const out = (t) => { const p = $('[data-out]', body); p.textContent = t; p.classList.remove('hidden'); };
  $('[data-up]', body).onclick = guard(async () => {
    const f = $('[data-file]', body).files[0];
    if (!f) throw new Error('请选择 APK 文件');
    const r = await api('POST', `/apps/upload?filename=${encodeURIComponent(f.name)}`, f);
    toast(`${r.label || r.package} ${r.versionName || ''} 已上传`, 'ok'); render(O.el);
  });
  $('[data-panel]', body).onclick = guard(async () => { const r = await api('POST', '/panel', { action: 'open' }); toast(r.msg, 'ok'); });
  $$('[data-ins]', body).forEach(b => b.onclick = guard(async () => {
    const adb = b.dataset.mode === 'adb';
    if (!await confirmBox(adb ? '通过无线调试 adb 静默安装这个 APK？' : '在手机上弹出安装界面？之后需要在手机上点「安装」。', '安装', false)) return;
    b.disabled = true; const t = b.textContent; b.textContent = '安装中…';
    try { const r = await api('POST', `/apps/${b.dataset.ins}/install`, { mode: b.dataset.mode }); toast(r.msg || '完成', 'ok'); }
    finally { b.disabled = false; b.textContent = t; }
    setTimeout(() => O.alive && O.tab === 'app' && render(O.el), 1500);
  }));
  $$('[data-del]', body).forEach(b => b.onclick = guard(async () => { if (!await confirmBox('删除这个 APK？')) return; await api('DELETE', `/apps/${b.dataset.del}`); render(O.el); }));
  $('[data-pair]', body).onclick = guard(async () => { const r = await api('POST', '/adb/pair', { port: $('[data-pport]', body).value, code: $('[data-pcode]', body).value }); out(r.output); toast('配对成功', 'ok'); });
  $('[data-conn]', body).onclick = guard(async () => { const r = await api('POST', '/adb/connect', { port: $('[data-cport]', body).value }); out(r.output); render(O.el); });
  $('[data-mdns]', body).onclick = guard(async () => { const r = await api('POST', '/adb/mdns', {}); out(r.output || '(没有发现)'); });
  $$('[data-disc]', body).forEach(b => b.onclick = guard(async () => { await api('POST', '/adb/disconnect', { serial: b.dataset.disc }); render(O.el); }));
}

// ---------------------------------------------------------------------------- 服务与日志
async function tabServices(body) {
  const [s, lg] = await Promise.all([api('GET', '/services'), api('GET', '/logs')]);
  $('[data-ver]', O.el).textContent = `当前版本 ${s.hub.version}${s.hub.title ? ' · ' + s.hub.title : ''}`;
  const kv = (k, v) => `<div class="kv"><span>${k}</span><span>${v}</span></div>`;
  const local = s.nodes.filter(n => n.local_code);
  body.innerHTML = `<div class="grid grid-cols-2 gap-4 mb-5">
    <div class="card p-4"><div class="flex justify-between items-center mb-2"><span class="panel-title">资源平台</span><button class="btn-soft !py-1" data-rh><i data-lucide="rotate-cw" class="w-3.5 h-3.5"></i>重启平台</button></div>
      ${kv('版本', `<span class="font-mono">${esc(s.hub.version)}</span>`)}${kv('进程', `${s.hub.pid} · Python ${esc(s.hub.python)}`)}${kv('已运行', ago(s.hub.started).replace('前', ''))}${kv('代码目录', `<span class="font-mono">${esc(s.hub.root)}</span>`)}</div>
    <div class="card p-4"><div class="flex justify-between items-center mb-2"><span class="panel-title">节点代理</span><button class="btn-soft !py-1" data-ra ${local.length ? '' : 'disabled title="没有运行在本机代码目录上的节点代理"'}><i data-lucide="rotate-cw" class="w-3.5 h-3.5"></i>重启本机节点代理</button></div>
      ${s.nodes.map(n => kv(`${esc(n.name)} <span class="text-slate-400 font-mono">${esc(n.host)}</span>`, `${n.online ? '<span class="text-emerald-600">在线</span>' : '<span class="text-slate-400">离线</span>'} · ${esc(n.runtime || '—')}${n.local_code ? ' · 本机代码' : ''}`)).join('') || '<div class="text-xs text-slate-400">无</div>'}
      <div class="text-[11px] text-slate-500 mt-2">重启节点代理时，它上面运行中的实例会先停止，代理上线后自动重新部署。</div></div>
    <div class="card p-4"><div class="panel-title mb-2">运行中的实例</div>${s.instances.map(i => kv(`${esc(i.id)} · ${esc(i.name)}`, esc(i.status))).join('') || '<div class="text-xs text-slate-400">无</div>'}</div>
    <div class="card p-4"><div class="panel-title mb-2">安卓助手</div>${s.helper ? kv('状态', `<span class="text-emerald-600">运行中</span> · pid ${s.helper.pid}`) + kv('adb 设备', (s.helper.adb || []).map(d => esc(d.serial + ' ' + d.state)).join(', ') || '无')
      : `<div class="text-xs text-slate-500">${s.termux ? '未运行 (随 start_agv.sh 启动)' : '平台不在安卓设备上，无需安卓助手'}</div>`}</div></div>
    <div class="panel-title mb-2">日志文件</div>
    <div class="card overflow-hidden"><table class="w-full text-xs"><tbody>${lg.logs.map(l => `<tr class="border-t border-slate-100 first:border-0"><td class="px-4 py-2 font-mono">${esc(l.name)}</td>
      <td class="px-4 text-slate-500">${bytes(l.size)}</td><td class="px-4 text-slate-500">${dt(l.mtime)}</td>
      <td class="px-4 text-right"><a class="btn-ghost !py-1" href="/api/hub/admin/logs/${encodeURIComponent(l.name)}?token=${encodeURIComponent(getTok())}" download><i data-lucide="download" class="w-3.5 h-3.5"></i>下载</a></td></tr>`).join('') ||
      '<tr><td class="px-4 py-4 text-center text-slate-400">没有找到日志文件 (只在手机 Termux 部署时提供)</td></tr>'}</tbody></table></div>`;
  $('[data-rh]', body).onclick = guard(async () => {
    if (!await confirmBox('重启资源平台？约 10 秒不可用，运行中的仿真不受影响。', '重启')) return;
    const r = await api('POST', '/restart', { target: 'hub' }); toast(r.msg, 'ok');
    await sleep(4000);
    for (let k = 0; k < 30; k++) { try { await http('GET', '/api/hub/health'); break; } catch (e) { await sleep(1000); } }
    render(O.el);
  });
  const ra = $('[data-ra]', body);
  if (ra) ra.onclick = guard(async () => {
    if (!await confirmBox('重启本机节点代理？上面运行中的实例会先停止，代理重新上线后自动重新部署。', '重启')) return;
    const j = await api('POST', '/restart', { target: 'agent' }); openJob(j.id);
  });
  O.timer = setInterval(() => { if (O.alive && O.tab === 'svc' && !document.querySelector('body > .fixed.inset-0.z-50')) render(O.el); }, 10000);
}

// ---------------------------------------------------------------------------- 设置
async function tabSettings(body) {
  body.innerHTML = `<div class="card p-4 max-w-md"><div class="panel-title mb-3">修改管理员密码</div>
    <input type="password" class="inp mb-2" data-old placeholder="当前密码" autocomplete="current-password">
    <input type="password" class="inp mb-2" data-new placeholder="新密码 (至少 6 位)" autocomplete="new-password">
    <input type="password" class="inp mb-3" data-new2 placeholder="再输入一次新密码" autocomplete="new-password">
    <button class="btn-primary" data-go>修改</button></div>`;
  $('[data-go]', body).onclick = guard(async () => {
    const n = $('[data-new]', body).value;
    if (n !== $('[data-new2]', body).value) throw new Error('两次输入的新密码不一致');
    const r = await api('POST', '/password', { old: $('[data-old]', body).value, new: n });
    setTok(r.token); toast('密码已修改', 'ok'); render(O.el);
  });
}
