// ============================================================================
// 入口: hash 路由  #/platform/<page>  |  #/wb[/<实例>/<tab>]  |  #/replay
// ============================================================================
import { $, $$, hub, icons, user, ensureUser, toast } from './core.js';
import * as Platform from './platform.js';
import * as WB from './workbench.js';

const state = { view: null };

function setView(v) {
  state.view = v;
  $('#view-platform').classList.toggle('hidden', v !== 'platform');
  $('#view-workbench').classList.toggle('hidden', v !== 'workbench');
  $('#nav-platform').classList.toggle('active', v === 'platform');
  $('#nav-workbench').classList.toggle('active', v === 'workbench');
}

async function route() {
  document.querySelectorAll('body > .fixed.inset-0.z-50').forEach(m => m.querySelector('[data-close]')?.click());
  const h = location.hash.replace(/^#\/?/, '') || 'platform/overview';
  const [a, b, c] = h.split('/');
  if (a === 'wb') {
    setView('workbench');
    Platform.leave();
    await WB.open(b || null, c || null);
  } else {
    setView('platform');
    WB.leave();
    const page = b || 'overview';
    $$('#view-platform .nav-item').forEach(n => n.classList.toggle('active', n.dataset.page === page));
    await Platform.show(page, c);
  }
  icons();
}

function renderUser() {
  $('#user-name').textContent = user.name || '未登录';
  $('#user-avatar').textContent = (user.name || '?').slice(0, 1);
}

async function pill() {
  try {
    const o = await hub.get('/overview');
    const el = $('#hub-pill');
    el.innerHTML = `<span class="w-2 h-2 rounded-full bg-emerald-500"></span> 平台在线 · 节点 ${o.nodes.online}/${o.nodes.total} · 实例 ${o.instances.active}`;
    const cnt = { instances: o.instances.active, nodes: o.nodes.total, models: o.models.total, scenes: o.scenes.total, packages: o.packages.total };
    Object.entries(cnt).forEach(([k, v]) => { const e = $(`[data-cnt="${k}"]`); if (e) e.textContent = v; });
    Platform.setOverview(o);
  } catch (e) {
    $('#hub-pill').innerHTML = `<span class="w-2 h-2 rounded-full bg-rose-500"></span> 平台未连接`;
  }
}

window.addEventListener('hashchange', route);
window.addEventListener('DOMContentLoaded', async () => {
  icons();
  ensureUser(); renderUser();
  $('#user-btn').onclick = () => {
    const v = (prompt('使用人姓名', user.name) || '').trim();
    if (v) { user.name = v; renderUser(); toast(`当前使用人: ${v}`, 'ok'); }
  };
  $('#btn-new-sim').onclick = () => Platform.openDeploy();
  $('#nav-workbench').onclick = (e) => {
    const last = WB.lastInstance();
    if (last) { e.preventDefault(); location.hash = `#/wb/${last}`; }
  };
  await pill();
  setInterval(pill, 5000);
  route();
});
