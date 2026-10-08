// ============================================================================
// 公共: API 客户端、用户、提示、弹窗、格式化、图标
// ============================================================================
export const $ = (s, r = document) => r.querySelector(s);
export const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));
export const esc = (s) => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
export const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '—' : (typeof v === 'number' ? v.toFixed(d) : String(v));
export const sleep = (ms) => new Promise(r => setTimeout(r, ms));

// ---------------------------------------------------------------- 用户 (姓名即身份)
export const user = {
  get name() { try { return localStorage.getItem('amr.user') || ''; } catch (e) { return ''; } },
  set name(v) { try { localStorage.setItem('amr.user', v); } catch (e) { } },
};
export function ensureUser() {
  if (!user.name) {
    const v = (prompt('请输入使用人姓名 (用于控制权与部署记录)', '') || '').trim();
    user.name = v || '访客';
  }
  return user.name;
}

// ---------------------------------------------------------------- HTTP
export class ApiError extends Error { constructor(status, msg, code) { super(msg); this.status = status; this.code = code; } }
export async function http(method, url, body, headers = {}) {
  const h = Object.assign({ 'X-User': encodeURIComponent(user.name || '') }, headers);
  let data = body;
  if (body !== undefined && !(body instanceof Blob) && !(body instanceof ArrayBuffer) && !(body instanceof File)) {
    h['Content-Type'] = 'application/json';
    data = JSON.stringify(body);
  }
  let r;
  try {
    r = await fetch(url, { method, headers: h, body: data });
  } catch (e) {
    throw new ApiError(0, `网络错误: ${e.message}`);
  }
  const ct = r.headers.get('content-type') || '';
  const j = ct.includes('json') ? await r.json().catch(() => ({})) : null;
  // 错误体按 RFC 9457 (application/problem+json): detail 为说明、code 为错误码；旧服务的 {error:{code,message}} 也兼容
  if (!r.ok) throw new ApiError(r.status, j?.detail || j?.error?.message || j?.message || r.statusText, j?.code || j?.error?.code);
  return j !== null ? j : r;
}
export const hub = {
  get: (p) => http('GET', '/api/hub' + p),
  post: (p, b) => http('POST', '/api/hub' + p, b === undefined ? {} : b),
  patch: (p, b) => http('PATCH', '/api/hub' + p, b),
  put: (p, b) => http('PUT', '/api/hub' + p, b),
  del: (p) => http('DELETE', '/api/hub' + p),
};
// 上传 (带进度): XHR
export function upload(url, file, onProgress) {
  return new Promise((resolve, reject) => {
    const x = new XMLHttpRequest();
    x.open('POST', url);
    x.setRequestHeader('X-User', encodeURIComponent(user.name || ''));
    x.upload.onprogress = (e) => onProgress && e.lengthComputable && onProgress(e.loaded / e.total);
    x.onload = () => {
      let j = {};
      try { j = JSON.parse(x.responseText || '{}'); } catch (e) { }
      if (x.status >= 400) reject(new ApiError(x.status, j.detail || j.error?.message || x.statusText, j.code || j.error?.code)); else resolve(j);
    };
    x.onerror = () => reject(new ApiError(0, '上传失败 (网络)'));
    x.send(file);
  });
}
export function download(url, filename) {
  const a = document.createElement('a');
  a.href = url; if (filename) a.download = filename;
  document.body.appendChild(a); a.click(); a.remove();
}
export function downloadJSON(obj, filename) {
  const b = new Blob([JSON.stringify(obj, null, 2)], { type: 'application/json' });
  const u = URL.createObjectURL(b); download(u, filename); setTimeout(() => URL.revokeObjectURL(u), 3000);
}

// ---------------------------------------------------------------- 提示
export function toast(msg, kind = 'info') {
  const box = $('#toast-box');
  const el = document.createElement('div');
  const c = { info: 'bg-slate-800', ok: 'bg-emerald-600', warn: 'bg-amber-500', err: 'bg-rose-600' }[kind] || 'bg-slate-800';
  el.className = `${c} text-white text-xs font-medium px-4 py-2.5 rounded-lg shadow-lg fade-in max-w-md`;
  el.textContent = msg;
  box.appendChild(el);
  setTimeout(() => { el.style.opacity = '0'; el.style.transition = 'opacity .3s'; setTimeout(() => el.remove(), 300); }, kind === 'err' ? 5000 : 2800);
}
export function guard(fn) {
  return async (...a) => {
    try { return await fn(...a); } catch (e) { toast(e.message || String(e), 'err'); console.error(e); }
  };
}

// ---------------------------------------------------------------- 图标 (lucide 本地)
export function icons(root) {
  if (window.lucide) window.lucide.createIcons({ attrs: { 'stroke-width': 2 }, nameAttr: 'data-lucide', root: root || document });
}

// ---------------------------------------------------------------- 弹窗
export function modal({ title, icon = 'layers', badge = '', body = '', footer = '', size = 'max-w-2xl', full = false, onClose, headExtra = '' }) {
  const wrap = document.createElement('div');
  wrap.className = 'fixed inset-0 z-50 flex items-center justify-center bg-slate-900/40 backdrop-blur-sm p-4';
  wrap.innerHTML = `
    <div class="bg-white rounded-2xl shadow-2xl w-full ${full ? 'max-w-[1400px] h-[92vh]' : size} flex flex-col overflow-hidden fade-in">
      <div class="px-6 py-4 border-b border-slate-100 flex items-center justify-between shrink-0">
        <div class="flex items-center gap-2.5 min-w-0">
          <div class="w-8 h-8 rounded-lg bg-blue-50 text-blue-600 flex items-center justify-center shrink-0"><i data-lucide="${icon}" class="w-4 h-4"></i></div>
          <h3 class="text-sm font-bold text-slate-800 truncate">${title}</h3>${badge}
        </div>
        <div class="flex items-center gap-2">${headExtra}
          <button data-close class="p-1.5 rounded-md text-slate-400 hover:text-slate-700 hover:bg-slate-100"><i data-lucide="x" class="w-4 h-4"></i></button>
        </div>
      </div>
      <div data-body class="${full ? 'flex-1 min-h-0 overflow-hidden' : 'p-6 overflow-y-auto max-h-[72vh]'}">${body}</div>
      ${footer ? `<div data-foot class="px-6 py-4 bg-slate-50 border-t border-slate-100 flex items-center justify-between shrink-0">${footer}</div>` : ''}
    </div>`;
  document.body.appendChild(wrap);
  const close = () => { wrap.remove(); onClose && onClose(); };
  wrap.addEventListener('mousedown', (e) => { if (e.target === wrap) close(); });
  $$('[data-close]', wrap).forEach(b => b.addEventListener('click', close));
  icons(wrap);
  return { el: wrap, body: $('[data-body]', wrap), foot: $('[data-foot]', wrap), close };
}
export function confirmBox(msg, okText = '确定', danger = true) {
  return new Promise(res => {
    const m = modal({
      title: '请确认', icon: danger ? 'alert-triangle' : 'help-circle', size: 'max-w-md',
      body: `<p class="text-sm text-slate-600 leading-relaxed">${msg}</p>`,
      footer: `<button data-no class="px-4 py-1.5 border border-slate-200 bg-white rounded-lg text-xs font-semibold text-slate-600">取消</button>
               <button data-ok class="px-5 py-1.5 ${danger ? 'bg-rose-600 hover:bg-rose-700' : 'bg-blue-600 hover:bg-blue-700'} text-white rounded-lg text-xs font-bold">${okText}</button>`,
      onClose: () => res(false),
    });
    $('[data-no]', m.el).onclick = () => { m.close(); };
    $('[data-ok]', m.el).onclick = () => { res(true); m.el.remove(); };
  });
}

// ---------------------------------------------------------------- 格式
export function dur(sec) {
  if (sec === null || sec === undefined) return '—';
  sec = Math.max(0, Math.floor(sec));
  const h = Math.floor(sec / 3600), m = Math.floor(sec % 3600 / 60), s = sec % 60;
  return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}
export function mmss(sec) {
  sec = Math.max(0, sec || 0);
  const m = Math.floor(sec / 60), s = sec - m * 60;
  return `${String(m).padStart(2, '0')}:${s.toFixed(1).padStart(4, '0')}`;
}
export function bytes(n) {
  if (!n) return '—';
  const u = ['B', 'KB', 'MB', 'GB'];
  let i = 0;
  while (n >= 1024 && i < u.length - 1) { n /= 1024; i++; }
  return `${n.toFixed(i ? 1 : 0)} ${u[i]}`;
}
export function ago(t) {
  if (!t) return '—';
  const s = Date.now() / 1000 - t;
  if (s < 60) return `${Math.floor(s)} 秒前`;
  if (s < 3600) return `${Math.floor(s / 60)} 分钟前`;
  if (s < 86400) return `${Math.floor(s / 3600)} 小时前`;
  return new Date(t * 1000).toLocaleString('zh-CN', { hour12: false });
}
export function dt(t) { return t ? new Date(t * 1000).toLocaleString('zh-CN', { hour12: false }) : '—'; }

export const STATUS = {
  running: ['运行中', 'bg-emerald-50 text-emerald-700 border-emerald-200', 'bg-emerald-500'],
  deploying: ['部署中', 'bg-blue-50 text-blue-700 border-blue-200', 'bg-blue-500'],
  degraded: ['异常', 'bg-amber-50 text-amber-700 border-amber-200', 'bg-amber-500'],
  stopping: ['终止中', 'bg-slate-100 text-slate-600 border-slate-200', 'bg-slate-400'],
  stopped: ['已终止', 'bg-slate-100 text-slate-500 border-slate-200', 'bg-slate-400'],
  error: ['部署失败', 'bg-rose-50 text-rose-700 border-rose-200', 'bg-rose-500'],
  idle: ['空闲', 'bg-emerald-50 text-emerald-700 border-emerald-200', 'bg-emerald-500'],
  offline: ['离线', 'bg-slate-100 text-slate-500 border-slate-200', 'bg-slate-400'],
};
export function badge(st, extra = '') {
  const s = STATUS[st] || [st, 'bg-slate-100 text-slate-600 border-slate-200', 'bg-slate-400'];
  return `<span class="px-2 py-0.5 rounded-full text-[11px] font-bold border ${s[1]} inline-flex items-center gap-1 whitespace-nowrap"><span class="w-1.5 h-1.5 rounded-full ${s[2]}"></span>${s[0]}${extra}</span>`;
}
export const CHASSIS_LABEL = { single_steer: '单舵轮', dual_steer: '双舵轮', diff_drive: '差速双驱', quad_steer: '四舵轮', omni: '全向', mecanum: '麦克纳姆轮' };
export const TAG_STYLE = {
  TSK: 'bg-purple-50 text-purple-700 border-purple-200', NAV: 'bg-blue-50 text-blue-700 border-blue-200',
  OBS: 'bg-amber-50 text-amber-700 border-amber-200', INJ: 'bg-orange-50 text-orange-700 border-orange-200',
  RST: 'bg-slate-100 text-slate-700 border-slate-300', DRV: 'bg-emerald-50 text-emerald-700 border-emerald-200',
  SAF: 'bg-rose-50 text-rose-700 border-rose-200', SYS: 'bg-slate-50 text-slate-500 border-slate-200',
};
export const STEP_LABEL = { move: '导航移动 (Move)', lift: '顶升取货 (Lift)', drop: '下降放货 (Drop)', wait: '等待 (Wait)' };
