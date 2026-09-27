// 工作台共享状态与实例 API (经平台代理 /inst/<id>/ 访问实例网关)
import { http, toast, user } from './core.js';

export const W = {
  iid: null, inst: null, info: null, base: '', tab: '31', tel: {}, stat: {}, lock: { held: false, mine: false },
  token: '', events: [], evAfter: 0, evTags: {}, timers: [], tabs: {}, flows: [], flowSel: null, sceneId: null,
  replay: null, offline: false,
};
export const lockKey = () => `amr.lock.${W.iid}`;
export function saveToken(t) { W.token = t || ''; try { t ? localStorage.setItem(lockKey(), t) : localStorage.removeItem(lockKey()); } catch (e) { } }
export function loadToken() { try { return localStorage.getItem(lockKey()) || ''; } catch (e) { return ''; } }

export function ia(method, path, body) {
  return http(method, W.base + path, body, W.token ? { 'X-Lock-Token': W.token } : {}).catch(e => {
    if (e.status === 423) toast('当前为只读：控制权由其他使用人持有，请先获取控制权', 'warn');
    throw e;
  });
}
export const iget = (p) => ia('GET', p);
export const ipost = (p, b) => ia('POST', p, b === undefined ? {} : b);
export const iput = (p, b) => ia('PUT', p, b);
export const idel = (p) => ia('DELETE', p);
export function needLock() {
  if (!W.lock.mine) { toast('请先获取控制权 (顶栏「获取控制权」)', 'warn'); return false; }
  return true;
}
export function every(ms, fn) { const h = setInterval(fn, ms); W.timers.push(h); return h; }
export const me = () => user.name;
