import { preferences } from './composables/preferences';
export async function api(path, options = {}) {
  const {timeoutMs,signal:externalSignal,...requestOptions}=options;
  const controller=timeoutMs>0?new AbortController():null;
  const abort=()=>controller?.abort(externalSignal?.reason);
  if(externalSignal?.aborted)abort();else externalSignal?.addEventListener('abort',abort,{once:true});
  const timer=controller?setTimeout(()=>controller.abort(),timeoutMs):null;
  try{
  const response = await fetch(path, {
    cache: 'no-store',
    ...requestOptions,
    signal:controller?.signal||externalSignal,
    headers: { ...(options.body instanceof FormData ? {} : { 'Content-Type': 'application/json' }), ...options.headers },
  });
  const type = response.headers.get('content-type') || '';
  const payload = type.includes('application/json') ? await response.json() : null;
  if (!response.ok) throw new Error(payload?.error || `请求失败（${response.status}）`);
  return payload;
  }finally{if(timer)clearTimeout(timer);externalSignal?.removeEventListener('abort',abort);}
}

export const postJson = (path, body) => api(path, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body),
});

export function formatTime(value, options = {}) {
  if (!value) return '--';
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return String(value);
  return new Intl.DateTimeFormat('zh-CN', {
    hour12: preferences.hour12,
    month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit',
    ...(options.seconds ? { second: '2-digit' } : {}),
  }).format(date);
}

export function downloadCsv(filename, rows) {
  const escape = (value) => `"${String(value ?? '').replaceAll('"', '""')}"`;
  const content = `\ufeff${rows.map((row) => row.map(escape).join(',')).join('\r\n')}`;
  const url = URL.createObjectURL(new Blob([content], { type: 'text/csv;charset=utf-8' }));
  const link = document.createElement('a');
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

const inspectionControlOrigin=import.meta.env.VITE_INSPECTION_CONTROL_ORIGIN||`${window.location.protocol}//${window.location.hostname}:8019`;
export function inspectionControlApi(path,options={}){
  return api(inspectionControlOrigin+path,options);
}
export const postInspectionControl=(path,body)=>inspectionControlApi(path,{method:'POST',body:JSON.stringify(body)});
