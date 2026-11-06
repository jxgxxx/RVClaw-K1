import { preferences } from './preferences';
export function pointLabel(id) {
  const index = ['A','B','C'].indexOf(String(id));
  return index >= 0 ? '仪表 ' + String(index + 1).padStart(2,'0') : /ORIGIN/.test(String(id)) ? '返航点' : String(id || '未记录');
}
export function meterName(point) {
  if (!point) return '未记录';
  const name = point.meter_name || point.meter_id || '';
  return name || (/pointer/.test(point.meter_kind||'')?'指针':'数字')+'仪表 '+pointLabel(point.point || point.name || point.waypoint).replace('仪表 ','');
}
export function readingLabel(item) {
  const status = String(item?.status || '').toLowerCase();
  if(item?.result_state==='failure'||['failed','error','unreadable','low_confidence','ambiguous','unsupported','pointer_outside_scale','inference_error','inference_offline','invalid_request','timeout','refused'].includes(status)) return '识别失败';
  if (status === 'out_of_range') return '超出范围';
  if (item?.value != null && String(item.value).trim() !== '') return '识别成功';
  return '未记录';
}
export function badgeColor(text) {
  return /失败|异常|超出|待处理|未连接|离线|中止/.test(text) ? 'red' : /成功|完成|已读数|已处理|正常|已连接|在线/.test(text) ? 'green' : /进行|巡检中|前往/.test(text) ? 'blue' : '';
}
export function readingValue(value) {
  if(value==null||String(value).trim()==='')return '未记录';
  const n=Number(value);
  return Number.isFinite(n)?String(Number(n.toFixed(2))):String(value);
}
export function clock(value) {
  if (!value) return '未记录';
  const date = new Date(typeof value === 'number' && value < 1e12 ? value * 1000 : value);
  return Number.isNaN(date.getTime()) ? '未记录' : date.toLocaleTimeString('zh-CN', { hour12:preferences.hour12 });
}

