// Camera startup and device sampling have separate update intervals. Keep a
// bounded transition across those intervals without hiding real hardware faults.
export const CAMERA_SWITCH_TIMEOUT = 40000;
const STATUS_FRESHNESS = 8000;

export function createCameraActivity() {
  return { operation: 0, pending: false, mode: '', startedAt: 0, deadline: 0,
    status: null, statusAt: 0, backendSince: 0, backendGeneration: null,
    failure: '', failureAt: 0 };
}

export function beginCameraChange(activity, mode, now = Date.now()) {
  activity.operation++;
  Object.assign(activity, { pending: true, mode, startedAt: now,
    deadline: now + CAMERA_SWITCH_TIMEOUT, status: null, statusAt: 0,
    failure: '', failureAt: 0 });
  return activity.operation;
}

function trackBackend(activity, camera, now) {
  if (!camera) return;
  const changed = activity.backendGeneration != null && camera.generation != null
    && camera.generation !== activity.backendGeneration;
  if (camera.switching || (changed && !camera.online)) {
    if (!activity.backendSince || changed) activity.backendSince = now;
  } else if (camera.online) activity.backendSince = 0;
  if (camera.generation != null) activity.backendGeneration = camera.generation;
}

export function acceptCameraStatus(activity, payload, now = Date.now(), requestedAt = now) {
  const item = payload?.cameras?.[0];
  if (!item || requestedAt < activity.startedAt) return;
  const camera = { ...item, active_mode: payload.active_mode,
    generation: payload.generation, switching: payload.switching };
  if (activity.status?.generation != null && camera.generation < activity.status.generation) return;
  activity.status = camera; activity.statusAt = now;
  trackBackend(activity, camera, now);
  if (camera.online && !camera.switching) { activity.failure = ''; activity.failureAt = 0; }
}

export function acceptCameraSnapshot(activity, snapshot, now = Date.now()) {
  const camera = snapshot?.camera;
  if (!camera) return;
  // A cached snapshot from an older generation cannot undo a newer direct status.
  if (activity.status?.generation != null && camera.generation < activity.status.generation) return;
  trackBackend(activity, camera, now);
  const healthy = camera.online && !camera.switching
    && snapshot.health?.hardware?.find(row => row.id === 'camera')?.state === 'ok';
  if (healthy && (!activity.mode || camera.active_mode === activity.mode)) {
    activity.pending = false; activity.failure = ''; activity.failureAt = 0;
  }
}

export function finishCameraChange(activity, operation, failure = '', now = Date.now()) {
  if (operation !== activity.operation) return;
  activity.pending = false;
  activity.failure = failure; activity.failureAt = failure ? now : 0;
}

export function withCameraHealth(data, snapshot, activity, now = Date.now()) {
  if (!data) return data;
  const original = data.hardware?.find(row => row.id === 'camera');
  if (!original) return data;
  const sampled = snapshot?.camera || {};
  const latest = activity.status;
  const fresh = latest && now >= activity.statusAt && now - activity.statusAt <= STATUS_FRESHNESS
    && (sampled.generation == null || latest.generation >= sampled.generation);
  const localPending = activity.pending && now < activity.deadline;
  const backendTimedOut = activity.backendSince > 0
    && now - activity.backendSince >= CAMERA_SWITCH_TIMEOUT;
  const timedOut = activity.pending && now >= activity.deadline || backendTimedOut;
  const explicitFailure = fresh && !latest.switching && !latest.online && Boolean(latest.last_error);
  const backendPending = activity.backendSince > 0
    && now - activity.backendSince < CAMERA_SWITCH_TIMEOUT && !explicitFailure;
  const justFailed = activity.failure && now - activity.failureAt <= STATUS_FRESHNESS;
  const ready = fresh && latest.online && !latest.switching
    && (latest.frame_age_ms == null || latest.frame_age_ms <= 5000);
  let state, status, reason, advice = '';
  if (localPending || (!timedOut && !justFailed && backendPending)) {
    state = 'unknown'; status = '切换中'; reason = '正在切换相机模式，等待新画面';
  } else if (timedOut || justFailed) {
    state = 'error'; status = '异常'; reason = timedOut ? '相机模式切换超时，未收到新画面' : activity.failure;
    advice = '检查相机连接和当前识别模式，稍后重试。';
  } else if (explicitFailure) {
    state = 'error'; status = '异常'; reason = '相机模式启动失败，未收到新画面';
    advice = '检查相机连接和当前识别模式，稍后重试。';
  } else if (ready) {
    state = 'ok'; status = '正常'; reason = '相机画面持续更新';
  } else return data;
  const row = { ...original, state, status, reason, advice };
  const hardware = data.hardware.map(item => item.id === 'camera' ? row : item);
  const metrics = data.metrics || [];
  const issues = (data.issues || []).filter(item => item.id !== 'camera');
  if (state === 'error') issues.push(row);
  const overall = issues.length ? 'error'
    : [...hardware, ...metrics].some(item => item.state === 'unknown') ? 'unknown'
    : hardware.some(item => item.state === 'idle') ? 'idle' : 'ok';
  return { ...data, hardware, issues, state: overall,
    status: overall === 'error' ? '监测异常' : status === '切换中' ? '相机模式切换中'
      : overall === 'ok' ? '监测正常' : overall === 'idle' ? '设备已连接，待机中' : '部分状态待确认' };
}
