import { reactive } from 'vue';
import { api } from '../api';
import { preferences } from './preferences';
import { createCameraActivity, beginCameraChange, acceptCameraStatus, acceptCameraSnapshot, finishCameraChange } from './cameraHealth';

export const device = reactive({
  snapshot: null,
  voiceHealth: null,
  observedAt: Date.now(),
  samples: [],
  error: '',
  refreshing: false,
  lastUpdated: null,
});

export const cameraActivity = reactive(createCameraActivity());
export function beginCameraModeChange(mode) {
  device.observedAt = Date.now();
  return beginCameraChange(cameraActivity, mode, device.observedAt);
}
export function recordCameraStatus(payload, requestedAt = Date.now()) {
  device.observedAt = Date.now();
  acceptCameraStatus(cameraActivity, payload, device.observedAt, requestedAt);
}
export function finishCameraModeChange(operation, failure = '') {
  device.observedAt = Date.now();
  finishCameraChange(cameraActivity, operation, failure, device.observedAt);
}

let timer;
let freshnessTimer;
let running = false;
let refreshMilliseconds = 5000;
let sampleLimit = 60;

function readPreferences() {
  try {
    const preferences = JSON.parse(localStorage.getItem('rvclaw-studio-preferences-v2') || '{}');
    refreshMilliseconds = [3, 5, 10].includes(Number(preferences.refreshSeconds)) ? Number(preferences.refreshSeconds) * 1000 : 5000;
    sampleLimit = [24, 40, 60].includes(Number(preferences.sampleLimit)) ? Number(preferences.sampleLimit) : 60;
  } catch { refreshMilliseconds = 5000; sampleLimit = 60; }
}

export async function refreshDevice() {
  if (device.refreshing) return;
  device.refreshing = true;
  try {
    const requestedAt = performance.now();
    const [deviceResult, voiceResult] = await Promise.allSettled([
      api('/api/device/snapshot', { signal: AbortSignal.timeout(7000) }).then(snapshot => ({
        snapshot, networkLatency: Math.round((performance.now() - requestedAt) * 10) / 10,
      })),
      api('/voice-health.json', { signal: AbortSignal.timeout(4000) }),
    ]);
    device.voiceHealth = voiceResult.status === 'fulfilled' ? voiceResult.value : null;
    device.observedAt = Date.now();
    if (deviceResult.status !== 'fulfilled') throw deviceResult.reason;
    const { snapshot, networkLatency } = deviceResult.value;
    device.snapshot = snapshot;
    acceptCameraSnapshot(cameraActivity, snapshot, device.observedAt);
    device.error = '';
    device.lastUpdated = new Date();
    const cpu = snapshot.cpu || {};
    const policies = cpu.policies || [];
    const clusters = (cpu.temperatures || []).filter((item) => /^(thermal_cluster|cluster\d+_thermal)/.test(item.type) && Number.isFinite(Number(item.celsius)));
    const averageTemperature = clusters.length
      ? clusters.reduce((sum, item) => sum + Number(item.celsius), 0) / clusters.length
      : Number(cpu.temperature_celsius);
    const actual = cpu.current_frequencies_mhz || cpu.actual_frequencies_mhz || null;
    device.samples.push({
      time: new Date(),
      networkLatency,
      temperature: averageTemperature,
      highestTemperature: clusters.length ? Math.max(...clusters.map((item) => Number(item.celsius))) : Number(cpu.temperature_celsius),
      policy0: Number(policies[0]?.frequency_mhz),
      policy8: Number(policies[1]?.frequency_mhz),
      actual: Array.isArray(actual) ? actual.map(Number) : null,
      frameAge: snapshot.network?.frame_age_ms,
      cameraFps: snapshot.camera?.fps,
      cpuUsage: snapshot.cpu?.usage_percent ?? snapshot.resources?.cpu_percent,
      memoryUsage: snapshot.memory?.total_bytes ? snapshot.memory.used_bytes / snapshot.memory.total_bytes * 100 : null,
    });
    while (device.samples.length > sampleLimit) device.samples.shift();
  } catch (error) {
    device.error = error.message || '设备状态读取失败';
  } finally {
    device.refreshing = false;
  }
}

async function pollDevice() {
  if (!running) return;
  if (!document.hidden || !preferences.pauseBackground) await refreshDevice();
  if (running) timer = window.setTimeout(pollDevice, refreshMilliseconds);
}

export function startDevicePolling() {
  if (running) return;
  readPreferences();
  running = true;
  freshnessTimer = window.setInterval(() => { device.observedAt = Date.now(); }, 1000);
  document.addEventListener('device-refresh-settings-change', onPreferencesChange);
  pollDevice();
}

function onPreferencesChange(event) {
  const refreshSeconds = Number(event.detail?.refreshSeconds);
  const nextLimit = Number(event.detail?.sampleLimit);
  refreshMilliseconds = [3, 5, 10].includes(refreshSeconds) ? refreshSeconds * 1000 : 5000;
  sampleLimit = [24, 40, 60].includes(nextLimit) ? nextLimit : 60;
  while (device.samples.length > sampleLimit) device.samples.shift();
  if (running) {
    window.clearTimeout(timer);
    timer = window.setTimeout(pollDevice, refreshMilliseconds);
  }
}

export function stopDevicePolling() {
  running = false;
  window.clearInterval(freshnessTimer);
  window.clearTimeout(timer);
  document.removeEventListener('device-refresh-settings-change', onPreferencesChange);
}
