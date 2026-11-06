import { reactive, watch } from 'vue';
const key = 'rvclaw-studio-preferences-v2';
const defaults = { refreshSeconds:5, sampleLimit:60, defaultPage:'overview', compact:false, pauseBackground:true, cameraFit:'cover', autoCamera:true, mapRange:'task', routeAnimation:true, alertNotice:true, pageSize:20, hour12:false };
let stored = {};
try { stored = JSON.parse(localStorage.getItem(key) || '{}'); } catch {}
export const preferences = reactive({ ...defaults, ...stored, cameraFit: stored.cameraDisplayVersion === 'fill-v1' ? stored.cameraFit || defaults.cameraFit : defaults.cameraFit, cameraDisplayVersion:'fill-v1', compact: stored.designVersion === 'concept-v1' ? Boolean(stored.compact) : false, designVersion:'concept-v1' });
if(![5,10,20].includes(Number(preferences.pageSize)))preferences.pageSize=10;
watch(preferences, value => {
  localStorage.setItem(key, JSON.stringify(value));
  document.dispatchEvent(new CustomEvent('device-refresh-settings-change', { detail:value }));
  window.dispatchEvent(new CustomEvent('device-refresh-settings-change', { detail:value }));
}, { deep:true });

