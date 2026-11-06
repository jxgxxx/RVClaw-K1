<script setup>
import { computed, onMounted, onBeforeUnmount, ref, watch } from 'vue';
import { api } from '../api';
import CameraModeSelect from '../components/CameraModeSelect.vue';
import CameraPreview from '../components/CameraPreview.vue';
import { device } from '../composables/device';
import { preferences } from '../composables/preferences';
import { registerPageRefresher } from '../composables/pageRefresh';
import { pointLabel,readingLabel,badgeColor,clock } from '../composables/presentation';
import { monitoring,monitorColor } from '../composables/monitor';
import { missionProgress } from '../composables/mission';
import { inspectionTask } from '../composables/inspectionState';
const progress=ref(null),progressReceivedAt=ref(0),assets=ref(null),records=ref([]);
const cameraModeSelect=ref(null),cameraStatus=ref(null),cameraSwitching=ref(false);
function acceptCameraStatus(value){if(value?.cameras?.[0])cameraStatus.value={...value.cameras[0],active_mode:value.active_mode,generation:value.generation,switching:value.switching};}
let timer,unregister,loadingProgress=false;
const snapshot=computed(()=>device.snapshot||{}),camera=computed(()=>cameraStatus.value||snapshot.value.camera||{});
watch(()=>snapshot.value.camera,value=>{
 const live=cameraStatus.value;
 if(live&&value?.active_mode===live.active_mode&&value.generation===live.generation&&!value.switching&&(value.online||!live.online))cameraStatus.value=null;
});
const cameraConnecting=computed(()=>cameraSwitching.value||camera.value.switching||monitoring.value.hardware.some(row=>row.id==='camera'&&row.status==='切换中'));
const cpu=computed(()=>snapshot.value.cpu||{});
const latencySamples=computed(()=>device.samples.map(s=>s.networkLatency).filter(Number.isFinite)),latency=computed(()=>device.error?null:latencySamples.value.at(-1));
const temperatureSamples=computed(()=>device.samples.map(s=>s.temperature).filter(Number.isFinite));
const temperatureRange=computed(()=>temperatureSamples.value.length?Math.min(...temperatureSamples.value).toFixed(1)+' – '+Math.max(...temperatureSamples.value).toFixed(1)+' °C':'—');
const temperatures=computed(()=>(cpu.value.temperatures||[]).filter(p=>/^(thermal_cluster|cluster\d+_thermal)/.test(p.type)&&Number.isFinite(Number(p.celsius))));
const temperature=computed(()=>temperatures.value.length?temperatures.value.reduce((s,p)=>s+Number(p.celsius),0)/temperatures.value.length:cpu.value.temperature_celsius);
const displayProgress=computed(()=>inspectionTask(assets.value,progress.value));
const percent=computed(()=>missionProgress(displayProgress.value).percent);
const stage=computed(()=>{
 const step=displayProgress.value?.steps?.find(s=>s.id===displayProgress.value.current_step_id);
 return step?(/ORIGIN/.test(step.id)?'返回起点':step.kind==='waypoint'?'前往'+pointLabel(step.id):step.label):displayProgress.value?.state_text||'等待任务';
});
const events=computed(()=>{
 const items=(progress.value?.steps||[]).filter(s=>s.at).map(s=>({id:s.id,time:s.at,icon:s.kind==='waypoint'?'meter':'route',title:s.kind==='waypoint'?pointLabel(s.id)+' 已到达':/ORIGIN/.test(s.id)?'小车已返回起点':s.label,detail:progress.value?.state_text||'巡检任务'}));
 for(const r of records.value.slice(0,4))items.push({id:r.evidence_id||r.created_at,time:r.created_at,icon:'meter',title:(r.kind==='pointer'?'指针仪表':'数字仪表')+' · '+readingLabel(r),detail:r.value==null?'读数未记录':String(r.value)});
 return items.sort((a,b)=>new Date(typeof b.time==='number'?b.time*1000:b.time)-new Date(typeof a.time==='number'?a.time*1000:a.time)).slice(0,4);
});
const cameraUrl=computed(()=>preferences.autoCamera && camera.value.online?camera.value.stream_url:'');
function nav(page){document.dispatchEvent(new CustomEvent('studio-page-change',{detail:{page}}));}
function progressVersion(value){return JSON.stringify([value?.task_id,value?.state,value?.current_step_id,value?.steps?.map(s=>[s.id,s.status,s.at]),value?.evidence?.dir,value?.evidence?.photo_count]);}
async function loadProgress(){if(loadingProgress||(document.hidden&&preferences.pauseBackground))return;loadingProgress=true;try{const next=await api('/inspection-progress.json'),changed=progressVersion(next)!==progressVersion(progress.value);progress.value=next;progressReceivedAt.value=Date.now();if(changed&&assets.value)assets.value=await api('/api/inspection-assets');}catch{}finally{loadingProgress=false;}}
async function load(){
 const result=await Promise.allSettled([loadProgress(),api('/api/inspection-assets'),api('/api/studio/meter-history'),cameraModeSelect.value?.refresh()]);
 if(result[1].status==='fulfilled')assets.value=result[1].value;
 if(result[2].status==='fulfilled')records.value=result[2].value.records||[];
}
onMounted(()=>{load();timer=setInterval(loadProgress,3000);unregister=registerPageRefresher('overview',load);});
onBeforeUnmount(()=>{clearInterval(timer);unregister?.();});
</script>
<template>
<div class="overview-grid">
 <UiCard title="相机画面" glyph="camera" class="camera-card">
  <template #extra><CameraModeSelect ref="cameraModeSelect" :camera="camera" @status="acceptCameraStatus" @switching="cameraSwitching=$event" :locked="['running','active','in_progress'].includes(progress?.state)"/></template>
  <CameraPreview :src="cameraUrl" :generation="camera.generation" :mode="camera.active_mode" :online="Boolean(camera.online)" :switching="Boolean(cameraConnecting)" :enabled="preferences.autoCamera" :pause-background="preferences.pauseBackground" :fit="preferences.cameraFit" :width="camera.width" :height="camera.height"/>
  <template #foot><span>{{preferences.cameraFit==='cover'?'填满画面':'完整画面'}} · {{camera.width||'—'}} × {{camera.height||'—'}}</span><span>{{camera.fps?Number(camera.fps).toFixed(1):'—'}} 帧/秒</span></template>
 </UiCard>
 <UiCard title="巡检进度" glyph="route"><template #extra><button class="btn plain" @click="nav('inspection')"><UiIcon name="arrow" :size="15"/>查看任务</button></template>
  <div class="card-content overview-patrol"><div class="progress-heading"><div><span class="subtle">{{assets?.map?.name||'巡检任务'}}</span><div class="progress-value number" style="margin-top:8px">{{Math.round(percent)}}<small>%</small></div></div><span class="badge" :class="badgeColor(displayProgress?.state_text||'')">{{displayProgress?.state_text||'等待任务'}}</span></div><div class="progress-bar"><span :style="{width:percent+'%'}"></span></div><div class="mini-map"><TaskMap :map="assets?.map" :route="displayProgress?.route||[]" :progress="percent" :task-key="displayProgress?.task_id" :live-pose="displayProgress?.task_id===progress?.task_id?progress?.live_pose:null" :status-at="progressReceivedAt" :moving="['running','active','in_progress'].includes(displayProgress?.state)&&Boolean(displayProgress?.startup?.first_motion_at||percent>0)"/></div><div class="mini-checkpoints"><span v-for="id in ['A','B','C','ORIGIN']" :key="id"><i :class="displayProgress?.steps?.find(s=>s.id===id)?.status==='done'?'done':''">{{displayProgress?.steps?.find(s=>s.id===id)?.status==='done'?'✓':'·'}}</i>{{id==='ORIGIN'?'返航点':pointLabel(id)}}</span></div></div>
 </UiCard>
 <UiCard title="硬件与运行状态" glyph="chip" class="overview-health-card"><template #extra><button class="btn plain" @click="nav('devices')">RVBench<UiIcon name="arrow" :size="13"/></button></template>
  <div class="overview-health"><div class="health-list" tabindex="0" role="region" aria-label="硬件实时状态"><div v-for="row in monitoring.hardware" :key="row.id" :data-hardware="row.id" class="health-list-row"><span><UiIcon :name="row.id==='voice'?'mic':row.id==='camera'?'camera':row.id==='navigation'?'route':row.id==='localization'?'map':'chip'" :size="15"/>{{row.label}}</span><span class="badge" :class="monitorColor(row.state)">{{row.status}}</span></div><div v-if="!monitoring.hardware.length" class="empty-inline">正在获取硬件状态…</div></div><button v-if="monitoring.issues.length" class="health-issue-link" @click="nav('devices')"><UiIcon name="alert" :size="17"/><span><b>{{monitoring.issues.length}} 项运行异常需要关注</b><small>{{monitoring.issues[0].reason}}</small></span><UiIcon name="arrow" :size="14"/></button><div v-else class="health-foot green"><UiIcon name="check" :size="15"/>{{monitoring.status}}</div></div>
 </UiCard>
 <UiCard title="处理器平均温度" glyph="temp" class="metric-card"><div class="card-content"><div class="metric-value"><div class="temperature-value"><b class="number">{{temperature==null?'—':Number(temperature).toFixed(1)}}</b><span class="muted">°C</span></div><span v-if="temperature!=null" class="badge" :class="Number(temperature)>=80?'red':'green'">{{Number(temperature)>=80?'温度偏高':'正常'}}</span></div><TrendPlot :values="temperatureSamples" :times="device.samples.map(s=>s.time)" :plot-height="205" unit="°C" label="处理器平均温度" legend/><div class="metric-summary"><div><small>采样次数</small><b>{{temperatureSamples.length}} 次</b></div><div><small>温度范围</small><b>{{temperatureRange}}</b></div><div><small>最近更新</small><b>{{device.lastUpdated?clock(device.lastUpdated).slice(0,5):'—'}}</b></div></div></div></UiCard>
 <UiCard title="网络延时" glyph="wifi" class="metric-card"><div class="card-content"><div class="metric-value"><div class="temperature-value"><b class="number">{{latency==null?'—':Math.round(latency)}}</b><span class="muted">ms</span></div><span class="badge" :class="device.error?'red':'green'">{{device.error?'连接异常':'已连接'}}</span></div><TrendPlot :values="latencySamples" :times="device.samples.map(s=>s.time)" :plot-height="205" unit="ms" label="网络延时" legend/><div class="metric-summary"><div><small>采样次数</small><b>{{latencySamples.length}} 次</b></div><div><small>平均延时</small><b>{{latencySamples.length?Math.round(latencySamples.reduce((a,b)=>a+b,0)/latencySamples.length)+' ms':'—'}}</b></div><div><small>最近更新</small><b>{{device.lastUpdated?clock(device.lastUpdated).slice(0,5):'—'}}</b></div></div></div></UiCard>
 <UiCard title="设备资源" glyph="chart" class="overview-resources"><template #extra><span class="badge" :class="device.error?'red':'green'">{{device.error?'连接异常':'实时更新'}}</span></template><div class="card-content"><ResourceBars/><div class="resource-chart-heading"><span class="subtle">处理器与内存占用</span></div><TrendPlot :times="device.samples.map(s=>s.time)" :plot-height="130" unit="%" :minimum="0" :maximum="100" legend :series="[{name:'处理器',color:'#b79bf7',values:device.samples.map(s=>s.cpuUsage)},{name:'内存',color:'#52d4d5',values:device.samples.map(s=>s.memoryUsage)}]"/></div></UiCard>
</div>
</template>

