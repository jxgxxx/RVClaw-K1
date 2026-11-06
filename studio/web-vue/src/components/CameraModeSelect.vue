<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { ElMessage } from 'element-plus';
import { api } from '../api';
import { refreshDevice, beginCameraModeChange, recordCameraStatus, finishCameraModeChange } from '../composables/device';
import { preferences } from '../composables/preferences';

const props=defineProps({camera:{type:Object,default:()=>({})},locked:Boolean});
const emit=defineEmits(['status','switching']);
const modes=ref([]),activeMode=ref(''),loading=ref(true),switching=ref(false),error=ref('');
let pending,retryTimer,controller,stopped=false,retryDelay=5000;

watch(()=>props.camera.active_mode,mode=>{
 if(typeof mode==='string'&&mode)activeMode.value=mode;
},{immediate:true});
const selectedMode=computed({
 get:()=>modes.value.some(mode=>mode.id===activeMode.value)?activeMode.value:'',
 set:mode=>{void changeMode(mode);},
});
const placeholder=computed(()=>error.value?'正在重试…':loading.value?'正在获取模式…':'请选择相机模式');
const disabled=computed(()=>loading.value||switching.value||props.camera.switching||props.locked||!modes.value.length);

function scheduleRetry(){
 if(stopped||retryTimer)return;
 retryTimer=window.setTimeout(()=>{
  retryTimer=undefined;
  if(document.hidden&&preferences.pauseBackground){scheduleRetry();return;}
  void refresh();
 },retryDelay);
 retryDelay=Math.min(retryDelay*2,30000);
}

function refresh(){
 if(stopped)return Promise.resolve();
 if(pending)return pending;
 window.clearTimeout(retryTimer);retryTimer=undefined;
 loading.value=true;
 controller=new AbortController();
 const timeout=window.setTimeout(()=>controller.abort(),7000);
 const requestedAt=Date.now();
 pending=(async()=>{
  try{
   const result=await api('/api/cameras',{signal:controller.signal});
   if(stopped)return;
   const available=(result?.modes||[]).filter(mode=>typeof mode.id==='string'&&mode.id&&typeof mode.label==='string'&&mode.label.trim());
   if(!available.length)throw new Error('未获取到相机模式');
   modes.value=available;
   activeMode.value=result.active_mode||props.camera.active_mode||'';
   error.value='';retryDelay=5000;
   recordCameraStatus(result,requestedAt);
   emit('status',result);
   return result;
  }catch{
   if(stopped)return;
   error.value='相机模式获取失败，将自动重试';
   scheduleRetry();
  }finally{
   window.clearTimeout(timeout);
   if(!stopped)loading.value=false;
   pending=undefined;
  }
 })();
 return pending;
}

const requestMode=mode=>api('/api/camera-mode',{method:'POST',body:JSON.stringify({mode}),signal:AbortSignal.timeout(10000)});
function modeLabel(mode){return modes.value.find(item=>item.id===mode)?.label.replace(/^Astra\s*/,'')||'相机';}
async function waitForMode(mode){
 const deadline=Date.now()+30000;
 while(!stopped&&Date.now()<deadline){
  const status=await refresh();
  if(status){
   const camera=status.cameras?.[0];
   if(status.active_mode!==mode)throw new Error('相机模式已被其他操作更改');
   if(!status.switching&&camera?.online)return status;
   if(!status.switching&&camera?.last_error)throw new Error('识别模式启动失败');
  }
  await new Promise(resolve=>window.setTimeout(resolve,750));
 }
 if(!stopped)throw new Error('相机画面连接超时');
}
async function changeMode(mode){
 if(disabled.value||!modes.value.some(item=>item.id===mode)||mode===activeMode.value)return;
 const previous=activeMode.value;
 const operation=beginCameraModeChange(mode);
 let failure='';
 switching.value=true;emit('switching',true);
 try{
  const initial=await requestMode(mode);
  if(stopped)return;
  recordCameraStatus(initial);
  activeMode.value=mode;emit('status',initial);
  await waitForMode(mode);
  if(stopped)return;
  await refreshDevice();
  ElMessage.success('相机模式已更新');
 }catch(e){
  if(stopped)return;
  let restored=false;
  // Restore the last working view only when this request still owns the mode.
  try{
   const status=await refresh();
   if(status?.active_mode===mode&&!status.switching&&!props.locked){
    await requestMode(previous);
    await waitForMode(previous);restored=!stopped;
   }
  }catch{}
  if(!restored)failure=e.message||'相机模式切换失败';
  if(!stopped)ElMessage.error(restored?modeLabel(mode)+'启动失败，已恢复'+modeLabel(previous):e.message||'相机模式切换失败');
 }finally{if(!stopped){finishCameraModeChange(operation,failure);switching.value=false;emit('switching',false);}}
}

defineExpose({refresh});
onMounted(()=>{void refresh();});
onBeforeUnmount(()=>{stopped=true;window.clearTimeout(retryTimer);controller?.abort();});
</script>

<template>
 <select v-model="selectedMode" class="camera-mode-select" aria-label="相机识别模式" :aria-busy="loading||switching" :title="error||(locked?'巡检期间请保持相机模式':'选择相机识别模式')" :disabled="disabled">
  <option v-if="!selectedMode" value="" disabled>{{placeholder}}</option>
  <option v-for="mode in modes" :key="mode.id" :value="mode.id">{{mode.label.replace(/^Astra\s*/, '')}}</option>
 </select>
</template>

<style scoped>
.camera-mode-select{flex:0 0 auto;width:112px;max-width:112px;min-height:29px;margin-left:8px;text-overflow:ellipsis}
.camera-mode-select:disabled{opacity:1;color:var(--muted);cursor:wait}
@media(max-width:520px){.camera-mode-select{width:108px;max-width:108px;margin-left:4px}}
</style>
