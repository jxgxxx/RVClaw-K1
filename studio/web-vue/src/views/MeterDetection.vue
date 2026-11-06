<script setup>
import {computed,onMounted,onBeforeUnmount,ref,watch} from 'vue';
import {ElMessage} from 'element-plus';
import {api,postJson,formatTime} from '../api';
import {readingLabel,badgeColor} from '../composables/presentation';
import {registerPageRefresher} from '../composables/pageRefresh';
import {usePagination} from '../composables/pagination';
import UiContentState from '../components/UiContentState.vue';
const meterType=ref('digital'),fileInput=ref(null),file=ref(null),imageUrl=ref(''),result=ref(null),history=ref([]),busy=ref(false),error=ref(''),source=ref(''),detail=ref(null),detailOpen=ref(false);
const uploadRequiredOpen=ref(false),historyLoading=ref(true),historyLoaded=ref(false),historyError=ref('');
let historyController,disposed=false;
const typeLabel=computed(()=>meterType.value==='pointer'?'指针仪表':'数字仪表');
const historyFilter=ref('all');
const filteredHistory=computed(()=>history.value.filter(record=>historyFilter.value==='all'||(historyFilter.value==='success'?['识别成功','超出范围'].includes(readingLabel(record)):readingLabel(record)==='识别失败')));
const {page,pageSize,visible}=usePagination(filteredHistory);let unregister;
watch(historyFilter,()=>page.value=1,{flush:'sync'});
function revoke(){if(imageUrl.value.startsWith('blob:'))URL.revokeObjectURL(imageUrl.value);}
function resetUpload(){uploadRequiredOpen.value=false;revoke();file.value=null;imageUrl.value='';result.value=null;error.value='';source.value='';detail.value=null;detailOpen.value=false;if(fileInput.value)fileInput.value.value='';}
watch(meterType,resetUpload,{flush:'sync'});
function choose(event){const next=event.target.files?.[0];event.target.value='';if(!next)return;if(!next.type.startsWith('image/'))return ElMessage.warning('请选择图片文件。');if(next.size>10*1024*1024)return ElMessage.warning('图片不能超过 10 MB。');revoke();file.value=next;imageUrl.value=URL.createObjectURL(next);result.value=null;source.value='本次上传';error.value='';}
function dropImage(event){if(!busy.value&&event.dataTransfer?.files?.length)choose({target:{files:event.dataTransfer.files,value:''}});}
function reason(r){if(r?.reason_zh)return r.reason_zh;const text=String(r?.reason||'');if(/403|401/.test(text))return '识别服务拒绝了请求，请稍后重试。';if(/inference_error|inference_offline/.test(r?.status||''))return '识别服务暂时不可用，请稍后重试。';if(/[\u4e00-\u9fff]/.test(text))return text;return '未能可靠读取仪表，请减少反光并确保画面清晰。';}
async function load(){
 if(historyController||disposed)return;
 const controller=new AbortController();historyController=controller;historyLoading.value=true;historyError.value='';
 try{const data=await api('/api/studio/meter-history',{signal:AbortSignal.any([controller.signal,AbortSignal.timeout(12000)])});if(disposed)return;history.value=data.records||[];historyLoaded.value=true;}
 catch{if(!disposed)historyError.value='请检查设备连接后重试。';}
 finally{historyLoading.value=false;historyController=null;}
}
async function read(capture=false){
 if(busy.value)return;if(!capture&&!file.value){uploadRequiredOpen.value=true;return;}busy.value=true;error.value='';
 try{
 let response;
 if(capture){response=await postJson('/api/meter-current-read',{meter_type:meterType.value,meter_id:'studio-'+meterType.value,required_camera_mode:'astra_raw'});result.value=response.trial||response;revoke();imageUrl.value=result.value.image_url||result.value.scene_url||imageUrl.value;source.value='相机拍照';file.value=null;}
 else{const data=await new Promise((resolve,reject)=>{const r=new FileReader();r.onload=()=>resolve(r.result);r.onerror=()=>reject(new Error('图片读取失败'));r.readAsDataURL(file.value);});response=await postJson('/api/meter-image-read',{meter_type:meterType.value,image:data,filename:file.value.name});result.value=response.trial||response;}
 if(readingLabel(result.value)==='识别失败')error.value=reason(result.value);
 await load();page.value=1;
 }catch(e){result.value={kind:meterType.value,status:'error',value:null,created_at:new Date().toISOString()};error.value=/camera mode|astra_raw/.test(e.message)?'请先在总览页面将相机切换为原始画面，再拍照识别。':/[\u4e00-\u9fff]/.test(e.message)?e.message:'识别失败，请检查服务连接后重试。';}finally{busy.value=false;}
}
function chooseFromPrompt(){uploadRequiredOpen.value=false;fileInput.value?.click();}
function view(record){detail.value=record;detailOpen.value=true;}
onMounted(async()=>{unregister=registerPageRefresher('meter',load);await load();if(disposed)return;const id=new URLSearchParams(location.hash.split('?')[1]||'').get('record');if(id){const index=history.value.findIndex(row=>row.id===id);if(index>=0){page.value=Math.floor(index/pageSize.value)+1;view(history.value[index]);}else ElMessage.warning('该识别记录暂未找到，请刷新历史记录。');}});onBeforeUnmount(()=>{disposed=true;historyController?.abort();resetUpload();unregister?.();});
</script>
<template>
<div class="detect-grid">
 <UiCard class="detection-image-card"><template #title><div class="tabbar meter-type-tabs" role="group" aria-label="仪表类型"><button class="tab" :class="{active:meterType==='digital'}" :aria-pressed="meterType==='digital'" :disabled="busy" @click="meterType='digital'">数字仪表</button><button class="tab" :class="{active:meterType==='pointer'}" :aria-pressed="meterType==='pointer'" :disabled="busy" @click="meterType='pointer'">指针仪表</button></div></template><template #extra><span class="detection-picture-label"><UiIcon name="camera" :size="16"/>待识别画面</span></template><div class="detect-picture" @dragover.prevent @drop.prevent="dropImage"><input ref="fileInput" type="file" accept="image/*" @change="choose"><img v-if="imageUrl" :src="imageUrl" :alt="typeLabel+'图片'"><button v-else @click="fileInput?.click()"><span class="upload-art layered-art" aria-hidden="true"><span class="back-paper"></span><span class="front-paper"><svg viewBox="0 0 32 32" width="32" height="32" fill="none" stroke="currentColor" stroke-width="1.5"><rect x="4" y="4" width="24" height="24" rx="4"/><circle cx="11" cy="11" r="2.3"/><path d="m5 23 8-8 5 5 4-4 5 7"/></svg></span><span class="upload-dot"><UiIcon name="upload" :size="18"/></span></span><b>选择仪表图片</b><span>点击选择，或拖放图片到这里</span><small>JPG、PNG、WebP · 最大 10 MB</small></button></div><div class="action-row centered"><button class="btn primary" :disabled="busy" @click="read()"><UiIcon name="camera" :size="15"/>{{busy?'正在识别…':'开始读数'}}</button><button class="btn" :disabled="busy" @click="fileInput?.click()"><UiIcon name="upload" :size="15"/>选择图片</button></div><div class="detection-result"><span class="result-icon"><UiIcon name="meter" :size="23"/></span><div><b :class="result?badgeColor(readingLabel(result)):''">{{busy?'正在识别…':result?readingLabel(result):'等待上传图片'}}</b><small>{{error||(result?source+' · '+(result.created_at?formatTime(result.created_at,{seconds:true}):''):'选择图片后，点击开始读数')}}</small></div><strong class="number" :class="result?badgeColor(readingLabel(result)):''">{{result?.value??'—'}}</strong></div></UiCard>
<UiCard title="最近识别" glyph="clock" class="recent-table"><template #extra><div class="meter-history-tools"><span v-if="historyLoading&&historyLoaded" class="history-refresh-label" role="status">正在更新…</span><label class="history-filter" for="meter-history-filter">识别状态<select id="meter-history-filter" v-model="historyFilter"><option value="all">全部记录</option><option value="success">识别成功</option><option value="failure">识别失败</option></select></label></div></template><div v-if="historyError&&history.length" class="history-error" role="status">记录更新失败，已保留上次结果<button class="btn plain" @click="load">重试</button></div><div class="table-scroll" :class="{'has-state':!filteredHistory.length}" :aria-busy="historyLoading"><table><thead><tr><th>时间</th><th>仪表类型</th><th>来源</th><th>读数</th><th>状态</th><th>操作</th></tr></thead><tbody><tr v-for="record in visible" :key="record.id"><td class="number">{{formatTime(record.created_at,{seconds:true})}}</td><td>{{record.kind==='pointer'?'指针仪表':'数字仪表'}}</td><td>{{record.source_label}}</td><td><span :class="badgeColor(readingLabel(record))">{{record.value??'识别失败'}}</span></td><td><span class="badge" :class="badgeColor(readingLabel(record))">{{readingLabel(record)}}</span></td><td><button class="btn plain" @click="view(record)"><UiIcon name="eye" :size="15"/>查看</button></td></tr></tbody></table><UiContentState v-if="historyLoading&&!historyLoaded" loading skeleton title="正在读取识别记录"/><UiContentState v-else-if="!history.length&&historyError" icon="wifi" title="识别记录暂时无法读取" :description="historyError" retry @retry="load"/><UiContentState v-else-if="!filteredHistory.length" icon="meter" :title="history.length?(historyFilter==='success'?'暂无识别成功的记录':'暂无识别失败的记录'):'还没有识别记录'" :description="history.length?'切换识别状态，查看其他记录':'上传仪表图片并开始读数，识别结果将保存在这里'"/></div><UiPagination v-model="page" v-model:page-size="pageSize" :total="filteredHistory.length"/></UiCard>
</div>
<UiModal v-model="uploadRequiredOpen" title="尚未上传图片" width="min(460px,92vw)"><div class="modal-heading upload-required-message"><span class="icon-box"><UiIcon name="upload" :size="23"/></span><div><b>请先上传仪表图片</b><small>请选择一张{{typeLabel}}图片，再点击“开始读数”。</small></div></div><template #footer><button class="btn" type="button" @click="uploadRequiredOpen=false">知道了</button><button class="btn primary" type="button" @click="chooseFromPrompt"><UiIcon name="upload" :size="15"/>选择图片</button></template></UiModal>
<UiModal v-model="detailOpen" title="仪表识别记录" width="min(820px,92vw)"><template v-if="detail"><img class="photo-dialog-img" :src="detail.image_url" alt="本次仪表识别图片"><div class="detail-strip"><span>{{formatTime(detail.created_at,{seconds:true})}}</span><b :class="badgeColor(readingLabel(detail))">{{detail.value??'识别失败'}}</b><span class="badge" :class="badgeColor(readingLabel(detail))">{{readingLabel(detail)}}</span></div><p v-if="readingLabel(detail)==='识别失败'" class="note error">{{reason(detail)}}</p></template><template #footer><button class="btn" @click="detailOpen=false">关闭</button></template></UiModal>
</template>

<style scoped>
.meter-history-tools{display:flex;align-items:center;gap:12px;min-width:0}.history-filter{display:flex;align-items:center;gap:8px;font-size:11px;color:var(--muted);white-space:nowrap}.history-filter select{min-height:32px;padding:4px 9px;border:1px solid var(--line);border-radius:5px;background:var(--input);color:var(--ink);font:inherit;cursor:pointer}.history-filter select:focus-visible{outline:2px solid var(--blue);outline-offset:3px}
@media(max-width:600px){.meter-history-tools{position:relative}.meter-history-tools>.history-refresh-label{position:absolute;top:calc(100% + 6px);right:0;white-space:nowrap}.history-filter{gap:6px}.history-filter select{padding-inline:6px}}
@media(pointer:coarse){.history-filter select{min-height:44px}}
.recent-table .table-scroll.has-state{display:flex;flex-direction:column}.has-state>table{flex:none}.history-refresh-label{font-size:11px;color:var(--muted)}.history-error{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 16px;font-size:11px;color:var(--amber);border-bottom:1px solid var(--line)}
.upload-required-message{align-items:flex-start;margin-bottom:0}
.upload-required-message small{line-height:1.7;margin-top:6px}
</style>
