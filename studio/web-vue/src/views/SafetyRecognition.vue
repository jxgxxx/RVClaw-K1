<script setup>
import { computed, onMounted, onBeforeUnmount, ref, watch } from 'vue';
import { api, postJson, formatTime } from '../api';
import CameraPreview from '../components/CameraPreview.vue';
import UiContentState from '../components/UiContentState.vue';
import { preferences } from '../composables/preferences';
import { registerPageRefresher } from '../composables/pageRefresh';

const equipment=[{id:'helmet',label:'安全帽'},{id:'reflective',label:'反光工作服'},{id:'mask',label:'防尘口罩/面罩'},{id:'insulating',label:'绝缘工作服'}];
const labels=Object.fromEntries(equipment.map(x=>[x.id,x.label]));
const names={compliant:'已佩戴',violation:'未佩戴',unknown:'无法判断',no_people:'未发现人员',error:'识别失败',pending:'识别中'};
const colors={compliant:'green',violation:'red',unknown:'amber',no_people:'blue',error:'red',pending:'blue'};
const state=ref(null),records=ref([]),maps=ref([]),error=ref(''),recordsError=ref(''),loaded=ref(false),loading=ref(true),busy=ref(''),notice=ref('');
const filter=ref('all'),modeFilter=ref('all'),page=ref(1),pageSize=ref(5),now=ref(Date.now());
const configOpen=ref(false),saving=ref(false),configError=ref(''),draft=ref(null),detail=ref(null),detailOpen=ref(false);
const cloudSource=ref('upload'),uploadTab=ref(null),cameraTab=ref(null),fileInput=ref(null),uploadUrl=ref(''),uploadImage=ref(''),uploadName=ref(''),uploadError=ref(''),preparing=ref(false),uploadRecordId=ref('');
const uploadRecord=computed(()=>records.value.find(r=>r.id===uploadRecordId.value));
const uploadPending=computed(()=>busy.value==='upload'||(uploadRecordId.value&&!uploadRecord.value)||uploadRecord.value?.status==='pending');
watch(()=>uploadRecord.value?.status,status=>{if(status&&status!=='pending'&&notice.value==='图片已提交，正在云端识别')notice.value='';});
let selection=0;
let timer,clockTimer,unregister,inflight=false,alive=true,lastRecords=0;
const camera=computed(()=>state.value?.camera||{}),cam=computed(()=>camera.value.cameras?.[0]||{});
const local=computed(()=>state.value?.local||{}),cloud=computed(()=>state.value?.cloud||{});
const remaining=computed(()=>Math.max(0,Math.ceil((cloud.value.last_capture_at||0)+120-now.value/1000)));
const cloudLabel=computed(()=>cloud.value.busy?'正在识别':!cloud.value.ready?'服务未就绪':!state.value?.config?.cloud_enabled?'自动检测关闭':cloud.value.point?'检测点内':'等待到达检测点');
const cloudMessage=computed(()=>cloud.value.busy?'本次照片正在识别，完成后自动保存结果。':cloud.value.point?'当前点位：'+cloud.value.point.name:!state.value?.config?.points?.some(p=>p.enabled)?'尚未设置安全防护点位。配置后，到达点位自动检测。':'到达选定点位拍照，停留期间每两分钟采集一次，离开后停止。');
const filtered=computed(()=>records.value.filter(r=>(filter.value==='all'||r.status===filter.value)&&(modeFilter.value==='all'||r.mode===modeFilter.value)));
const visible=computed(()=>filtered.value.slice((page.value-1)*pageSize.value,page.value*pageSize.value));
watch([filter,modeFilter,pageSize],()=>page.value=1);
watch(()=>filtered.value.length,()=>{page.value=Math.min(page.value,Math.max(1,Math.ceil(filtered.value.length/pageSize.value)));});
const detailRow=computed(()=>records.value.find(r=>r.id===detail.value?.id)||detail.value);
function missing(row){return (row.missing||[]).map(k=>labels[k]).join('、');}
function outcome(row){return row.status==='violation'?'未佩戴：'+missing(row):row.reason||names[row.status]||'无法判断';}
async function loadRecords(){
 try{const value=await api('/api/safety/records',{signal:AbortSignal.timeout(12000)});if(!alive)return;records.value=value.records||[];loaded.value=true;recordsError.value='';lastRecords=Date.now();
  const id=new URLSearchParams(location.hash.split('?')[1]||'').get('record');if(id&&!detail.value){const found=records.value.find(r=>r.id===id);if(found)view(found);}
 }catch(e){if(alive)recordsError.value=e.message;}
}
async function load(force=false){
 if(inflight||!alive)return;inflight=true;
 try{
  const value=await api('/api/safety/status',{signal:AbortSignal.timeout(10000)});if(!alive)return;
  const finished=state.value?.cloud?.busy&&!value.cloud?.busy;state.value=value;error.value='';
  if(force||!loaded.value||finished||Date.now()-lastRecords>10000)await loadRecords();
 }catch(e){if(alive){error.value=e.message;if(!loaded.value)recordsError.value=e.message;}}
 finally{inflight=false;loading.value=false;}
}
async function action(kind){
 if(busy.value)return;busy.value=kind;notice.value='';
 try{
  if(kind==='local'){await postJson('/api/safety/local',{enabled:!local.value.active});notice.value=local.value.active?'本地识别已关闭':'本地识别已开启，正在准备画面';}
  else{
   if(kind==='upload'&&!uploadImage.value){uploadError.value='请先选择需要识别的图片。';return;}
   const row=await postJson(kind==='upload'?'/api/safety/upload':'/api/safety/capture',kind==='upload'?{image:uploadImage.value}:{});
   if(kind==='upload'){uploadRecordId.value=row.id;records.value=[row,...records.value.filter(r=>r.id!==row.id)];}
   if(state.value)state.value.cloud={...state.value.cloud,busy:true,last_capture_at:Date.now()/1000};
   notice.value=kind==='upload'?'图片已提交，正在云端识别':'已开始拍照验证';
  }
  await load(true);
 }catch(e){if(kind==='upload')uploadError.value=e.message;else notice.value=e.message;}finally{busy.value='';}
}
async function chooseImage(file){
 if(!file||preparing.value||busy.value==='upload'||uploadPending.value)return;
 const token=++selection;uploadError.value='';
 if(!['image/jpeg','image/png','image/webp'].includes(file.type)){uploadError.value='请选择照片或静态图片。';return;}
 if(!file.size||file.size>10*1024*1024){uploadError.value='图片不能超过 10 兆字节。';return;}
 const url=URL.createObjectURL(file);let accepted=false;preparing.value=true;
 try{
  const image=new Image();image.src=url;await image.decode();
  if(!image.naturalWidth||image.naturalWidth*image.naturalHeight>20000000)throw new Error('图片像素总数不能超过 2000 万。');
  const encoded=await new Promise((resolve,reject)=>{const reader=new FileReader();reader.onload=()=>resolve(reader.result);reader.onerror=()=>reject(new Error('图片读取失败，请重新选择。'));reader.readAsDataURL(file);});
  if(!alive||token!==selection)return;
  if(uploadUrl.value)URL.revokeObjectURL(uploadUrl.value);
  uploadUrl.value=url;uploadImage.value=encoded;uploadName.value=file.name;uploadRecordId.value='';accepted=true;
 }catch(e){if(alive&&token===selection)uploadError.value=e.message||'图片无法读取，请重新选择。';}
 finally{if(!accepted)URL.revokeObjectURL(url);if(token===selection)preparing.value=false;}
}
function selectImage(event){const file=event.target.files?.[0];event.target.value='';chooseImage(file);}
function dropImage(event){event.preventDefault();chooseImage(event.dataTransfer?.files?.[0]);}
function clearImage(){selection++;if(uploadUrl.value)URL.revokeObjectURL(uploadUrl.value);uploadUrl.value='';uploadImage.value='';uploadName.value='';uploadRecordId.value='';uploadError.value='';}
async function configure(){
 if(!state.value)return;draft.value=JSON.parse(JSON.stringify(state.value.config));configError.value='';configOpen.value=true;
 try{maps.value=(await api('/api/inspection-maps',{signal:AbortSignal.timeout(8000)})).maps||[];}catch{configError.value='地图列表读取失败，请重新打开配置。';}
}
function addPoint(){
 draft.value.points.push({id:'safety-'+Date.now().toString(36),name:'安全防护点 '+(draft.value.points.length+1),map_id:maps.value.find(m=>m.ready)?.id||'',x:0,y:0,radius:.8,enabled:false});
}
async function save(){
 if(!draft.value.required.length){configError.value='请至少选择一项防护装备。';return;}
 saving.value=true;configError.value='';
 try{state.value=await postJson('/api/safety/config',draft.value);configOpen.value=false;notice.value='检测配置已保存';}
 catch(e){configError.value=e.message;}finally{saving.value=false;}
}
function view(row){detail.value=row;detailOpen.value=true;}
function boxStyle(box){return {left:box[0]*100+'%',top:box[1]*100+'%',width:(box[2]-box[0])*100+'%',height:(box[3]-box[1])*100+'%'};}
onMounted(()=>{
 load(true);clockTimer=setInterval(()=>now.value=Date.now(),1000);
 timer=setInterval(()=>{if(!document.hidden||!preferences.pauseBackground)load();},3000);
 unregister=registerPageRefresher('safety',()=>load(true));
});
onBeforeUnmount(()=>{alive=false;selection++;if(uploadUrl.value)URL.revokeObjectURL(uploadUrl.value);clearInterval(timer);clearInterval(clockTimer);unregister?.();});
</script>

<template>
<div class="safety-page">
 <div v-if="error||state?.error" class="safety-message error" role="alert">{{error||state.error}}<button class="btn plain" @click="load(true)">重新连接</button></div>
 <div v-if="notice" class="safety-message" role="status">{{notice}}<button class="btn plain" aria-label="关闭提示" @click="notice=''"><UiIcon name="x" :size="15"/></button></div>
 <div class="safety-main">
  <UiCard title="本地识别" glyph="shield" class="safety-camera">
   <template #extra><span class="badge" :class="local.ready?'green':local.active?'amber':''">{{local.ready?'识别中':local.active?'准备中':'未开启'}}</span></template>
   <CameraPreview :src="cam.online?cam.stream_url:''" :generation="camera.generation" :mode="camera.active_mode" :online="Boolean(cam.online)" :switching="Boolean(camera.switching||busy==='local')" :enabled="preferences.autoCamera" :pause-background="preferences.pauseBackground" fit="cover" :width="cam.width" :height="cam.height"/>
   <div class="safety-local-stats" aria-live="polite"><div><strong class="number">{{local.counts?.persons??'—'}}</strong><span>画面人员</span></div><div class="green"><strong class="number">{{local.counts?.helmet??'—'}}</strong><span>佩戴安全帽</span></div><div class="red"><strong class="number">{{local.counts?.no_helmet??'—'}}</strong><span>未佩戴</span></div><div class="amber"><strong class="number">{{local.counts?.checking??'—'}}</strong><span>待确认</span></div></div>
   <div class="safety-local-control"><div class="voice-command"><UiIcon name="mic" :size="17"/><span>“小微小微，开始安全防护识别”</span></div><button class="btn" :class="{primary:!local.active}" title="使用本地安全帽模型检测 Astra 相机画面" :disabled="!state||Boolean(error)||Boolean(busy)||camera.switching" @click="action('local')"><UiIcon :name="local.active?'x':'play'" :size="14"/>{{busy==='local'?'正在切换…':local.active?'关闭本地识别':'开启本地识别'}}</button></div>
  </UiCard>
  <UiCard title="云端辅助识别" glyph="eye" class="safety-cloud">
   <template #extra><button class="btn plain" :disabled="!state" @click="configure"><UiIcon name="sliders" :size="15"/>检测配置</button></template>
   <div class="safety-cloud-body">
    <div class="cloud-status"><div class="cloud-source" role="tablist" aria-label="云端图片来源"><button id="safety-upload-tab" ref="uploadTab" role="tab" :aria-selected="cloudSource==='upload'" aria-controls="safety-upload-panel" :tabindex="cloudSource==='upload'?0:-1" @keydown.right.prevent="cloudSource='camera';$nextTick(()=>cameraTab?.focus())" @click="cloudSource='upload'">上传图片</button><button id="safety-camera-tab" ref="cameraTab" role="tab" :aria-selected="cloudSource==='camera'" aria-controls="safety-camera-panel" :tabindex="cloudSource==='camera'?0:-1" @keydown.left.prevent="cloudSource='upload';$nextTick(()=>uploadTab?.focus())" @click="cloudSource='camera'">相机拍照</button></div><span class="badge" :class="cloud.busy?'blue':cloud.point?'green':'amber'">{{loading?'正在连接':cloudLabel}}</span></div>
    <section v-show="cloudSource==='upload'" id="safety-upload-panel" role="tabpanel" aria-labelledby="safety-upload-tab" class="cloud-upload-panel">
     <div class="safety-upload-zone" @dragover.prevent @drop="dropImage">
      <input ref="fileInput" type="file" accept="image/jpeg,image/png,image/webp" aria-label="选择安全防护图片" :disabled="preparing||Boolean(uploadPending)" @change="selectImage">
      <button v-if="!uploadUrl" type="button" class="safety-upload-empty" :disabled="preparing||Boolean(uploadPending)" @click="fileInput?.click()"><span class="upload-symbol"><UiIcon name="upload" :size="28"/></span><b>{{preparing?'正在读取图片…':'选择安全防护图片'}}</b><span>点击选择，或将图片拖放到这里</span></button>
      <div v-else class="safety-upload-preview"><img :src="uploadUrl" alt="待识别的安全防护图片"><div v-if="uploadPending" class="upload-progress"><UiContentState loading title="正在云端识别"/></div><div v-else class="upload-image-actions"><button class="btn" type="button" @click="fileInput?.click()">更换图片</button><button class="btn" type="button" aria-label="移除安全防护图片" @click="clearImage"><UiIcon name="x" :size="15"/></button></div></div>
     </div>
     <div v-if="uploadName" class="upload-file-name" :title="uploadName">{{uploadName}}</div>
     <p v-if="uploadError" class="upload-error" role="alert">{{uploadError}}</p>
     <div v-if="uploadRecord&&uploadRecord.status!=='pending'" class="upload-result" role="status"><span class="badge" :class="colors[uploadRecord.status]">{{names[uploadRecord.status]}}</span><span class="upload-outcome">{{outcome(uploadRecord)}}</span><button class="btn plain" @click="view(uploadRecord)">查看结果<UiIcon name="arrow" :size="14"/></button></div>
    </section>
    <section v-show="cloudSource==='camera'" id="safety-camera-panel" role="tabpanel" aria-labelledby="safety-camera-tab" class="camera-capture-panel"><div class="cloud-emblem"><UiIcon name="camera" :size="28"/></div><p class="cloud-explanation">{{cloudMessage}}</p><div class="safety-schedule"><span><UiIcon name="clock" :size="15"/>采集间隔</span><b>2 分钟 / 张</b></div><div class="safety-schedule"><span><UiIcon name="map" :size="15"/>已启用点位</span><b>{{state?.config?.points.filter(p=>p.enabled).length||0}} 个</b></div></section>
    <section class="required-equipment"><h2>检测装备</h2><div class="gear-list"><div v-for="item in equipment" :key="item.id" class="gear-item" :class="{selected:state?.config?.required.includes(item.id)}"><UiIcon :name="state?.config?.required.includes(item.id)?'check':'x'" :size="15"/><span>{{item.label}}</span></div></div></section>
    <div v-if="cloud.point&&!cloud.busy" class="next-capture" role="status">{{remaining?'下次采集约 '+remaining+' 秒后':'等待采集新照片'}}</div>
    <div class="cloud-actions"><span v-if="remaining>0&&!cloud.busy" class="upload-cooldown" role="status">{{remaining}} 秒后可再次识别</span><button v-if="cloudSource==='upload'" class="btn primary" :disabled="!uploadImage||preparing||!cloud.ready||cloud.busy||Boolean(busy)||Boolean(error)||remaining>0" @click="action('upload')"><UiIcon name="eye" :size="15"/>{{uploadPending?'正在识别…':'开始识别'}}</button><button v-else class="btn" :disabled="!cloud.ready||cloud.busy||Boolean(busy)||Boolean(error)||remaining>0||camera.switching||!cam.online" @click="action('capture')"><UiIcon name="camera" :size="15"/>{{cloud.busy?'正在识别…':'拍照验证'}}</button></div>
   </div>
  </UiCard>
 </div>
 <UiCard title="识别记录" glyph="file" class="safety-records">
  <template #extra><div class="safety-filters"><label>推理模式<select v-model="modeFilter" aria-label="筛选推理模式"><option value="all">全部模式</option><option value="local">本地识别</option><option value="cloud">云端识别</option></select></label><label>识别结果<select v-model="filter" aria-label="筛选识别结果"><option value="all">全部结果</option><option v-for="(label,key) in names" :key="key" :value="key">{{label}}</option></select></label></div></template>
  <div v-if="recordsError&&records.length" class="safety-message error" role="status">记录更新失败，已保留上次结果<button class="btn plain" @click="loadRecords">重试</button></div>
  <div class="safety-table-scroll"><table><thead><tr><th>采集时间</th><th>模式</th><th>点位 / 来源</th><th>识别结果</th><th>未佩戴装备</th><th>证据</th></tr></thead><tbody><tr v-for="row in visible" :key="row.id"><td class="number">{{formatTime(row.created_at,{seconds:true})}}</td><td>{{row.mode==='local'?'本地':'云端'}}</td><td>{{row.source==='upload'?'图片上传':row.point?.name||(row.mode==='cloud'?'手动验证':'实时相机')}}</td><td><span class="badge" :class="colors[row.status]">{{names[row.status]}}</span></td><td class="missing-cell" :class="{'red':row.missing?.length}">{{missing(row)||'—'}}</td><td><button class="btn plain" @click="view(row)"><UiIcon name="eye" :size="15"/>查看</button></td></tr></tbody></table>
   <UiContentState v-if="!loaded&&!recordsError" loading skeleton title="正在读取识别记录"/>
   <UiContentState v-else-if="!records.length&&recordsError" icon="wifi" title="识别记录读取失败" retry @retry="loadRecords"/>
   <UiContentState v-else-if="!filtered.length" icon="shield" :title="records.length?'没有符合条件的记录':'还没有安全防护识别记录'"/>
  </div><UiPagination v-model="page" v-model:page-size="pageSize" :total="filtered.length"/>
 </UiCard>
</div>

<UiModal v-model="configOpen" title="安全防护检测配置" width="min(920px,94vw)">
 <form v-if="draft" class="safety-config" @submit.prevent="save">
  <fieldset><legend>云端检测装备</legend><div class="gear-options"><label v-for="item in equipment" :key="item.id"><input v-model="draft.required" type="checkbox" :value="item.id">{{item.label}}</label></div><p v-if="draft.required.includes('insulating')" class="config-help">绝缘工作服的绝缘性能需要现场核验，普通画面无法确认时记为“无法判断”。</p></fieldset>
  <label class="auto-cloud"><input v-model="draft.cloud_enabled" type="checkbox">到达安全防护点位时自动开启云端识别</label>
  <div class="point-heading"><h2>安全防护点位</h2><button class="btn" type="button" :disabled="draft.points.length>=50" @click="addPoint"><UiIcon name="map" :size="14"/>添加点位</button></div>
  <p class="config-help">仅配置检测触发区域。使用所属地图的米制坐标，到达并停稳后拍照；路线经过该区域时生效。</p>
  <div v-if="!draft.points.length" class="no-points"><UiIcon name="map" :size="24"/><span>尚未设置安全防护点位</span></div>
  <div v-for="(point,index) in draft.points" :key="point.id" class="safety-point">
   <div class="point-line"><label class="point-name">点位名称<input v-model="point.name" required maxlength="60" :aria-label="'点位 '+(index+1)+' 名称'"></label><label class="point-map">所属地图<select v-model="point.map_id" required :aria-label="'点位 '+(index+1)+' 所属地图'"><option disabled value="">选择地图</option><option v-for="map in maps" :key="map.id" :value="map.id" :disabled="!map.ready">{{map.name}}</option></select></label><button class="btn plain red" type="button" :aria-label="'删除点位 '+point.name" @click="draft.points.splice(index,1)"><UiIcon name="x" :size="17"/></button></div>
   <div class="point-coordinates"><label>横坐标（米）<input v-model.number="point.x" required type="number" step="0.01" min="-10000" max="10000"></label><label>纵坐标（米）<input v-model.number="point.y" required type="number" step="0.01" min="-10000" max="10000"></label><label>触发半径（米）<input v-model.number="point.radius" required type="number" step="0.1" min="0.2" max="5"></label><label class="point-enable"><input v-model="point.enabled" type="checkbox">启用</label></div>
  </div>
  <p v-if="configError" class="config-help red" role="alert">{{configError}}</p>
  <div class="config-buttons"><button class="btn" type="button" :disabled="saving" @click="configOpen=false">取消</button><button class="btn primary" type="submit" :disabled="saving">{{saving?'正在保存…':'保存配置'}}</button></div>
 </form>
</UiModal>

<UiModal v-model="detailOpen" title="安全防护识别记录" width="min(980px,94vw)">
 <div v-if="detailRow" class="safety-detail">
  <div class="detail-summary"><span class="badge" :class="colors[detailRow.status]">{{names[detailRow.status]}}</span><span>{{detailRow.mode==='local'?'本地识别':'云端识别'}}</span><time>{{formatTime(detailRow.created_at,{seconds:true})}}</time><b v-if="detailRow.point">{{detailRow.point.name}}</b></div>
  <div v-if="detailRow.image_url" class="evidence-stage"><img :src="detailRow.image_url" alt="本次安全防护识别现场照片"><div v-for="person in detailRow.persons.filter(p=>p.bbox)" :key="person.id" class="person-box" :class="{'is-violation':person.missing.length}" :style="boxStyle(person.bbox)"><span>人员 {{person.id}}</span></div></div>
  <UiContentState v-else-if="detailRow.status==='pending'" loading title="正在采集照片"/>
  <p v-if="detailRow.status!=='compliant'&&detailRow.status!=='no_people'&&detailRow.status!=='pending'" :class="detailRow.status==='violation'?'red':'amber'">{{outcome(detailRow)}}</p>
  <div class="detail-required">本次检查：{{detailRow.required.map(k=>labels[k]).join('、')}}</div>
  <div v-if="detailRow.mode==='local'" class="detail-counts">人员 {{detailRow.counts.persons}}　佩戴安全帽 {{detailRow.counts.helmet}}　未佩戴 {{detailRow.counts.no_helmet}}　待确认 {{detailRow.counts.checking}}</div>
  <section v-for="person in detailRow.persons" :key="person.id" class="person-result"><h3>人员 {{person.id}}</h3><div v-for="(item,key) in person.equipment" :key="key" class="person-gear"><span>{{labels[key]}}</span><span class="badge" :class="item.state==='missing'?'red':item.state==='present'?'green':'amber'">{{item.state==='missing'?'未佩戴':item.state==='present'?'已佩戴':'无法判断'}}</span><span class="gear-reason">{{item.reason}}</span></div></section>
 </div><template #footer><a v-if="detailRow?.image_url" class="btn" :href="detailRow.image_url" :download="'安全防护证据-'+detailRow.id+'.jpg'"><UiIcon name="download" :size="15"/>保存照片</a><button class="btn" @click="detailOpen=false">关闭</button></template>
</UiModal>
</template>

<style scoped>
.cloud-status{justify-content:space-between;flex-wrap:wrap}.cloud-source{display:inline-flex;gap:3px;padding:3px;background:var(--input);border:1px solid var(--line);border-radius:7px}.cloud-source button{border:1px solid transparent;border-radius:4px;background:transparent;padding:7px 14px;font-size:12px;color:var(--muted);cursor:pointer}.cloud-source button[aria-selected=true]{background:var(--raised);border-color:var(--line);color:var(--ink)}.cloud-source button:hover{color:var(--ink)}.cloud-upload-panel{display:grid;gap:10px}.safety-upload-zone{min-width:0;position:relative;border:1px dashed #526371;border-radius:7px;background:var(--input);overflow:hidden}.safety-upload-zone:focus-within,.safety-upload-zone:hover{border-color:var(--cyan)}.safety-upload-zone>input{display:none}.safety-upload-empty{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:12px;width:100%;min-height:207px;border:0;background:transparent;white-space:normal;padding:20px;color:var(--muted);cursor:pointer}.safety-upload-empty b{color:var(--ink);font-size:14px;font-weight:500}.safety-upload-empty>span:last-child{font-size:12px}.upload-symbol{display:grid;place-items:center;width:52px;height:52px;border:1px solid #52d4d538;border-radius:12px;color:var(--cyan);background:#52d4d509}.safety-upload-preview{position:relative;height:207px;background:#080d12}.safety-upload-preview>img{display:block;width:100%;height:100%;object-fit:contain}.upload-image-actions{position:absolute;right:10px;bottom:10px;display:flex;gap:7px}.upload-image-actions .btn{background:#151b23ed}.upload-file-name{min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:var(--muted);font-size:11px}.upload-error{font-size:12px;line-height:1.6;color:var(--red);margin:0}.upload-result{display:flex;gap:10px;align-items:center;flex-wrap:wrap;border:1px solid var(--line);border-radius:6px;padding:9px 11px}.upload-outcome{min-width:0;flex:1;font-size:12px;overflow-wrap:anywhere}.upload-progress{position:absolute;inset:0;background:#080d12ed;display:flex}.upload-progress :deep(.content-state){min-height:0;padding:16px}.camera-capture-panel{display:grid;gap:18px;padding-block:4px}.upload-cooldown{font-size:12px;color:var(--muted);margin-right:auto}.cloud-actions{gap:14px;align-items:center;flex-wrap:wrap}.safety-cloud{min-width:0}.safety-cloud-body{min-width:0}
@media(max-width:650px){.cloud-source button{min-height:40px;padding-inline:10px}.cloud-status{gap:10px}.safety-upload-empty,.safety-upload-preview{min-height:180px;height:180px}.cloud-actions>.btn{min-height:42px}.safety-upload-empty>span:last-child{font-size:11px}.upload-result .btn{margin-left:auto}.safety-upload-preview .upload-image-actions{right:8px;bottom:8px}}
.safety-page{display:grid;gap:18px;padding-bottom:10px}.safety-main{display:grid;grid-template-columns:minmax(0,1.45fr) minmax(340px,1fr);gap:18px}.safety-camera{display:flex;flex-direction:column;min-width:0}.safety-camera :deep(.camera-body){min-height:240px;height:clamp(240px,25vw,390px);flex:none;padding:0;background:#080d12;overflow:hidden}.safety-camera :deep(.camera-body img){position:absolute;inset:0}.safety-local-stats{display:grid;grid-template-columns:repeat(4,1fr);padding:17px 12px;border-bottom:1px solid var(--line)}.safety-local-stats>div{display:flex;flex-direction:column;align-items:center;gap:2px}.safety-local-stats>div+div{border-left:1px solid var(--line)}.safety-local-stats strong{font-size:26px;line-height:1.25}.safety-local-stats span{font-size:12px;color:var(--muted)}.safety-local-control{display:flex;align-items:center;justify-content:space-between;gap:14px;padding:15px 18px}.voice-command{display:flex;gap:8px;align-items:center;color:var(--muted);font-size:12px}.voice-command svg{color:var(--cyan)}.safety-cloud-body{padding:22px 24px;display:flex;flex-direction:column;gap:17px;height:calc(100% - 53px)}.cloud-status{display:flex;align-items:center;gap:16px}.cloud-emblem{display:grid;place-items:center;width:62px;height:66px;background:var(--input);border:1px solid var(--line);border-radius:10px;color:var(--cyan)}.cloud-emblem.working{color:var(--amber)}.cloud-explanation{color:var(--muted);font-size:13px;line-height:1.8;margin:0;max-width:48em}.required-equipment h2,.point-heading h2{font-size:13px;font-weight:500;margin:0 0 13px}.gear-list{display:grid;grid-template-columns:1fr 1fr;gap:10px}.gear-item{display:flex;gap:9px;align-items:center;min-height:40px;border:1px solid var(--line);border-radius:6px;padding:9px 12px;color:#768393;background:var(--input);font-size:12px}.gear-item.selected{color:var(--ink);border-color:#52d4d54a;background:#52d4d508}.gear-item.selected svg{color:var(--cyan)}.safety-schedule{display:flex;align-items:center;justify-content:space-between;padding-top:13px;border-top:1px solid var(--line);gap:15px;font-size:12px}.safety-schedule>span{display:flex;align-items:center;gap:9px;color:var(--muted)}.safety-schedule b{font-weight:500}.cloud-actions{margin-top:auto;display:flex;justify-content:flex-end;padding-top:6px}.next-capture{color:var(--cyan);font-size:12px}.safety-message{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 15px;border:1px solid var(--line);border-radius:6px;color:var(--cyan);font-size:12px}.safety-message.error{color:var(--amber)}.safety-records{min-width:0}.safety-records :deep(.card-head){height:auto;min-height:58px;padding-block:10px;gap:12px;flex-wrap:wrap}.safety-filters{display:flex;align-items:center;gap:18px}.safety-filters label{display:flex;align-items:center;gap:8px;font-size:12px;color:var(--muted)}.safety-filters select{padding:5px 8px;min-height:32px}.safety-table-scroll{overflow:auto}.safety-table-scroll table{width:100%;min-width:670px}.safety-table-scroll td{height:58px}.missing-cell{max-width:260px;white-space:normal}.safety-config fieldset{border:0;margin:0 0 20px;padding:0}.safety-config legend{margin-bottom:14px;font-size:14px}.gear-options{display:grid;grid-template-columns:1fr 1fr;gap:12px}.gear-options label{display:flex;align-items:center;gap:10px;padding:12px;border:1px solid var(--line);border-radius:6px;background:var(--input)}input[type=checkbox]{width:16px;height:16px;accent-color:var(--cyan);flex:none}.auto-cloud{display:flex;align-items:center;gap:10px;padding:16px 0 22px;border-bottom:1px solid var(--line)}.point-heading{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-top:22px}.point-heading h2{margin:0;font-size:14px}.config-help{color:var(--muted);font-size:12px;line-height:1.8;margin:12px 0}.config-help.red{color:var(--red)}.no-points{display:flex;align-items:center;justify-content:center;gap:12px;min-height:116px;color:var(--muted);border:1px dashed var(--line);border-radius:7px}.safety-point{margin-top:14px;padding:16px;border:1px solid var(--line);border-radius:7px;background:var(--input)}.point-line{display:flex;gap:14px;align-items:flex-end}.point-name{flex:1;min-width:0}.point-map{flex:1.5;min-width:0}.safety-point label:not(.point-enable){display:flex;flex-direction:column;gap:7px;font-size:12px;color:var(--muted)}.safety-point input:not([type=checkbox]),.safety-point select{width:100%;min-width:0;min-height:36px;padding:7px 9px}.point-coordinates{display:grid;grid-template-columns:repeat(3,minmax(0,1fr)) auto;gap:14px;margin-top:15px;align-items:end}.point-enable{display:flex;align-items:center;gap:7px;min-height:36px;white-space:nowrap}.config-buttons{display:flex;justify-content:flex-end;gap:12px;border-top:1px solid var(--line);margin-top:24px;padding-top:18px}.detail-summary{display:flex;flex-wrap:wrap;gap:14px;align-items:center;margin-bottom:18px;color:var(--muted);font-size:12px}.evidence-stage{position:relative;width:100%;line-height:0;background:#080d12}.evidence-stage img{width:100%;height:auto;display:block}.person-box{position:absolute;border:2px solid var(--cyan);pointer-events:none}.person-box.is-violation{border-color:var(--red)}.person-box>span{display:inline-block;background:#10151be8;line-height:1.5;font-size:12px;padding:3px 6px;color:var(--ink)}.detail-required,.detail-counts{margin-top:15px;color:var(--muted);font-size:12px}.person-result{margin-top:18px;border-top:1px solid var(--line);padding-top:8px}.person-result h3{font-size:13px}.person-gear{display:grid;grid-template-columns:140px 76px 1fr;align-items:start;gap:14px;margin:12px 0;font-size:12px}.person-gear .badge{width:fit-content}.gear-reason{color:var(--muted);overflow-wrap:anywhere}
@media(min-width:1700px){.safety-main{grid-template-columns:minmax(0,1.55fr) minmax(390px,1fr)}.safety-cloud-body{padding:26px 30px;gap:23px}}
@media(max-width:1150px){.safety-main{grid-template-columns:minmax(0,1fr)}.safety-camera :deep(.camera-body){height:360px}.safety-cloud-body{height:auto}.gear-list{grid-template-columns:repeat(4,1fr)}.cloud-actions{padding-top:0}}
@media(max-width:650px){.safety-main{gap:14px}.safety-camera :deep(.camera-body){height:240px;min-height:200px}.safety-local-control{flex-direction:column;align-items:stretch}.safety-local-control>.btn{width:100%}.safety-local-stats{padding-inline:4px}.safety-local-stats strong{font-size:23px}.safety-local-stats span{font-size:10px}.gear-list{grid-template-columns:1fr 1fr}.safety-cloud-body{padding:20px}.safety-filters{gap:10px;flex-wrap:wrap}.safety-filters label{gap:6px;font-size:11px}.safety-filters select{max-width:115px}.point-line{flex-wrap:wrap;gap:10px}.point-name,.point-map{flex-basis:calc(50% - 5px)}.point-coordinates{grid-template-columns:1fr 1fr}.gear-options{grid-template-columns:1fr}.person-gear{grid-template-columns:1fr auto;gap:8px}.gear-reason{grid-column:1/-1}.safety-config .btn{min-height:40px}.safety-local-control .voice-command{font-size:11px}}
</style>
