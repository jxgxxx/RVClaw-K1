<script setup>
import { computed, defineAsyncComponent, onMounted, onUnmounted, ref } from 'vue';
import zhCn from 'element-plus/es/locale/lang/zh-cn';
import robotImage from './assets/robot-sidebar.webp';
import brandLogo from './assets/rvclaw-logo.svg';
import { device, refreshDevice, startDevicePolling, stopDevicePolling } from './composables/device';
import { refreshPageData } from './composables/pageRefresh';
import { preferences } from './composables/preferences';
import { monitoring,monitorSummary,monitorColor } from './composables/monitor';
const pages=[
 ['overview','grid','设备总览','设备总览','相机画面、巡检任务与设备运行状态。',()=>import('./views/Overview.vue')],
 ['meter','meter','仪表检测','仪表检测','选择数字或指针仪表，获取图片后直接读数。',()=>import('./views/MeterDetection.vue')],
 ['safety','shield','安全防护','安全防护','本地安全帽识别与云端防护装备检查。',()=>import('./views/SafetyRecognition.vue')],
 ['meter-settings','sliders','巡检设置','巡检设置','地图与设置同步预览，保存后生效，取消恢复已保存值。',()=>import('./views/MeterSettings.vue')],
 ['inspection','route','巡检进度','巡检进度','查看巡检读数、路线示意与起步用时。',()=>import('./views/Inspection.vue')],
 ['devices','chip','RVBench','RVBench','四个紧凑的状态宫格，统一查看设备资源。',()=>import('./views/RvBench.vue')],
 ['alerts','alert','告警记录','告警记录','查看异常内容、图片与视频，并进入对应事件处理。',()=>import('./views/Alerts.vue')],
 ['logs','file','运行日志','运行日志','按事件查看运行情况，每条记录可独立查看和下载。',()=>import('./views/Logs.vue')],
 ['settings','settings','设置','设置','统一显示偏好、更新方式与设备状态说明。',()=>import('./views/Settings.vue')],
].map(([id,icon,label,title,description,loader])=>({id,icon,label,title,description,component:defineAsyncComponent(loader)}));
function route(){const id=location.hash.replace(/^#\/?/,'').split('?')[0];return pages.some(p=>p.id===id)?id:pages.some(p=>p.id===preferences.defaultPage)?preferences.defaultPage:'overview';}
const currentPage=ref(route()),routeKey=ref(location.hash),refreshing=ref(false);
const pageEntry=document.querySelector('script[type="module"][src]')?.src;
const current=computed(()=>pages.find(p=>p.id===currentPage.value)||pages[0]);
const online=computed(()=>Boolean(device.snapshot?.network?.online)&&!device.error);
const boardName=computed(()=>{
 const identity=[device.snapshot?.host,device.snapshot?.model?.target,location.hostname].filter(Boolean).join(' ');
 if(/k1/i.test(identity)||location.hostname==='100.112.170.125')return 'Firefly K1';
 if(/k3/i.test(identity)||location.hostname==='100.81.182.90')return 'Firefly K3';
 return '设备待识别';
});
function navigate(id,target={}){const query=new URLSearchParams();for(const key of ['record','session','map_id','photo','waypoint'])if(target[key])query.set(key,String(target[key]));location.hash=id+(query.size?'?'+query:'');currentPage.value=id;routeKey.value=location.hash;}
function onHash(){currentPage.value=route();routeKey.value=location.hash;}
function onPage(event){if(pages.some(p=>p.id===event.detail?.page))navigate(event.detail.page,event.detail.target||{});}
async function refreshVersion(){
 if(import.meta.env.DEV||!pageEntry)return;
 try{
  const response=await fetch(location.pathname+location.search,{cache:'no-store',signal:AbortSignal.timeout(4000)});
  if(!response.ok)return;
  const nextEntry=new DOMParser().parseFromString(await response.text(),'text/html').querySelector('script[type="module"][src]')?.getAttribute('src');
  if(nextEntry&&new URL(nextEntry,location.href).href!==pageEntry)location.reload();
 }catch{/* Record refresh remains available when the version check cannot connect. */}
}
async function refresh(){if(refreshing.value)return;refreshing.value=true;try{await Promise.allSettled([refreshVersion(),refreshDevice(),refreshPageData(currentPage.value)]);}finally{refreshing.value=false;}}
onMounted(()=>{window.addEventListener('hashchange',onHash);document.addEventListener('studio-page-change',onPage);startDevicePolling();});
onUnmounted(()=>{window.removeEventListener('hashchange',onHash);document.removeEventListener('studio-page-change',onPage);stopDevicePolling();});
</script>
<template>
<el-config-provider :locale="zhCn">
<div class="shell" :class="{'density-compact':preferences.compact}">
 <header class="topbar global-header">
  <button class="brand" type="button" aria-label="RVClaw 返回总览" @click="navigate('overview')"><div class="brand-mark"><img :src="brandLogo" alt="" width="32" height="32" aria-hidden="true"></div><b>RVClaw <span>Studio</span></b></button>
  <div class="header-board"><i class="dot" :class="online?'green':'red'"></i><b>{{boardName}}</b><span>巡检机器人</span><span class="board-address">{{device.snapshot?.network?.tailscale_ipv4||device.snapshot?.network?.ipv4||''}}</span></div>
  <div class="top-actions"><button v-if="currentPage==='overview'" type="button" class="top-health" :class="monitorColor(monitoring.state)" :title="monitorSummary+'，点击进入 RVBench 排查'" aria-label="查看设备监测与异常排查" @click="navigate('devices')"><UiIcon :name="monitoring.state==='error'?'alert':monitoring.state==='ok'?'check':'clock'" :size="15"/><span role="status" aria-atomic="true">{{monitorSummary}}</span><UiIcon name="arrow" :size="13"/></button><span class="connection" :class="{disconnected:!online}"><i class="dot"></i>{{online?'设备已连接':'设备连接中'}}</span><button class="btn" type="button" :disabled="refreshing" aria-label="刷新当前页面" @click="refresh"><UiIcon name="refresh" :size="15" :class="{'is-spinning':refreshing}"/>刷新</button></div>
 </header>
 <aside class="sidebar">
  <nav class="nav" aria-label="主导航"><button v-for="page in pages" :key="page.id" type="button" class="nav-item" :class="{active:currentPage===page.id}" :data-page="page.id" :aria-current="currentPage===page.id?'page':undefined" @click="navigate(page.id)"><UiIcon :name="page.icon" :size="18"/><span>{{page.label}}</span></button></nav>
  <section class="robot-card"><div class="robot-image"><img :src="robotImage" alt="RVClaw 巡检机器人完整照片"></div><div class="robot-name"><b>{{boardName}}</b><span><i class="dot"></i>{{online?'设备在线':'等待连接'}}</span></div></section>
  <div class="sidebar-foot">RVClaw · 设备巡检工作台</div>
 </aside>
 <section class="workspace">
  <main class="page-main" :key="routeKey" :data-route="currentPage">
   <div class="page-heading"><h1 class="page-title">{{current.title}}</h1></div>
   <component :is="current.component"/>
  </main>
 </section>
</div>
</el-config-provider>
</template>


<style scoped>
.global-header .brand-mark{flex:none}
.global-header .brand-mark img{display:block;width:100%;height:100%;object-fit:contain}
.global-header .brand b{color:#f0f2f7}
.global-header .brand b>span{color:#aeb6c6}
</style>
