<script setup>
import { computed } from 'vue';
import { preferences as p } from '../composables/preferences';
import { device } from '../composables/device';
import { monitoring, monitorColor } from '../composables/monitor';
const voice = computed(() => monitoring.value.hardware.find(row => row.id === 'voice') || {state:'unknown',status:'待确认'});
import { clock } from '../composables/presentation';
const snapshot=computed(()=>device.snapshot||{});
const groups=[
 {title:'显示与更新',icon:'sun',rows:[
 {key:'refreshSeconds',title:'状态更新间隔',description:'选择状态信息的更新频率',options:[[3,'3 秒'],[5,'5 秒'],[10,'10 秒']]},
 {key:'defaultPage',title:'默认打开页面',description:'下次访问时首先显示的页面',options:[['overview','总览'],['meter','仪表检测'],['meter-settings','巡检设置'],['inspection','巡检进度'],['devices','RVBench'],['alerts','告警记录'],['logs','运行日志'],['settings','设置']]},
 {key:'compact',title:'界面密度',description:'调整列表与控件的显示间距',options:[[false,'标准'],[true,'紧凑']]},
 {key:'pauseBackground',title:'后台暂停更新',description:'切换到其他标签页后暂停状态更新',toggle:true},
 ]},
 {title:'相机与地图',icon:'camera',rows:[
 {key:'cameraFit',title:'相机显示方式',description:'等比例显示；填满区域时可能裁去边缘',options:[['contain','完整画面'],['cover','填满区域']]},
 {key:'autoCamera',title:'自动打开相机',description:'进入相机相关页面时打开画面',toggle:true},
 {key:'mapRange',title:'地图默认范围',description:'优先显示任务点位所在区域',options:[['task','任务区域'],['full','完整地图']]},
 {key:'routeAnimation',title:'巡检路线动画',description:'显示随实际任务到点确认更新的位置',toggle:true},
 ]},
 {title:'告警与记录',icon:'file',rows:[
 {key:'alertNotice',title:'新告警提示',description:'出现新记录时在页面内提示',toggle:true},
 {key:'pageSize',title:'每页记录数量',description:'设置各记录列表的默认显示数量',options:[[5,'5 条'],[10,'10 条'],[20,'20 条']]},
 {key:'hour12',title:'时间显示',description:'使用一致的时间显示格式',options:[[false,'24 小时制'],[true,'12 小时制']]},
 ]}
];
</script>
<template>
<div class="settings-grid">
 <UiCard v-for="group in groups" :key="group.title" :title="group.title" :glyph="group.icon"><div class="card-content"><div v-for="row in group.rows" :key="row.key" class="setting-row"><div><b :id="'label-'+row.key">{{row.title}}</b><small>{{row.description}}</small></div><button v-if="row.toggle" class="toggle" role="switch" :aria-checked="p[row.key]" :aria-labelledby="'label-'+row.key" @click="p[row.key]=!p[row.key]"></button><select v-else v-model="p[row.key]" :aria-labelledby="'label-'+row.key"><option v-for="option in row.options" :key="String(option[0])" :value="option[0]">{{option[1]}}</option></select></div></div></UiCard>
 <UiCard title="设备信息" glyph="chip"><div class="card-content">
 <div class="setting-row"><div><b>当前连接设备</b><small>设备名称随连接自动更新</small></div><b style="font-size:13px">{{/k1/i.test(snapshot.model?.target||snapshot.host||'')?'Firefly K1':'Firefly K3'}}</b></div>
 <div class="setting-row"><div><b>相机名称</b><small>当前设备连接的相机</small></div><b style="font-size:13px">{{snapshot.camera?.name||'等待数据'}}</b></div>
 <div class="setting-row"><div><b>画面分辨率</b><small>相机当前输出的画面尺寸</small></div><span class="badge">{{snapshot.camera?.width||'—'}} × {{snapshot.camera?.height||'—'}}</span></div>
 <div class="setting-row" data-voice-info><div><b>语音模块</b><small>语音控制与录音链路</small></div><span class="badge" :class="monitorColor(voice.state)" :title="voice.reason">{{voice.status}}</span></div>
 <div class="setting-row"><div><b>最近状态更新</b><small>设备信息最后一次采样的时间</small></div><b class="number" style="font-size:13px">{{clock(snapshot.sampled_at)}}</b></div>
 </div></UiCard>
</div>
</template>

