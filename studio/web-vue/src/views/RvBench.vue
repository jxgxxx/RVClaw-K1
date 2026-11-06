<script setup>
import {computed,ref} from 'vue';
import {device} from '../composables/device';
import {preferences} from '../composables/preferences';
import {monitoring,monitorColor} from '../composables/monitor';
import {clock} from '../composables/presentation';
const rulesOpen=ref(false),snapshot=computed(()=>device.snapshot||{}),camera=computed(()=>snapshot.value.camera||{});
const metric=id=>monitoring.value.metrics.find(row=>row.id===id)||{state:'unknown',status:'待确认',value:null,reason:'等待有效采样'};
const hardware=id=>monitoring.value.hardware.find(row=>row.id===id)||{state:'unknown',status:'待确认',reason:'等待硬件反馈'};
const number=value=>typeof value==='number'&&Number.isFinite(value)?value.toFixed(1):'—';
const gb=value=>typeof value==='number'&&Number.isFinite(value)?(value/1024**3).toFixed(1):'—';
const samples=computed(()=>device.samples),times=computed(()=>samples.value.map(row=>row.time));
const latency=computed(()=>device.error?null:samples.value.at(-1)?.networkLatency);
const networkState=computed(()=>device.error||monitoring.value.issues.some(row=>row.id==='network')?'error':snapshot.value.sampled_at?'ok':'unknown');
const networkStatus=computed(()=>networkState.value==='error'?'网络异常':networkState.value==='ok'?'连接正常':'正在连接');
const cores=computed(()=>snapshot.value.cpu?.frequencies_mhz?.length||null);
const healthy=computed(()=>monitoring.value.hardware.filter(row=>row.state==='ok').length);
const range=key=>{const values=samples.value.map(row=>row[key]).filter(Number.isFinite);return values.length?Math.min(...values).toFixed(1)+' – '+Math.max(...values).toFixed(1):'—';};
const resourceRows=computed(()=>[
 {id:'cpu',name:'处理器占用',text:number(metric('cpu').value)+' %',value:metric('cpu').value,color:'#b79bf7'},
 {id:'memory',name:'内存占用',text:gb(snapshot.value.memory?.used_bytes)+' / '+gb(snapshot.value.memory?.total_bytes)+' GB',value:metric('memory').value,color:'#52d4d5'},
 {id:'storage',name:'存储占用',text:gb(snapshot.value.storage?.used_bytes)+' / '+gb(snapshot.value.storage?.total_bytes)+' GB',value:metric('storage').value,color:'#9cacc5'},
]);
</script>
<template>
<section class="rvbench-report" aria-label="小车硬件与主控实时监测">
 <div class="rvb-heading"><div><h2>RVBench <span>设备运行监测</span></h2></div><div class="rvb-report-meta"><div><span>最近采样</span><b class="number">{{snapshot.sampled_at?clock(snapshot.sampled_at):'等待采样'}}</b></div><div><span>更新间隔</span><b>{{preferences.refreshSeconds}} 秒 / 次</b></div><div><span>监测状态</span><span class="badge" :class="monitorColor(monitoring.state)">{{monitoring.status}}</span></div></div></div>
 <div class="rvb-metrics">
  <section class="rvb-block rvb-device"><h3><UiIcon name="settings" :size="17"/>设备信息</h3><dl><div><dt>当前主控</dt><dd>{{snapshot.model?.target||'设备连接中'}}</dd></div><div><dt>处理器核心</dt><dd>{{cores||'—'}} 核</dd></div><div><dt>内存容量</dt><dd>{{gb(snapshot.memory?.total_bytes)}} GB</dd></div><div><dt>存储容量</dt><dd>{{gb(snapshot.storage?.total_bytes)}} GB</dd></div><div><dt>硬件反馈</dt><dd>{{healthy}} / {{monitoring.hardware.length||'—'}} 项正常</dd></div><div><dt>当前采样</dt><dd>{{samples.length}} 次</dd></div></dl></section>
  <section class="rvb-block rvb-metric" data-metric="network"><UiIcon name="wifi" :size="27"/><h3>网络响应</h3><strong class="number" :class="networkState==='error'?'red':''">{{latency==null?'—':Math.round(latency)}}<small>ms</small></strong><p>最近范围 {{range('networkLatency')}} ms</p><span class="badge" :class="monitorColor(networkState)">{{networkStatus}}</span></section>
  <section class="rvb-block rvb-metric" data-metric="temperature"><UiIcon name="temp" :size="27"/><h3>处理器平均温度</h3><strong class="number" :class="metric('temperature').state==='error'?'red':''">{{number(metric('temperature').value)}}<small>°C</small></strong><p>最近范围 {{range('temperature')}} °C</p><span class="badge" :class="monitorColor(metric('temperature').state)">{{metric('temperature').state==='error'?'温度异常':metric('temperature').status}}</span></section>
  <section class="rvb-block rvb-metric" data-metric="cpu"><UiIcon name="chip" :size="27"/><h3>处理器占用</h3><strong class="number" :class="metric('cpu').state==='error'?'red':''">{{number(metric('cpu').value)}}<small>%</small></strong><p>最近范围 {{range('cpuUsage')}} %</p><span class="badge" :class="monitorColor(metric('cpu').state)">{{metric('cpu').state==='error'?'持续高负载':metric('cpu').status}}</span></section>
  <section class="rvb-block rvb-metric" data-metric="camera"><UiIcon name="camera" :size="27"/><h3>相机链路</h3><strong class="number" :class="hardware('camera').state==='error'?'red':''">{{hardware('camera').state==='ok'?number(camera.fps):'—'}}<small>帧/秒</small></strong><p>完整画面 · {{camera.width||'—'}} × {{camera.height||'—'}}</p><span class="badge" :class="monitorColor(hardware('camera').state)">{{hardware('camera').state==='error'?'相机异常':hardware('camera').state==='ok'?'相机已连接':hardware('camera').status==='切换中'?'切换中':'等待相机反馈'}}</span></section>
 </div>
 <div class="rvb-charts">
  <section class="rvb-block rvb-chart"><h3>网络响应延时<span>ms</span></h3><TrendPlot label="网络响应延时" unit="ms" :times="times" :plot-height="182" :series="[{name:'网络响应延时',color:'#52d4d5',values:samples.map(row=>row.networkLatency)}]"/><div class="rvb-chart-footer"><span><i></i>网络响应延时</span><span>最近 {{samples.length}} 次采样</span></div></section>
  <section class="rvb-block rvb-chart"><h3>处理器占用趋势<span>%</span></h3><TrendPlot label="处理器占用" unit="%" :times="times" :minimum="0" :maximum="100" :plot-height="182" :series="[{name:'处理器占用',color:'#b79bf7',values:samples.map(row=>row.cpuUsage)}]"/><div class="rvb-chart-footer"><span><i style="background:#b79bf7"></i>处理器占用</span><span>最近 {{samples.length}} 次采样</span></div></section>
  <section class="rvb-block rvb-chart"><h3>处理器温度<span>°C</span></h3><TrendPlot label="处理器平均温度" unit="°C" :times="times" :plot-height="182" :series="[{name:'处理器平均温度',color:'#52d4d5',values:samples.map(row=>row.temperature)}]"/><div class="rvb-chart-footer"><span><i></i>处理器平均温度</span><span>最近 {{samples.length}} 次采样</span></div></section>
 </div>
 <div class="rvb-bottom">
  <section class="rvb-block rvb-hardware"><div class="rvb-section-head"><h3><UiIcon name="chip" :size="16"/>硬件与运行链路</h3><span>{{healthy}} 项正常</span></div><div class="rvb-hardware-table"><table><thead><tr><th>监测项目</th><th>状态</th><th>当前反馈</th></tr></thead><tbody><tr v-for="row in monitoring.hardware" :key="row.id" :data-hardware="row.id"><td>{{row.label}}</td><td><span class="badge" :class="monitorColor(row.state)">{{row.status}}</span></td><td :class="row.state==='error'?'red':''">{{row.reason}}</td></tr></tbody></table><div v-if="!monitoring.hardware.length" class="empty-inline">正在获取硬件反馈…</div></div></section>
  <section class="rvb-block rvb-diagnosis" :class="{'has-errors':monitoring.issues.length}"><div class="rvb-section-head"><h3><UiIcon :name="monitoring.issues.length?'alert':'check'" :size="17"/>{{monitoring.issues.length?'异常排查':'运行概览'}}</h3><span class="badge" :class="monitorColor(monitoring.state)">{{monitoring.issues.length?monitoring.issues.length+' 项异常':monitoring.status}}</span></div><div v-if="monitoring.issues.length" class="rvb-issues" tabindex="0" aria-label="异常原因与排查建议"><article v-for="issue in monitoring.issues" :key="issue.id"><b class="red">{{issue.label}}异常</b><p>{{issue.reason}}</p><small>{{issue.advice}}</small></article></div><div v-else class="rvb-clear" :class="monitorColor(monitoring.state)"><UiIcon :name="monitoring.state==='ok'?'check':'clock'" :size="35"/><strong>{{monitoring.status}}</strong><p>{{monitoring.state==='ok'?'硬件反馈与主控指标均处于监测范围内':monitoring.state==='idle'?'设备已连接，任务启动后将自动采集并检查实时反馈':'正在核实实时反馈，尚未确认的项目不会标记为正常'}}</p></div></section>
  <section class="rvb-block rvb-resources"><h3>主控资源</h3><div v-for="row in resourceRows" :key="row.id" class="rvb-resource"><span>{{row.name}}<b class="number" :class="metric(row.id).state==='error'?'red':''">{{row.text}}</b></span><div class="rvb-resource-track"><i :style="{width:(row.value??0)+'%',background:metric(row.id).state==='error'?'var(--red)':row.color}"></i></div><small v-if="metric(row.id).state==='error'" class="red">{{metric(row.id).reason}}</small></div><div class="rvb-rule-note"><UiIcon name="eye" :size="16"/><div><b>只读监测</b><p>读取真实状态，不运行压测，不解除保护。</p></div></div><button class="btn rvb-rules-button" @click="rulesOpen=true"><UiIcon name="sliders" :size="14"/>查看异常判定规则</button></section>
 </div>
</section>
<UiModal v-model="rulesOpen" title="异常判定规则" width="min(610px,92vw)"><dl class="rvb-rule-list"><div><dt>处理器温度</dt><dd>任一处理器核心温度达到 80 °C 时提示异常，主卡显示平均温度。</dd></div><div><dt>处理器占用</dt><dd>利用率连续 30 秒达到 90% 以上时提示异常；短时计算峰值不直接认定为故障。</dd></div><div><dt>内存与存储</dt><dd>占用达到 90% 时分别提示内存或存储空间不足。</dd></div><div><dt>硬件反馈</dt><dd>无任务且采集服务按需休眠时，已连接的设备显示待机；设备拔出或权限不足仍提示异常。任务运行中，反馈超过 5 秒未更新或明确报告离线时提示异常。</dd></div><div><dt>语音模块</dt><dd>控制串口由语音服务持有、语音录音正在采集时显示正常；未接设备显示异常，并说明缺少的连接。不需要通过说话确认在线。</dd></div><div><dt>监测链路</dt><dd>监测心跳超过 8 秒未更新时，不沿用旧的正常状态。</dd></div><div><dt>网络响应</dt><dd>网页连续三次接口响应超过 1000 ms 时提示延时异常；连接失败时直接提示。</dd></div></dl><p class="subtle">以上为网页的监测提示规则，不会修改小车的运行参数或硬件保护策略。</p><template #footer><button class="btn primary" @click="rulesOpen=false">知道了</button></template></UiModal>
</template>
