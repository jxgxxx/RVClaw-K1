import {computed} from 'vue';
import {device,cameraActivity} from './device';
import {withCameraHealth} from './cameraHealth';
import {withVoiceHealth} from './voiceHealth';

export const monitoring=computed(()=>{
 const data=withCameraHealth(withVoiceHealth(device.snapshot,device.voiceHealth,device.observedAt)?.health,device.snapshot,cameraActivity,device.observedAt);
 if(device.error)return {state:'error',status:'监测异常',hardware:(data?.hardware||[]).map(row=>({...row,state:'unknown',status:'待确认',reason:'设备状态读取失败，历史状态不代表当前情况'})),metrics:(data?.metrics||[]).map(row=>({...row,state:'unknown',status:'待确认'})),issues:[{id:'connection',label:'设备连接',state:'error',status:'异常',reason:'设备状态读取失败',advice:'检查小车供电、网络连接和网页服务。'}]};
 if(!data)return {state:'unknown',status:'正在确认',hardware:[],metrics:[],issues:[]};
 const issues=[...data.issues],recent=device.samples.slice(-3);
 if(recent.length===3&&recent.every(row=>row.networkLatency>=1000))issues.push({id:'network',label:'网络响应',state:'error',status:'异常',reason:'连续三次响应延时超过 1000 ms',advice:'检查当前网络质量与设备负载。'});
 return {...data,issues,state:issues.length?'error':data.state,status:issues.length?'监测异常':data.status};
});
export const monitorColor=state=>state==='error'?'red':state==='ok'?'green':'amber';
export const monitorSummary=computed(()=>{
 const data=monitoring.value;if(data.issues.length){const names=data.issues.slice(0,2).map(row=>row.label+'异常');return names.join('、')+(data.issues.length>2?' 等 '+data.issues.length+' 项':'');}
 if(data.hardware.some(row=>row.id==='camera'&&row.status==='切换中'))return '相机模式切换中';
 return data.state==='ok'?'设备监测正常':data.state==='idle'?'设备已连接，待机中':'正在确认硬件状态';
});
