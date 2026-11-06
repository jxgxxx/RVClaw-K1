<script setup>
import { computed, nextTick, onBeforeUnmount, ref, watch } from 'vue';
import { pointLabel } from '../composables/presentation';
import { appendLiveTrack } from '../composables/liveTrack';
const props=defineProps({map:Object,points:{type:Array,default:()=>[]},selected:String,progress:{type:Number,default:null},taskKey:String,livePose:Object,liveTrack:Object,trajectory:Object,moving:Boolean,statusAt:Number,full:Boolean,controls:Boolean,route:{type:Array,default:()=>[]}});
const emit=defineEmits(['position']);
const background=ref(''),source=ref(null),crop=ref({x:0,y:0,width:310,height:67});
const normalized=computed(()=>{
 if(!props.map?.origin||props.map.available===false)return [];
 const raw=props.points.length?props.points:(props.map?.points||[]);
 const pts=raw.filter(p=>(p.x_m??p.x)!=null&&(p.y_m??p.y)!=null).map(p=>({id:p.point||p.name,x:Number(p.x_m??p.x),y:Number(p.y_m??p.y),yaw:Number(p.yaw_deg??(p.yaw||0)*180/Math.PI)})).filter(p=>Number.isFinite(p.x)&&Number.isFinite(p.y));
 if(!pts.some(p=>/ORIGIN/.test(p.id)))pts.unshift({id:'ORIGIN',x:-.15,y:.67,yaw:0});
 return pts;
});
function pixel(p){const m=props.map;if(!m)return[0,0];const[ox,oy,yaw=0]=m.origin,c=Math.cos(yaw),s=Math.sin(yaw);return[(c*(p.x-ox)+s*(p.y-oy))/m.resolution,m.height-(-s*(p.x-ox)+c*(p.y-oy))/m.resolution];}
const visual=computed(()=>normalized.value.map(p=>{const[x,y]=pixel(p);return{...p,px:(x-crop.value.x)*310/crop.value.width,py:(y-crop.value.y)*310/crop.value.width};}));
const routePoints=computed(()=> {
 const ids=props.route.length?['ORIGIN',...props.route.filter(id=>id!=='ORIGIN')]:['ORIGIN',...visual.value.filter(p=>!/^ORIGIN/.test(p.id)).map(p=>p.id)];
 if(props.route.at(-1)==='ORIGIN')ids.push(...ids.slice(0,-1).reverse());
 return ids.map(id=>visual.value.find(p=>p.id===id)).filter(Boolean);
});
const routePath=computed(()=>routePoints.value.map(p=>p.px+','+p.py).join(' '));
const legs=computed(()=>{const ids=props.route.length?props.route:['A','B','C','ORIGIN'],out=[];let previous=visual.value.find(p=>/ORIGIN/.test(p.id));const visited=[previous];for(const id of ids){const next=visual.value.find(p=>p.id===id);if(!previous||!next)continue;const points=/ORIGIN/.test(id)&&visited.length>2?[...visited].reverse():[previous,next];out.push(points.filter(Boolean));visited.push(next);previous=next;}return out;});
const animationClock=ref(Date.now()),actualPose=ref(null),track=ref([]);
const statusFresh=computed(()=>!props.statusAt||animationClock.value-props.statusAt<7000);
watch([()=>props.taskKey,()=>props.map?.id],()=>{actualPose.value=null;track.value=[];},{flush:'sync'});
watch([()=>props.liveTrack,()=>props.livePose,()=>props.taskKey,()=>props.map?.id],([value,pose])=>{
 // Merge older source samples before the latest marker, regardless of prop
 // update order. Otherwise the latest stamp would discard all backfill.
 if(value&&value.task_id===props.taskKey&&value.map_id===props.map?.id){
  for(const sample of value.points||[]){
   if(!['x','y','yaw','sampled_at'].every(key=>typeof sample[key]==='number'&&Number.isFinite(sample[key])))continue;
   track.value=appendLiveTrack(track.value,sample);
  }
 }
 if(!pose||pose.task_id!==props.taskKey||pose.map_id!==props.map?.id)return;
 if(!['x','y','yaw','sampled_at'].every(key=>typeof pose[key]==='number'&&Number.isFinite(pose[key])))return;
 const age=Date.now()/1000-pose.sampled_at;if(age<0||age>7||pose.sampled_at<=(actualPose.value?.sampled_at||0))return;
 actualPose.value={...pose};track.value=appendLiveTrack(track.value,pose);
},{immediate:true});
const freshnessTimer=setInterval(()=>{animationClock.value=Date.now();},500);
onBeforeUnmount(()=>clearInterval(freshnessTimer));
const liveFresh=computed(()=>statusFresh.value&&actualPose.value&&animationClock.value/1000-actualPose.value.sampled_at<7);
const location=computed(()=>{
 if(!actualPose.value)return null;const[x,y]=pixel(actualPose.value);
 return{x:(x-crop.value.x)*310/crop.value.width,y:(y-crop.value.y)*310/crop.value.width};
});
const savedTrack=computed(()=>props.trajectory?.task_id===props.taskKey&&props.trajectory?.map_id===props.map?.id?(props.trajectory.points||[]):[]);
const traveledPaths=computed(()=>{const segments=[],points=props.moving&&track.value.length?track.value:savedTrack.value.length?savedTrack.value:track.value;for(const p of points){if(!['x','y','sampled_at'].every(k=>typeof p[k]==='number'&&Number.isFinite(p[k])))continue;if(p.breakBefore||!segments.length)segments.push([]);const[x,y]=pixel(p);segments.at(-1).push([(x-crop.value.x)*310/crop.value.width,(y-crop.value.y)*310/crop.value.width].join(','));}return segments.map(s=>s.join(' '));});
const mapHeight=computed(()=>310*crop.value.height/crop.value.width);
const zoom=ref(1);
const viewBox=computed(()=>{const h=mapHeight.value+27,w=310/zoom.value,height=h/zoom.value;return [(310-w)/2,-13+(h-height)/2,w,height].join(' ');});
function changeZoom(factor){zoom.value=Math.max(1,Math.min(4,Number((zoom.value*factor).toFixed(3))));}
watch([()=>props.map?.image_url,()=>props.full],()=>{zoom.value=1;});
const scaleLength=computed(()=>310/(crop.value.width*(props.map?.resolution||1)));
async function renderBackground(){
 if(!source.value||!props.map)return;
 const m=props.map,pixels=normalized.value.map(pixel);
 if(props.full)crop.value={x:0,y:0,width:m.width,height:m.height};
 else {
  const x=Math.max(0,Math.floor(Math.min(...pixels.map(p=>p[0]))-1.8/m.resolution));
  const right=Math.min(m.width,Math.ceil(Math.max(...pixels.map(p=>p[0]))+4.2/m.resolution));
  const width=Math.max(1,right-x),height=Math.min(m.height,Math.max(width/(310/67),Math.max(...pixels.map(p=>p[1]))-Math.min(...pixels.map(p=>p[1]))+1.6/m.resolution));
  const center=(Math.min(...pixels.map(p=>p[1]))+Math.max(...pixels.map(p=>p[1])))/2;
  crop.value={x,y:Math.max(0,Math.min(m.height-height,Math.round(center-height/2))),width,height};
 }
 const c=document.createElement('canvas');const renderScale=Math.min(4,2048/Math.max(crop.value.width,crop.value.height));c.width=Math.max(1,Math.round(crop.value.width*renderScale));c.height=Math.max(1,Math.round(crop.value.height*renderScale));
 const ctx=c.getContext('2d');ctx.imageSmoothingEnabled=false;const r=crop.value;ctx.drawImage(source.value,r.x,r.y,r.width,r.height,0,0,c.width,c.height);
 const data=ctx.getImageData(0,0,c.width,c.height);
 for(let i=0;i<data.data.length;i+=4){const v=data.data[i];const color=v<100?[132,151,172]:v>245?[45,59,73]:[24,35,47];data.data[i]=color[0];data.data[i+1]=color[1];data.data[i+2]=color[2];}
 ctx.putImageData(data,0,0);background.value=c.toDataURL();
}
let loadGeneration=0;
watch(()=>props.map?.image_url,async url=>{const generation=++loadGeneration;source.value=null;background.value='';if(!url)return;const img=new Image();img.src=url;try{await img.decode();if(generation!==loadGeneration)return;source.value=img;await nextTick();renderBackground();}catch{if(generation===loadGeneration)background.value='';}},{immediate:true});
watch([()=>props.full,normalized],renderBackground,{deep:true});
function click(event){
 if(!source.value||!props.map)return;
 const svg=event.currentTarget,pt=svg.createSVGPoint();pt.x=event.clientX;pt.y=event.clientY;const p=pt.matrixTransform(svg.getScreenCTM().inverse());
 const r=crop.value,u=r.x+p.x/310*r.width,v=r.y+p.y/310*r.width;
 if(p.x<0||p.x>310||p.y<0||p.y>r.height*310/r.width)return;
 const original=document.createElement('canvas');original.width=props.map.width;original.height=props.map.height;const ctx=original.getContext('2d');ctx.drawImage(source.value,0,0);
 if(ctx.getImageData(Math.floor(u),Math.floor(v),1,1).data[0]<250)return;
 const[ox,oy,yaw=0]=props.map.origin,c=Math.cos(yaw),s=Math.sin(yaw),wy=props.map.height-v;
 emit('position',{x:Number((ox+c*u*props.map.resolution-s*wy*props.map.resolution).toFixed(3)),y:Number((oy+s*u*props.map.resolution+c*wy*props.map.resolution).toFixed(3))});
}
</script>
<template>
<div class="task-map" :class="{'has-controls':controls}">
<svg v-if="background" class="map-svg" :class="{'map-tall':mapHeight>105}" :viewBox="viewBox" role="img" aria-label="任务区域地图和巡检点位" @click="click">
 <image :href="background" x="0" y="0" width="310" :height="mapHeight" preserveAspectRatio="none"/>
 <polyline :points="routePath" fill="none" stroke="#91a4bf" stroke-width="1" stroke-dasharray="2 2"/>
 <polyline v-for="(path,index) in traveledPaths" :key="index" :points="path" fill="none" stroke="#52d4d5" stroke-width="1.2"/>
 <g v-for="point in visual" :key="point.id">
  <g v-if="point.id===selected" :transform="'translate('+point.px+' '+point.py+') rotate('+(-point.yaw)+')'"><path d="m0 0 12 -6a13 13 0 0 1 0 12z" fill="#b79bf730"/><path d="M0 0h13" stroke="#b79bf7" stroke-width=".8"/></g>
  <circle :cx="point.px" :cy="point.py" :r="point.id===selected?3.2:2.4" :fill="/ORIGIN/.test(point.id)?'#eeb073':'#b79bf7'" stroke="#202b38" stroke-width="1"/>
  <rect :x="point.px-(mapHeight>105?20:12)" :y="point.id==='C'?point.py-(mapHeight>105?20:15):point.py+7" :width="mapHeight>105?40:24" :height="mapHeight>105?14:9" rx="2" fill="#111922eb" stroke="#53677b" stroke-width=".3"/>
  <text :x="point.px" :y="point.id==='C'?point.py-8:point.py+(mapHeight>105?18:14)" fill="#d9e4ef" :font-size="mapHeight>105?9:5.3" text-anchor="middle" font-weight="500">{{pointLabel(point.id)}}</text>
 </g>
 <g v-if="location&&progress!=null" class="robot-position" :class="{'is-moving':moving&&liveFresh}" :style="{transform:`translate(${location.x}px, ${location.y}px)`}" :data-sampled-at="actualPose.sampled_at" :data-running="moving&&liveFresh"><circle r="2.5" fill="#111" stroke="#fff" stroke-width=".8"/><title>小车位置</title></g>
 <g :transform="'translate(12 '+(mapHeight-3)+')'"><path :d="'M0 0h'+scaleLength+'M0-2v4m'+scaleLength+'-4v4'" stroke="#a4b7c9" stroke-width=".7"/><text :x="scaleLength/2" y="11" fill="#a4b7c9" font-size="4.8" text-anchor="middle">1 米</text></g>
</svg>
<div v-if="!background" class="empty-state">{{map?.available===false?map.message||'暂无可显示的任务地图':'正在读取任务地图…'}}</div>
<div v-if="controls&&background" class="map-controls" role="group" aria-label="地图缩放操作" @click.stop>
 <button type="button" aria-label="放大地图" title="放大" :disabled="zoom>=4" @click="changeZoom(1.25)"><svg viewBox="0 0 24 24" width="17" height="17" fill="none" stroke="currentColor" stroke-width="1.6"><path d="M5 12h14M12 5v14"/></svg></button>
 <button type="button" aria-label="缩小地图" title="缩小" :disabled="zoom<=1" @click="changeZoom(.8)"><svg viewBox="0 0 24 24" width="17" height="17" fill="none" stroke="currentColor" stroke-width="1.6"><path d="M5 12h14"/></svg></button>
 <button type="button" aria-label="复位地图" title="复位" @click="zoom=1"><svg viewBox="0 0 24 24" width="17" height="17" fill="none" stroke="currentColor" stroke-width="1.6"><path d="M4 9V4h5m6 0h5v5M4 15v5h5m6 0h5v-5"/></svg></button>
</div>
</div>
</template>
<style scoped>
.robot-position{transform-origin:0 0;transform-box:view-box;}
.robot-position.is-moving{transition:transform 1s linear;}
.live-position-status{font-size:12px;color:var(--text-muted,#a4b7c9);padding:4px 8px;}
@media(prefers-reduced-motion:reduce){.robot-position{transition:none;}}
</style>

