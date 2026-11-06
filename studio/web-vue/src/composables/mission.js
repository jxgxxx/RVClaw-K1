// Task completion percentages come only from confirmed backend arrivals.
// TaskMap interpolates actual coordinates separately from confirmed progress.
export function missionProgress(task){
 const route=(task?.route||['A','B','C','ORIGIN']).filter((id,i,a)=>a.indexOf(id)===i),steps=task?.steps||[];
 let arrived=0;for(const id of route){if(steps.find(s=>s.id===id)?.status==='done')arrived++;else break;}
 const terminal=task?.state==='completed'&&arrived===route.length;
 const percent=terminal?100:route.length?arrived/route.length*100:0;
 return{percent,arrived,route,stage:route[arrived]||null,completed:terminal};
}

export function missionStartup(task){
 const step=task?.steps?.find(item=>item.id==='start'),state=String(task?.state||'idle');
 const base={id:'start',title:'起步',done:false,active:false,failed:false};
 const arrived=(task?.route||[]).some(id=>task?.steps?.find(item=>item.id===id)?.status==='done');
 if(['failed','error','aborted'].includes(step?.status))return {...base,failed:true,label:'起步失败',description:'起步未能完成，请检查设备后重试。'};
 // Older progress producers mark start done on TASK_STARTED, before physical
 // departure is known. A terminal task with no arrivals cannot prove departure.
 if(['failed','aborted','startup_failed'].includes(state)&&!arrived)return {...base,failed:true,label:state==='aborted'?'已中止':task?.failure_phase==='startup'?'起步失败':'起步阶段异常',description:task?.failure_reason||'任务在到达首个点位前结束，请检查起步及首段巡检情况。'};
 if(arrived||state==='completed'||step?.departure_confirmed===true||task?.startup?.first_motion_at)return {...base,done:true,label:'已完成',description:task?.startup?.first_motion_at?`实际起步用时 ${task.startup.elapsed_s} 秒`:'已启动并进入巡检路线。'};
 if(['running','active','in_progress','localizing'].includes(state)||step?.status==='active'){
  const labels={accepted:'正在准备巡检',environment_ready:'正在启动导航',navigation_service_ready:'正在检查地图',control_priority_ready:'正在检查定位',route_ready:'正在检查定位',localization_verified_in_route:'定位已确认，准备起步'};
  const startup=task?.startup;
  return {...base,active:true,label:'起步中',description:startup?`${labels[startup.stage]||'正在检查硬件与定位'} · 已等待 ${startup.elapsed_s} 秒`:'正在检查硬件与定位。'};
 }
 return {...base,label:'待执行',description:'等待巡检任务启动。'};
}
