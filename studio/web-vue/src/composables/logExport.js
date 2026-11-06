export function logDate(value) {
 const date=new Date(typeof value==='number'&&value<1e12?value*1000:value);
 return value!=null&&value!==''&&!Number.isNaN(date.getTime())?date:null;
}
export function dateKey(value) {
 const date=logDate(value);if(!date)return '';
 return [date.getFullYear(),String(date.getMonth()+1).padStart(2,'0'),String(date.getDate()).padStart(2,'0')].join('-');
}
export function fullLogTime(value) {
 const date=logDate(value);if(!date)return '未记录';
 return dateKey(date)+' '+[date.getHours(),date.getMinutes(),date.getSeconds()].map(n=>String(n).padStart(2,'0')).join(':');
}
export function exportRangeError(mode,start,end) {
 if(mode!=='range')return '';
 if(!start||!end)return '请选择开始日期和结束日期。';
 if(!/^\d{4}-\d{2}-\d{2}$/.test(start)||!/^\d{4}-\d{2}-\d{2}$/.test(end))return '请输入有效日期。';
 if(start>end)return '结束日期不能早于开始日期。';
 return '';
}
export function logsInRange(records,mode,start,end) {
 if(mode!=='range')return records;
 if(exportRangeError(mode,start,end))return [];
 return records.filter(row=>{const key=dateKey(row.time);return key&&key>=start&&key<=end;});
}
export function saveFile(blob,filename) {
 const url=URL.createObjectURL(blob),link=document.createElement('a');
 link.href=url;link.download=filename;document.body.append(link);link.click();link.remove();
 setTimeout(()=>URL.revokeObjectURL(url),1000);
}
const readable=value=>value==null||value===''?'未记录':String(value);
const percent=value=>Number.isFinite(value)?value+' %':'未记录';
const space=value=>Number.isFinite(value)?(value/1024**3).toFixed(2)+' GB':'未记录';
function usage(label,item){const ratio=item?.usage_percent??(item?.total_bytes>0&&Number.isFinite(item.used_bytes)?Math.round(item.used_bytes/item.total_bytes*1000)/10:null);return `${label}：${percent(ratio)}（已用 ${space(item?.used_bytes)} / 总计 ${space(item?.total_bytes)}）`;}
function snapshotLines(snapshot) {
 const temperatures=(snapshot.cpu?.temperatures||[]).filter(row=>/^(thermal_cluster|cluster\d+_thermal)/.test(row.type)&&Number.isFinite(row.celsius));
 const temperature=temperatures.length?(temperatures.reduce((sum,row)=>sum+row.celsius,0)/temperatures.length).toFixed(1)+' °C':'未记录';
 const frequency=(snapshot.cpu?.policies||[]).filter(row=>Number.isFinite(row.frequency_mhz)).map(row=>(row.frequency_mhz/1000).toFixed(2)+' GHz').join(' / ')||'未记录';
 const connection=item=>item?.online===true?'已连接':item?.online===false?'未连接':'未记录';
 return [`采样时间：${fullLogTime(snapshot.sampled_at)}`,`处理器平均温度：${temperature}`,`处理器频率：${frequency}`,`处理器占用：${percent(snapshot.cpu?.usage_percent)}`,usage('内存占用',snapshot.memory),usage('存储占用',snapshot.storage),`网络连接：${connection(snapshot.network)}`,`相机链路：${connection(snapshot.camera)}`];
}
export function logText(row,currentSnapshot={}) {
 const lines=['RVClaw 运行记录','',`记录时间：${fullLogTime(row.time)}`,`记录类别：${readable(row.category)}`,`记录来源：${readable(row.source_label)}`,`事件摘要：${readable(row.summary)}`,`事件说明：${readable(row.description)}`,`结果：${row.failed?'异常':'正常'}`,`仪表读数：${readable(row.value)}`,'','记录时设备信息',...(row.snapshot?snapshotLines(row.snapshot):['该条历史记录未保存当时的设备信息。']),'','下载时设备信息（仅反映当前状态）',...snapshotLines(currentSnapshot)];
 return '\ufeff'+lines.join('\r\n')+'\r\n';
}
export function workbookRows(records,currentSnapshot={}) {
 return records.map((row,index)=>[index+1,fullLogTime(row.time),readable(row.category),readable(row.source_label),readable(row.summary),readable(row.description),row.failed?'异常':'正常',readable(row.value),row.snapshot?snapshotLines(row.snapshot).join('\n'):'该条历史记录未保存当时的设备信息。',snapshotLines(currentSnapshot).join('\n'),logText(row,currentSnapshot).replace(/^\ufeff/,'')]);
}
export async function logsWorkbook(records,currentSnapshot={}) {
 // Excel generation is loaded on demand and runs entirely in the user's browser.
 const {default:writeXlsxFile}=await import('write-excel-file/browser');
 const headings=['序号','记录时间','记录类别','记录来源','事件摘要','事件说明','结果','仪表读数','记录时设备信息','导出时设备信息（仅反映当前状态）','完整记录内容（与单条 TXT 一致）'];
 const data=[headings.map(value=>({value,fontWeight:'bold',backgroundColor:'#EDF1FF',color:'#24334D',height:30,alignVertical:'center'})),...workbookRows(records,currentSnapshot).map((values,index)=>values.map((value,column)=>({value,type:column===0?Number:String,wrap:true,alignVertical:'top',...(column===6?{color:records[index].failed?'#B42318':'#087443'}:{})})))];
 return writeXlsxFile(data,{sheet:'运行记录',columns:[7,23,15,16,28,50,10,18,58,58,85].map(width=>({width})),stickyRowsCount:1},{fontFamily:'Microsoft YaHei',fontSize:11}).toBlob();
}
