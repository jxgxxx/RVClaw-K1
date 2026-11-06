import {computed,ref,watch} from 'vue';
import {preferences} from './preferences';
export function usePagination(rows){
 const page=ref(1),pageSize=ref([5,10,20].includes(Number(preferences.pageSize))?Number(preferences.pageSize):10);
 const pageCount=computed(()=>Math.max(1,Math.ceil(rows.value.length/pageSize.value)));
 const visible=computed(()=>rows.value.slice((page.value-1)*pageSize.value,page.value*pageSize.value));
 watch(pageSize,()=>page.value=1);watch(()=>preferences.pageSize,n=>pageSize.value=Number(n));watch(pageCount,n=>page.value=Math.min(page.value,n));
 return{page,pageSize,pageCount,visible};
}
