<script setup>
import { computed } from 'vue';
const props=defineProps({modelValue:{type:Number,default:1},pageSize:{type:Number,default:10},total:{type:Number,default:0}});
const emit=defineEmits(['update:modelValue','update:pageSize']);
const count=computed(()=>Math.max(1,Math.ceil(props.total/props.pageSize)));
const numbers=computed(()=>{const values=new Set([1,count.value,props.modelValue-1,props.modelValue,props.modelValue+1]);const sorted=[...values].filter(v=>v>=1&&v<=count.value).sort((a,b)=>a-b),out=[];for(const n of sorted){if(out.length&&n-Number(out.at(-1))>1)out.push('gap-'+n);out.push(n);}return out;});
function go(n){if(n>=1&&n<=count.value)emit('update:modelValue',n);}
</script>
<template><div class="pagination unified-pagination"><span>共 {{total}} 条</span><div class="pagination-controls"><label>每页 <select :value="pageSize" aria-label="每页记录条数" @change="emit('update:pageSize',Number($event.target.value));go(1)"><option v-for="n in [5,10,20]" :key="n" :value="n">{{n}} 条</option></select></label><button class="page-nav" :disabled="modelValue<=1" @click="go(modelValue-1)">上一页</button><template v-for="n in numbers" :key="n"><span v-if="typeof n==='string'" class="page-gap">···</span><button v-else class="page-no" :class="{active:modelValue===n}" :aria-label="'第 '+n+' 页'" :aria-current="modelValue===n?'page':undefined" @click="go(n)">{{n}}</button></template><button class="page-nav" :disabled="modelValue>=count" @click="go(modelValue+1)">下一页</button></div></div></template>
