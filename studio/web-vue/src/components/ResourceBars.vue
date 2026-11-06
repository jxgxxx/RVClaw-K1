<script setup>
import { computed } from 'vue';
import { device } from '../composables/device';
const rows = computed(() => {
  const s = device.snapshot || {}, r=s.resources || {}, memory=s.memory || r.memory || {}, disk=s.storage || r.storage || {};
  const percent = s.cpu?.usage_percent ?? r.cpu_percent;
  const size = value => value == null ? null : (Number(value)/1024**3).toFixed(1);
  return [
    { label:'处理器占用', value:percent == null ? '未提供' : Number(percent).toFixed(1)+' %', percent:percent || 0 },
    { label:'内存占用', value:memory.total_bytes ? size(memory.used_bytes)+' / '+size(memory.total_bytes)+' GB' : '未提供', percent:memory.total_bytes ? memory.used_bytes/memory.total_bytes*100 : 0 },
    { label:'存储占用', value:disk.total_bytes ? size(disk.used_bytes)+' / '+size(disk.total_bytes)+' GB' : '未提供', percent:disk.total_bytes ? disk.used_bytes/disk.total_bytes*100 : 0 },
  ];
});
</script>
<template><div class="resource-bars"><div v-for="row in rows" :key="row.label"><div class="bar-label"><span>{{ row.label }}</span><b class="number">{{ row.value }}</b></div><div class="meter-bar"><span :style="{width:Math.max(0,Math.min(100,row.percent))+'%'}"></span></div></div></div></template>

