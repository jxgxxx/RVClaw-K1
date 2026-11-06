<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { initChart } from '../chart';

const props = defineProps({ title: String, rows: { type: Array, default: () => [] }, color: { type: String, default: '#087e78' }, unit: { type: String, default: '' } });
const host = ref(null);
let chart;
let observer;
const option = computed(() => ({
  animation: false,
  grid: { left: 48, right: 14, top: 10, bottom: 24 },
  tooltip: { trigger: 'axis', confine: true, valueFormatter: (value) => `${value}${props.unit ? ` ${props.unit}` : ''}` },
  xAxis: { type: 'category', data: props.rows.map((row) => row.label), axisLine: { lineStyle: { color: '#dce3e5' } }, axisLabel: { color: '#77858c', fontSize: 9, hideOverlap: true } },
  yAxis: { type: 'value', splitLine: { lineStyle: { color: '#e8edef', type: 'dashed' } }, axisLabel: { color: '#77858c', fontSize: 9 } },
  series: [{ type: 'bar', data: props.rows.map((row) => row.value), barMaxWidth: 26, itemStyle: { color: props.color, borderRadius: [3, 3, 0, 0] } }],
}));
watch(option, async (value) => { await nextTick(); chart?.setOption(value, true); }, { deep: true });
onMounted(() => { chart = initChart(host.value); chart.setOption(option.value); observer = new ResizeObserver(() => chart?.resize()); observer.observe(host.value); });
onBeforeUnmount(() => { observer?.disconnect(); chart?.dispose(); });
</script>

<template>
  <article class="resource-chart-panel"><div class="resource-title">{{ title }}</div><div ref="host" class="resource-chart" /></article>
</template>

<style scoped>
.resource-chart-panel { min-width:0; padding:11px; border:1px solid #e2e8ea; border-radius:6px; background:#fff; }
.resource-title { color:#53636b; font-size:10px; font-weight:600; }
.resource-chart { width:100%; height:170px; }
@media (max-height:680px) and (min-width:721px) { .resource-chart { height:132px; } }
</style>
