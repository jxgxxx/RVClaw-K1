<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { initChart } from '../chart';

const props = defineProps({ title: String, points: { type: Array, default: () => [] }, kind: String });
const host = ref(null);
let chart;
let observer;

const option = computed(() => {
  const time = props.points.map((sample) => new Date(sample.time).toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit' }));
  const temperature = props.kind === 'temperature';
  const actualAvailable = !temperature && props.points.some((point) => point.actual?.length);
  const lines = temperature
    ? [{ name: '平均温度', key: 'temperature', color: '#078b82' }]
    : actualAvailable
      ? [{ name: '实测核心频率', key: 'actual', color: '#b97813' }]
      : [
          { name: '策略组 0-7', key: 'policy0', color: '#375e9a' },
          { name: '策略组 8-15', key: 'policy8', color: '#b97813' },
        ];
  return {
    animation: false,
    grid: { left: 42, right: 10, top: 16, bottom: 26 },
    tooltip: { trigger: 'axis', confine: true, valueFormatter: (value) => `${value} ${temperature ? '°C' : 'MHz'}` },
    xAxis: { type: 'category', data: time, boundaryGap: false, axisLine: { lineStyle: { color: '#cbd3db' } }, axisLabel: { color: '#718090', hideOverlap: true, fontSize: 10 } },
    yAxis: { type: 'value', scale: true, splitLine: { lineStyle: { color: '#e6eaee', type: 'dashed' } }, axisLabel: { color: '#718090', fontSize: 10 } },
    series: lines.map((line) => ({
      name: line.name, type: 'line', showSymbol: props.points.length < 2, symbolSize: 3, smooth: 0.25,
      connectNulls: false, lineStyle: { color: line.color, width: 1.25 },
      itemStyle: { color: line.color },
      data: props.points.map((point) => {
        const value = point[line.key];
        if (Array.isArray(value)) return value.length ? value.reduce((sum, item) => sum + item, 0) / value.length : null;
        return Number.isFinite(value) ? value : null;
      }),
    })),
  };
});

function resize() { chart?.resize(); }
watch(option, async (next) => { await nextTick(); chart?.setOption(next, true); }, { deep: true });
onMounted(() => {
  chart = initChart(host.value);
  chart.setOption(option.value);
  observer = new ResizeObserver(resize);
  observer.observe(host.value);
});
onBeforeUnmount(() => { observer?.disconnect(); chart?.dispose(); });
</script>

<template>
  <article class="chart-panel">
    <div class="chart-heading"><h3>{{ title }}</h3><span>{{ kind === 'temperature' ? '°C' : 'MHz' }}</span></div>
    <div ref="host" class="chart-canvas" :aria-label="`${title}趋势图`" />
  </article>
</template>
