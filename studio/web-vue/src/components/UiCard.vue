<script setup>
import UiIcon from './UiIcon.vue';
defineProps({ title: String, glyph: String });
</script>
<template>
  <article class="card"><div class="card-head"><div class="card-title"><slot name="title"><UiIcon v-if="glyph" :name="glyph" />{{ title }}</slot></div><slot name="extra" /></div><slot /><div v-if="$slots.foot" class="card-foot"><slot name="foot" /></div></article>
</template>

