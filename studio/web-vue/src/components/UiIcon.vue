<script setup>
import { icons } from '../icons';
defineProps({ name: { type:String, default:'grid' }, size: { type:[Number,String], default:18 } });
</script>
<template><svg aria-hidden="true" :width="size" :height="size" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.65" stroke-linecap="round" stroke-linejoin="round" v-html="icons[name] || icons.grid"></svg></template>

