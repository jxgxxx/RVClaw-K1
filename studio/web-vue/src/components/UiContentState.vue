<script setup>
import UiIcon from './UiIcon.vue';
defineProps({loading:Boolean,skeleton:Boolean,icon:{type:String,default:'file'},title:String,description:String,retry:Boolean});
defineEmits(['retry']);
</script>

<template>
 <div class="content-state" :class="{'is-loading':loading}" role="status" aria-live="polite" aria-atomic="true">
  <span class="state-icon" aria-hidden="true"><svg v-if="loading" class="state-spinner" viewBox="0 0 32 32" fill="none"><circle cx="16" cy="16" r="12" stroke="currentColor" stroke-width="2.4" opacity=".16"/><path d="M16 4a12 12 0 0 1 12 12" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"/></svg><UiIcon v-else :name="icon" :size="25"/></span>
  <div class="state-copy"><b>{{title}}</b><p v-if="description">{{description}}</p></div>
  <div v-if="loading&&skeleton" class="state-skeleton" aria-hidden="true"><div v-for="row in 3" :key="row" class="skeleton-row"><span class="skeleton-mark"></span><span class="skeleton-copy"><i></i><i></i></span><span class="skeleton-tag"></span></div></div>
  <button v-if="retry" type="button" class="btn state-retry" @click="$emit('retry')"><UiIcon name="refresh" :size="14"/>重新加载</button>
 </div>
</template>

<style scoped>
.content-state{flex:1;min-width:0;min-height:200px;width:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:18px;padding:32px 24px;text-align:center;color:var(--muted)}
.state-icon{position:relative;display:grid;place-items:center;flex:none;width:58px;height:58px;border:1px solid var(--line);border-radius:16px;background:var(--input);color:var(--muted)}
.is-loading .state-icon{border:0;border-radius:0;background:transparent;color:var(--cyan,#52d4d5)}
.state-spinner{width:32px;height:32px;animation:state-turn 1s linear infinite}
.state-copy{max-width:320px}.state-copy b{display:block;color:var(--ink);font-size:14px;font-weight:500;line-height:1.6}.state-copy p{font-size:12px;line-height:1.8;margin:7px 0 0;overflow-wrap:anywhere}
.state-skeleton{width:100%;max-width:430px;margin-top:6px;display:grid;gap:14px;animation:state-breathe 1.8s ease-in-out infinite}
.skeleton-row{display:flex;align-items:center;gap:13px;min-height:49px;padding-bottom:14px;border-bottom:1px solid var(--line-soft,var(--line))}
.skeleton-mark{flex:none;width:27px;height:31px;background:var(--line);border-radius:5px;opacity:.55}.skeleton-copy{flex:1;display:grid;gap:9px}.skeleton-copy i{display:block;height:7px;width:76%;border-radius:3px;background:var(--line);opacity:.8}.skeleton-copy i+ i{width:48%;opacity:.45}.skeleton-row:nth-child(2) .skeleton-copy i:first-child{width:62%}.skeleton-tag{width:39px;height:15px;border-radius:4px;background:var(--line);opacity:.5}
.state-retry{margin-top:2px;min-height:34px;font-size:12px}.state-retry:focus-visible{outline:2px solid var(--blue);outline-offset:3px}
@keyframes state-turn{to{transform:rotate(360deg)}}@keyframes state-breathe{50%{opacity:.45}}
@media(max-width:520px){.content-state{padding:26px 16px;gap:15px}.state-skeleton{gap:10px}.state-copy p{font-size:11px}}
@media(prefers-reduced-motion:reduce){.state-spinner,.state-skeleton{animation:none}}
</style>
