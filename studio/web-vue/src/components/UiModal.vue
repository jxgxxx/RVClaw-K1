<script setup>
import { nextTick,onBeforeUnmount,ref,useId,watch } from 'vue';
const props=defineProps({modelValue:Boolean,title:String,width:{type:String,default:'640px'}});
const emit=defineEmits(['update:modelValue']);
const panel=ref(null),id=useId();let previousFocus,previousOverflow;
function close(){emit('update:modelValue',false);}
function keydown(event){
 if(!props.modelValue)return;
 if(event.key==='Escape'){event.preventDefault();close();}
 if(event.key==='Tab'){
  const elements=[...panel.value.querySelectorAll('button:not(:disabled),a[href],input:not(:disabled),select:not(:disabled),video[controls],[tabindex="0"]')].filter(e=>e.getClientRects().length);
  if(!elements.length){event.preventDefault();panel.value.focus();return;}
  const first=elements[0],last=elements.at(-1);
  if(event.shiftKey&&document.activeElement===first){event.preventDefault();last.focus();}
  else if(!event.shiftKey&&document.activeElement===last){event.preventDefault();first.focus();}
 }
}
watch(()=>props.modelValue,async open=>{
 if(open){previousFocus=document.activeElement;previousOverflow=document.body.style.overflow;document.body.style.overflow='hidden';document.addEventListener('keydown',keydown);await nextTick();panel.value?.querySelector('button')?.focus();}
 else{document.body.style.overflow=previousOverflow||'';document.removeEventListener('keydown',keydown);previousFocus?.focus?.();}
});
onBeforeUnmount(()=>{if(props.modelValue)document.body.style.overflow=previousOverflow||'';document.removeEventListener('keydown',keydown);});
</script>
<template><Teleport to="body"><div v-if="modelValue" class="modal-backdrop" @click.self="close"><div ref="panel" class="modal" role="dialog" aria-modal="true" :aria-labelledby="id" tabindex="-1" :style="{width}"><div class="modal-head"><b :id="id">{{title}}</b><button class="modal-close" type="button" aria-label="关闭弹窗" @click="close"><UiIcon name="x" :size="19"/></button></div><div class="modal-body"><slot/></div><div v-if="$slots.footer" class="action-row"><slot name="footer"/></div></div></div></Teleport></template>

