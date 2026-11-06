<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import { MjpegFrames } from '../composables/mjpeg';

const props = defineProps({ src: String, generation: [Number, String], mode: String, online: Boolean, switching: Boolean, enabled: Boolean, pauseBackground: Boolean, fit: String, width: Number, height: Number });
const frameUrl = ref(''), loaded = ref(false), reconnecting = ref(false), frameCount = ref(0);
const frameSize = ref(null);
const aspectRatio = computed(() => `${frameSize.value?.width || props.width || 1280} / ${frameSize.value?.height || props.height || 720}`);
const hidden = ref(document.hidden);
const paused = computed(() => hidden.value && props.pauseBackground);
let token = 0, controller, retryTimer, watchdog, reader, lastFrameAt = 0;
let pendingFrame = null, decoding = false, retiredUrl = '', attempt = 0, stopped = false;

function releaseImages() {
  if (frameUrl.value) URL.revokeObjectURL(frameUrl.value);
  if (retiredUrl) URL.revokeObjectURL(retiredUrl);
  frameUrl.value = ''; retiredUrl = ''; pendingFrame = null; decoding = false;
  loaded.value = false;
}

function publish(frame) {
  // At most one image is decoding and one newest frame is waiting.
  if (decoding) { pendingFrame = frame; return; }
  decoding = true;
  retiredUrl = frameUrl.value;
  frameUrl.value = URL.createObjectURL(new Blob([frame], { type: 'image/jpeg' }));
}

function imageLoaded(event) {
  const image = event.currentTarget;
  if (image.naturalWidth && image.naturalHeight && (frameSize.value?.width !== image.naturalWidth || frameSize.value?.height !== image.naturalHeight)) {
    frameSize.value = { width: image.naturalWidth, height: image.naturalHeight };
  }
  loaded.value = true; reconnecting.value = false; decoding = false;
  if (retiredUrl) URL.revokeObjectURL(retiredUrl);
  retiredUrl = '';
  if (pendingFrame) { const next = pendingFrame; pendingFrame = null; publish(next); }
}

function imageFailed() {
  releaseImages();
  controller?.abort();
}

function stop() {
  token++;
  window.clearTimeout(retryTimer); window.clearInterval(watchdog);
  controller?.abort(); controller = undefined;
  reader?.cancel().catch(() => {}); reader = undefined;
  releaseImages();
}

async function connect(expectedToken) {
  if (stopped || expectedToken !== token) return;
  const current = new AbortController(); controller = current;
  let currentReader, received = 0;
  lastFrameAt = Date.now();
  watchdog = window.setInterval(() => {
    if (expectedToken === token && Date.now() - lastFrameAt > 7000) {
      reconnecting.value = true; current.abort();
    }
  }, 1000);
  try {
    const url = new URL(props.src, window.location.href);
    // A fresh URL also prevents browsers/proxies from reusing an ended stream
    // when two camera modes have the same /stream/astra.mjpg path.
    url.searchParams.set('_preview', Date.now().toString(36) + '-' + expectedToken + '-' + attempt);
    const response = await fetch(url, { signal: current.signal, cache: 'no-store', credentials: 'same-origin' });
    if (!response.ok || !response.body || !/multipart\/x-mixed-replace/i.test(response.headers.get('Content-Type') || '')) throw new Error('相机视频连接失败');
    currentReader = response.body.getReader(); reader = currentReader;
    const parser = new MjpegFrames(frame => {
      if (expectedToken !== token || stopped) return;
      lastFrameAt = Date.now(); received++; frameCount.value++;
      if (received >= 4) attempt = 0;
      publish(frame);
    });
    while (expectedToken === token && !stopped) {
      const { done, value } = await currentReader.read();
      if (done) throw new Error('相机视频连接已结束');
      parser.push(value);
    }
  } catch {
    if (expectedToken !== token || stopped) return;
    reconnecting.value = true;
  } finally {
    if (currentReader) await currentReader.cancel().catch(() => {});
    current.abort();
    if (expectedToken === token) {
      window.clearInterval(watchdog);
      reader = undefined; controller = undefined;
      if (!stopped) retryTimer = window.setTimeout(() => connect(expectedToken), Math.min(10000, 500 * 2 ** Math.min(attempt++, 5)));
    }
  }
}

function restart() {
  stop(); frameCount.value = 0; attempt = 0; reconnecting.value = false;
  if (!stopped && props.enabled && props.src && !props.switching && !paused.value) void connect(token);
}

watch(() => [props.src, props.generation, props.mode, props.switching, props.enabled, paused.value], restart, { immediate: true });
function visibilityChanged() { hidden.value = document.hidden; }
onMounted(() => document.addEventListener('visibilitychange', visibilityChanged));
onBeforeUnmount(() => { stopped = true; document.removeEventListener('visibilitychange', visibilityChanged); stop(); });

const message = computed(() => !props.enabled ? '自动打开相机已关闭' : paused.value ? '页面恢复显示后自动连接' : props.switching ? '正在切换相机模式…' : reconnecting.value ? '正在重连相机画面…' : props.online ? '正在连接相机画面…' : '相机当前离线');
</script>

<template>
  <div class="camera-body" :style="{'--camera-aspect-ratio':aspectRatio}" :data-frame-count="frameCount" :aria-busy="!loaded || reconnecting">
    <img v-if="frameUrl" :src="frameUrl" :alt="fit==='cover'?'相机实时画面':'相机实时完整画面'" :style="{objectFit:fit}" @load="imageLoaded" @error="imageFailed">
    <div v-if="!loaded || reconnecting" class="camera-placeholder" role="status"><UiIcon name="camera" :size="28"/><span>{{message}}</span></div>
    <span class="image-label" :class="{'is-offline':!online&&!switching}"><i class="dot" aria-hidden="true"></i>{{switching?'相机切换中':reconnecting?'画面重连中':online?'实时相机画面':'相机离线'}}</span>
  </div>
</template>
