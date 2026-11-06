import { createApp } from 'vue';
import { ElMessage } from 'element-plus';
import 'element-plus/es/components/message/style/css';
import App from './App.vue';
import './style.css';
import './implementation.css';
import './approved-polish.css';
import './rvbench-report.css';
import './dark-theme.css';
import UiCard from './components/UiCard.vue';
import UiIcon from './components/UiIcon.vue';
import ResourceBars from './components/ResourceBars.vue';
import TrendPlot from './components/TrendPlot.vue';
import TaskMap from './components/TaskMap.vue';
import UiModal from './components/UiModal.vue';
import UiPagination from './components/UiPagination.vue';

const app = createApp(App);
app.component('UiCard',UiCard).component('UiIcon',UiIcon).component('UiModal',UiModal).component('UiPagination',UiPagination).component('ResourceBars',ResourceBars).component('TrendPlot',TrendPlot).component('TaskMap',TaskMap);
app.config.errorHandler = (error) => {
  console.error(error);
  ElMessage.error('页面发生错误，请刷新后重试');
};
app.mount('#app');
