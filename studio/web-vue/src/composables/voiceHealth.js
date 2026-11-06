const unknown = () => ({
  id: 'voice', label: '语音模块', state: 'unknown', status: '待确认',
  reason: '暂时无法取得有效的语音监测数据。', advice: '请稍后刷新，或检查语音监测服务。',
  components: [],
});

export function voiceHealth(payload, now = Date.now()) {
  const age = now / 1000 - payload?.sampled_at;
  const row = payload?.voice;
  if (payload?.version !== 1 || !Number.isFinite(payload?.sampled_at)
      || age < -5 || age > 10 || row?.id !== 'voice'
      || !['ok', 'error', 'unknown'].includes(row?.state)) return unknown();
  const text = (value, fallback) => typeof value === 'string' && value.trim() ? value.slice(0, 300) : fallback;
  return {
    ...unknown(), state: row.state,
    status: row.state === 'ok' ? '正常' : row.state === 'error' ? '异常' : (row.status === '准备中' ? '准备中' : '待确认'),
    reason: text(row.reason, unknown().reason), advice: text(row.advice, ''),
  };
}

export function withVoiceHealth(snapshot, payload, now = Date.now()) {
  if (!snapshot) return snapshot;
  const base = snapshot.health || {};
  const row = voiceHealth(payload, now);
  const hardware = [...(base.hardware || []).filter(item => item.id !== 'voice'), row];
  const metrics = base.metrics || [];
  const issues = (base.issues || []).filter(item => item.id !== 'voice');
  if (row.state === 'error') issues.push(row);
  const state = issues.length || base.state === 'error' ? 'error'
    : base.state === 'idle' && row.state === 'ok' ? 'idle'
    : base.state === 'ok' && row.state === 'ok' ? 'ok' : 'unknown';
  return {...snapshot, health: {...base, hardware, metrics, issues, state,
    status: state === 'ok' ? '监测正常' : state === 'idle' ? '设备已连接，待机中' : state === 'error' ? '监测异常' : '部分状态待确认'}};
}
