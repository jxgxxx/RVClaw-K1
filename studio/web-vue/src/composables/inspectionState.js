const started = task => Number(task?.started_at || 0);

// Current tasks exist independently from camera/photo batches. Only explicit
// map identity or the same task ID may join those two sources.
export function inspectionTask(assets, progress, session = '', pending = null) {
  const saved = assets?.task || null;
  const mapId = assets?.map?.id;
  const matches = progress?.task_id && (
    (mapId && progress.map_id === mapId) || progress.task_id === saved?.task_id
  );
  let selected = saved;
  if (matches && (!session || progress.task_id === saved?.task_id) && started(progress) >= started(saved)) {
    selected = progress;
  }
  // Before either request returns, both map IDs can be undefined. Equality
  // alone cannot prove that a pending task exists or belongs to this map.
  if (!session && mapId && pending?.task_id && pending.map_id === mapId && pending.task_id !== selected?.task_id && started(pending) >= started(selected)) {
    selected = pending;
  }
  return selected;
}

export function taskPhotos(assets, task) {
  if (!task?.task_id && !assets?.task?.task_id) return assets?.photos || [];
  return task?.task_id && task.task_id === assets?.task?.task_id ? assets?.photos || [] : [];
}
