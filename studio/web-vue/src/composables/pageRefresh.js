const pageRefreshers = new Map();

export function registerPageRefresher(page, refresh) {
  pageRefreshers.set(page, refresh);
  return () => {
    if (pageRefreshers.get(page) === refresh) pageRefreshers.delete(page);
  };
}

export async function refreshPageData(page) {
  const refresh = pageRefreshers.get(page);
  if (refresh) await refresh();
}
