# 巡检进度服务

`inspection_progress_watcher.py` 被动观察任务数据库、持久到点事件和 route trace，输出网页的当前进度、历史摘要及轨迹。它不启动巡检、不控制底盘、不改变导航进程。

正式入口由 `deploy/k1/snapshot/systemd-user/rvclaw-inspection-progress.service` 配置。任务数据库是 `runtime/voice_tasks.sqlite3`，到点事件目录是 `runtime/waypoint-arrivals`，输出及账本位于 `runtime/inspection-progress`。进度 API 和动画由网页网关及 Vue 页面消费。

```bash
systemctl --user status rvclaw-inspection-progress.service
journalctl --user -u rvclaw-inspection-progress -f
```

安装使用 [K1 复现指南](../../docs/K1-复现指南.md) 的统一入口，无需临时安装目录或独立演示网页。接口行为见 `contracts/studio-patrol-live.md`。
