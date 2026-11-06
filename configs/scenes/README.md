# 地图与巡检配置

`inspection-k1-reviewed` 是当前 K1 使用的走廊场景，包含地图、A/B/C/ORIGIN 坐标、朝向、仪表检测任务及定位参考。`deploy/k1/snapshot/inspection-profile.json` 记录默认选择。

其余场景与 `configs/studio-maps` 保留网页已有可选地图的数据，不包含运行日志。更换场地时需重新建图、配置点位和参考图，不能直接把这些坐标用到另一处环境。算法源码与场地配置分开维护。
