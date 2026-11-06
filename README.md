# RVClaw

K1 巡检小车的完整工程源码：V550 底盘控制、ROS2 导航与建图、语音任务、仪表本地/云端识别，以及网页控制台。

本次整理以 **2026-11-06 K1 实际部署文件**为依据，按本仓库已有目录交付。运行文件与构建入口已对齐；不在本次整理中修改小车运动、定位、识别或语音算法。交付范围仅 K1，包含整个项目的业务源码、底盘固件及构建所需 SDK 源码。

| 目录 | 内容 |
| --- | --- |
| `robot/firmware` | STM32 C30D/V8.1 固件源工程及 Keil 项目 |
| `robot/ros2_ws/src` | 底盘串口协议、控制与遥测、雷达、ROS2 节点及巡检算法 |
| `robot/profiles` | 导航、建图和任务配置的源码覆盖层；工作区由部署脚本生成 |
| `robot/vendor` | K1 构建所需 SDK 及第三方源代码，保留原许可 |
| `services` | 语音、仪表识别、本地 API、任务控制、进度及网页网关 |
| `studio/web-vue` | Vue 3 网页源代码、素材和 pnpm 锁文件 |
| `deploy/k1` | K1 组装、原生编译、设备权限及 systemd 配置 |
| `configs` | 地图、巡检点、阈值、仪表参考图及设备配置 |
| `contracts` | 控制、状态、识别及网页接口约定 |
| `models` | 模型和语音资源的路径、大小、SHA256 清单 |
| `scripts` | 产品构建、资源导入及交付检查入口 |
| `runtime` | 运行时生成目录；不提交历史任务、缓存或日志 |
| `samples`、`tests` | 保留原有目录占位；不加入现场测试代码 |

从 [K1 复现指南](docs/K1-复现指南.md) 开始。资源交付状态见 [资源说明](models/README.md)，接口和调用关系见 [工程导航](docs/工程导航.md)。

不需要连接小车即可检查源码和构建网页：

```bash
python3 scripts/check_delivery.py
cd studio/web-vue
pnpm install --frozen-lockfile
pnpm run build
```

K1 组装使用 `deploy/k1/assemble_snapshot.py`，安装服务使用 `deploy/k1/install_snapshot.py`；所有部署和构建入口均在 K1 目录，不依赖其他板型的部署文件。

模型权重和原音色 WAV 通过配套资源包导入，不放入 Git 历史；API 密钥需使用者自行配置。现场采集日志、录音、测试脚本、任务数据库、编译目录、`node_modules` 和设备密钥不在源码提交中。

当前版本边界及检查结果见 [交付说明](docs/交付说明.md)。源码整理与一次完整巡检成功，不代表所有场地和负载条件已完成验收。
