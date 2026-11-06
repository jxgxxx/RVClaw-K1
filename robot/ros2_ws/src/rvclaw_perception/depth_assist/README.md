# Astra depth assist

当前阶段：工程接入准备完成。用户已向商家确认，当前不支持 RISC-V；K1 原生深度接入被驱动支持阻塞。未启用服务，未参与导航决策。

`openni_depth_reader.cpp` 只打开 `2bc5:0403` 深度设备，不打开 RGB，不发布底盘控制、初始位姿或地图。仅选择不超过 640×480、30 fps 的毫米深度模式，优先选择像素吞吐量最小的模式。每秒最多输出 3 次左、中、右区域距离摘要，保存在用户 runtime 目录；无图像、点云广播或持续磁盘录像。

距离是深度光学坐标系 Z 值，限定 0.6–4.0 m；无效像素不当作空旷，覆盖不足输出 null。没有验证深度与 RGB 配准、相机与底盘外参，因此这些数值不能直接当作仪表距离、底盘净空或地图位置。`observation.py` 拒绝过期、跨启动、错误时钟、错误格式和无效距离；数据缺失不会影响现有导航。

工程位置：源代码在本目录；接口在 `contracts/depth_observation.schema.json`；配置在 `configs/perception/astra_depth.env`；K1 启动、服务和权限模板在 `deploy/k1/`；构建产物在 `build/depth-assist/`；采样、验证记录在 `runtime/k1/diagnostics/depth-assist-review/`。

K1 本机编译：

```bash
cmake -S /data/RVClaw/robot/ros2_ws/src/rvclaw_perception/depth_assist -B /data/RVClaw/build/depth-assist -DCMAKE_BUILD_TYPE=Release
cmake --build /data/RVClaw/build/depth-assist -j2
```

K1 当前系统 OpenNI2 可编译接口，但不能枚举 Astra。商家 ROS2 包只附 x64/arm/arm64 的 `liborbbec.so`；不能直接用于 RISC-V。需取得支持 Astra Pro（不是 Pro Plus）的 RISC-V OpenNI2 库和设备插件，或可合法编译的驱动源码。供应商库应隔离到 `runtime/k1/vendor/openni2/`，按其实际 ABI 构建，不能覆盖系统共享库。`udev` 模板尚未安装，USB 深度设备目前属于 root，只读权限不足以采集。

服务模板限制为低优先级、CPU 4–6、最多 0.3 核、128 MiB。它尚未安装或自启，不会反复尝试失败的驱动。实际开销和 USB 带宽必须取得真实帧后测量，不能把限额当作性能实测。

启用前依次验证：原生驱动和权限；真实毫米深度与已知距离一致；RGB 与雷达、底盘心跳同时稳定；静态外参及仪表 ROI 配准；至少一次现场影子采样对照雷达候选。只有经这些验证、证据能区分候选后，才接入定位核验或到点判断。局部距离本身无法解决所有相似地图位置，不能自动写入 AMCL 或降低既有门槛。
