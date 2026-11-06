# 底盘 STM32 固件

`c30d-v8.1/USER/WHEELTEC.uvprojx` 是 C30D 控制器的 Keil 源工程，保留厂家 STM32/CMSIS/FreeRTOS 源码及原文件编码。工程内已含主机握手、命令超时和 PS2 人工优先的修改。

运动入口与状态机在 `BALANCE/system.c` / `system.h`，串口解析在 `HARDWARE/usartx.c`，电机/编码器/手柄相关驱动在 `HARDWARE`。上位机协议及里程计实现位于 `robot/ros2_ws/src/rvclaw_stm32_base`。

使用匹配的 Keil/ARM 工具链打开项目，按实际 C30D 板型、车型与厂家设备支持包配置后构建。当前设备已有合适固件时无需重刷。此次仅交付源工程，不包含下载器配置、历史编译日志、`.hex`/`.axf` 或个人 IDE 状态，也不执行刷写。
