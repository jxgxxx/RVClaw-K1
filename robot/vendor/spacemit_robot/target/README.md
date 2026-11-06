# 目标配置

本目录存放 **构建目标配置文件**（`*.json`），用于 `lunch` 选择、`m` / `build.sh` 构建时指定要编译的包及其选项。

## 目录简介

- **作用**：选择启用的 CMake 包与 ROS2 包、外设驱动、并行数等，一次 `lunch` 后，`m` 与 `build.sh` 均按该配置构建。
- **格式**：JSON，需 `jq` 解析（构建系统依赖）。

## 现有目标

| 文件 | 说明 |
| ---- | ---- |
| `k3-com260-minimal.json` | K3 COM260 板卡，最小配置 |
| `k3-com260-lekiwi.json` | K3 COM260，lekiwi 产品配置 |
| `k3-com260-diablo.json` | K3 COM260，diablo 产品配置 |

## 如何使用

```bash
# 交互选择（source envsetup 后）
source build/envsetup.sh
lunch

# 直接指定
lunch k3-com260-minimal

# 或通过环境变量（脚本/CI）
BUILD_TARGET=k3-com260-minimal ./build/build.sh all
```

## JSON 字段说明

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `version` | string | 配置版本，如 `"1.0"` |
| `board` | string | 板卡标识，如 `"k3-com260"` |
| `product` | string | 产品名，如 `"minimal"`、`"lekiwi"` |
| `description` | string | 配置描述（lunch 菜单显示） |
| `enabled_packages` | string[] | 启用的包列表，路径如 `components/peripherals/motor`、`middleware/ros2/mlink/cpp` |
| `enabled_package_options` | object | 各包的选项，如外设启用的驱动 |
| `options` | object | 全局选项 |

### enabled_package_options

用于给特定包传递构建选项。外设包常用 `enabled_drivers` 指定要启用的驱动：

```json
"enabled_package_options": {
  "components/peripherals/motor": {
    "enabled_drivers": ["drv_can_dm", "drv_uart_feetech"]
  },
  "components/peripherals/lidar": {
    "enabled_drivers": ["drv_uart_ydlidar", "drv_uart_rplidar"]
  }
}
```

构建时会转换为 CMake 参数，例如：`-DSROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS=drv_can_dm;drv_uart_feetech`。

### options

| 字段 | 类型 | 说明 |
| ---- | ---- | ---- |
| `parallel_jobs` | number | 并行编译任务数 |
| `auto_resolve_dependencies` | bool | 是否自动解析依赖 |

## 新增目标

1. 复制现有配置（如 `k3-com260-minimal.json`）并重命名，如 `k3-com260-myproduct.json`。
2. 修改 `board`、`product`、`description`。
3. 按需调整 `enabled_packages`（添加/删除包）。
4. 按需在 `enabled_package_options` 中为外设设置 `enabled_drivers`。
5. 执行 `lunch` 验证新目标是否出现在菜单中。

## 相关说明

- 构建系统会读取 `enabled_packages`，并自动解析包依赖，得到完整构建列表。
- 外设的 `enabled_drivers` 需与各组件 `CMakeLists.txt` 中支持的驱动名一致，具体见各外设目录。

## License

本目录文件遵循项目 LICENSE（见上级目录或本目录 LICENSE 文件）。
