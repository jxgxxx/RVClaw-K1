# Build 脚本

## 项目简介

本目录提供 **spacemit robot sdk 仓库的统一构建与环境脚本**，用于从源码编译 CMake 包与 ROS2 包、选择目标配置、安装依赖及生成部署目录。

- **做什么**：提供 `envsetup.sh`（环境与快捷命令）、`build.sh`（实际构建逻辑）、以及依赖与目标配置解析；支持全量构建、按类型（CMake/ROS2）构建、单包构建与 rootfs 部署。
- **解决什么问题**：统一入口与布局（output/staging、output/rootfs、target 配置），避免各组件各自写构建脚本；通过 `target/*.json` 选择启用包与选项，通过 `lunch` / `m` / `mm` 快速完成选型与编译。

## 功能特性

| 类别       | 支持                                                                 |
| ---------- | -------------------------------------------------------------------- |
| 构建类型   | CMake 包、ROS2（ament_cmake/ament_python）包；全量 / 仅 CMake / 仅 ROS2 |
| 命令       | `lunch` 选目标，`m` 全量或分类型构建，`mm` 单包构建，`build.sh` 直接调用 |
| 目标配置   | `target/*.json` 指定启用包、包选项、并行数等                          |
| 输出布局   | `output/staging` 安装前缀，`output/rootfs` 部署目录（deploy-rootfs） |
| 依赖       | 系统依赖检查与可选自动安装（apt），包级依赖解析                       |
| Docker 构建 | 通过 `m_enable_docker_build` 或显式设置 `SROBOTIS_USE_DOCKER_BUILD=1` 后进入 Bianbu Docker 编译环境 |
| 交叉编译   | 通过 `m_enable_cross_build` 或直接调用 `build/cross_build.sh`，在 x86_64 主机上生成 riscv64 目标产物 |
| Python 环境 | `m_env_build`、`python_env_build.sh` 为应用构建虚拟环境          |
| Python wheel | 可选：在 **non-ROS2** 包安装成功后，用 PEP 517 生成 `.whl` 到 `output/wheels/`（见下文） |

| 类别       | 不支持 / 说明                                                       |
| ---------- | -------------------------------------------------------------------- |
| 其他构建系统 | 仅支持 CMake 与 ROS2 colcon，不负责其他语言/框架的构建               |
| 构建模式   | 默认走本地环境；Docker 构建和交叉编译均需在当前 shell 显式启用       |

## 用户场景与预期效果

本节作为构建系统的行为验收标准。下表中的 `m`、`mm` 是 `source build/envsetup.sh`
后提供的快捷命令；脚本或 CI 也可以直接调用等价的 `./build/build.sh ...` 入口。

`mm` 的依赖语义与是否选择 target 无关：普通 `mm` 只构建当前包，`mm --with-deps`
递归构建当前包声明的 SDK 依赖并构建当前包，`mm --deps` 只递归构建依赖。`lunch`
只提供 target 上下文，例如包选项、平台/Docker 选择和 target 相关环境；只有 `m` /
`build.sh all|cmake|ros2` 这类全量或分类型构建才以 target `enabled_packages` 作为构建根集合。

### 构建系统命令场景

| 编号 | 场景 | 示例命令 | 预期效果 |
| ---- | ---- | -------- | -------- |
| B1 | 不选择 target，单独编译一个 non-ROS2 包 | `./build/build.sh package components/foo` | 不要求先 `lunch`；只编译当前包；检查基础构建依赖和当前包系统依赖；产物安装到 `output/staging`；不主动编译 SDK 依赖包。 |
| B2 | 不选择 target，编译一个 non-ROS2 包及其依赖 | `./build/build.sh package application/native/app --with-deps` | 递归解析 SDK `<depend>` 闭包；按依赖顺序构建依赖包后再构建当前包；检查当前包和所有依赖包的 `<system_depend>`；依赖环、缺失依赖、不可构建依赖应明确失败。 |
| B3 | 不选择 target，只编译一个 non-ROS2 包的依赖 | `./build/build.sh package application/native/app --deps` | 递归构建当前包的 SDK 依赖闭包，但不构建当前包自身；系统依赖检查范围与 `--with-deps` 一致。 |
| B4 | 选择 target 后，单独编译一个 non-ROS2 包 | `lunch k3-xxx && mm` | 依赖语义与未选择 target 的 `mm` 相同：只编译当前包；target 只影响包选项、平台/Docker 等上下文，不因为 target 中启用了其他包就触发全量构建。 |
| B5 | 选择 target 后，编译 non-ROS2 包及其依赖 | `lunch k3-xxx && mm --with-deps` | 依赖语义与未选择 target 的 `mm --with-deps` 相同：以当前包为根递归构建 SDK 依赖，再构建当前包；target/Docker 环境只提供构建上下文。 |
| B6 | 单独编译 ROS2 包 | `./build/build.sh package application/ros2/demo` | 根据 `package.xml` 的 `ament_cmake` / `ament_python` 识别 ROS2 包；加载 `ROS_SETUP`；执行 `colcon build --packages-select <pkg>`；不构建整个 ROS2 workspace。 |
| B7 | 编译 ROS2 包及其依赖 | `./build/build.sh package application/ros2/demo --with-deps` | 先构建 ROS2 包声明中可映射到 SDK non-ROS2 组件的 underlay 依赖；这些 SDK 依赖也要递归处理；再执行 `colcon build --packages-up-to <pkg>`。 |
| B8 | 只编译 ROS2 包的依赖 | `./build/build.sh package application/ros2/demo --deps` | 准备可映射的 SDK underlay 依赖；通过 `colcon list --packages-up-to` 找到 ROS2 依赖包；只构建依赖包，不构建当前 ROS2 包。 |
| B9 | 选择 target 后全量构建 | `lunch k3-xxx && m` 或 `BUILD_TARGET=k3-xxx ./build/build.sh all` | 读取 target `enabled_packages` 并递归展开 SDK 依赖；先构建需要的 non-ROS2 包，再构建需要的 ROS2 middleware/application；完成后生成 `output/rootfs`。 |
| B10 | 不选择 target 全量构建 | `./build/build.sh all` | 作为开发 fallback，发现并构建仓库内可构建的 non-ROS2 包；存在 ROS2 workspace 时尝试构建 ROS2；正式板级验证应优先使用 target。 |
| B11 | 只构建 non-ROS2 / CMake 包 | `m -C` 或 `./build/build.sh cmake` | 只执行 non-ROS2 构建；选择 target 时按 target 包集合构建，不选择 target 时按仓库发现结果构建；不调用 colcon。 |
| B12 | 只构建 ROS2 包 | `m -R` 或 `./build/build.sh ros2` | 加载 ROS 环境并构建 ROS2 middleware/application；选择 target 时只构建 target 关联 ROS2 包；不执行无关 non-ROS2 全量构建。 |
| B13 | 全量清理 | `m clean` 或 `./build/build.sh clean all` | 清理 `output/build`、`output/staging`、`output/rootfs` 以及 ROS2 build/log 目录；不删除源码和用户工作区文件。 |
| B14 | 清理单个 non-ROS2 包 | `./build/build.sh package components/foo clean` | 只清理该包的 CMake build 目录，例如 `output/build/cmake/pkgs/components_foo`；不清理整个 staging，不影响其他包。 |
| B15 | 清理单个 ROS2 包 | `./build/build.sh package application/ros2/demo clean` | 应清理该包 ROS2 build 目录和 install 输出，包括 `output/staging/share/<pkg>`、`output/staging/lib/<pkg>`、ament index、colcon registry 等。 |
| B16 | 使用 Bianbu Docker 构建 | `SROBOTIS_USE_DOCKER_BUILD=1 BUILD_TARGET=k3-xxx ./build/build.sh all` | k1 target 使用 `bianbu:2.3`，k3 target 使用 `bianbu:4.0`；先以 root 检查/安装系统依赖，再用默认构建用户执行真实构建；外部命令语义与真机构建保持一致。 |
| B17 | 使用交叉编译全量构建 | `lunch k3-xxx && m_enable_cross_build && m` | `m` 路由到 `build/cross_build.sh all`；在 Ubuntu host 容器中执行构建，在 Bianbu 容器中准备 riscv64 sysroot；产物输出到 `output/cross/<target>/`。 |
| B18 | 使用交叉编译构建单包 | `lunch k3-xxx && m_enable_cross_build && cd components/foo && mm --with-deps` | `mm` 路由到 `build/cross_build.sh package <pkg>`；依赖语义与本地 `mm` 一致，但系统依赖会按 host / target sysroot 拆分。 |
| B19 | 查看交叉编译依赖拆分 | `BUILD_TARGET=k3-xxx ./build/cross_build.sh deps all` | 只打印依赖，不编译；输出分为 `Host dependencies` 和 `Target sysroot dependencies`，用于确认哪些包安装到 Ubuntu host 容器、哪些包安装到 Bianbu sysroot。 |

### CI 调用场景

CI 不定义新的构建语义，只负责在合适的时机调用上面的构建入口，并收集结果、日志和通知。

| 编号 | 场景 | CI 调用方式 | 预期效果 |
| ---- | ---- | ----------- | -------- |
| C1 | PR 单组件验证 | `build.sh package <module>`，必要时附加 `--with-deps` 或 target 信息 | 只验证 PR 相关模块；lint、构建和模块 `test.yaml` 分阶段报告；如果修改了 `package.xml`、`build`、`target` 或 `scripts/test`，额外运行构建系统测试。 |
| C2 | Nightly / release target 全编译 | `BUILD_TARGET=<target> ./build/build.sh all` | 先运行构建系统自测，再执行 target 全量构建和 target 范围测试；归档构建日志、测试报告和 rootfs；失败时能区分依赖、编译、ROS2、rootfs 或测试阶段。 |

## 环境准备

### 基础要求

- **Shell**：Bash 或 Zsh（脚本兼容两者）。
- **系统**：推荐 Bianbu 2.2 以上、Ubuntu 24.04 以上。
- **必需**：`jq` 用于解析 `target/*.json`。

### 必须依赖

构建前请确保已安装编译工具链与 CMake：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config jq
```

### ROS2 依赖

若目标包含 ROS2 包（`middleware/ros2/`、`application/ros2/`），需先安装 ROS2 并加载环境，具体安装方式待补充。

### Docker 编译

默认使用本地环境编译，不会根据 x86/amd64 主机环境自动进入 Docker。如果需要使用 Bianbu Docker
编译，推荐在 `source build/envsetup.sh` 后执行 `m_enable_docker_build`；也可为单次命令显式设置
`SROBOTIS_USE_DOCKER_BUILD=1`。在已经通过 `lunch` 或 `BUILD_TARGET` 选择了
k1/k3 target 后，`m`、`mm` 和 `./build/build.sh all|cmake|ros2|package <dir>` 会进入 Docker
编译环境，外部命令用法不变。

重新 `source build/envsetup.sh` 会把 Docker 编译开关重置为关闭，需要再次执行
`m_enable_docker_build` 才会重新进入 Docker 编译。执行 `m_enable_docker_build disable` 可在当前 shell
内关闭 Docker 编译。

Docker 编译封装的目标是把普通 `build/build.sh` 放进匹配目标板的 Bianbu 容器里执行，外部入口仍然是
`m`、`mm` 或 `./build/build.sh`。这不是交叉编译：编译器、系统依赖和 ROS2 环境都来自 Bianbu 容器，
产物仍写回当前源码目录挂载的 `output/`。

Docker 封装只包裹需要 target 上下文的构建命令：

- `all`、`cmake` / `C`、`ros2` / `R`
- `package <dir>` / `pkg <dir>`，但不包括 `package <dir> clean`
- `clean`、`deploy-rootfs` 等命令不进入 Docker 包裹逻辑

- k1 target 使用 `bianbu:2.3`，若本地没有则执行 `docker pull harbor.spacemit.com/bianbu/bianbu:2.3`
- k3 target 使用 `bianbu:4.0`，若本地没有则执行 `docker pull harbor.spacemit.com/bianbu/bianbu:4.0`
- 如果未安装 Docker，构建入口会提示先安装和配置 Docker 环境。
- 默认以 `linux/riscv64` 平台启动 Bianbu 镜像。
- Docker 容器会保留并复用：同一 SDK 路径和同一 bianbu 版本的容器已运行时直接 `docker exec`，
  已存在但停止时先 `docker start`，不存在时才创建。
- 容器名默认由 SDK 路径 hash 和 Bianbu tag 组成，避免多个源码目录共用同一个容器；也可通过
  `SROBOTIS_DOCKER_CONTAINER_NAME` 固定。
- 新建容器默认只挂载当前 SDK 根目录到容器相同路径；默认容器名包含当前 SDK 路径标识，因此同一环境下
  不同 SDK 源码目录会使用不同 Docker 容器。
- 如果容器已存在但没有挂载当前 `REPO_ROOT`，会删除并重建，避免复用到错误工作区。
- 当 target 配置启用 `auto_resolve_dependencies` 时，会先以 root 在 Docker 内安装系统依赖；
  实际编译步骤默认使用宿主 uid/gid，避免 output 产物变成 root-owned。
- 依赖自动安装分两步：先以 root 执行一次 `build.sh` 依赖检查/安装，随后真实构建设置
  `SROBOTIS_SKIP_DEPS_CHECK=1`，避免重复检查。
- 非 root Docker 编译会在容器内补齐宿主 uid/gid 对应的用户信息，并使用 `output/.docker_home`
  作为持久化 HOME。
- 依赖安装后会默认把当前仓库的 `output/` 属主修正回宿主 uid/gid，避免历史 root-owned 文件阻塞后续编译。
- Docker 构建中如果未显式指定 `-jN`，默认使用宿主 `nproc` 作为并行度，以加快编译。
- k1 / Bianbu 2.3 容器会自动补齐 apt suite：将 `bianbu-v2.2-updates` 调整为
  `bianbu-v2.3-updates`，并加入 `noble-ros`；同时设置对应 apt pin。
- `SROBOTIS_DOCKER_DEVICES` 可把宿主 `/dev/*` 设备透传进容器，适合需要访问板卡设备或特殊节点的构建
  / 测试流程；非 `/dev/` 路径会被拒绝。
- Docker 封装逻辑在 `build/docker_build.sh`，`build/build.sh` 只保留入口判断；板端直接编译不会进入
  Docker 流程。

可通过环境变量调整：

```bash
# 推荐：source 环境后启用 Bianbu Docker 编译
source build/envsetup.sh
m_enable_docker_build
./build/build.sh all

# 当前 shell 关闭 Bianbu Docker 编译
m_enable_docker_build disable

# 单次命令显式启用 Bianbu Docker 编译
SROBOTIS_USE_DOCKER_BUILD=1 ./build/build.sh all

# 强制编译步骤也在容器内以 root 运行
SROBOTIS_DOCKER_RUN_AS_ROOT=1 ./build/build.sh all

# 覆盖 Docker 平台
SROBOTIS_DOCKER_PLATFORM=linux/riscv64 ./build/build.sh all

# 覆盖默认容器名
SROBOTIS_DOCKER_CONTAINER_NAME=srobotis-k3-build ./build/build.sh all

# 覆盖 Docker 挂载范围
SROBOTIS_DOCKER_MOUNT_SRC=/home/user SROBOTIS_DOCKER_MOUNT_DST=/home/user ./build/build.sh all

# 透传宿主设备到 Docker，多个设备可用逗号或空格分隔
SROBOTIS_DOCKER_DEVICES=/dev/tcm ./build/build.sh all
SROBOTIS_DOCKER_DEVICES="/dev/tcm,/dev/null:/dev/null" ./build/build.sh all

# 禁用自动修正 output 属主
SROBOTIS_DOCKER_FIX_OUTPUT_OWNER=0 ./build/build.sh all

# Docker 构建不自动使用最大线程，改回 target/环境中的 PARALLEL_JOBS
SROBOTIS_DOCKER_MAX_JOBS=0 ./build/build.sh all
```

完整入口示例：

```bash
source build/envsetup.sh
lunch k3-com260-minimal
m_enable_docker_build

m              # 在 Bianbu Docker 中构建全量 target
m -C           # 在 Bianbu Docker 中只构建 CMake / non-ROS2 包
m -R           # 在 Bianbu Docker 中只构建 ROS2 包

cd components/peripherals/lidar
mm --with-deps # 在 Bianbu Docker 中构建当前包及其依赖
```

### 交叉编译

交叉编译用于在 x86_64 主机上构建 riscv64 目标产物，入口是 `build/cross_build.sh`。它和上面的
“Bianbu Docker 编译”不是同一个模式：

- Docker 编译：直接在 Bianbu riscv64 容器里运行普通 `build/build.sh`，更接近板端原生编译。
- 交叉编译：使用 Ubuntu host 容器执行 CMake/colcon/Cargo 构建，使用 Bianbu 容器安装目标依赖并导出
  riscv64 sysroot。

推荐用法是在 `source build/envsetup.sh` 后显式启用交叉编译；重新 source 会重置为关闭：

```bash
source build/envsetup.sh
lunch k3-com260-reachy-mini
m_enable_cross_build

m              # 交叉编译全量 target
m -C           # 只交叉编译 non-ROS2 / CMake 包
m -R           # 只交叉编译 ROS2 包
m clean        # 清理 output/cross/<target>

cd components/peripherals/motor
mm --with-deps # 交叉编译当前包及其 SDK 依赖

m_enable_cross_build disable
```

脚本或 CI 也可以直接调用 `cross_build.sh`，不依赖 shell 快捷函数：

```bash
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh all
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh cmake
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh ros2
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh package components/peripherals/motor --with-deps
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh clean all
```

交叉编译要求已经选择 k1/k3 target。镜像按 target family 自动选择：

| Target family | Ubuntu host 容器 | Bianbu sysroot 容器 |
| ------------- | ---------------- | ------------------- |
| k1            | `ubuntu:24.04`   | `bianbu:2.3`        |
| k3            | `ubuntu:26.04`   | `bianbu:4.0`        |

如果本地没有 `bianbu:<tag>`，交叉编译会优先拉取
`harbor.spacemit.com/bianbu/bianbu:<tag>`，并尽量 tag 回 `bianbu:<tag>` 供后续复用。可通过
`SROBOTIS_CROSS_BIANBU_IMAGE` 覆盖完整 Bianbu 镜像名。

交叉编译流程：

1. 读取 `target/*.json`，解析启用包和包选项。
2. 创建或复用两个容器：`srobotis-cross-ubuntu-*` 与 `srobotis-cross-bianbu-*`。
3. 收集系统依赖并按 `realm` 拆分：host 依赖安装到 Ubuntu 容器，target 依赖安装到 Bianbu 容器。
4. 从 Bianbu 容器导出 sysroot 到 `output/cross/<target>/sysroot`。
5. 生成 `toolchain-riscv64.cmake` 和 `meson-riscv64.ini`。
6. 在 Ubuntu host 容器中运行普通 `build/build.sh`，同时注入交叉编译 CMake、pkg-config、Python、ROS2
   和 Rust/Cargo 参数。

输出布局：

```text
output/cross/<target>/
  host/                    # host 侧工具前缀，供 CMAKE_PROGRAM_PATH 等查找
  sysroot/                 # 从 Bianbu 容器导出的 riscv64 目标 sysroot
  staging/                 # 交叉编译安装前缀
  rootfs/                  # deploy-rootfs 生成的部署目录
  toolchain-riscv64.cmake  # CMake toolchain file
  meson-riscv64.ini        # Meson cross file
  .cargo/                  # Cargo 缓存与配置目录
```

常用环境变量：

| 变量 | 默认值 / 说明 |
| ---- | ------------- |
| `SROBOTIS_USE_CROSS_BUILD` | `m_enable_cross_build` 设置为 `1`；`source build/envsetup.sh` 会重置为 `0` |
| `SROBOTIS_CROSS_OUTPUT_ROOT` | 覆盖 `output/cross/<target>` |
| `SROBOTIS_CROSS_SYSROOT` | 覆盖目标 sysroot 目录 |
| `SROBOTIS_CROSS_HOST_PREFIX` | 覆盖 host 工具前缀目录 |
| `SROBOTIS_CROSS_BIANBU_TAG` | 覆盖 Bianbu tag，例如 `2.3` / `4.0` |
| `SROBOTIS_CROSS_BIANBU_IMAGE` | 覆盖完整 Bianbu 镜像名 |
| `SROBOTIS_CROSS_UBUNTU_IMAGE` | 覆盖完整 Ubuntu host 镜像名 |
| `SROBOTIS_CROSS_BIANBU_PLATFORM` | Bianbu 容器平台，默认 `linux/riscv64` |
| `SROBOTIS_CROSS_UBUNTU_PLATFORM` | Ubuntu host 容器平台，默认不强制指定 |
| `SROBOTIS_CROSS_REFRESH_SYSROOT` | 设为 `1` 强制重新导出 sysroot |
| `SROBOTIS_CROSS_SKIP_SYSROOT_SYNC` | 设为 `1` 且 sysroot 已存在时复用旧 sysroot |
| `SROBOTIS_CROSS_FIX_OUTPUT_OWNER` | 设为 `0` 可禁用构建后修正 `output/cross` 属主 |
| `SROBOTIS_CROSS_RUST_VERSION` | Rust 工具链版本，默认 `1.91` |
| `SROBOTIS_CROSS_CARGO_EXECUTABLE` | 覆盖 host Cargo 路径，默认 `/usr/bin/cargo-${SROBOTIS_CROSS_RUST_VERSION}` |
| `SROBOTIS_CROSS_RUSTC_EXECUTABLE` | 覆盖 host rustc 路径，默认 `/usr/bin/rustc-${SROBOTIS_CROSS_RUST_VERSION}` |
| `SROBOTIS_CROSS_DISABLE_ROSIDL_GENERATOR_PY` | 设为 `1` 可临时禁用 ROS2 Python 消息生成；默认启用，以支持板端 `ros2 topic echo` 自定义消息 |

辅助命令：

```bash
# 只看依赖拆分，不实际编译
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh deps all
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh deps package components/peripherals/motor

# 从当前 cross rootfs/staging 扫描 ELF 动态库，反推板端运行时 apt 包
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh runtime-deps all
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh runtime-deps --strict all
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh runtime-deps --include-base all
```

## 依赖检查机制

构建入口会在实际编译前检查系统依赖：

- `m` / `./build/build.sh all|cmake|ros2`：先读取当前 `target/*.json`，根据 `enabled_packages`
  和各包 `package.xml` 里的 `<depend>` 递归展开完整包集合，再收集这些包的 `<system_depend>`。
- `mm` / `./build/build.sh package <pkg>`：只检查基础构建依赖、当前包的系统依赖；若当前包是
  `ament_cmake` / `ament_python`，还会额外检查 ROS2 基础依赖。
- 构建基础依赖始终来自 `build/package.xml`；只有本次构建确实包含 ROS2 包时，才会额外读取
  `build/package_ros2.xml`。

`package.xml` 中两类依赖含义不同：

```xml
<depend>audio</depend>
<depend arch="x86_64">mujoco</depend>
<system_depend check="pkg-config --exists yaml-cpp">libyaml-cpp-dev</system_depend>
```

- `<depend>` 是 SDK 内部包依赖，影响自动展开包集合和 non-ROS2 包构建顺序。
- `<system_depend>` 是系统包依赖，影响编译前的 apt 依赖检查和缺包安装提示。

平台相关 SDK 内部包依赖可在 `<depend>` 上使用可选 `arch` 属性；不带 `arch` 表示所有平台都依赖，
带 `arch` 时只在匹配平台加入依赖闭包。例如包只在 x86_64 构建时依赖 MuJoCo：

```xml
<depend arch="x86_64">mujoco</depend>
```

`<system_depend>` 的文本内容是要安装的系统包名；默认检查命令是 `dpkg -s <包名>`。如果声明了
`check="..."`，则使用该命令判断依赖是否已满足：

```xml
<system_depend>cmake</system_depend>
<system_depend check="pkg-config --exists eigen3">libeigen3-dev</system_depend>
<system_depend check="command -v espeak-ng">espeak-ng</system_depend>
```

平台相关系统依赖使用可选 `arch` 属性声明。构建平台默认来自 `uname -m`，也可通过
`SDK_BUILD_ARCH` 覆盖；构建脚本会将 `amd64` 归一为 `x86_64`，将 `rv64` / `riscv` 归一为
`riscv64`。不带 `arch` 表示所有平台都需要；带 `arch` 时只在匹配平台检查和安装：

```xml
<system_depend arch="x86_64" check="pkg-config --exists glfw3">libglfw3-dev</system_depend>
<system_depend arch="riscv64">spacemit-onnxruntime</system_depend>
<system_depend check="pkg-config --exists yaml-cpp">libyaml-cpp-dev</system_depend>
```

依赖检查失败时，构建系统会汇总缺失的 required 依赖，并提示安装；设置
`AUTO_INSTALL_DEPS=yes` 或 `AUTO_INSTALL_DEPS=true` 时会直接执行依赖安装。以 root 运行时使用
`apt install -y ...`，非 root 运行且存在 sudo 时使用 `sudo apt install -y ...`。

### 交叉编译依赖声明

交叉编译会复用 `package.xml` 中的 `<system_depend>`，并额外识别 `when`、`realm`、`check_kind`、
`check_arg`、`board`、`option_key` 和 `option_value` 等属性：

```xml
<!-- 只在交叉编译时安装到 Ubuntu host 容器 -->
<system_depend when="cross" realm="host" check_kind="command" check_arg="meson">meson</system_depend>

<!-- 只在交叉编译时安装到 Bianbu target sysroot -->
<system_depend when="cross" realm="target" arch="riscv64">python3-dev</system_depend>

<!-- 只在启用指定驱动时需要，且用文件存在性检查目标 sysroot 是否满足 -->
<system_depend when="cross" realm="target" arch="riscv64"
               check_kind="file"
               check_arg="/usr/lib/rust-1.91/lib/rustlib/riscv64a23-unknown-linux-gnu/lib"
               option_key="enabled_drivers"
               option_value="drv_uart_xl330,uart_xl330">libstd-rust-1.91-dev</system_depend>
```

- `when="cross"` 表示只在交叉编译依赖收集时生效；`when="native"` / `when="docker"` 可用于排除交叉编译。
- `realm="host"` 表示安装到 Ubuntu host 容器；`realm="target"` 表示安装到 Bianbu sysroot；
  `realm="both"` 表示两边都需要；`realm="skip"` 表示交叉编译忽略该系统依赖。
- 未指定 `realm` 时，`build/package.xml`、`build/package_cross.xml`、`build/package_ros2_cross.xml`
  默认属于 host，普通组件包默认属于 target。
- `check_kind` 支持 `dpkg`、`command`、`pkg-config`、`file`、`rustlib`；不写时默认按 dpkg 包检查。
- `board="k1"` / `board="k3"` 可按目标板系列过滤；`arch` 仍按 realm 对应架构过滤。
- `option_key` / `option_value` 可按 target 中的 `enabled_package_options` 过滤，例如只在启用某个驱动时
  安装 Rust 工具链或板端库。

交叉编译全局依赖由以下文件声明：

- `build/package.xml`：普通构建基础依赖，也会进入交叉编译 host 依赖。
- `build/package_cross.xml`：交叉编译专用全局依赖，例如 riscv64 gcc/g++、binutils、meson、ninja。
- `build/package_ros2_cross.xml`：交叉编译 ROS2 包时才加入的 host/target 依赖。

## 如何编译

### 完整编译

在**仓库根目录**执行。两种方式等价，`m` 为快捷命令，`build.sh` 适合脚本或 CI。

**方式一：快捷 m 命令**

```bash
# 1. 加载环境与快捷命令
source build/envsetup.sh

# 2. 选择目标（交互菜单或直接指定）
lunch
# 或：lunch k3-com260-minimal

# 3. 全量构建
m                    # CMake + ROS2
m -C                 # 仅 CMake 包
m -R                 # 仅 ROS2 包
m -j8                # 指定 8 并行任务
m -py                # 全量构建并在 non-ROS2 包 install 成功后打 Python wheel（见下文）
```

**方式二：build.sh**

```bash
# 需先 lunch 或设置 BUILD_TARGET
source build/envsetup.sh
lunch k3-com260-minimal

# 全量 / 分类型构建
./build/build.sh all        # CMake + ROS2，构建完自动 deploy-rootfs
./build/build.sh cmake      # 仅 CMake 包（同 m -C）
./build/build.sh ros2       # 仅 ROS2 包（同 m -R）

# 选项
./build/build.sh -j8 all    # 指定 8 并行
./build/build.sh -v all     # 详细输出到终端
./build/build.sh --py cmake # 仅 CMake，并在各包 install 成功后打 Python wheel（见「Python wheel 包」）

# 脚本/CI 场景：用环境变量指定目标，可省去 lunch
BUILD_TARGET=k3-com260-minimal ./build/build.sh all

# 清理与部署
./build/build.sh clean all          # 清理全部
./build/build.sh clean cmake        # 仅清理 CMake
./build/build.sh deploy-rootfs      # 从 staging 生成 rootfs
```

### 交叉编译

交叉编译的外部命令尽量保持与普通 `m` / `mm` 一致，只需要先启用 `m_enable_cross_build`：

```bash
source build/envsetup.sh
lunch k3-com260-reachy-mini
m_enable_cross_build

m              # 等价于 ./build/cross_build.sh all
m -C           # 等价于 ./build/cross_build.sh cmake
m -R           # 等价于 ./build/cross_build.sh ros2
m -j8          # 指定并行度
m clean        # 清理 output/cross/<target>
```

单包交叉编译：

```bash
source build/envsetup.sh
lunch k3-com260-reachy-mini
m_enable_cross_build

cd components/peripherals/motor
mm
mm --with-deps
mm clean
```

直接脚本调用：

```bash
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh all
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh package components/peripherals/motor --with-deps
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh runtime-deps all
```

### 单组件编译

**方式一：快捷 mm 命令**（需先 `source envsetup.sh` 且 `lunch`，在包目录内执行）

```bash
source build/envsetup.sh
lunch k3-com260-minimal
cd components/peripherals/lidar
mm
mm -py               # 构建当前包并在 install 成功后打 wheel（若存在 pyproject 约定路径）
mm --with-deps       # 先构建当前包依赖，再构建当前包
mm --deps            # 只构建当前包依赖，不构建当前包
# 传递 CMake 参数（仅对 CMake 包生效）
mm -DBUILD_STREAM_DEMO=ON
mm -- -DOPT1=ON -DOPT2=OFF
```

`mm --with-deps` / `mm --deps` 对 CMake 包解析 SDK 内部 `<depend>` 依赖；对 ROS2 包会先从标准
`package.xml` 依赖中预构建可映射到 `components/` 的 SDK underlay 组件，再使用 colcon 的 workspace
包依赖（`--packages-up-to`）语义。

**方式二：build.sh package**（支持相对路径，无需绝对路径）

```bash
source build/envsetup.sh
lunch k3-com260-minimal

# 从仓库根目录
./build/build.sh package components/peripherals/lidar
./build/build.sh package components/peripherals/lidar clean

# 已 cd 进包目录时，可用 .
cd components/peripherals/lidar
./build/build.sh package .
```

`./build/build.sh help` 可查看完整子命令与选项。

### Python wheel 包（可选）

部分组件在 **`python/pyproject.toml`** 或 **包根目录 `pyproject.toml`** 下提供可发布的 Python 包。默认 **不** 打 wheel；需要时在**同一次**全量/CMake/单包构建中加上 **`--py`**（或通过 `m -py` / `mm -py` 转发给 `build.sh`）。

**前置**：系统已安装 **`python3`（例如 3.14）** 只表示解释器可用；打 wheel 还需要：**`python3-build`**（提供 `python3 -m build`）与 **`pybind11-dev`**（多数 C++ bindings 需要 `find_package(pybind11 CONFIG)`）。使用 **`m -py` / `mm -py` / `./build/build.sh --py`** 时，若缺这些依赖，会在 wheel 阶段直接报错并中止（不再跳过）。

**入口示例**

```bash
source build/envsetup.sh
lunch k3-com260-minimal

# 全量或仅 CMake：在对应包 install 成功后打 wheel
m -py
m -py -C
./build/build.sh --py cmake
./build/build.sh --py all

# 单包（在包目录下）
cd components/model_zoo/asr
mm -py

# 仅重打 wheel（不重跑 CMake）：在仓库根目录
./build/python_wheels.sh components/model_zoo/asr
./build/python_wheels.sh components/agent_tools/mlink/gateway
```

**产物路径**：`output/wheels/<包路径中 / 与空格替换为 __>/` 下生成 `.whl`；日志：`output/log/cmake/pkgs/<同上>.wheel.log`。

**约定与限制**

- 跳过 `components/thirdparty/` 下的包（避免对 vendor 误跑）。
- wheel 构建会使用与 CMake 安装一致的 **`PREFIX`**（默认 `output/staging`），并前置 `CMAKE_PREFIX_PATH` / `PKG_CONFIG_PATH` / `LD_LIBRARY_PATH`，便于扩展模块找到已安装的库。
- 若 `pyproject.toml` 不在上述两种路径（例如仅在子目录 `py_wheel/`），当前构建系统**不会**自动发现；可单独执行 `./build/python_wheels.sh` 并传入该组件在仓库中的包路径，或后续在组件内自行调用同一逻辑。

实现上，wheel 逻辑集中在 **`build/python_wheels.sh`**：被 `nonros2.sh` **source** 时提供函数 `srobotis_maybe_build_python_wheel`；**直接执行**该文件时则仅对命令行列出的包打 wheel。

## 如何运行

### 运行非 ROS2 应用

构建产物在 `output/staging`。运行前需 `source build/envsetup.sh`（PATH 与 LD_LIBRARY_PATH 已包含 staging）：

```bash
test_lidar_uart YDLIDAR /dev/ttyUSB0 230400   # 示例：test_lidar_uart测试用例（staging/bin 已在 PATH 中）
```


### 运行 ROS2 应用

需先加载 ROS2 与项目环境：

```bash
sros2_setup
ros2 run peripherals_lidar_node lidar_2d_node   # 示例：2D 雷达 ROS2 节点
```

## 详细使用

- 环境变量说明、`build.sh` 子命令、`target/*.json` 字段、依赖与 Python 环境等详见 **官方文档**（链接待补充）。

## 常见问题

### source envsetup 后为什么没有自动进入 Docker 编译？

`source build/envsetup.sh` 会显式把 `SROBOTIS_USE_DOCKER_BUILD` 重置为 `0`。这是为了避免复用 shell
时误把本地构建放进 Docker。需要 Docker 编译时，每次 source 后重新执行：

```bash
m_enable_docker_build
```

### Docker 编译到底执行了几次 build.sh？

当 target 配置启用了 `options.auto_resolve_dependencies=true`，或外部显式设置了
`AUTO_INSTALL_DEPS=yes` / `true` 时，Docker 封装会先以 root 运行一次依赖检查/安装，并带上
`SROBOTIS_DEPS_ONLY=1`；依赖安装完成后再以宿主 uid/gid 运行真实构建，并带上
`SROBOTIS_SKIP_DEPS_CHECK=1`。这样既能安装 apt 依赖，又能避免最终产物变成 root-owned。

如果未启用自动依赖安装，则直接进入真实构建，依赖缺失时由容器内的 `build.sh` 报错。

### Docker 容器什么时候会复用，什么时候会重建？

默认容器名包含 SDK 路径和 Bianbu 版本，所以同一路径、同一 target family 会复用容器。容器已停止时会
`docker start` 后复用；容器存在但没有正确挂载当前 `REPO_ROOT` 时会删除并重建。

需要强制使用固定容器名时可设置：

```bash
SROBOTIS_DOCKER_CONTAINER_NAME=srobotis-k3-build ./build/build.sh all
```

### source envsetup 后为什么没有自动进入交叉编译？

`source build/envsetup.sh` 会显式把 `SROBOTIS_USE_CROSS_BUILD` 重置为 `0`。这是为了避免一个 shell
复用到另一个 SDK 或 target 时误走交叉编译。需要交叉编译时，每次 source 后重新执行：

```bash
m_enable_cross_build
```

### 交叉编译和 Bianbu Docker 编译怎么选？

- 想尽量模拟板端原生环境，优先用 `m_enable_docker_build`。
- 想在 x86_64 主机上产出 riscv64 目标文件，并复用主机侧 CMake/colcon/Cargo 工具链，使用
  `m_enable_cross_build`。

两者都是显式开关，不会自动启用；同一个 shell 中不要同时启用两种模式。

### 修改 target 依赖后为什么 sysroot 没变？

交叉编译会根据 target 依赖指纹复用 `output/cross/<target>/sysroot`。如果需要强制重新导出：

```bash
SROBOTIS_CROSS_REFRESH_SYSROOT=1 BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh all
```

### 如何确认板端还需要安装哪些运行时包？

先完成交叉编译和 rootfs 生成，再执行：

```bash
BUILD_TARGET=k3-com260-reachy-mini ./build/cross_build.sh runtime-deps all
```

输出中的 `Install command` 是根据当前 ELF 动态库依赖反推的板端 apt 安装命令；`Unresolved runtime libraries`
非空时可加 `--strict` 让命令以失败退出。

## 版本与发布

本目录为仓库构建脚本，无独立版本号，随仓库一起发布；

| 版本       | 说明 |
| ---------- | ---- |
| 随仓库发布 | 构建与环境脚本（envsetup、build.sh、target 配置等）随仓库版本发布。|

## 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

### 提交流程

1. **Fork 并克隆仓库**（若为托管平台协作流程）
2. **创建分支**：`git checkout -b fix/xxx` 或 `git checkout -b feat/xxx`
3. **修改代码**，完成后执行 `shellcheck build/*.sh` 等检查
4. **提交**：
   ```bash
   git add <修改的文件>
   git commit -m "build(readme): 简短描述本次修改"
   git push origin <分支名>
   ```
5. **创建 Pull Request**：在托管平台从个人分支向主仓库目标分支发起 MR/PR

### 提交规范

- **编码规范**：本目录 Shell 脚本遵循 [Google Shell 风格指南](https://google.github.io/styleguide/shellguide.html)，请按该规范编写与修改代码。
- **commit message**：遵循 [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) 规范，格式为 `<type>(<scope>): <description>`，如 `build(xxx): fix envsetup for zsh`、`docs(readme): add build.sh usage`。
- **提交前检查**：请在提交前运行本仓库的脚本检查，确保通过风格检查：
  ```bash
  # 在仓库根目录执行（检查 build 目录下所有 .sh）
  shellcheck build/*.sh
  ```
  若未安装 `shellcheck`，可先执行：`sudo apt install shellcheck`（或见 [ShellCheck](https://github.com/koalaman/shellcheck) 安装说明）。

## License

本目录源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
