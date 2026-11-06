# K1 部署入口

完整命令见 [K1 复现指南](../../docs/K1-复现指南.md)。

1. 导入模型/原音色资源，准备固定版本的 K1 系统和 SDK。
2. `assemble_snapshot.py --bind-address <K1地址>` 从公共源码及当前 K1 覆盖层组装新运行目录。
3. 运行 middleware、compiler、workspace、native、scan_map_kernel 和 AMCL 构建。
4. 开发电脑使用 pnpm 锁文件构建网页，将 `studio/web` 部署到 K1。
5. 安装设备规则、用户组和优先级权限，再用 `install_snapshot.py --bind-address <K1地址> --install` 安装服务。
6. 显式启动服务，现场就位后由用户开始巡检。

`snapshot/` 保存当前已部署环境、服务模板、地图选择和源码映射。组装/安装不下达运动命令，且拒绝覆盖已有运行目录或服务。

旧迁移入口不随本次提交；编译统一使用 `build_workspaces.sh`、`build_native.sh` 和 K1 目录内的其他构建脚本。
