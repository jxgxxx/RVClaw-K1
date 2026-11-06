# 产品脚本

- `check_delivery.py`：不接触硬件的源码/依赖清单检查。
- `import_k1_assets.py`：校验并导入配套模型及原音色资源包。
- `robot/`：组装清单使用的建图、可视化和设备/任务入口。
- `run_sample_flow.py` / `serve_studio.py`：保留项目原有 API/网页辅助入口。

正式服务与巡检使用 `deploy/k1` 的统一部署及调度入口；现场试验记录、临时复现脚本和回放测试不在此目录交付。
