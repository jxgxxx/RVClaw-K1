# 本地与云端并行仪表识别

仪表服务包 `meter_gauge/cloud_meter.py` 提供云端识别兼容接口；
`meter_gauge/hybrid.py` 在本地首次拒识后按需提交当前帧。代码不打开相机，不控制底盘。

配置：`configs/device/meter/cloud_meter.env`。
密钥：`runtime/k1/secrets/meter_cloud_api_key`，不纳入版本控制。
接口结构：`contracts/cloud_meter_reading.schema.json`、`contracts/meter_cloud_assistance.schema.json`。

## 巡检行为

本地首次识别正常时保持原有逻辑：A/B 两次有效数字读数按容差一致；
C 两次正常/异常条件一致并取中位数。该路径不提交云端请求。

本地拒识后，用该次留档 JPEG 启动一个网络任务，本地继续获取新帧并识别。
正向定位到表盘/显示区时先做带边缘余量的裁剪；没有可信区域时发送当前全景。
不发送模板上的数字或正常阈值，也不读取参考照片的旧数值。

本地两次先一致就立即完成，不等待后台网络线程。
云端候选必须与另一个独立采集帧交叉确认：可以是后续本地有效读数，
也可以是在本地仍失败时对另一帧追加的一次云端确认。每次检测最多两次云端请求。
同一帧的本地结果与云端结果不能算两份独立证据。

云端结果检查类型、数字格式、单位、物理量程、置信度、帧时效与点位/会话编号。
指针表的云端交叉确认额外要求数值误差不超过该点的容差，不能仅因阈值判断相同就互相确认。
数值冲突不按返回快慢或模型自报置信度裁决；仅允许上述一次新帧确认。
单张高置信度云端回答不会直接变成巡检成功。

配置的网络请求等待上限为 20 秒；从首次本地失败起的辅助窗口为 30 秒；
本地尝试结束后最多额外等 8 秒，均受点位原有总截止时间限制。
这些是程序预算，不是云端速度承诺。底层 socket 超时并非严格的整次请求墙钟上限；
超过辅助窗口或检测结束后的返回值会被隔离，不改写已完成结果。
进程内最多一个云端网络任务，不排队堆积；网络错误后熔断 30 秒。

定位、转向、停车及重新对位的原有条件保持不变，云端不能绕过这些条件。
设置 `RVCLAW_METER_HYBRID_ENABLED=0` 可关闭辅助，不改本地识别配置。
未部署辅助模块或加载失败时仍走原有本地路径。

## 使用与记录

```bash
# 本地配置检查：不联网、不识别、不打开相机
python3 /data/RVClaw/services/meter_gauge/run_cloud_meter.py --check

# 会上传指定 JPEG 并使用云端 API 额度
python3 /data/RVClaw/services/meter_gauge/run_cloud_meter.py \
  --image /path/to/meter.jpg --meter-type digital
```

指针表使用 `--meter-type pointer`；可用 `--calibration` 指定量程 JSON。
独立命令使用配置中的单次请求超时，巡检辅助使用自己的更短预算。
程序只需 Python 标准库；已有 Pillow 用于按需裁剪，无 Pillow 时保留原图。
不新建常驻模型，不额外启动 OCR/ncnn/ROS 或相机服务。

巡检点结果增加 `cloud_assistance` 审计字段；后台返回另存到
`runtime/k1/vision/cloud_meter/<request_id>/attempt_1.json` 等文件。
字段记录来源帧、裁剪区域、候选、是否迟到，不记录密钥。

历史板端实图与测速记录不随源码发布。此前实测数字表可以读出正确值，也出现过超时；指针表出现过低置信度和数值偏差。
云端是有预算的辅助渠道，仍需现场巡检验证。
