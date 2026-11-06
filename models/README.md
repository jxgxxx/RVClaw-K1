# 模型与语音资源

| 资源包 | 内容 |
| --- | --- |
| `assets.tar.gz` | SenseVoice、VAD、仪表 NCNN 权重、原音色提示音 WAV |
| `vision-models.tar.gz` | 仪表、OCR、人员/安全帽模型，导入 `~/.cache/models/vision` |

`k1-assets.json` 逐项记录路径、大小及 SHA256，`resource-bundles.json` 记录资源包本身的 SHA256。ASR 的 `tokens.txt` 是字表，不是访问凭据。

```bash
python3 scripts/import_k1_assets.py \
  --portable-archive /path/to/assets.tar.gz \
  --vision-archive /path/to/vision-models.tar.gz
```

本稿尚处于本地审阅阶段：两个资源包已准备，**尚未上传，没有已发布下载链接**。审核后应随源码一起交付，并补充真实附件链接。仅 clone 源码不能完整运行推理及原音色播报。资源包不包含 API 密钥、现场日志或录音。

本快照按 `k1-assets.json` 导入资源。资源版权及使用遵循各自模型、厂商条款。
