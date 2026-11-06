# 模型与语音资源

| 资源包 | 内容 |
| --- | --- |
| `assets.tar.gz` | SenseVoice、VAD、仪表 NCNN 权重、原音色提示音 WAV |
| `vision-models.tar.gz` | 仪表、OCR、人员/安全帽模型，导入 `~/.cache/models/vision` |

`k1-assets.json` 逐项记录路径、大小及 SHA256，`resource-bundles.json` 记录资源包本身的 SHA256。ASR 的 `tokens.txt` 是字表，不是访问凭据。

```bash
curl -fL --retry 3 -o /tmp/rvclaw-k1-assets.tar.gz \
  https://github.com/jxgxxx/RVClaw-K1/releases/download/k1-resources-20261106/assets.tar.gz
curl -fL --retry 3 -o /tmp/rvclaw-k1-vision-models.tar.gz \
  https://github.com/jxgxxx/RVClaw-K1/releases/download/k1-resources-20261106/vision-models.tar.gz
python3 scripts/import_k1_assets.py \
  --portable-archive /tmp/rvclaw-k1-assets.tar.gz \
  --vision-archive /tmp/rvclaw-k1-vision-models.tar.gz
```

两个资源包通过 [K1 配套资源预发布](https://github.com/jxgxxx/RVClaw-K1/releases/tag/k1-resources-20261106) 提供，合计约 540 MB。下载后按清单验证；导入器逐项检查 174 项资源的大小和 SHA256。仅 clone 源码不能完整运行推理及原音色播报。资源包不包含 API 密钥、现场日志或录音。

本快照按 `k1-assets.json` 导入资源。资源版权及使用遵循各自模型、厂商条款。
