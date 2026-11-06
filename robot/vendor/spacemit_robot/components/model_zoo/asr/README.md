# ASR 组件

## 1. 项目简介

本组件为通用 ASR 封装，提供统一的 C++ 接口与 Python 绑定，**支持本地与云端**多种识别引擎，便于集成到 AI Agent 等应用中。当前已支持 SenseVoice（本地 ONNX），接口可扩展其他本地或云端后端。功能特性如下：

| 类别     | 支持                                                                 |
| -------- | -------------------------------------------------------------------- |
| 部署方式 | **本地**（如 ONNX 推理）、**云端**（可扩展 HTTP/API 等）             |
| 识别方式 | 文件/内存阻塞识别 `Call()`、`Recognize()`；流式识别 `Start()` + `SendAudioFrame()` + `Flush()` / `Stop()` |
| 后端     | SenseVoice（本地 ONNX）、Zipformer CTC（本地 ONNX 流式）、Qwen3-ASR（llama-server） |
| 语言     | 中文、英文、日文、韩文、粤语、自动检测                               |
| 接口     | C++（`include/asr_service.h`）、Python（`spacemit_asr`）             |

## 2. 验证模型

按以下顺序完成依赖安装、模型准备与示例运行。

### 2.1. 安装依赖

- **编译环境**：CMake ≥ 3.15，C++17 编译器（GCC/Clang/MSVC）。
- **必选**：libsndfile、libfftw3、libcurl（若使用默认后端）。

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake \
  libsndfile1-dev libfftw3-dev libcurl4-openssl-dev
```

**可选：**

- **Python 绑定**：`pip install pybind11` 或 `apt install python3-pybind11`
- **流式示例（C++）**：需 audio 组件 + PortAudio，`apt install portaudio19-dev`。SDK 编译时默认开启，独立编译时默认关闭（`cmake .. -DBUILD_STREAM_DEMO=ON`）

**CMake 编译选项：**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `USE_SPACEMIT_EP` | **ON** | 启用 SpaceMIT EP 加速（K3 平台）。非 K3 平台如未安装 libspacemit_ep 会自动跳过并打印 warning |
| `ASR_MODEL_FETCH_OFF` | **ON** | 默认禁用 cmake 配置阶段的自动模型下载。设为 OFF 可开启自动下载（`cmake .. -DASR_MODEL_FETCH_OFF=OFF`） |
| `BUILD_STREAM_DEMO` | OFF（独立编译） | 编译流式 demo，需 audio 组件 + PortAudio |
| `BUILD_PYTHON_BINDINGS` | ON | 编译 Python 绑定，需 pybind11 |

### 2.2. 下载模型

> **默认行为**：cmake 配置阶段**不会**自动下载模型。程序运行时如检测到模型缺失会自动下载（覆盖交叉编译部署场景）。
>
> 如需在 cmake 配置阶段自动下载模型：`cmake .. -DASR_MODEL_FETCH_OFF=OFF`

#### 2.2.1 SenseVoice 模型（本地 ONNX）

使用 SenseVoice 时需将模型放到默认路径 **`~/.cache/models/asr/sensevoice/`**，目录内需包含 `model_quant_optimized.onnx`、`tokens.txt`、`am.mvn`。

**手动下载：**

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr
wget https://archive.spacemit.com/spacemit-ai/model_zoo/asr/sensevoice.tar.gz
tar -xzf sensevoice.tar.gz
```

#### 2.2.2 Qwen3-ASR 模型（llama-server）

Qwen3-ASR 通过 llama-server 提供服务，需要安装 llama.cpp 工具包并下载模型。

注意：Qwen3-ASR 模型默认上下文较大，未限制上下文时在 8G 内存板卡上启动 llama-server 可能因内存不足失败或被系统 kill。下面示例通过 `-c 4096` 降低 KV cache 内存占用；如仍内存不足，建议配置 swap 或使用更大内存配置。

**1. 安装 llama-server：**

```bash
sudo apt install llama.cpp-tools-spacemit
```

**2. 下载模型：**

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr

# Qwen3-ASR 0.6B（轻量，速度优先）
wget https://archive.spacemit.com/spacemit-ai/model_zoo/asr/qwen3-asr-0.6B-dynq-q40.tar.gz
tar -xzf qwen3-asr-0.6B-dynq-q40.tar.gz

# Qwen3-ASR 1.7B（更大，质量优先）
wget https://archive.spacemit.com/spacemit-ai/model_zoo/asr/qwen3-asr-1.7B-dynq-q40.tar.gz
tar -xzf qwen3-asr-1.7B-dynq-q40.tar.gz
```

解压后目录结构：
```
qwen3-asr-0.6B-dynq-q40/
├── Qwen3-ASR-0.6B-text-q40.gguf
├── Qwen3-ASR-0.6B-encoder-frontend.dynq.onnx
├── Qwen3-ASR-0.6B-encoder-backend.dynq.onnx
└── config.json

qwen3-asr-1.7B-dynq-q40/
├── Qwen3-ASR-1.7B-text-q40.gguf                 # LLM 文本解码器
├── Qwen3-ASR-1.7B-encoder-frontend.dynq.onnx    # 音频编码器前端
├── Qwen3-ASR-1.7B-encoder-backend.dynq.onnx     # 音频编码器后端
├── Qwen3-ASR-1.7B-encoder-split-metadata.json   # encoder split metadata
└── config.json
```

**3. 启动 llama-server：**

启动哪个模型，就把 `-m` 和 `--smt-config-dir` 指向对应目录。ASR 组件只访问
llama-server endpoint，不直接切换本地模型文件。

Qwen3-ASR 0.6B：

```bash
llama-server \
    -m ~/.cache/models/asr/qwen3-asr-0.6B-dynq-q40/Qwen3-ASR-0.6B-text-q40.gguf \
    --media-backend smt \
    --smt-config-dir ~/.cache/models/asr/qwen3-asr-0.6B-dynq-q40/ \
    --host 127.0.0.1 --port 8063 \
    --alias qwen3-asr \
    -t 4 -c 4096
```

Qwen3-ASR 1.7B：

```bash
llama-server \
    -m ~/.cache/models/asr/qwen3-asr-1.7B-dynq-q40/Qwen3-ASR-1.7B-text-q40.gguf \
    --media-backend smt \
    --smt-config-dir ~/.cache/models/asr/qwen3-asr-1.7B-dynq-q40/ \
    --host 127.0.0.1 --port 8063 \
    --alias qwen3-asr \
    -t 4 -c 4096
```

关键参数说明：
- `-m` 与 `--smt-config-dir`：决定当前服务加载的 Qwen3-ASR 模型版本。
- `--media-backend smt`：启用 SpacemiT 媒体后端（处理音频输入）
- `--smt-config-dir`：指定包含 ONNX 音频编码器的目录
- `--alias qwen3-asr`：设置 OpenAI API 请求中的模型名。保持该 alias 时，`asr_file_demo --engine qwen3-asr` 无需额外指定 `--model`。
- `-c 4096`：限制 llama.cpp 上下文长度，降低 8G 板卡上的 KV cache 内存占用；长音频可按需调大。

**4. 验证服务：**

```bash
curl http://127.0.0.1:8063/health
# 应返回 {"status":"ok"}
```

#### 2.2.3 Zipformer 模型（本地 ONNX 流式）

Zipformer CTC 是轻量级流式 ASR 模型，适合实时识别场景。

当前 Zipformer 后端在 K3 + SpaceMIT EP 下需要临时禁用 `Conv` 算子，用于规避现阶段 SpaceMIT EP 侧兼容问题。后续 bug 修复后，该环境变量可移除。

**手动下载：**

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr
wget https://archive.spacemit.com/spacemit-ai/model_zoo/asr/zipformer.tar.gz
tar -xzf zipformer.tar.gz
```

**使用：**

```bash
export SPACEMIT_EP_DISABLE_OP_TYPE_FILTER="Conv"
./build/bin/asr_file_demo audio.wav --engine zipformer
```

### 2.3. 下载测试音频

建议先下载一段示例音频用于快速验证（16kHz 单声道 wav 更佳）：

```bash
mkdir -p ~/.cache/models/assets/audio
cd ~/.cache/models/assets/audio
wget https://archive.spacemit.com/spacemit-ai/model_zoo/assets/audio/001_zh_daily_weather.wav
```

更多音频资源可在 `https://archive.spacemit.com/spacemit-ai/model_zoo/assets/audio` 按需下载。

### 2.4. 测试

本节提供示例程序的编译与运行方式，便于开发者快速验证效果。使用前需先按下列两种方式之一完成编译，再运行对应示例。

- **在 SDK 中验证**（2.4.1）：在已拉取的 SpacemiT Robot SDK 工程内用 `mm` 编译，产物部署到 `output/staging`，适合整机集成或与 LLM、TTS 等模块联调。
- **独立构建下验证**（2.4.2）：在 ASR 组件目录下用 CMake 本地编译，不依赖完整 SDK，适合快速体验或在不使用 repo 的环境下使用。

#### 2.4.1. 在 SDK 中验证

**编译**：本组件已纳入 SpacemiT Robot SDK 时，在 SDK 根目录下执行。SDK 拉取与初始化见 [SpacemiT Robot SDK Manifest](https://github.com/spacemit-robotics/manifest)（使用 repo 时需先完成 `repo init`、`repo sync` 等）。

```bash
source build/envsetup.sh
cd components/model_zoo/asr
mm
```

构建产物会安装到 `output/staging`。

**运行**：运行前在 SDK 根目录执行 `source build/envsetup.sh`，使 PATH 与库路径指向 `output/staging`，然后可执行：

**C++ 文件识别（SenseVoice）：**

```bash
asr_file_demo ~/.cache/models/assets/audio/001_zh_daily_weather.wav
```

**C++ 文件识别（Zipformer）：**

```bash
export SPACEMIT_EP_DISABLE_OP_TYPE_FILTER="Conv"
asr_file_demo ~/.cache/models/assets/audio/001_zh_daily_weather.wav --engine zipformer
```

**C++ 文件识别（Qwen3-ASR，需先启动 llama-server）：**

```bash
asr_file_demo ~/.cache/models/assets/audio/001_zh_daily_weather.wav --engine qwen3-asr
# 指定远程服务器
asr_file_demo audio.wav --engine qwen3-asr --endpoint http://10.0.90.72:8063/v1/chat/completions
```

**Python 文件识别**（直接运行 `python python/examples/...` 前，需当前 Python 环境已安装 wheel，或设置 `PYTHONPATH` 指向 SDK 构建产物）：

```bash
python python/examples/asr_file_demo.py ~/.cache/models/assets/audio/001_zh_daily_weather.wav
```

**流式识别**（SDK 编译时默认已开启，可直接运行）：

```bash
asr_stream_demo -l              # 列出麦克风设备
asr_stream_demo -i 0 -t 5       # 设备 0，录音 5 秒
```

**Python 流式识别**（需已安装 `spacemit_asr` 和 `spacemit_audio`，或设置好 `PYTHONPATH`）：

```bash
python python/examples/asr_stream_demo.py -l
python python/examples/asr_stream_demo.py --duration 5
python python/examples/asr_stream_demo.py --duration 5 --channels 2
```

#### 2.4.2. 独立构建下验证

在 ASR 组件目录下完成编译后，运行下列示例。

**C++ 文件识别（默认构建即包含）：**

```bash
cd /path/to/asr
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# SenseVoice（默认）
./bin/asr_file_demo ~/.cache/models/assets/audio/001_zh_daily_weather.wav

# Zipformer（K3 + SpaceMIT EP 下临时禁用 Conv）
export SPACEMIT_EP_DISABLE_OP_TYPE_FILTER="Conv"
./bin/asr_file_demo ~/.cache/models/assets/audio/001_zh_daily_weather.wav --engine zipformer

# Qwen3-ASR（需先启动 llama-server，见 2.2.2）
./bin/asr_file_demo ~/.cache/models/assets/audio/001_zh_daily_weather.wav --engine qwen3-asr
```

**Python 文件识别：**

```bash
cd /path/to/asr
cmake --build build --target asr-install-python   # 或设置 PYTHONPATH
python python/examples/asr_file_demo.py ~/.cache/models/assets/audio/001_zh_daily_weather.wav
```

**流式识别（默认未开启）**：需先安装 PortAudio（见 2.1 可选依赖）和 audio 组件，然后开启流式示例重新构建：

```bash
cd build
cmake .. -DBUILD_STREAM_DEMO=ON
make -j$(nproc)
./bin/asr_stream_demo -l              # 列出麦克风设备
./bin/asr_stream_demo -i 0 -t 5      # 设备 0，录音 5 秒
```

Python 流式示例无需额外编译选项，安装 `spacemit_asr` 和 `spacemit_audio` 后直接运行：

```bash
python python/examples/asr_stream_demo.py -l
python python/examples/asr_stream_demo.py --duration 5
python python/examples/asr_stream_demo.py --duration 5 --channels 2
```

## 3. 应用开发

本章说明如何在自有工程中**集成 ASR 并调用 API**。环境与依赖见 [2.1](#21-安装依赖)，模型准备见 [2.2](#22-下载模型)，编译与运行示例见 [2.4](#24-测试)。

### 3.1. 构建与集成产物

无论通过 [2.4.1](#241-在-sdk-中验证)（SDK）或 [2.4.2](#242-独立构建下验证)（独立构建）哪种方式编译，完成后**应用开发所需**的库与头文件如下，集成时只需**包含头文件并链接对应库**：

| 产物 | 说明 |
| ---- | ---- |
| `include/asr_service.h` | **C++ API 头文件**，应用侧只需包含此头文件并链接下方库即可调用 |
| `build/lib/libasr.a` | C++ 核心库，链接时使用 |
| `build/lib/libsensevoice.a` | SenseVoice 后端库，链接时使用 |
| `build/python/spacemit_asr/` | Python 包，`cmake --build build --target asr-install-python` 安装后 `import spacemit_asr` |

示例可执行文件（非集成必需）：`build/bin/asr_file_demo`、`build/bin/asr_stream_demo`（SDK 默认开启，独立编译需 `-DBUILD_STREAM_DEMO=ON`）。运行与验证步骤见 [2.4.1](#241-在-sdk-中验证) 或 [2.4.2](#242-独立构建下验证)。

### 3.2. API 使用

**C++**：头文件 `include/asr_service.h` 为唯一 API 入口，实现为 PIMPL，无额外依赖。在业务代码中 `#include "asr_service.h"`，链接 `libasr.a` 与 `libsensevoice.a`（及 libsndfile 等），即可使用。兼容旧工程时仍安装 `libstt.a`。

```cpp
#include "asr_service.h"
using namespace SpacemiT;

AsrConfig config = AsrConfig::Preset("sensevoice");
config.language = "auto";
config.punctuation = true;
auto engine = std::make_shared<AsrEngine>(config);

// 文件识别
auto result = engine->Call("~/.cache/models/assets/audio/001_zh_daily_weather.wav");
if (result) std::cout << result->GetText() << std::endl;

// 内存识别（16kHz mono）
std::vector<float> audio = load_audio();
result = engine->Recognize(audio, 16000);
```

**Python**：安装后 `import spacemit_asr`，详见 `python/examples/` 与 [API.md](API.md)。

```python
import spacemit_asr
text = spacemit_asr.recognize_file("~/.cache/models/assets/audio/001_zh_daily_weather.wav")
# 或
with spacemit_asr.Engine() as engine:
    result = engine.recognize_file("~/.cache/models/assets/audio/001_zh_daily_weather.wav")
    print(result.text, result.rtf)
```

**CMake 集成**：将本组件作为子目录引入，并链接 `asr`、包含头文件路径即可。

```cmake
add_subdirectory(asr)
target_link_libraries(your_target PRIVATE asr)
target_include_directories(your_target PRIVATE ${ASR_SOURCE_DIR}/include)
```

## 4. 常见问题

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 首次识别很慢 | 模型加载和 warmup 被计入首次运行 | 以第二次及之后结果评估性能。 |
| 识别文本为空或很短 | 输入音频过短、静音或采样率不匹配 | 用 `audio_demo play` 回放，确认 WAV 是 16kHz 有声内容。 |
| 流式采集报设备错误 | 默认输入设备不符合预期 | 先运行 `asr_stream_demo -l`，再用 `-i <id>` 指定设备。 |
| Zipformer 报 SpaceMIT EP/Conv 相关错误 | 当前 K3 + SpaceMIT EP 下 `Conv` 算子存在临时兼容问题 | 运行前设置 `SPACEMIT_EP_DISABLE_OP_TYPE_FILTER="Conv"`。 |
| Qwen3-ASR 调用失败 | `llama-server` 未启动或 endpoint 不正确 | 用 `curl http://127.0.0.1:8063/health` 确认服务状态。 |
| Qwen3-ASR 启动失败或被 kill | 8G 内存板卡可能内存不足 | 检查系统内存；如必须在 8G 板卡上运行，先配置 swap 后再启动 `llama-server`。 |

## 5. 版本与发布

版本以本组件文档或仓库 tag 为准。

| 版本   | 说明 |
| ------ | ---- |
| 1.0.2  | 同步 C++、Python、组件清单和各 backend 版本号，用于发布包含 SenseVoice 情绪识别开关的 ASR 包。 |
| 0.1.0  | 提供 C++ / Python 接口，支持 SenseVoice、Zipformer CTC、Qwen3-ASR，文件/内存阻塞识别与流式识别。 |

## 6. 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

- **编码规范**：C++ 代码遵循 [Google C++ 风格指南](https://google.github.io/styleguide/cppguide.html)。
- **提交前检查**：若仓库提供 lint 脚本，请在提交前运行并通过检查。

## 7. License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。

## 8. 附录：性能指标

以下数据基于 K3 平台实测，为阶段性信息，持续优化中，请以最新文档为准。

### SenseVoice (INT8, SpaceMIT EP, 2 线程)

| 测试文件 | 音频时长 | 处理时间 | RTF |
|----------|----------|----------|-----|
| 004_zh_selling_sausages.wav | 14158 ms | 5090 ms | 0.360 |
| 001_zh_daily_weather.wav | 1619 ms | 212 ms | 0.131 |
| 002_en_daily_weather.wav | 1802 ms | 232 ms | 0.129 |
| 003_zh_en_search.wav | 2324 ms | 299 ms | 0.129 |
| **合计** | **19903 ms** | **5833 ms** | **0.293** |

### Qwen3-ASR (Q4_0, llama-server, K3)

测试命令使用 `llama-server -c 4096`，并通过 `asr_file_demo --engine qwen3-asr --rounds 3`
连续识别 `001_zh_daily_weather.wav` 与 `004_zh_selling_sausages.wav`。

| 模型 | llama-server 线程数 | 测试文件 | 总音频时长 | 总处理时间 | RTF |
|------|---------------------|----------|------------|------------|-----|
| Qwen3-ASR 0.6B | 4 | 2 个文件 x 3 轮 | 47331 ms | 10257 ms | 0.217 |
| Qwen3-ASR 0.6B | 8 | 2 个文件 x 3 轮 | 47331 ms | 7977 ms | 0.169 |
| Qwen3-ASR 1.7B | 4 | 2 个文件 x 3 轮 | 47331 ms | 19883 ms | 0.420 |
| Qwen3-ASR 1.7B | 8 | 2 个文件 x 3 轮 | 47331 ms | 15572 ms | 0.329 |

### Zipformer CTC (CPU, 4 线程)

| 测试文件 | 音频时长 | 处理时间 | RTF |
|----------|----------|----------|-----|
| ref.wav (14s 中文) | 14158 ms | 6622 ms | 0.468 |

测试音频文件可从 [archive.spacemit.com](https://archive.spacemit.com/spacemit-ai/model_zoo/assets/audio) 下载：
```bash
mkdir -p ~/.cache/models/assets/audio
cd ~/.cache/models/assets/audio
wget https://archive.spacemit.com/spacemit-ai/model_zoo/assets/audio/001_zh_daily_weather.wav
./build/bin/asr_file_demo ~/.cache/models/assets/audio/001_zh_daily_weather.wav
```
