# VAD API

语音活动检测框架，支持 C++ 和 Python。

## 功能特性

- **单帧检测**: 检测单帧音频是否为语音
- **流式检测**: 实时音频流检测，支持回调模式
- **状态跟踪**: 语音开始/结束事件通知
- **多后端**: Silero VAD (深度学习)、Energy VAD (轻量级)

---

## C++ API

```cpp
namespace SpacemiT {

// =============================================================================
// VadState - VAD 状态
// =============================================================================

enum class VadState {
    SILENCE,       // 静音
    SPEECH_START,  // 语音开始
    SPEECH,        // 语音中
    SPEECH_END     // 语音结束
};

// =============================================================================
// VadBackendType - 后端类型
// =============================================================================

enum class VadBackendType {
    SILERO,   // Silero VAD (深度学习)
    ENERGY,   // Energy VAD (轻量级)
    WEBRTC,   // WebRTC VAD
    CUSTOM    // 自定义后端
};

// =============================================================================
// VadConfig - 配置
// =============================================================================

struct VadConfig {
    VadBackendType backend = VadBackendType::SILERO;
    std::string model_dir;           // 模型目录
    int sample_rate = 16000;         // 采样率
    int window_size = 512;           // 窗口大小 (32ms @ 16kHz)
    float trigger_threshold = 0.5f;  // 语音开始阈值
    float stop_threshold = 0.35f;    // 语音结束阈值
    int smoothing_window = 10;       // 平滑窗口大小

    // 静态工厂
    static VadConfig Preset(const std::string& name);
    static std::vector<std::string> AvailablePresets();

    // 链式调用
    VadConfig withTriggerThreshold(float t) const;
    VadConfig withStopThreshold(float t) const;
    VadConfig withSmoothingWindow(int w) const;
};

// =============================================================================
// VadResult - 检测结果
// =============================================================================

class VadResult {
public:
    float GetProbability() const;          // 原始语音概率
    float GetSmoothedProbability() const;  // 平滑后概率
    bool IsSpeech() const;                 // 是否为语音
    VadState GetState() const;             // VAD 状态
    int64_t GetTimestampMs() const;        // 时间戳 (ms)
    bool IsSpeechStart() const;            // 是否语音开始
    bool IsSpeechEnd() const;              // 是否语音结束
};

// =============================================================================
// VadEngineCallback - 回调接口
// =============================================================================

class VadEngineCallback {
public:
    virtual void OnOpen() {}
    virtual void OnEvent(std::shared_ptr<VadResult> result) {}
    virtual void OnSpeechStart(int64_t timestamp_ms) {}
    virtual void OnSpeechEnd(int64_t timestamp_ms, int duration_ms) {}
    virtual void OnComplete() {}
    virtual void OnError(const std::string& message) {}
    virtual void OnClose() {}
};

// =============================================================================
// VadEngine - VAD 引擎
// =============================================================================

class VadEngine {
public:
    explicit VadEngine(const VadConfig& config = VadConfig());

    // 单帧检测
    std::shared_ptr<VadResult> Detect(const std::vector<float>& audio, int sample_rate = 16000);
    std::shared_ptr<VadResult> Detect(const std::vector<int16_t>& audio, int sample_rate = 16000);

    // 流式检测
    void SetCallback(std::shared_ptr<VadEngineCallback> callback);
    void Start();
    void SendAudioFrame(const std::vector<float>& data);
    void SendAudioFrame(const float* data, size_t size);
    void Stop();

    // 状态
    void Reset();
    VadState GetCurrentState() const;
    bool IsInSpeech() const;
    bool IsInitialized() const;
    std::string GetEngineName() const;
    VadBackendType GetBackendType() const;

    // 动态配置
    void SetTriggerThreshold(float threshold);
    void SetStopThreshold(float threshold);
};

}  // namespace SpacemiT
```

### C++ 示例

```cpp
#include "vad_service.h"
using namespace SpacemiT;

int main() {
    // 配置
    auto config = VadConfig::Preset("silero")
        .withTriggerThreshold(0.6f)
        .withStopThreshold(0.35f);

    // 创建引擎
    auto engine = std::make_shared<VadEngine>(config);

    // 单帧检测
    std::vector<float> audio(512);  // 32ms @ 16kHz
    auto result = engine->Detect(audio);
    if (result->IsSpeech()) {
        std::cout << "语音概率: " << result->GetProbability() << std::endl;
    }

    return 0;
}
```

### 流式回调示例

```cpp
#include "vad_service.h"
using namespace SpacemiT;

class MyCallback : public VadEngineCallback {
public:
    void OnSpeechStart(int64_t timestamp_ms) override {
        std::cout << "语音开始: " << timestamp_ms << "ms" << std::endl;
    }

    void OnSpeechEnd(int64_t timestamp_ms, int duration_ms) override {
        std::cout << "语音结束: " << timestamp_ms << "ms, 持续: " << duration_ms << "ms" << std::endl;
    }

    void OnEvent(std::shared_ptr<VadResult> result) override {
        std::cout << "概率: " << result->GetProbability() << std::endl;
    }
};

int main() {
    auto engine = std::make_shared<VadEngine>();
    engine->SetCallback(std::make_shared<MyCallback>());

    engine->Start();
    while (recording) {
        std::vector<float> audio = read_audio_frame();
        engine->SendAudioFrame(audio);
    }
    engine->Stop();

    return 0;
}
```

---

## Python API

```python
"""
Space VAD Python API
"""
import spacemit_vad
from spacemit_vad import VadEngine, VadConfig, VadCallback, VadResult, VadState, VadBackendType

# =============================================================================
# 快捷函数
# =============================================================================

def detect(audio: np.ndarray, sample_rate: int = 16000) -> VadResult:
    """
    快速检测单帧音频

    Args:
        audio: numpy 数组 (float32)
        sample_rate: 采样率

    Returns:
        VadResult 检测结果
    """

# =============================================================================
# VadState - VAD 状态
# =============================================================================

class VadState(Enum):
    SILENCE = ...      # 静音
    SPEECH_START = ... # 语音开始
    SPEECH = ...       # 语音中
    SPEECH_END = ...   # 语音结束

# =============================================================================
# VadBackendType - 后端类型
# =============================================================================

class VadBackendType(Enum):
    SILERO = ...  # Silero VAD
    ENERGY = ...  # Energy VAD
    WEBRTC = ...  # WebRTC VAD
    CUSTOM = ...  # 自定义

# =============================================================================
# VadConfig - 配置
# =============================================================================

class VadConfig:
    """VAD 配置"""

    def __init__(self): ...

    @staticmethod
    def preset(name: str) -> "VadConfig":
        """通过预设名称创建配置（如 'silero', 'energy'）"""

    @staticmethod
    def available_presets() -> list[str]:
        """获取所有可用预设名称"""

    # 链式配置
    def with_trigger_threshold(self, threshold: float) -> "VadConfig": ...
    def with_stop_threshold(self, threshold: float) -> "VadConfig": ...
    def with_smoothing_window(self, window: int) -> "VadConfig": ...

# =============================================================================
# VadResult - 检测结果
# =============================================================================

class VadResult:
    """检测结果"""

    @property
    def probability(self) -> float:
        """原始语音概率"""

    @property
    def smoothed_probability(self) -> float:
        """平滑后概率"""

    @property
    def is_speech(self) -> bool:
        """是否为语音"""

    @property
    def state(self) -> VadState:
        """VAD 状态"""

    @property
    def timestamp_ms(self) -> int:
        """时间戳 (毫秒)"""

    @property
    def is_speech_start(self) -> bool:
        """是否语音开始"""

    @property
    def is_speech_end(self) -> bool:
        """是否语音结束"""

# =============================================================================
# VadCallback - 回调类
# =============================================================================

class VadCallback:
    """流式检测回调"""

    def __init__(self): ...

    def on_event(self, callback: Callable[[VadResult], None]):
        """设置事件回调"""

    def on_speech_start(self, callback: Callable[[int], None]):
        """设置语音开始回调 (参数: timestamp_ms)"""

    def on_speech_end(self, callback: Callable[[int, int], None]):
        """设置语音结束回调 (参数: timestamp_ms, duration_ms)"""

    def on_error(self, callback: Callable[[str], None]):
        """设置错误回调"""

    def on_open(self, callback: Callable[[], None]):
        """设置打开回调"""

    def on_complete(self, callback: Callable[[], None]):
        """设置完成回调"""

    def on_close(self, callback: Callable[[], None]):
        """设置关闭回调"""

# =============================================================================
# VadEngine - VAD 引擎
# =============================================================================

class VadEngine:
    """VAD 引擎"""

    def __init__(self, config: Optional[VadConfig] = None):
        """
        Args:
            config: VAD 配置，不传则使用默认配置
        """

    def detect(self, audio: np.ndarray, sample_rate: int = 16000) -> VadResult:
        """
        检测单帧音频

        Args:
            audio: numpy 数组 (float32 或 int16)
            sample_rate: 采样率

        Returns:
            VadResult 检测结果
        """

    # 流式 API
    def set_callback(self, callback: VadCallback):
        """设置回调"""

    def start(self):
        """开始流式检测"""

    def send_audio_frame(self, audio: np.ndarray):
        """发送音频帧 (float32)"""

    def stop(self):
        """停止流式检测"""

    # 状态
    def reset(self):
        """重置状态"""

    @property
    def is_initialized(self) -> bool:
        """是否已初始化"""

    @property
    def engine_name(self) -> str:
        """引擎名称"""

    @property
    def backend_type(self) -> VadBackendType:
        """后端类型"""

    @property
    def last_probability(self) -> float:
        """上次检测的语音概率"""
```

### Python 示例

```python
import spacemit_vad
import numpy as np

# 快捷方式
audio = np.random.randn(512).astype(np.float32) * 0.1
result = spacemit_vad.detect(audio)
print(f"语音概率: {result.probability}")

# Engine 类
engine = spacemit_vad.VadEngine()
result = engine.detect(audio)
if result.is_speech:
    print("检测到语音")
```

### 流式回调示例

```python
import spacemit_vad
import numpy as np

# 创建回调
callback = spacemit_vad.VadCallback()
callback.on_speech_start(lambda ts: print(f"语音开始: {ts}ms"))
callback.on_speech_end(lambda ts, dur: print(f"语音结束: {ts}ms, 持续: {dur}ms"))
callback.on_event(lambda r: print(f"概率: {r.probability:.3f}"))

# 创建引擎并设置回调
engine = spacemit_vad.VadEngine()
engine.set_callback(callback)

# 流式检测
engine.start()
for i in range(100):
    audio = np.random.randn(512).astype(np.float32) * 0.1
    engine.send_audio_frame(audio)
engine.stop()
```

---

## 数据格式

- **采样率**: 16000 Hz
- **声道**: 单声道 (mono)
- **窗口大小**: 512 样本 (32ms @ 16kHz)
- **格式**:
  - C++: `std::vector<float>` 或 `std::vector<int16_t>`
  - Python: `np.ndarray` (float32 或 int16)
- **范围**:
  - float32: [-1.0, 1.0]
  - int16: [-32768, 32767]

```python
# PCM16 bytes -> float32
audio = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32) / 32768.0

# float32 -> PCM16 bytes
pcm_bytes = (audio * 32767).astype(np.int16).tobytes()
```
