#!/usr/bin/env bash
# Build native K1 binaries and Python 3.12 modules from the stable sources.
set -eo pipefail
ROBOT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
SDK_ROOT=${SROBOTIS_ROOT:-/data/spacemit_robot}
test "$(uname -m)" = riscv64
test -f "$ROBOT_ROOT/runtime/k1/environment/rvclaw-vendor-command.service.env"
test -d "$SDK_ROOT/application"
test -d "$SDK_ROOT/target"
test -d "$ROBOT_ROOT/robot/vendor/native/ncnn"

export SROBOTIS_THIRDPARTY_CACHE="$ROBOT_ROOT/robot/vendor/native/thirdparty"
mkdir -p "$HOME/Downloads"
install -m 644 "$SROBOTIS_THIRDPARTY_CACHE/kissfft-febd4caeed32e33ad8b2e0bb5ea77542c40f18ec.zip" "$HOME/Downloads/"
cd "$SDK_ROOT"
source build/envsetup.sh
export AUTO_INSTALL_DEPS=no
cd "$SDK_ROOT/components/multimedia/audio"
mm -j4 -DAUDIO_BUILD_EXAMPLES=OFF -DAUDIO_BUILD_PYTHON=ON \
  -DFETCHCONTENT_SOURCE_DIR_PORTAUDIO="$ROBOT_ROOT/robot/vendor/native/portaudio"
cd "$SDK_ROOT/components/model_zoo/asr"
mm -j4 -DBUILD_STREAM_DEMO=ON -DBUILD_PYTHON_BINDINGS=ON -DASR_MODEL_FETCH_OFF=ON
cd "$SDK_ROOT/components/model_zoo/vad"
mm -j4 -DBUILD_VAD_EXAMPLES=OFF -DBUILD_VAD_PYTHON=OFF
vad_build="$SDK_ROOT/output/build/cmake/pkgs/components_model_zoo_vad"
cmake -S "$SDK_ROOT/components/model_zoo/vad" -B "$vad_build" \
  -DBUILD_VAD_EXAMPLES=OFF -DBUILD_VAD_PYTHON=ON
cmake --build "$vad_build" --target _spacemit_vad --parallel 4
mkdir -p "$SDK_ROOT/output/staging/lib/python/spacemit_vad"
install -m 644 "$SDK_ROOT/components/model_zoo/vad/python/spacemit_vad/"*.py "$SDK_ROOT/output/staging/lib/python/spacemit_vad/"
vad_extension=$(find "$SDK_ROOT/output/build/cmake/pkgs/components_model_zoo_vad" -type f -name '_spacemit_vad.cpython-312-*.so' -print -quit)
test -n "$vad_extension"
install -m 755 "$vad_extension" "$SDK_ROOT/output/staging/lib/python/spacemit_vad/"
cd "$SDK_ROOT/components/model_zoo/vision"
mm -j4 -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_PYTHON_WHEEL=OFF -DSPACEMIT_DIR=/usr

compiler_root="$ROBOT_ROOT/build/toolchain/gcc14"
test -x "$compiler_root/usr/bin/g++-14"
cmake -S "$ROBOT_ROOT/robot/vendor/native/ncnn" -B "$ROBOT_ROOT/build/ncnn-gcc14" \
  -DCMAKE_C_COMPILER="$compiler_root/usr/bin/gcc-14" \
  -DCMAKE_CXX_COMPILER="$compiler_root/usr/bin/g++-14" \
  -DCMAKE_BUILD_TYPE=Release -DNCNN_VERSION=20260526 \
  -DNCNN_VULKAN=OFF -DNCNN_RVV=ON -DNCNN_XTHEADVECTOR=OFF \
  -DNCNN_ZFH=OFF -DNCNN_ZVFH=OFF -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
  -DNCNN_PYTHON=ON -DNCNN_SYSTEM_PYBIND11=ON \
  -DOpenMP_gomp_LIBRARY=/usr/lib/riscv64-linux-gnu/libgomp.so.1 \
  -DNCNN_BUILD_TESTS=OFF -DNCNN_BUILD_TOOLS=OFF -DNCNN_BUILD_EXAMPLES=OFF \
  -DNCNN_BUILD_BENCHMARK=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPython_EXECUTABLE=/usr/bin/python3
cmake --build "$ROBOT_ROOT/build/ncnn-gcc14" --target pyncnn --parallel 6
mkdir -p "$ROBOT_ROOT/runtime/k1/pylib/ncnn"
install -m 644 "$ROBOT_ROOT/robot/vendor/native/ncnn/python/ncnn/__init__.py" "$ROBOT_ROOT/runtime/k1/pylib/ncnn/"
install -m 755 "$ROBOT_ROOT/build/ncnn-gcc14/python/ncnn/"*.so "$ROBOT_ROOT/runtime/k1/pylib/ncnn/"
bash "$ROBOT_ROOT/deploy/k1/build_vision_native.sh"
echo 'K1 native components built; no robot service was started.'
