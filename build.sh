#!/usr/bin/env bash
set -euo pipefail

DEPLOY_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<'EOF'
用法：bash build.sh [all|real|mujoco] [CMake 参数...]

  all     编译 Go2 仿真和真机（默认），输出到 cmake_build
  real    编译 Go2 真机，包含键盘速度控制，输出到 cmake_build_keyboard
  mujoco  编译 Go2 MuJoCo 仿真，输出到 cmake_build_mujoco

默认启用 Torch 和 ONNX；可用 CMake 参数覆盖，例如：
  bash build.sh real -DUSE_TORCH=OFF -DUSE_ONNX=ON

环境变量：BUILD_JOBS（默认 4）、BUILD_DIR（默认使用对应模式目录）。
BUILD_DIR 为相对路径时，以项目根目录为基准。
EOF
}

DEPLOY_BUILD_MODE=all
case "${1:-}" in
    all|real|mujoco)
        DEPLOY_BUILD_MODE="$1"
        shift
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    ""|-*) ;;
    *)
        printf '未知编译模式：%s\n' "$1" >&2
        usage >&2
        exit 2
        ;;
esac

DEPLOY_USE_MUJOCO=ON
DEPLOY_BUILD_GO2_REAL=ON
DEPLOY_DEFAULT_BUILD_DIR=cmake_build
case "$DEPLOY_BUILD_MODE" in
    real)
        DEPLOY_USE_MUJOCO=OFF
        DEPLOY_DEFAULT_BUILD_DIR=cmake_build_keyboard
        ;;
    mujoco)
        DEPLOY_BUILD_GO2_REAL=OFF
        DEPLOY_DEFAULT_BUILD_DIR=cmake_build_mujoco
        ;;
esac

DEPLOY_BUILD_DIR="${BUILD_DIR:-$DEPLOY_DEFAULT_BUILD_DIR}"
if [[ "$DEPLOY_BUILD_DIR" != /* ]]; then
    DEPLOY_BUILD_DIR="$DEPLOY_PROJECT_ROOT/$DEPLOY_BUILD_DIR"
fi
DEPLOY_BUILD_JOBS="${BUILD_JOBS:-4}"
if [[ ! "$DEPLOY_BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then
    printf 'BUILD_JOBS 必须是正整数：%s\n' "$DEPLOY_BUILD_JOBS" >&2
    exit 2
fi

cmake -S "$DEPLOY_PROJECT_ROOT/src/legged_deploy" \
      -B "$DEPLOY_BUILD_DIR" \
      -DUSE_CMAKE=ON \
      -DUSE_MUJOCO="$DEPLOY_USE_MUJOCO" \
      -DBUILD_GO2_REAL="$DEPLOY_BUILD_GO2_REAL" \
      -DUSE_TORCH=ON \
      -DUSE_ONNX=ON \
      -DCMAKE_BUILD_TYPE=Release \
      "$@"
cmake --build "$DEPLOY_BUILD_DIR" --parallel "$DEPLOY_BUILD_JOBS"
printf '\n编译完成，输出目录：%s/bin\n' "$DEPLOY_BUILD_DIR"
