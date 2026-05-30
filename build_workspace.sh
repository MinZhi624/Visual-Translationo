#!/bin/bash
set -e

# 获取脚本所在目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS 2 环境（自动检测已 source 的 ROS distro）
if [ -z "$ROS_DISTRO" ]; then
    if [ -d "/opt/ros/jazzy" ]; then
        source /opt/ros/jazzy/setup.bash
        echo "[INFO] Sourced ROS 2 Jazzy"
    elif [ -d "/opt/ros/humble" ]; then
        source /opt/ros/humble/setup.bash
        echo "[INFO] Sourced ROS 2 Humble"
    else
        echo "[ERROR] No ROS 2 installation found in /opt/ros/"
        exit 1
    fi
else
    echo "[INFO] Using existing ROS 2 environment: $ROS_DISTRO"
fi

# 构建全部包（导出 compile_commands.json）
echo "[INFO] Building workspace..."
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"

# 合并 compile_commands.json 供 clangd 使用
echo "[INFO] Merging compile_commands.json..."
python3 "$SCRIPT_DIR/merge_compile_commands.py"

# Source 工作空间 + OpenVINO 库路径
echo "[INFO] Sourcing workspace..."
source "$SCRIPT_DIR/install/setup.bash"

# OpenVINO 运行时库（如果已安装）
OPENVINO_DIR="/home/$USER/intel/openvino_2026.1.0/runtime/lib/intel64"
if [ -d "$OPENVINO_DIR" ]; then
    export LD_LIBRARY_PATH="$OPENVINO_DIR:$LD_LIBRARY_PATH"
    echo "[INFO] OpenVINO LD_LIBRARY_PATH set"
fi

echo "[INFO] Done. Workspace is ready."
