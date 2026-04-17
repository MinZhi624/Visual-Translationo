#!/bin/bash
# 包装 colcon build，编译后自动恢复完整的 compile_commands.json

set -e

echo "=== Building workspace ==="
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"

echo "=== Merging compile_commands.json ==="
python3 merge_compile_commands.py

echo "=== Done ==="
