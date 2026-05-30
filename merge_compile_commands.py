#!/usr/bin/env python3
"""
合并 colcon build 生成的各包 compile_commands.json 到 build/compile_commands.json。
供 clangd 使用。
"""

import json
import os
from pathlib import Path


def main():
    workspace_dir = Path(__file__).parent.resolve()
    build_dir = workspace_dir / "build"
    output_file = build_dir / "compile_commands.json"

    if not build_dir.exists():
        print(f"[ERROR] Build directory not found: {build_dir}")
        return 1

    all_commands = []

    # 遍历 build 下的每个包目录，查找 compile_commands.json
    for pkg_dir in sorted(build_dir.iterdir()):
        if not pkg_dir.is_dir():
            continue
        compile_db = pkg_dir / "compile_commands.json"
        if compile_db.exists():
            try:
                with open(compile_db, "r", encoding="utf-8") as f:
                    commands = json.load(f)
                # 过滤：只保留 src/ 下的手写源码，排除 build/ 生成的中间文件
                # (rosidl 生成代码、CMake 自动生成的 .cpp 等会严重拖慢 clangd)
                filtered = [c for c in commands if "/src/" in c.get("file", "")]
                all_commands.extend(filtered)
                print(f"[INFO] Merged {len(filtered)}/{len(commands)} entries from {compile_db}")
            except (json.JSONDecodeError, OSError) as e:
                print(f"[WARN] Failed to read {compile_db}: {e}")

    if not all_commands:
        print("[WARN] No compile_commands.json entries left after filtering.")
        print("       Did you build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON?")
        return 1

    # 去重：基于 "file" 字段
    seen = {}
    unique_commands = []
    for entry in all_commands:
        file_path = entry.get("file")
        if file_path and file_path not in seen:
            seen[file_path] = True
            unique_commands.append(entry)

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(unique_commands, f, indent=2, ensure_ascii=False)

    print(f"[INFO] Written {len(unique_commands)} unique entries to {output_file}")
    return 0


if __name__ == "__main__":
    exit(main())
