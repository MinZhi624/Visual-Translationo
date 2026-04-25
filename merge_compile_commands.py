#!/usr/bin/env python3
"""
Merge all per-package compile_commands.json under build/ into the top-level
build/compile_commands.json. Run this after a partial colcon build to fix clangd.
"""
import json
import os
from pathlib import Path

BUILD_DIR = Path("build")
OUTPUT = BUILD_DIR / "compile_commands.json"

merged = []
for subdir in sorted(BUILD_DIR.iterdir()):
    cc = subdir / "compile_commands.json"
    if cc.is_file():
        with open(cc, "r") as f:
            merged.extend(json.load(f))

with open(OUTPUT, "w") as f:
    json.dump(merged, f, indent=2)

print(f"Merged {len(merged)} entries into {OUTPUT}")
