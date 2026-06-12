#!/usr/bin/env python3
"""
Yaw 搜索基准测试入口脚本。

用法:
    python3 Debug/run_yaw_search_benchmark.py --video /path/to/video.mp4
    python3 Debug/run_yaw_search_benchmark.py --video v1.mp4 --video v2.mp4 --build --force

退出码:
    0  成功并找到合格参数
    2  参数、视频或环境无效
    3  Test 运行失败或没有有效样本
    4  分析成功，但没有参数通过门槛
    5  CSV 缺列、版本错误或结果不完整
"""

import argparse
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_REPEATS = 3
DEFAULT_STEPS_DEG = [0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0]
DEFAULT_ITERATIONS = [5, 8, 10, 12, 15]


def repo_root() -> Path:
    """从脚本路径定位仓库根目录。"""
    return Path(__file__).resolve().parents[1]


def sha256_file(path: str) -> str:
    """计算视频文件的 SHA256。"""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def fmt_array(arr) -> str:
    """将 Python 列表格式化为 ROS 2 参数数组字符串。"""
    return "[" + ",".join(str(x) for x in arr) + "]"


def validate_videos(args) -> None:
    """校验视频文件存在（其他环境校验在构建之后进行）。"""
    if not args.video:
        print("错误：至少指定一个 --video", file=sys.stderr)
        sys.exit(2)

    for v in args.video:
        if not os.path.isfile(v):
            print(f"错误：视频不存在: {v}", file=sys.stderr)
            sys.exit(2)


def validate_build_artifacts() -> None:
    """构建后校验 setup.bash、Test 可执行文件以及 ROS 环境。"""
    root = repo_root()

    setup_bash = root / "install" / "setup.bash"
    if not setup_bash.exists():
        print(f"错误：找不到 {setup_bash}，请先构建工作空间", file=sys.stderr)
        sys.exit(2)

    test_exe = root / "install" / "armor_plate_identification" / "lib" / "armor_plate_identification" / "Test"
    if not test_exe.exists():
        print(f"错误：找不到 Test 可执行文件: {test_exe}", file=sys.stderr)
        sys.exit(2)

    if not os.environ.get("AMENT_PREFIX_PATH"):
        print("错误：AMENT_PREFIX_PATH 未设置，请 source install/setup.bash 或用 --build",
              file=sys.stderr)
        sys.exit(2)


BUILD_PACKAGES = [
    "armor_plate_interfaces",
    "armor_plate_common",
    "armor_plate_identification",
]


def build_workspace(root: Path) -> None:
    """调用 build_workspace.sh 构建 identification 相关包。"""
    script = root / "build_workspace.sh"
    if not script.exists():
        print(f"错误：找不到构建脚本 {script}", file=sys.stderr)
        sys.exit(2)
    cmd = [str(script), "--packages-select"] + BUILD_PACKAGES
    print(f"[INFO] 开始构建: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=str(root))
    if result.returncode != 0:
        print("错误：工作空间构建失败", file=sys.stderr)
        sys.exit(3)


def capture_sourced_env(root: Path) -> dict:
    """source install/setup.bash 并捕获环境变量。"""
    cmd = f'source "{root}/install/setup.bash" >/dev/null 2>&1 && env -0'
    result = subprocess.run(
        ["bash", "-c", cmd],
        cwd=str(root),
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print("错误：无法 source install/setup.bash", file=sys.stderr)
        sys.exit(3)

    env = dict(os.environ)
    for item in result.stdout.split("\0"):
        if "=" in item:
            key, value = item.split("=", 1)
            env[key] = value
    return env


def add_openvino_library_path(env: dict) -> dict:
    """如果检测到 OpenVINO 运行时目录，将其加入 LD_LIBRARY_PATH。"""
    candidates = []
    user = os.environ.get("USER", "")
    if user:
        candidates.append(f"/home/{user}/intel/openvino_2026.1.0/runtime/lib/intel64")
    candidates.extend([
        "/opt/intel/openvino_2026/runtime/lib/intel64",
        "/opt/intel/openvino_2025/runtime/lib/intel64",
        "/opt/intel/openvino/runtime/lib/intel64",
    ])
    ov_dir = os.environ.get("INTEL_OPENVINO_DIR")
    if ov_dir:
        candidates.insert(0, os.path.join(ov_dir, "runtime", "lib", "intel64"))

    for cand in candidates:
        if os.path.isdir(cand):
            old = env.get("LD_LIBRARY_PATH", "")
            if old:
                env["LD_LIBRARY_PATH"] = f"{cand}:{old}"
            else:
                env["LD_LIBRARY_PATH"] = cand
            break
    return env


def git_info(root: Path):
    """返回 (commit, dirty)。"""
    try:
        commit = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=False,
        ).stdout.strip()
        dirty_out = subprocess.run(
            ["git", "-C", str(root), "status", "--porcelain"],
            capture_output=True, text=True, check=False,
        ).stdout.strip()
        dirty = bool(dirty_out)
    except Exception:
        commit = "unknown"
        dirty = None
    return commit, dirty


def get_cpu_model() -> str:
    """读取 /proc/cpuinfo 获取 CPU 型号。"""
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except Exception:
        pass
    return "unknown"


def get_build_type(root: Path) -> str:
    """从 CMakeCache.txt 读取 CMAKE_BUILD_TYPE。"""
    cache = root / "build" / "armor_plate_identification" / "CMakeCache.txt"
    if not cache.exists():
        return "unknown"
    try:
        with open(cache) as f:
            for line in f:
                if line.startswith("CMAKE_BUILD_TYPE:"):
                    parts = line.split("=", 1)
                    if len(parts) == 2:
                        return parts[1].strip()
    except Exception:
        pass
    return "unknown"


def get_compiler_version() -> str:
    """获取 C++ 编译器版本第一行。"""
    try:
        out = subprocess.run(
            ["c++", "--version"],
            capture_output=True, text=True, check=False,
        ).stdout.strip()
        if out:
            return out.splitlines()[0].strip()
    except Exception:
        pass
    return "unknown"


def make_run_id() -> str:
    """生成运行 ID（时间戳）。"""
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def video_output_dir(run_dir: Path, video_path: str, sha256: str) -> Path:
    """每个视频使用“文件名 + SHA256 前 8 位”作为输出目录名。"""
    stem = Path(video_path).stem
    return run_dir / f"{stem}_{sha256[:8]}"


def run_test_once(env: dict, root: Path, video_path: str, csv_path: Path) -> int:
    """启动一次 Test 节点执行 Yaw 搜索基准。返回进程退出码。"""
    exe = str(root / "install" / "armor_plate_identification" / "lib" / "armor_plate_identification" / "Test")
    cmd = [
        exe, os.path.abspath(video_path), "--ros-args",
        "-p", "headless:=true",
        "-p", "debug_timecontrol:=false",
        "-p", "delay_time:=0",
        "-p", "debug_frame:=false",
        "-p", "debug_frame_count:=0",
        "-p", "yaw_benchmark_enabled:=true",
        "-p", f"yaw_benchmark_output_csv:={csv_path}",
        "-p", f"yaw_benchmark_steps_deg:={fmt_array(DEFAULT_STEPS_DEG)}",
        "-p", f"yaw_benchmark_iterations:={fmt_array(DEFAULT_ITERATIONS)}",
    ]
    print(f"[INFO] 启动: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=str(root), env=env)
    return result.returncode


def run_analyzer(*video_dirs: Path, output_dir: Path = None) -> int:
    """调用 analyze_yaw_search.py。返回进程退出码。"""
    analyzer = repo_root() / "Debug" / "analyze_yaw_search.py"
    cmd = ["python3", str(analyzer)]
    if output_dir is not None:
        cmd.extend(["--output-dir", str(output_dir)])
    for d in video_dirs:
        cmd.append(str(d))
    print(f"[INFO] 分析: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=str(repo_root()))
    return result.returncode


def collect_video_results(run_dir: Path, video_infos: list) -> dict:
    """汇总各视频的分析结果与整体推荐。"""
    overall = {}
    overall_path = run_dir / "recommendation.json"
    if overall_path.exists():
        try:
            with open(overall_path) as f:
                overall = json.load(f)
        except Exception:
            pass
    return {
        "per_video": video_infos,
        "overall": overall,
    }


def write_run_json(run_dir: Path, args, video_infos: list, overall_result: dict,
                   env: dict, exit_code: int) -> None:
    """写入 run.json 元数据。"""
    root = repo_root()
    commit, dirty = git_info(root)
    run_json = {
        "schema_version": 1,
        "timestamp": datetime.datetime.now().isoformat(),
        "command_line": sys.argv,
        "exit_code": exit_code,
        "git_commit": commit,
        "git_dirty": dirty,
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "cpu_model": get_cpu_model(),
        "build_type": get_build_type(root),
        "compiler_version": get_compiler_version(),
        "parameter_grid": {
            "steps_deg": DEFAULT_STEPS_DEG,
            "iterations": DEFAULT_ITERATIONS,
            "sample_stride": 5,
            "max_samples": 2000,
            "warmup_samples": 50,
        },
        "forced_params": {
            "headless": True,
            "debug_timecontrol": False,
            "delay_time": 0,
            "debug_frame": False,
            "debug_frame_count": 0,
            "yaw_benchmark_enabled": True,
        },
        "videos": video_infos,
        "overall_result": overall_result,
    }
    with open(run_dir / "run.json", "w") as f:
        json.dump(run_json, f, indent=2, ensure_ascii=False)


def main():
    parser = argparse.ArgumentParser(
        description="Yaw 搜索自动化调参入口",
    )
    parser.add_argument(
        "--video", action="append", required=True,
        help="待测视频路径（可多次指定）",
    )
    parser.add_argument(
        "--build", action="store_true",
        help="先构建工作空间再运行测试",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="强制覆盖已存在的运行目录",
    )
    parser.add_argument(
        "--repeats", type=int, default=DEFAULT_REPEATS,
        help=f"每个视频的重复运行次数（默认 {DEFAULT_REPEATS}）",
    )
    parser.add_argument(
        "--run-id", default=None,
        help="自定义运行 ID（默认使用时间戳）",
    )
    args = parser.parse_args()

    if args.repeats < 1:
        print("错误：--repeats 必须 >= 1", file=sys.stderr)
        sys.exit(2)

    # 校验视频存在（不依赖构建状态）
    validate_videos(args)
    root = repo_root()

    # 构建（如果需要）
    if args.build:
        build_workspace(root)

    # 捕获环境：source install/setup.bash（确保 LD_LIBRARY_PATH 等正确设置）
    env = capture_sourced_env(root)
    env = add_openvino_library_path(env)

    # 校验构建产物和环境变量
    # 临时设置环境以通过 validate_build_artifacts 的 AMENT_PREFIX_PATH 检查
    os.environ.update(env)
    validate_build_artifacts()

    run_id = args.run_id or make_run_id()
    run_dir = root / "Debug" / "YawSearch" / "runs" / run_id
    if run_dir.exists():
        if args.force:
            shutil.rmtree(run_dir)
        else:
            print(f"错误：运行目录已存在: {run_dir}，使用 --force 覆盖",
                  file=sys.stderr)
            sys.exit(2)
    run_dir.mkdir(parents=True, exist_ok=True)

    video_infos = []
    per_video_codes = []

    for video_path in args.video:
        sha256 = sha256_file(video_path)
        vid_dir = video_output_dir(run_dir, video_path, sha256)
        vid_dir.mkdir(parents=True, exist_ok=True)

        video_info = {
            "path": os.path.abspath(video_path),
            "sha256": sha256,
            "output_dir": str(vid_dir),
            "repeats": [],
            "analyzer_exit_code": None,
        }

        repeat_csvs = []
        for r in range(args.repeats):
            csv_path = vid_dir / f"repeat_{r}.csv"
            video_info["repeats"].append(str(csv_path))
            code = run_test_once(env, root, video_path, csv_path)
            repeat_csvs.append(csv_path)
            if code != 0:
                print(f"[ERROR] Test 运行失败，退出码 {code}: {video_path} repeat_{r}",
                      file=sys.stderr)
                video_info["test_exit_code"] = code
                per_video_codes.append(3)
                break
        else:
            video_info["test_exit_code"] = 0

        if video_info.get("test_exit_code", 0) != 0:
            video_infos.append(video_info)
            continue

        # 校验 CSV 文件已生成
        missing_csv = [str(c) for c in repeat_csvs if not c.exists()]
        if missing_csv:
            print(f"[ERROR] 缺少 CSV 输出: {missing_csv}", file=sys.stderr)
            video_info["analyzer_exit_code"] = 5
            per_video_codes.append(5)
            video_infos.append(video_info)
            continue

        code = run_analyzer(vid_dir)
        video_info["analyzer_exit_code"] = code
        per_video_codes.append(code)
        video_infos.append(video_info)

    # 整体分析（跨视频）
    overall_code = None
    if len(video_infos) >= 1 and all(
        v.get("test_exit_code", 0) == 0 and v.get("analyzer_exit_code") in (0, 4)
        for v in video_infos
    ):
        overall_code = run_analyzer(
            *[Path(v["output_dir"]) for v in video_infos],
            output_dir=run_dir,
        )

    # 计算最终退出码
    final_code = 0
    if any(c == 5 for c in per_video_codes):
        final_code = 5
    elif any(c == 3 for c in per_video_codes):
        final_code = 3
    elif overall_code is not None:
        final_code = overall_code
    elif any(c == 4 for c in per_video_codes):
        final_code = 4

    overall_result = collect_video_results(run_dir, video_infos)
    write_run_json(run_dir, args, video_infos, overall_result, env, final_code)

    print(f"[INFO] 运行结果保存至: {run_dir}")
    print(f"[INFO] 最终退出码: {final_code}")
    return final_code


if __name__ == "__main__":
    sys.exit(main())
