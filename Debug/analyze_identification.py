#!/usr/bin/env python3
"""
PnP 解算质量分析 — 可视化世界坐标系下的观测轨迹，最小二乘拟合旋转圆。
用法: python3 Debug/analyze_identification.py
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import math
import argparse


# ============================================================
# 1. 数据加载
# ============================================================

def load_log(path):
    """加载 tracker_log.txt，返回 dict of numpy arrays"""
    result = {}
    with open(path, "r") as f:
        header = f.readline().strip().split()
        rows = []
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 18:
                rows.append(parts)
    if not rows:
        return None

    result["sec"] = np.array([int(r[0]) for r in rows])
    result["nanosec"] = np.array([int(r[1]) for r in rows])
    result["timestamp"] = result["sec"] + result["nanosec"] * 1e-9
    result["target_world_x"] = np.array([float(r[2]) for r in rows])
    result["target_world_y"] = np.array([float(r[3]) for r in rows])
    result["target_world_z"] = np.array([float(r[4]) for r in rows])
    result["raw_yaw"] = np.array([float(r[8]) for r in rows])
    result["solve_ok"] = np.array([int(r[16]) for r in rows])
    result["method"] = [r[15] for r in rows]
    return result


# ============================================================
# 2. 最小二乘拟合圆
# ============================================================

def fit_circle_least_squares(x, y):
    """
    最小二乘法拟合圆: (x - cx)^2 + (y - cy)^2 = r^2
    展开: x^2 + y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)
    令 A = 2*cx, B = 2*cy, C = r^2 - cx^2 - cy^2
    则: A*x + B*y + C = x^2 + y^2
    返回 (cx, cy, r)
    """
    n = len(x)
    if n < 3:
        return None, None, None

    # 构建线性方程组: [x, y, 1] * [A, B, C]^T = x^2 + y^2
    A_mat = np.column_stack([x, y, np.ones(n)])
    b = x**2 + y**2

    # 最小二乘求解
    result, residuals, rank, sv = np.linalg.lstsq(A_mat, b, rcond=None)
    A, B, C = result

    cx = A / 2.0
    cy = B / 2.0
    r = np.sqrt(C + cx**2 + cy**2)

    return cx, cy, r


def compute_fit_quality(x, y, cx, cy, r):
    """计算拟合质量: 残差统计"""
    dist = np.sqrt((x - cx)**2 + (y - cy)**2)
    residual = dist - r
    return {
        "mean": np.mean(np.abs(residual)),
        "std": np.std(residual),
        "max": np.max(np.abs(residual)),
        "median": np.median(np.abs(residual)),
    }


# ============================================================
# 3. 可视化
# ============================================================

def plot_observation(method, data, output_dir):
    """
    绘制观测轨迹 + 最小二乘拟合圆
    """
    # 只用 solve_ok=1 的有效帧
    valid = data["solve_ok"] == 1
    if not np.any(valid):
        print(f"  [{method}] 无有效观测帧，跳过")
        return

    x = data["target_world_x"][valid]
    y = data["target_world_y"][valid]
    yaw = data["raw_yaw"][valid]
    t = data["timestamp"][valid] - data["timestamp"][0]

    # 最小二乘拟合圆
    cx, cy, r = fit_circle_least_squares(x, y)
    if cx is None:
        print(f"  [{method}] 数据点不足，无法拟合")
        return

    quality = compute_fit_quality(x, y, cx, cy, r)

    # --- 图 1: 观测轨迹 + 拟合圆 ---
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    # 按 yaw 着色
    scatter = ax.scatter(x, y, c=yaw, cmap="hsv", s=12, alpha=0.7, zorder=3)
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
    cbar.set_label("yaw (rad)")

    # 拟合圆
    theta = np.linspace(0, 2 * np.pi, 200)
    circle_x = cx + r * np.cos(theta)
    circle_y = cy + r * np.sin(theta)
    ax.plot(circle_x, circle_y, "k--", linewidth=1.5, alpha=0.6,
            label=f"fitted circle r={r:.4f}m")

    # 圆心
    ax.scatter([cx], [cy], c="red", s=120, marker="x", linewidths=3, zorder=5,
               label=f"center ({cx:.4f}, {cy:.4f})")

    # 每个观测点到圆心的连线（采样画，避免太密）
    step = max(1, len(x) // 30)
    for i in range(0, len(x), step):
        ax.plot([x[i], cx], [y[i], cy], "gray", linewidth=0.3, alpha=0.3)

    ax.set_xlabel("X world (m)")
    ax.set_ylabel("Y world (m)")
    ax.set_title(f"[{method}] Observation Trajectory & LS Circle Fit\n"
                 f"center=({cx:.4f}, {cy:.4f})  r={r:.4f}m  "
                 f"residual: mean={quality['mean']:.4f} std={quality['std']:.4f}m")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")
    fig.tight_layout()

    out_path = os.path.join(output_dir, f"{method}_observation_circle.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  [{method}] Saved {out_path}")

    # --- 图 2: 残差分布 ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    dist = np.sqrt((x - cx)**2 + (y - cy)**2)
    residual = dist - r

    axes[0].plot(t, residual, "b.-", markersize=2)
    axes[0].axhline(0, color="k", linestyle="-", alpha=0.3)
    axes[0].axhline(quality["mean"], color="r", linestyle="--", alpha=0.5,
                     label=f"mean |res|={quality['mean']:.4f}")
    axes[0].axhline(-quality["mean"], color="r", linestyle="--", alpha=0.5)
    axes[0].set_ylabel("residual (m)")
    axes[0].set_title(f"[{method}] Circle Fit Residual (dist - r)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, dist, "g.-", markersize=2, label="dist to center")
    axes[1].axhline(r, color="r", linestyle="--", alpha=0.5, label=f"r={r:.4f}")
    axes[1].set_ylabel("distance (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title(f"[{method}] Distance to Fitted Center")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    out_path2 = os.path.join(output_dir, f"{method}_residual.png")
    fig.savefig(out_path2, dpi=150)
    plt.close(fig)
    print(f"  [{method}] Saved {out_path2}")

    return {"cx": cx, "cy": cy, "r": r, "quality": quality, "n_frames": len(x)}


# ============================================================
# 4. 统计摘要
# ============================================================

def print_summary(method, info):
    q = info["quality"]
    print(f"\n{'='*50}")
    print(f"  Method: {method}")
    print(f"{'='*50}")
    print(f"  Valid frames:     {info['n_frames']}")
    print(f"  Center:           ({info['cx']:.4f}, {info['cy']:.4f})")
    print(f"  Radius:           {info['r']:.4f} m")
    print(f"  Residual mean:    {q['mean']:.4f} m")
    print(f"  Residual std:     {q['std']:.4f} m")
    print(f"  Residual max:     {q['max']:.4f} m")
    print(f"  Residual median:  {q['median']:.4f} m")


# ============================================================
# 5. Main
# ============================================================

def find_test_folders(base_dir):
    """自动发现 Tracker/ 下包含 method 子目录的测试文件夹"""
    methods = ["ekf", "ls", "ba"]
    tracker_dir = os.path.join(base_dir, "Tracker")
    test_folders = []
    if os.path.isdir(tracker_dir):
        for d in sorted(os.listdir(tracker_dir)):
            if os.path.isdir(os.path.join(tracker_dir, d)) and d not in methods:
                if any(os.path.isdir(os.path.join(tracker_dir, d, m)) for m in methods):
                    test_folders.append(d)
    return test_folders


def find_log_files(base_dir, test_name):
    """扫描 Debug/Tracker/{test_name}/{method}/ 下所有 .txt 文件，跳过 temp/ 目录"""
    methods = ["ekf", "ls", "ba"]
    tracker_dir = os.path.join(base_dir, "Tracker")
    result = {}
    for method in methods:
        method_dir = os.path.join(tracker_dir, test_name, method)
        if not os.path.isdir(method_dir):
            continue
        txt_files = glob.glob(os.path.join(method_dir, "**", "*.txt"), recursive=True)
        txt_files = [f for f in txt_files if "/temp/" not in f]
        if txt_files:
            result[method] = sorted(txt_files)
    return result


def main():
    parser = argparse.ArgumentParser(description="PnP 解算质量分析")
    parser.add_argument("--test", "-t", default=None,
                        help="测试文件夹名（视频文件名去掉扩展名，如 BlueSlow）。不指定则处理所有测试。")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = script_dir  # Debug/

    # 确定要处理的测试文件夹
    if args.test:
        test_names = [args.test]
    else:
        test_names = find_test_folders(base_dir)
        if not test_names:
            print("No test folders found in Debug/Tracker/ (need {test}/{ekf,ls,ba}/ structure)")
            return
        print(f"Found test folders: {test_names}")

    for test_name in test_names:
        print(f"\n{'#'*60}")
        print(f"  Test: {test_name}")
        print(f"{'#'*60}")

        output_dir = os.path.join(base_dir, "Picture", "identification", test_name)
        os.makedirs(output_dir, exist_ok=True)

        log_files = find_log_files(base_dir, test_name)
        if not log_files:
            print(f"  No log files found in Debug/Tracker/{test_name}/{{ekf,ls,ba}}/")
            continue

        all_info = {}

        for method, files in log_files.items():
            print(f"\n  Processing {method}: {len(files)} file(s)")

            for fpath in files:
                fname = os.path.splitext(os.path.basename(fpath))[0]
                print(f"    Loading {fpath}...")

                data = load_log(fpath)
                if data is None:
                    print(f"    Skipped (empty)")
                    continue

                info = plot_observation(method, data, output_dir)
                if info:
                    print_summary(method, info)
                    all_info[method] = info

        # 多方法对比
        if len(all_info) > 1:
            print(f"\n  {'='*50}")
            print("  Cross-method comparison")
            print(f"  {'='*50}")
            for method, info in all_info.items():
                q = info["quality"]
                print(f"  {method:4s}  center=({info['cx']:.4f},{info['cy']:.4f})  "
                      f"r={info['r']:.4f}  res_mean={q['mean']:.4f}m  "
                      f"res_std={q['std']:.4f}m  frames={info['n_frames']}")

    print("\nDone.")


if __name__ == "__main__":
    main()
