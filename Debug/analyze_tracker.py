#!/usr/bin/env python3
"""
Tracker Debug 数据分析与可视化
用法: python3 Debug/analyze_tracker.py
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

COLUMNS = [
    "sec", "nanosec",
    "target_world_x", "target_world_y", "target_world_z",
    "filtered_world_x", "filtered_world_y", "filtered_world_z",
    "raw_yaw", "filter_yaw",
    "center_x", "center_y", "center_r",
    "center_vx", "center_vy",
    "method", "solve_ok", "time_cost"
]

NUM_COLS = [c for c in COLUMNS if c not in ("method",)]


def load_log(path):
    """加载单个 tracker_log.txt，返回 dict of numpy arrays"""
    data = {c: [] for c in COLUMNS}
    with open(path, "r") as f:
        header = f.readline()  # skip header
        for line in f:
            parts = line.strip().split()
            if len(parts) < 18:
                continue
            data["sec"].append(int(parts[0]))
            data["nanosec"].append(int(parts[1]))
            for i, col in enumerate(COLUMNS[2:]):
                if col == "method":
                    data[col].append(parts[15])
                elif col == "solve_ok":
                    data[col].append(int(parts[16]))
                elif col == "time_cost":
                    data[col].append(float(parts[17]))
                else:
                    data[col].append(float(parts[15 + i - 15]))
    # 更简洁的解析
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
    result["filtered_world_x"] = np.array([float(r[5]) for r in rows])
    result["filtered_world_y"] = np.array([float(r[6]) for r in rows])
    result["filtered_world_z"] = np.array([float(r[7]) for r in rows])
    result["raw_yaw"] = np.array([float(r[8]) for r in rows])
    result["filter_yaw"] = np.array([float(r[9]) for r in rows])
    result["center_x"] = np.array([float(r[10]) for r in rows])
    result["center_y"] = np.array([float(r[11]) for r in rows])
    result["center_r"] = np.array([float(r[12]) for r in rows])
    result["center_vx"] = np.array([float(r[13]) for r in rows])
    result["center_vy"] = np.array([float(r[14]) for r in rows])
    result["method"] = [r[15] for r in rows]
    result["solve_ok"] = np.array([int(r[16]) for r in rows])
    result["time_cost"] = np.array([float(r[17]) for r in rows])
    return result


# ============================================================
# 2. 衍生指标计算
# ============================================================

MUTATION_THRESH = 0.1  # 突变阈值 (m)


def compute_metrics(data):
    """计算衍生指标，返回新 dict"""
    m = {}
    n = len(data["timestamp"])
    t = data["timestamp"]
    t_rel = t - t[0]  # 相对时间

    # --- 帧间差分 ---
    m["dx_raw"] = np.diff(data["target_world_x"], prepend=data["target_world_x"][0])
    m["dy_raw"] = np.diff(data["target_world_y"], prepend=data["target_world_y"][0])
    m["dx_flt"] = np.diff(data["filtered_world_x"], prepend=data["filtered_world_x"][0])
    m["dy_flt"] = np.diff(data["filtered_world_y"], prepend=data["filtered_world_y"][0])
    m["dyaw"] = np.diff(data["raw_yaw"], prepend=data["raw_yaw"][0])
    # wrap dyaw to [-pi, pi]
    m["dyaw"] = np.array([d - 2*math.pi if d > math.pi else d + 2*math.pi if d < -math.pi else d for d in m["dyaw"]])

    m["dpos_raw"] = np.sqrt(m["dx_raw"]**2 + m["dy_raw"]**2)
    m["dpos_flt"] = np.sqrt(m["dx_flt"]**2 + m["dy_flt"]**2)

    # --- 突变标记 ---
    m["mutation"] = (np.abs(m["dx_raw"]) > MUTATION_THRESH) | (np.abs(m["dy_raw"]) > MUTATION_THRESH)

    # --- 残差 (观测 vs center 模型) ---
    r = data["center_r"]
    yaw = data["raw_yaw"]
    # 当 center_r=0 时残差无意义，用 nan 标记
    valid_r = r > 0.01
    m["res_x"] = np.where(valid_r,
                          data["target_world_x"] - (data["center_x"] + r * np.sin(yaw)),
                          np.nan)
    m["res_y"] = np.where(valid_r,
                          data["target_world_y"] - (data["center_y"] - r * np.cos(yaw)),
                          np.nan)
    m["res_norm"] = np.sqrt(np.nan_to_num(m["res_x"])**2 + np.nan_to_num(m["res_y"])**2)
    m["res_norm"] = np.where(valid_r, m["res_norm"], np.nan)

    # --- 可观性 ---
    neg90 = -math.pi / 2.0
    m["yaw_from_neg90"] = np.abs(data["raw_yaw"] - neg90)

    # --- 滤波效果 ---
    valid = data["solve_ok"] == 1
    if np.any(valid):
        mean_tx = np.mean(data["target_world_x"][valid])
        mean_ty = np.mean(data["target_world_y"][valid])
        m["err_raw_x"] = data["target_world_x"] - mean_tx
        m["err_raw_y"] = data["target_world_y"] - mean_ty
        m["err_flt_x"] = data["filtered_world_x"] - mean_tx
        m["err_flt_y"] = data["filtered_world_y"] - mean_ty
        std_raw = np.std(data["target_world_x"][valid])
        std_flt = np.std(data["filtered_world_x"][valid])
        m["filter_ratio_x"] = std_flt / std_raw if std_raw > 0 else np.nan
        std_raw_y = np.std(data["target_world_y"][valid])
        std_flt_y = np.std(data["filtered_world_y"][valid])
        m["filter_ratio_y"] = std_flt_y / std_raw_y if std_raw_y > 0 else np.nan
    else:
        m["err_raw_x"] = np.zeros(n)
        m["err_raw_y"] = np.zeros(n)
        m["err_flt_x"] = np.zeros(n)
        m["err_flt_y"] = np.zeros(n)
        m["filter_ratio_x"] = np.nan
        m["filter_ratio_y"] = np.nan

    # --- 中心稳定性 ---
    m["center_x_std"] = np.std(data["center_x"])
    m["center_y_std"] = np.std(data["center_y"])

    # --- solve_ok 统计 ---
    m["solve_rate"] = np.mean(data["solve_ok"])
    # 连续失败最大长度
    max_fail = 0
    cur_fail = 0
    for s in data["solve_ok"]:
        if s == 0:
            cur_fail += 1
            max_fail = max(max_fail, cur_fail)
        else:
            cur_fail = 0
    m["max_consecutive_fail"] = max_fail

    # --- 耗时 ---
    m["time_mean"] = np.mean(data["time_cost"])
    m["time_std"] = np.std(data["time_cost"])
    m["time_max"] = np.max(data["time_cost"])
    m["time_min"] = np.min(data["time_cost"])

    # --- 相对时间 ---
    m["t_rel"] = t_rel

    return m


# ============================================================
# 3. 保存分析结果
# ============================================================

def save_analysis(data, metrics, output_path):
    """合并原始数据 + 衍生指标，保存为 *_any.txt"""
    n = len(data["timestamp"])
    header = (
        "t_rel "
        "target_x target_y target_z "
        "filtered_x filtered_y filtered_z "
        "raw_yaw filter_yaw "
        "center_x center_y center_r center_vx center_vy "
        "solve_ok time_cost "
        "dx_raw dy_raw dpos_raw "
        "dx_flt dy_flt dpos_flt "
        "dyaw mutation "
        "res_x res_y res_norm "
        "yaw_from_neg90 "
        "err_raw_x err_raw_y err_flt_x err_flt_y"
    )
    with open(output_path, "w") as f:
        f.write(header + "\n")
        for i in range(n):
            row = [
                f"{metrics['t_rel'][i]:.6f}",
                f"{data['target_world_x'][i]:.6f}",
                f"{data['target_world_y'][i]:.6f}",
                f"{data['target_world_z'][i]:.6f}",
                f"{data['filtered_world_x'][i]:.6f}",
                f"{data['filtered_world_y'][i]:.6f}",
                f"{data['filtered_world_z'][i]:.6f}",
                f"{data['raw_yaw'][i]:.6f}",
                f"{data['filter_yaw'][i]:.6f}",
                f"{data['center_x'][i]:.6f}",
                f"{data['center_y'][i]:.6f}",
                f"{data['center_r'][i]:.6f}",
                f"{data['center_vx'][i]:.6f}",
                f"{data['center_vy'][i]:.6f}",
                f"{data['solve_ok'][i]}",
                f"{data['time_cost'][i]:.6f}",
                f"{metrics['dx_raw'][i]:.6f}",
                f"{metrics['dy_raw'][i]:.6f}",
                f"{metrics['dpos_raw'][i]:.6f}",
                f"{metrics['dx_flt'][i]:.6f}",
                f"{metrics['dy_flt'][i]:.6f}",
                f"{metrics['dpos_flt'][i]:.6f}",
                f"{metrics['dyaw'][i]:.6f}",
                f"{int(metrics['mutation'][i])}",
                f"{metrics['res_x'][i]:.6f}" if not np.isnan(metrics['res_x'][i]) else "nan",
                f"{metrics['res_y'][i]:.6f}" if not np.isnan(metrics['res_y'][i]) else "nan",
                f"{metrics['res_norm'][i]:.6f}" if not np.isnan(metrics['res_norm'][i]) else "nan",
                f"{metrics['yaw_from_neg90'][i]:.6f}",
                f"{metrics['err_raw_x'][i]:.6f}",
                f"{metrics['err_raw_y'][i]:.6f}",
                f"{metrics['err_flt_x'][i]:.6f}",
                f"{metrics['err_flt_y'][i]:.6f}",
            ]
            f.write(" ".join(row) + "\n")


# ============================================================
# 4. 可视化
# ============================================================

def plot_single(method, data, metrics, picture_dir):
    """单方法 6 张图"""
    t = metrics["t_rel"]
    prefix = os.path.join(picture_dir, method)

    # --- 图 1: center_x / center_y 时序 ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(t, data["center_x"], "b.-", markersize=2)
    axes[0].set_ylabel("center_x (m)")
    axes[0].set_title(f"[{method}] Rotation Center X")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, data["center_y"], "r.-", markersize=2)
    axes[1].set_ylabel("center_y (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title(f"[{method}] Rotation Center Y")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{prefix}_center_xy.png", dpi=150)
    plt.close(fig)

    # --- 图 2: 观测轨迹 vs 滤波轨迹 (x-y) ---
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    valid = data["solve_ok"] == 1
    ax.scatter(data["target_world_x"], data["target_world_y"],
               c="red", s=8, alpha=0.5, label="observed")
    ax.scatter(data["filtered_world_x"][valid], data["filtered_world_y"][valid],
               c="green", s=8, alpha=0.5, label="filtered")
    # center 点（所有帧 + 平均值）
    valid_center = data["center_r"] > 0.01
    ax.scatter(data["center_x"][valid_center], data["center_y"][valid_center],
               c="blue", s=8, alpha=0.3, label="center per frame")
    cx, cy = np.mean(data["center_x"][valid_center]), np.mean(data["center_y"][valid_center])
    ax.scatter([cx], [cy], c="blue", s=100, marker="*", zorder=5,
               label=f"center mean ({cx:.3f},{cy:.3f})")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"[{method}] Trajectory: Observed vs Filtered")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(f"{prefix}_trajectory.png", dpi=150)
    plt.close(fig)

    # --- 图 3: 帧间差分 (突变检测) ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(t, np.abs(metrics["dx_raw"]), "b.-", markersize=2, label="|dx_raw|")
    axes[0].plot(t, np.abs(metrics["dx_flt"]), "g.-", markersize=2, label="|dx_flt|")
    mut = metrics["mutation"]
    if np.any(mut):
        axes[0].scatter(t[mut], np.abs(metrics["dx_raw"][mut]),
                        c="red", s=20, zorder=5, label="mutation")
    axes[0].axhline(MUTATION_THRESH, color="red", linestyle="--", alpha=0.5)
    axes[0].set_ylabel("|dx| (m)")
    axes[0].set_title(f"[{method}] Frame-to-Frame X Displacement")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, np.abs(metrics["dy_raw"]), "b.-", markersize=2, label="|dy_raw|")
    axes[1].plot(t, np.abs(metrics["dy_flt"]), "g.-", markersize=2, label="|dy_flt|")
    if np.any(mut):
        axes[1].scatter(t[mut], np.abs(metrics["dy_raw"][mut]),
                        c="red", s=20, zorder=5, label="mutation")
    axes[1].axhline(MUTATION_THRESH, color="red", linestyle="--", alpha=0.5)
    axes[1].set_ylabel("|dy| (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title(f"[{method}] Frame-to-Frame Y Displacement")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{prefix}_mutation.png", dpi=150)
    plt.close(fig)

    # --- 图 4: 残差时序 ---
    fig, ax = plt.subplots(1, 1, figsize=(12, 4))
    valid_r = ~np.isnan(metrics["res_norm"])
    if np.any(valid_r):
        ax.plot(t[valid_r], metrics["res_norm"][valid_r], "b.-", markersize=2)
        ax.set_ylabel("residual (m)")
    else:
        ax.text(0.5, 0.5, "center_r = 0, residual N/A",
                transform=ax.transAxes, ha="center", va="center", fontsize=14)
    ax.set_xlabel("time (s)")
    ax.set_title(f"[{method}] Observation Residual (vs center model)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{prefix}_residual.png", dpi=150)
    plt.close(fig)

    # --- 图 5: yaw 分布 + 可观性 ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(t, data["raw_yaw"], "b.-", markersize=2, label="raw_yaw")
    axes[0].plot(t, data["filter_yaw"], "g.-", markersize=2, label="filter_yaw")
    axes[0].axhline(-math.pi / 2, color="red", linestyle="--", alpha=0.5, label="-90 deg")
    axes[0].set_ylabel("yaw (rad)")
    axes[0].set_title(f"[{method}] Yaw: Raw vs Filtered")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, np.degrees(metrics["yaw_from_neg90"]), "m.-", markersize=2)
    axes[1].axhline(2.0, color="red", linestyle="--", alpha=0.5, label="filter threshold 2 deg")
    axes[1].set_ylabel("|yaw - (-90 deg)| (deg)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title(f"[{method}] Observability: Distance from -90 deg")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{prefix}_yaw.png", dpi=150)
    plt.close(fig)

    # --- 图 6: 耗时 + solve_ok ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].bar(t, data["time_cost"], width=0.012, color="steelblue", alpha=0.7)
    axes[0].set_ylabel("time_cost (ms)")
    axes[0].set_title(f"[{method}] Computation Time (mean={metrics['time_mean']:.2f}ms)")
    axes[0].grid(True, alpha=0.3)
    colors = ["green" if s == 1 else "red" for s in data["solve_ok"]]
    axes[1].bar(t, data["solve_ok"], width=0.012, color=colors, alpha=0.7)
    axes[1].set_ylabel("solve_ok")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title(f"[{method}] Solve Status (rate={metrics['solve_rate']*100:.1f}%)")
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{prefix}_perf.png", dpi=150)
    plt.close(fig)

    print(f"  [{method}] 6 plots saved to {picture_dir}/")


def plot_comparison(all_results, picture_dir):
    """三方法对比图（仅对有数据的方法）"""
    methods = list(all_results.keys())
    if len(methods) < 2:
        return

    colors = {"ekf": "blue", "ls": "orange", "ba": "green"}

    # --- 对比 1: center_x ---
    fig, axes = plt.subplots(len(methods), 1, figsize=(12, 3 * len(methods)), sharex=False)
    if len(methods) == 1:
        axes = [axes]
    for ax, method in zip(axes, methods):
        data, metrics = all_results[method]
        ax.plot(metrics["t_rel"], data["center_x"], ".-",
                color=colors.get(method, "gray"), markersize=2)
        ax.set_ylabel("center_x (m)")
        ax.set_title(f"{method}  std={metrics['center_x_std']:.4f}m")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Center X Comparison", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(picture_dir, "compare_center_x.png"), dpi=150)
    plt.close(fig)

    # --- 对比 2: center_y ---
    fig, axes = plt.subplots(len(methods), 1, figsize=(12, 3 * len(methods)), sharex=False)
    if len(methods) == 1:
        axes = [axes]
    for ax, method in zip(axes, methods):
        data, metrics = all_results[method]
        ax.plot(metrics["t_rel"], data["center_y"], ".-",
                color=colors.get(method, "gray"), markersize=2)
        ax.set_ylabel("center_y (m)")
        ax.set_title(f"{method}  std={metrics['center_y_std']:.4f}m")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Center Y Comparison", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(picture_dir, "compare_center_y.png"), dpi=150)
    plt.close(fig)

    # --- 对比 3: 轨迹 ---
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    for method in methods:
        data, _ = all_results[method]
        ax.scatter(data["target_world_x"], data["target_world_y"],
                   s=8, alpha=0.3, color="red", label="_nolegend_")
        valid = data["solve_ok"] == 1
        ax.scatter(data["filtered_world_x"][valid], data["filtered_world_y"][valid],
                   s=8, alpha=0.5, color=colors.get(method, "gray"), label=f"{method} filtered")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory Comparison")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(os.path.join(picture_dir, "compare_trajectory.png"), dpi=150)
    plt.close(fig)

    # --- 对比 4: 耗时 ---
    fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    means = []
    stds = []
    labels = []
    for method in methods:
        _, metrics = all_results[method]
        means.append(metrics["time_mean"])
        stds.append(metrics["time_std"])
        labels.append(method)
    x = np.arange(len(labels))
    ax.bar(x, means, yerr=stds, capsize=5, color=[colors.get(m, "gray") for m in labels], alpha=0.7)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("time_cost (ms)")
    ax.set_title("Computation Time Comparison")
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(picture_dir, "compare_time.png"), dpi=150)
    plt.close(fig)

    print(f"  Comparison plots saved to {picture_dir}/")


# ============================================================
# 5. 统计摘要
# ============================================================

def print_summary(method, metrics):
    """打印单方法统计摘要"""
    print(f"\n{'='*50}")
    print(f"  Method: {method}")
    print(f"{'='*50}")
    print(f"  Frames:           {len(metrics['t_rel'])}")
    print(f"  Solve rate:       {metrics['solve_rate']*100:.1f}%")
    print(f"  Max consec fail:  {metrics['max_consecutive_fail']}")
    print(f"  Center X std:     {metrics['center_x_std']:.4f} m")
    print(f"  Center Y std:     {metrics['center_y_std']:.4f} m")
    print(f"  Filter ratio X:   {metrics['filter_ratio_x']:.4f}" if not np.isnan(metrics['filter_ratio_x']) else "  Filter ratio X:   N/A")
    print(f"  Filter ratio Y:   {metrics['filter_ratio_y']:.4f}" if not np.isnan(metrics['filter_ratio_y']) else "  Filter ratio Y:   N/A")
    print(f"  Mutations:        {np.sum(metrics['mutation'])}")
    print(f"  Time:             {metrics['time_mean']:.2f} +/- {metrics['time_std']:.2f} ms")
    print(f"  Time range:       [{metrics['time_min']:.2f}, {metrics['time_max']:.2f}] ms")


# ============================================================
# 6. Main
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
    parser = argparse.ArgumentParser(description="Tracker Debug 数据分析与可视化")
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

        picture_dir = os.path.join(base_dir, "Picture", "tracker", test_name)
        os.makedirs(picture_dir, exist_ok=True)

        log_files = find_log_files(base_dir, test_name)
        if not log_files:
            print(f"  No log files found in Debug/Tracker/{test_name}/{{ekf,ls,ba}}/")
            continue

        all_results = {}

        for method, files in log_files.items():
            print(f"\n  Processing {method}: {len(files)} file(s)")
            anal_dir = os.path.join(base_dir, "Tracker", test_name, "Anal", method)
            os.makedirs(anal_dir, exist_ok=True)

        for fpath in files:
            fname = os.path.splitext(os.path.basename(fpath))[0]
            print(f"  Loading {fpath}...")

            data = load_log(fpath)
            if data is None:
                print(f"  Skipped (empty)")
                continue

            metrics = compute_metrics(data)

            # 保存分析结果
            out_path = os.path.join(anal_dir, f"{fname}_any.txt")
            save_analysis(data, metrics, out_path)
            print(f"  Saved {out_path}")

            # 打印摘要
            print_summary(method, metrics)

            # 绘图
            plot_single(method, data, metrics, picture_dir)

            # 保存用于对比（取每个方法最后一个文件）
            all_results[method] = (data, metrics)

    # 三方法对比
    if len(all_results) > 1:
        plot_comparison(all_results, picture_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
