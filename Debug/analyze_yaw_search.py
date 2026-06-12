#!/usr/bin/env python3
"""
Yaw 搜索基准分析脚本。

读取一个或多个 CSV（同一视频的多次重复），生成：
  - summary.csv：每参数组合的聚合统计
  - recommendation.json：推荐参数组合（或 null）
  - 热力图：精度、P50/P95 耗时
  - Pareto 图：精度 vs 耗时
  - 边界/可观测/无效样本统计图

用法:
    python3 Debug/analyze_yaw_search.py <csv_or_dir> [<csv_or_dir> ...]
    python3 Debug/analyze_yaw_search.py --output-dir /path/to/output <csv_or_dir> ...
"""

import argparse
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# ============================================================
# 1. CSV Schema
# ============================================================

SCHEMA_VERSION = 1

REQUIRED_COLUMNS = [
    "schema_version", "video", "sample_id", "frame_index",
    "armor_index", "armor_name", "armor_type",
    "distance_m", "image_center_distance_px",
    "init_yaw_rad", "reference_yaw_rad", "reference_error_sum_px",
    "reference_boundary", "observable", "invalid",
    "step_deg", "iterations",
    "coarse_yaw_rad", "refined_yaw_rad",
    "coarse_error_sum_px", "refined_error_sum_px",
    "yaw_error_deg", "error_regret_sum_px", "error_regret_per_corner_px",
    "coarse_in_local_basin", "evaluation_count",
    "elapsed_ns", "timing_valid",
]

# Columns that must be numeric (float or int)
NUMERIC_COLUMNS = [
    "distance_m", "image_center_distance_px",
    "init_yaw_rad", "reference_yaw_rad", "reference_error_sum_px",
    "step_deg", "iterations",
    "coarse_yaw_rad", "refined_yaw_rad",
    "coarse_error_sum_px", "refined_error_sum_px",
    "yaw_error_deg", "error_regret_sum_px", "error_regret_per_corner_px",
    "evaluation_count", "elapsed_ns",
]

# Boolean-like columns (0/1)
BOOL_COLUMNS = [
    "reference_boundary", "observable", "invalid",
    "coarse_in_local_basin", "timing_valid",
]

# Thresholds (from plan section 8)
THRESH_YAW_P99 = 0.1          # degrees
THRESH_YAW_OUTLIER = 0.5      # degrees
THRESH_CORNER_P99 = 0.01      # px
THRESH_CORNER_OUTLIER = 0.05  # px
MIN_TIMING_VALID = 200
MIN_OBSERVABLE = 50
LATENCY_TIE_TOLERANCE = 0.02  # 2%

# Plot annotation text
ANNOTATION_TEXT = (
    "Note: 0.01 px / 0.1° thresholds are relative to\n"
    "dense reference search consistency, not real yaw accuracy."
)


# ============================================================
# 2. CSV Loading
# ============================================================

def collect_csv_files(paths):
    """展开目录或文件列表为 CSV 文件列表。"""
    csv_files = []
    for p in paths:
        if os.path.isdir(p):
            for fname in sorted(os.listdir(p)):
                if fname.endswith(".csv"):
                    csv_files.append(os.path.join(p, fname))
        elif os.path.isfile(p):
            csv_files.append(p)
        else:
            print(f"[WARN] 路径不存在: {p}", file=sys.stderr)
    return csv_files


def load_csv(path):
    """
    读取一个 CSV 文件，返回 dict of numpy arrays + 元信息。
    失败时返回 (None, error_msg)。
    """
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return None, f"CSV 为空或缺少表头: {path}"
        # 校验列
        missing = [c for c in REQUIRED_COLUMNS if c not in reader.fieldnames]
        if missing:
            return None, f"CSV 缺少必需列 {missing}: {path}"
        rows = list(reader)

    if not rows:
        return None, f"CSV 无数据行: {path}"

    # 校验 schema_version
    try:
        sv = int(rows[0]["schema_version"])
    except (ValueError, KeyError):
        return None, f"schema_version 无法解析: {path}"
    if sv != SCHEMA_VERSION:
        return None, f"schema_version={sv}，期望 {SCHEMA_VERSION}: {path}"

    # 解析为 numpy arrays
    data = {}
    n = len(rows)

    for col in REQUIRED_COLUMNS:
        if col in NUMERIC_COLUMNS:
            vals = np.empty(n, dtype=np.float64)
            for i, row in enumerate(rows):
                try:
                    vals[i] = float(row[col])
                except (ValueError, KeyError):
                    vals[i] = np.nan
            data[col] = vals
        elif col in BOOL_COLUMNS:
            vals = np.empty(n, dtype=np.int32)
            for i, row in enumerate(rows):
                try:
                    vals[i] = int(row[col])
                except (ValueError, KeyError):
                    vals[i] = 0
            data[col] = vals
        else:
            data[col] = [row.get(col, "") for row in rows]

    return data, None


def load_all_csvs(csv_files):
    """读取多个 CSV，纵向拼接。返回 (data_dict, error_list)。"""
    all_data = {col: [] for col in REQUIRED_COLUMNS}
    errors = []

    for fpath in csv_files:
        data, err = load_csv(fpath)
        if err:
            errors.append(err)
            continue
        n = len(data["schema_version"])
        for col in REQUIRED_COLUMNS:
            if isinstance(data[col], np.ndarray):
                all_data[col].append(data[col])
            else:
                all_data[col].extend(data[col])

    if errors:
        return None, errors

    # 合并 numpy arrays
    result = {}
    for col in REQUIRED_COLUMNS:
        if isinstance(all_data[col][0] if all_data[col] else [], np.ndarray):
            result[col] = np.concatenate(all_data[col])
        else:
            result[col] = all_data[col]
    return result, []


# ============================================================
# 3. Analysis
# ============================================================

def theoretical_evaluations(step_deg, iterations):
    """计算理论误差评估次数：枚举 + 三分搜索。"""
    # 枚举点数 = floor(2 * 30 / step) + 1
    enum_points = int(math.floor(60.0 / step_deg)) + 1
    return enum_points + int(iterations) * 2


def analyze_parameter_combo(data, step_deg, iterations):
    """
    分析单个参数组合（step_deg × iterations）。
    返回 dict of 统计量，失败返回 None。
    """
    # 筛选当前参数组合的行
    mask_step = np.isclose(data["step_deg"], step_deg)
    mask_iter = np.isclose(data["iterations"], iterations)
    mask = mask_step & mask_iter
    if not np.any(mask):
        return None

    observable = data["observable"][mask].astype(bool)
    invalid = data["invalid"][mask].astype(bool)
    boundary = data["reference_boundary"][mask].astype(bool)
    timing_valid = data["timing_valid"][mask].astype(bool)
    finite = np.isfinite(data["yaw_error_deg"][mask])
    basin = data["coarse_in_local_basin"][mask].astype(bool)

    yaw_err = data["yaw_error_deg"][mask]
    corner_err = data["error_regret_per_corner_px"][mask]
    elapsed_ns = data["elapsed_ns"][mask]

    # --- 门槛 1: 非有限或搜索失败 ---
    n_nonfinite = int(np.sum(~finite & ~invalid))

    # --- 门槛 2: 可观测样本粗搜不在局部盆地 ---
    obs_mask = observable & ~invalid
    n_not_in_basin = int(np.sum(~basin[obs_mask])) if np.any(obs_mask) else 0

    # --- 门槛 3 & 4: 可观测样本 yaw 误差 ---
    obs_yaw = yaw_err[obs_mask] if np.any(obs_mask) else np.array([])
    yaw_p99 = float(np.percentile(obs_yaw, 99)) if len(obs_yaw) > 0 else np.nan
    n_yaw_outlier = int(np.sum(obs_yaw > THRESH_YAW_OUTLIER)) if len(obs_yaw) > 0 else 0

    # --- 门槛 5 & 6: 有效非边界样本每角点误差增量 ---
    valid_nonboundary = ~invalid & ~boundary
    corner_valid = corner_err[valid_nonboundary]
    corner_p99 = float(np.percentile(corner_valid, 99)) if len(corner_valid) > 0 else np.nan
    n_corner_outlier = int(np.sum(corner_valid > THRESH_CORNER_OUTLIER)) if len(corner_valid) > 0 else 0

    # --- 门槛 7: 最小样本量 ---
    n_timing_valid = int(np.sum(timing_valid))
    n_observable = int(np.sum(obs_mask))

    # --- 耗时统计（仅 timing-valid 样本） ---
    tv_elapsed = elapsed_ns[timing_valid]
    if len(tv_elapsed) > 0:
        p50_ns = float(np.percentile(tv_elapsed, 50))
        p95_ns = float(np.percentile(tv_elapsed, 95))
        mean_ns = float(np.mean(tv_elapsed))
    else:
        p50_ns = p95_ns = mean_ns = np.nan

    eval_count = theoretical_evaluations(step_deg, iterations)

    return {
        "step_deg": step_deg,
        "iterations": int(iterations),
        "evaluation_count": eval_count,
        # 门槛指标
        "n_nonfinite_or_failed": n_nonfinite,
        "n_not_in_basin": n_not_in_basin,
        "yaw_error_p99_deg": yaw_p99,
        "n_yaw_outlier": n_yaw_outlier,
        "corner_error_p99_px": corner_p99,
        "n_corner_outlier": n_corner_outlier,
        "n_timing_valid": n_timing_valid,
        "n_observable": n_observable,
        # 耗时
        "elapsed_p50_ns": p50_ns,
        "elapsed_p95_ns": p95_ns,
        "elapsed_mean_ns": mean_ns,
        # 样本统计
        "n_total": int(np.sum(mask)),
        "n_invalid": int(np.sum(invalid)),
        "n_boundary": int(np.sum(boundary)),
        # 是否通过所有门槛
        "pass_thresholds": None,  # 待计算
    }


def check_thresholds(stats):
    """检查单个参数组合是否通过所有门槛。返回 (bool, failed_list)。"""
    failed = []
    if stats["n_nonfinite_or_failed"] > 0:
        failed.append(f"nonfinite_or_failed={stats['n_nonfinite_or_failed']}")
    if stats["n_not_in_basin"] > 0:
        failed.append(f"not_in_basin={stats['n_not_in_basin']}")
    if np.isnan(stats["yaw_error_p99_deg"]) or stats["yaw_error_p99_deg"] > THRESH_YAW_P99:
        failed.append(f"yaw_p99={stats['yaw_error_p99_deg']:.4f}> {THRESH_YAW_P99}")
    if stats["n_yaw_outlier"] > 0:
        failed.append(f"yaw_outlier={stats['n_yaw_outlier']}")
    if np.isnan(stats["corner_error_p99_px"]) or stats["corner_error_p99_px"] > THRESH_CORNER_P99:
        failed.append(f"corner_p99={stats['corner_error_p99_px']:.6f}> {THRESH_CORNER_P99}")
    if stats["n_corner_outlier"] > 0:
        failed.append(f"corner_outlier={stats['n_corner_outlier']}")
    if stats["n_timing_valid"] < MIN_TIMING_VALID:
        failed.append(f"timing_valid={stats['n_timing_valid']}< {MIN_TIMING_VALID}")
    if stats["n_observable"] < MIN_OBSERVABLE:
        failed.append(f"observable={stats['n_observable']}< {MIN_OBSERVABLE}")
    return len(failed) == 0, failed


def run_full_analysis(data):
    """
    对所有参数组合运行分析。
    返回 (all_stats: list[dict], steps: list, iters: list)。
    """
    steps = sorted(set(float(s) for s in data["step_deg"][np.isfinite(data["step_deg"])]))
    iters = sorted(set(int(i) for i in data["iterations"][np.isfinite(data["iterations"])]))

    all_stats = []
    for step in steps:
        for it in iters:
            stats = analyze_parameter_combo(data, step, it)
            if stats is None:
                continue
            passed, failed = check_thresholds(stats)
            stats["pass_thresholds"] = passed
            stats["failed_thresholds"] = failed
            all_stats.append(stats)

    return all_stats, steps, iters


# ============================================================
# 4. Recommendation
# ============================================================

def recommend(all_stats):
    """
    从通过所有门槛的组合中选最优。
    规则：
      1. 各视频 P95 耗时最大值最低
      2. 2% 内选理论评估次数更少
      3. 仍相同选 yaw P99 更低
    返回推荐 dict 或 None。
    """
    passing = [s for s in all_stats if s["pass_thresholds"]]
    if not passing:
        return None

    # 对于单视频场景，P95 latency 就是 elapsed_p95_ns
    # （跨视频场景需要外部聚合）
    def sort_key(s):
        return (
            s["elapsed_p95_ns"] if not np.isnan(s["elapsed_p95_ns"]) else float("inf"),
            s["evaluation_count"],
            s["yaw_error_p99_deg"] if not np.isnan(s["yaw_error_p99_deg"]) else float("inf"),
        )

    passing.sort(key=sort_key)
    best = passing[0]
    best_p95 = best["elapsed_p95_ns"]

    # 检查是否有 2% 内的候选
    candidates = [s for s in passing
                  if not np.isnan(s["elapsed_p95_ns"])
                  and abs(s["elapsed_p95_ns"] - best_p95) / max(best_p95, 1) < LATENCY_TIE_TOLERANCE]

    if len(candidates) > 1:
        # 按 eval count 排序
        candidates.sort(key=lambda s: s["evaluation_count"])
        best_eval = candidates[0]["evaluation_count"]
        tied = [s for s in candidates if s["evaluation_count"] == best_eval]
        if len(tied) > 1:
            # 按 yaw P99 排序
            tied.sort(key=lambda s: s["yaw_error_p99_deg"] if not np.isnan(s["yaw_error_p99_deg"]) else float("inf"))
            return tied[0]
        return candidates[0]

    return best


# ============================================================
# 5. Output
# ============================================================

def write_summary_csv(all_stats, output_path):
    """写入 summary.csv。"""
    if not all_stats:
        return
    fields = [
        "step_deg", "iterations", "evaluation_count",
        "n_nonfinite_or_failed", "n_not_in_basin",
        "yaw_error_p99_deg", "n_yaw_outlier",
        "corner_error_p99_px", "n_corner_outlier",
        "n_timing_valid", "n_observable",
        "elapsed_p50_ns", "elapsed_p95_ns", "elapsed_mean_ns",
        "n_total", "n_invalid", "n_boundary",
        "pass_thresholds", "failed_thresholds",
    ]
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for s in all_stats:
            row = dict(s)
            row["pass_thresholds"] = str(s["pass_thresholds"]).lower()
            row["failed_thresholds"] = "; ".join(s.get("failed_thresholds", []))
            writer.writerow(row)
    print(f"[INFO] 写入: {output_path}")


def write_recommendation_json(rec, all_stats, output_path):
    """写入 recommendation.json。"""
    result = {"schema_version": 1}

    if rec is not None:
        result["recommended"] = {
            "step_deg": rec["step_deg"],
            "iterations": rec["iterations"],
            "evaluation_count": rec["evaluation_count"],
            "yaw_error_p99_deg": rec["yaw_error_p99_deg"],
            "elapsed_p95_ns": rec["elapsed_p95_ns"],
            "elapsed_p50_ns": rec["elapsed_p50_ns"],
            "corner_error_p99_px": rec["corner_error_p99_px"],
        }
    else:
        result["recommended"] = None

    # 输出 Pareto 候选（按 P95 排序前 10）
    ranked = sorted(all_stats, key=lambda s: s["elapsed_p95_ns"] if not np.isnan(s["elapsed_p95_ns"]) else float("inf"))
    pareto = []
    for s in ranked[:10]:
        pareto.append({
            "step_deg": s["step_deg"],
            "iterations": s["iterations"],
            "evaluation_count": s["evaluation_count"],
            "yaw_error_p99_deg": s["yaw_error_p99_deg"],
            "elapsed_p95_ns": s["elapsed_p95_ns"],
            "pass_thresholds": s["pass_thresholds"],
            "failed_thresholds": s.get("failed_thresholds", []),
        })
    result["pareto_top_10"] = pareto

    with open(output_path, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"[INFO] 写入: {output_path}")


# ============================================================
# 6. Plots
# ============================================================

def ns_to_us(ns):
    """纳秒转微秒。"""
    if np.isnan(ns):
        return np.nan
    return ns / 1000.0


def build_grid(all_stats, steps, iters, field):
    """构建 step × iteration 的 2D 网格。"""
    grid = np.full((len(steps), len(iters)), np.nan)
    step_idx = {s: i for i, s in enumerate(steps)}
    iter_idx = {it: i for i, it in enumerate(iters)}
    for s in all_stats:
        si = step_idx.get(s["step_deg"])
        ii = iter_idx.get(s["iterations"])
        if si is not None and ii is not None:
            grid[si, ii] = s[field]
    return grid


def plot_accuracy_heatmap(all_stats, steps, iters, output_path):
    """精度热力图：step_deg × iterations → yaw_error P99。"""
    grid = build_grid(all_stats, steps, iters, "yaw_error_p99_deg")

    fig, ax = plt.subplots(figsize=(10, 7))
    im = ax.imshow(grid, cmap="RdYlGn_r", aspect="auto", origin="lower",
                   vmin=0, vmax=max(THRESH_YAW_OUTLIER, np.nanmax(grid) * 1.1))

    ax.set_xticks(range(len(iters)))
    ax.set_xticklabels([str(i) for i in iters])
    ax.set_yticks(range(len(steps)))
    ax.set_yticklabels([f"{s}°" for s in steps])
    ax.set_xlabel("Ternary iterations")
    ax.set_ylabel("Enumeration step (deg)")
    ax.set_title("Yaw Error P99 (degrees) — Observable Samples")

    # 在每个格子标数值
    for i in range(len(steps)):
        for j in range(len(iters)):
            val = grid[i, j]
            if not np.isnan(val):
                color = "white" if val > THRESH_YAW_P99 else "black"
                ax.text(j, i, f"{val:.3f}", ha="center", va="center",
                        fontsize=8, color=color, fontweight="bold")

    # 标注门槛线
    ax.axhline(y=-0.5, color="blue", linestyle="--", alpha=0)
    cbar = fig.colorbar(im, ax=ax, label="Yaw Error P99 (deg)")
    cbar.ax.axhline(y=THRESH_YAW_P99, color="blue", linestyle="--", linewidth=2)

    fig.text(0.5, 0.01, ANNOTATION_TEXT, ha="center", fontsize=8, style="italic", color="gray")
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"[INFO] 写入: {output_path}")


def plot_timing_heatmap(all_stats, steps, iters, output_path_prefix):
    """耗时热力图 P50 和 P95。"""
    for pct_label, field in [("P50", "elapsed_p50_ns"), ("P95", "elapsed_p95_ns")]:
        grid = build_grid(all_stats, steps, iters, field)
        # 转微秒
        grid_us = np.vectorize(ns_to_us)(grid)

        fig, ax = plt.subplots(figsize=(10, 7))
        im = ax.imshow(grid_us, cmap="YlOrRd", aspect="auto", origin="lower",
                       vmin=0, vmax=np.nanmax(grid_us) * 1.1 if np.any(np.isfinite(grid_us)) else 1)

        ax.set_xticks(range(len(iters)))
        ax.set_xticklabels([str(i) for i in iters])
        ax.set_yticks(range(len(steps)))
        ax.set_yticklabels([f"{s}°" for s in steps])
        ax.set_xlabel("Ternary iterations")
        ax.set_ylabel("Enumeration step (deg)")
        ax.set_title(f"Elapsed Time {pct_label} (µs) — Timing-Valid Samples")

        for i in range(len(steps)):
            for j in range(len(iters)):
                val = grid_us[i, j]
                if not np.isnan(val):
                    ax.text(j, i, f"{val:.0f}", ha="center", va="center",
                            fontsize=8, fontweight="bold")

        fig.colorbar(im, ax=ax, label=f"Elapsed {pct_label} (µs)")
        fig.text(0.5, 0.01, ANNOTATION_TEXT, ha="center", fontsize=8, style="italic", color="gray")
        fig.tight_layout(rect=[0, 0.04, 1, 1])
        out = f"{output_path_prefix}_{pct_label.lower()}.png"
        fig.savefig(out, dpi=150)
        plt.close(fig)
        print(f"[INFO] 写入: {out}")


def plot_pareto(all_stats, output_path):
    """Pareto 图：yaw P99 vs P95 延迟。"""
    fig, ax = plt.subplots(figsize=(10, 7))

    passing = [s for s in all_stats if s["pass_thresholds"]]
    failing = [s for s in all_stats if not s["pass_thresholds"]]

    # 失败的用灰色 x
    if failing:
        x_f = [ns_to_us(s["elapsed_p95_ns"]) for s in failing]
        y_f = [s["yaw_error_p99_deg"] for s in failing]
        ax.scatter(x_f, y_f, c="lightgray", marker="x", s=60, label="Failing", alpha=0.6)

    # 通过的用绿色圆
    if passing:
        x_p = [ns_to_us(s["elapsed_p95_ns"]) for s in passing]
        y_p = [s["yaw_error_p99_deg"] for s in passing]
        labels_p = [f"{s['step_deg']}°/{s['iterations']}it" for s in passing]
        ax.scatter(x_p, y_p, c="green", marker="o", s=80, label="Passing", zorder=5)
        for xi, yi, lab in zip(x_p, y_p, labels_p):
            ax.annotate(lab, (xi, yi), textcoords="offset points",
                        xytext=(5, 5), fontsize=7, color="darkgreen")

    # 门槛区域
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    ax.axhline(THRESH_YAW_P99, color="red", linestyle="--", alpha=0.5,
               label=f"P99 threshold ({THRESH_YAW_P99}°)")
    ax.set_xlim(xlim)
    ax.set_ylim(max(0, ylim[0] - 0.01), ylim[1])

    ax.set_xlabel("P95 Latency (µs)")
    ax.set_ylabel("Yaw Error P99 (deg)")
    ax.set_title("Pareto: Accuracy vs Latency")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.text(0.5, 0.01, ANNOTATION_TEXT, ha="center", fontsize=8, style="italic", color="gray")
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"[INFO] 写入: {output_path}")


def plot_sample_stats(all_stats, steps, iters, output_path):
    """边界/可观测/无效样本统计柱状图。"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    labels = [f"{s['step_deg']}°/{s['iterations']}it" for s in all_stats]
    x = np.arange(len(labels))

    # 无效样本
    n_invalid = [s["n_invalid"] for s in all_stats]
    axes[0].bar(x, n_invalid, color="red", alpha=0.7)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
    axes[0].set_ylabel("Count")
    axes[0].set_title("Invalid Samples")
    axes[0].grid(True, alpha=0.3, axis="y")

    # 边界样本
    n_boundary = [s["n_boundary"] for s in all_stats]
    axes[1].bar(x, n_boundary, color="orange", alpha=0.7)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
    axes[1].set_ylabel("Count")
    axes[1].set_title("Boundary Samples")
    axes[1].grid(True, alpha=0.3, axis="y")

    # 可观测样本
    n_obs = [s["n_observable"] for s in all_stats]
    axes[2].bar(x, n_obs, color="steelblue", alpha=0.7)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
    axes[2].set_ylabel("Count")
    axes[2].set_title("Observable Samples")
    axes[2].axhline(MIN_OBSERVABLE, color="red", linestyle="--", alpha=0.5,
                    label=f"min={MIN_OBSERVABLE}")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3, axis="y")

    fig.suptitle("Sample Statistics per Parameter Combo")
    fig.text(0.5, 0.01, ANNOTATION_TEXT, ha="center", fontsize=8, style="italic", color="gray")
    fig.tight_layout(rect=[0, 0.04, 1, 0.95])
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"[INFO] 写入: {output_path}")


def generate_plots(all_stats, steps, iters, output_dir):
    """生成所有图表。"""
    plot_accuracy_heatmap(all_stats, steps, iters,
                          os.path.join(output_dir, "heatmap_accuracy.png"))
    plot_timing_heatmap(all_stats, steps, iters,
                        os.path.join(output_dir, "heatmap_timing"))
    plot_pareto(all_stats,
                os.path.join(output_dir, "pareto.png"))
    plot_sample_stats(all_stats, steps, iters,
                      os.path.join(output_dir, "sample_stats.png"))


# ============================================================
# 7. Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Yaw 搜索基准分析",
    )
    parser.add_argument(
        "inputs", nargs="+",
        help="CSV 文件或包含 CSV 的目录",
    )
    parser.add_argument(
        "--output-dir", default=None,
        help="输出目录（默认使用第一个输入的父目录）",
    )
    args = parser.parse_args()

    # 收集 CSV 文件
    csv_files = collect_csv_files(args.inputs)
    if not csv_files:
        print("错误：未找到 CSV 文件", file=sys.stderr)
        return 5

    print(f"[INFO] 加载 {len(csv_files)} 个 CSV 文件...")
    data, errors = load_all_csvs(csv_files)
    if errors:
        for e in errors:
            print(f"[ERROR] {e}", file=sys.stderr)
        return 5

    n_rows = len(data["schema_version"])
    print(f"[INFO] 共 {n_rows} 行数据")

    # 分析
    all_stats, steps, iters = run_full_analysis(data)
    if not all_stats:
        print("[ERROR] 无有效参数组合", file=sys.stderr)
        return 5

    # 确定输出目录
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = os.path.dirname(csv_files[0])
    os.makedirs(output_dir, exist_ok=True)

    # 输出
    write_summary_csv(all_stats, os.path.join(output_dir, "summary.csv"))

    rec = recommend(all_stats)
    write_recommendation_json(rec, all_stats,
                              os.path.join(output_dir, "recommendation.json"))

    generate_plots(all_stats, steps, iters, output_dir)

    # 返回码
    if rec is not None:
        print(f"\n[INFO] 推荐参数: step={rec['step_deg']}°, iterations={rec['iterations']}, "
              f"evals={rec['evaluation_count']}, yaw_p99={rec['yaw_error_p99_deg']:.4f}°, "
              f"p95={ns_to_us(rec['elapsed_p95_ns']):.0f} µs")
        return 0
    else:
        print("\n[WARN] 没有参数组合通过所有门槛", file=sys.stderr)
        # 输出具体失败原因
        for s in all_stats:
            if s["failed_thresholds"]:
                print(f"  {s['step_deg']}°/{s['iterations']}it: {', '.join(s['failed_thresholds'])}")
        return 4


if __name__ == "__main__":
    sys.exit(main())
