#!/usr/bin/env python3
"""
可视化推理结果
"""
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

RESULT_FILE = "/home/minzhi/ws05_fourth_assessment/DeepLearning/inference_results.txt"
OUTPUT_DIR = "/home/minzhi/ws05_fourth_assessment/pthoto"

def parse_results(path):
    """解析推理结果txt"""
    dirs = {}
    current_dir = None
    
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("目录:"):
                m = re.search(r"目录:\s+(\S+)\s+\(共\s+(\d+)", line)
                if m:
                    current_dir = m.group(1)
                    dirs[current_dir] = {"total": int(m.group(2)), "files": []}
            elif current_dir and line.startswith("roi_"):
                parts = line.split()
                if len(parts) >= 6:
                    dirs[current_dir]["files"].append({
                        "file": parts[0],
                        "pred": parts[1],
                        "conf": float(parts[2]),
                        "prob3": float(parts[3]),
                        "prob4": float(parts[4]),
                        "prob_neg": float(parts[5]),
                    })
    return dirs

def plot_prediction_distribution(dirs):
    """图1: 各目录预测分布"""
    dir_names = list(dirs.keys())
    counts_3 = []
    counts_4 = []
    counts_neg = []
    
    for d in dir_names:
        c3 = sum(1 for f in dirs[d]["files"] if f["pred"] == "3")
        c4 = sum(1 for f in dirs[d]["files"] if f["pred"] == "4")
        cneg = sum(1 for f in dirs[d]["files"] if f["pred"] == "negative")
        counts_3.append(c3)
        counts_4.append(c4)
        counts_neg.append(cneg)
    
    x = np.arange(len(dir_names))
    width = 0.25
    
    fig, ax = plt.subplots(figsize=(14, 6))
    bars1 = ax.bar(x - width, counts_3, width, label='3', color='#3498db')
    bars2 = ax.bar(x, counts_4, width, label='4', color='#e74c3c')
    bars3 = ax.bar(x + width, counts_neg, width, label='negative', color='#95a5a6')
    
    ax.set_ylabel('Count')
    ax.set_title('Prediction Distribution by Directory (New Model)')
    ax.set_xticks(x)
    ax.set_xticklabels(dir_names, rotation=30, ha='right')
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    
    # 在柱子上标数字
    for bars in [bars1, bars2, bars3]:
        for bar in bars:
            h = bar.get_height()
            if h > 0:
                ax.text(bar.get_x() + bar.get_width()/2., h, f'{int(h)}',
                       ha='center', va='bottom', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(f"{OUTPUT_DIR}/viz_prediction_dist.png", dpi=150)
    plt.close()
    print(f"Saved: viz_prediction_dist.png")

def plot_confidence_by_dir(dirs):
    """图2: 各目录平均置信度"""
    dir_names = []
    avg_confs = []
    
    for d, data in dirs.items():
        if data["files"]:
            avg_conf = np.mean([f["conf"] for f in data["files"]])
            dir_names.append(d)
            avg_confs.append(avg_conf)
    
    fig, ax = plt.subplots(figsize=(12, 5))
    colors = ['#2ecc71' if c > 0.9 else '#f39c12' if c > 0.7 else '#e74c3c' for c in avg_confs]
    bars = ax.bar(dir_names, avg_confs, color=colors)
    ax.set_ylabel('Average Confidence')
    ax.set_title('Average Confidence by Directory')
    ax.set_ylim(0, 1.05)
    ax.set_xticklabels(dir_names, rotation=30, ha='right')
    ax.grid(axis='y', alpha=0.3)
    ax.axhline(y=0.9, color='r', linestyle='--', alpha=0.5, label='90% threshold')
    
    for bar, conf in zip(bars, avg_confs):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.01,
               f'{conf:.3f}', ha='center', va='bottom', fontsize=9)
    
    plt.tight_layout()
    plt.savefig(f"{OUTPUT_DIR}/viz_confidence.png", dpi=150)
    plt.close()
    print(f"Saved: viz_confidence.png")

def plot_confidence_histogram(dirs):
    """图3: 置信度分布直方图（分暗图/亮图）"""
    dark_dirs = ["3_mind_4000_8", "3_mind_5000_2"]
    bright_dirs = ["3_galaxy_5000_8_1", "3_galaxy_5000_8_2", "3_mind_5000_8_1", "3_mind_5000_8_2"]
    
    dark_confs = []
    bright_confs = []
    
    for d in dark_dirs:
        if d in dirs:
            dark_confs.extend([f["conf"] for f in dirs[d]["files"]])
    
    for d in bright_dirs:
        if d in dirs:
            bright_confs.extend([f["conf"] for f in dirs[d]["files"]])
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    
    axes[0].hist(dark_confs, bins=30, color='#e74c3c', alpha=0.7, edgecolor='black')
    axes[0].set_title(f'Dark Images (n={len(dark_confs)})\nmind_4000_8 + mind_5000_2')
    axes[0].set_xlabel('Confidence')
    axes[0].set_ylabel('Count')
    axes[0].axvline(x=np.mean(dark_confs), color='blue', linestyle='--', label=f'Mean={np.mean(dark_confs):.3f}')
    axes[0].legend()
    
    axes[1].hist(bright_confs, bins=30, color='#2ecc71', alpha=0.7, edgecolor='black')
    axes[1].set_title(f'Bright Images (n={len(bright_confs)})\ngalaxy + mind_5000_8')
    axes[1].set_xlabel('Confidence')
    axes[1].set_ylabel('Count')
    axes[1].axvline(x=np.mean(bright_confs), color='blue', linestyle='--', label=f'Mean={np.mean(bright_confs):.3f}')
    axes[1].legend()
    
    plt.tight_layout()
    plt.savefig(f"{OUTPUT_DIR}/viz_conf_hist.png", dpi=150)
    plt.close()
    print(f"Saved: viz_conf_hist.png")

def plot_summary_table(dirs):
    """图4: 汇总表格"""
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.axis('off')
    
    rows = []
    rows.append(["Directory", "Total", "Pred:3", "Pred:4", "Pred:neg", "Avg Conf", "Min Conf", "Max Conf"])
    
    for d in sorted(dirs.keys()):
        files = dirs[d]["files"]
        if not files:
            continue
        total = len(files)
        c3 = sum(1 for f in files if f["pred"] == "3")
        c4 = sum(1 for f in files if f["pred"] == "4")
        cneg = sum(1 for f in files if f["pred"] == "negative")
        confs = [f["conf"] for f in files]
        avg_c = np.mean(confs)
        min_c = np.min(confs)
        max_c = np.max(confs)
        rows.append([d, str(total), str(c3), str(c4), str(cneg), f"{avg_c:.3f}", f"{min_c:.3f}", f"{max_c:.3f}"])
    
    table = ax.table(cellText=rows, cellLoc='center', loc='center',
                     colWidths=[0.25, 0.08, 0.08, 0.08, 0.1, 0.1, 0.1, 0.1])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2)
    
    # 表头样式
    for i in range(len(rows[0])):
        table[(0, i)].set_facecolor('#34495e')
        table[(0, i)].set_text_props(color='white', weight='bold')
    
    # 暗图行高亮
    for i, row in enumerate(rows[1:], 1):
        if "4000_8" in row[0] or "5000_2" in row[0]:
            for j in range(len(row)):
                table[(i, j)].set_facecolor('#ffeaa7')
    
    ax.set_title('Inference Results Summary (Dark rows = previously misclassified)', fontsize=12, pad=20)
    plt.tight_layout()
    plt.savefig(f"{OUTPUT_DIR}/viz_summary_table.png", dpi=150)
    plt.close()
    print(f"Saved: viz_summary_table.png")

if __name__ == "__main__":
    dirs = parse_results(RESULT_FILE)
    plot_prediction_distribution(dirs)
    plot_confidence_by_dir(dirs)
    plot_confidence_histogram(dirs)
    plot_summary_table(dirs)
    print(f"\nAll visualizations saved to: {OUTPUT_DIR}/")
