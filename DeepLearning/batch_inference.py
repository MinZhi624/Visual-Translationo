#!/usr/bin/env python3
"""
批量推理 picture 目录下所有图片，输出结果到 txt
"""
import os
import sys
sys.path.insert(0, '/home/minzhi/ws05_fourth_assessment/DeepLearning')

import torch
import cv2
import numpy as np
from PIL import Image
from torchvision import transforms
from train import TinyNumberCNN, IMG_H, IMG_W

# ==================== 配置 ====================
PICTURE_DIR = "/home/minzhi/ws05_fourth_assessment/picture"
OUTPUT_FILE = "/home/minzhi/ws05_fourth_assessment/DeepLearning/inference_results.txt"
MODEL_PATH = "/home/minzhi/ws05_fourth_assessment/DeepLearning/best.pth"

# 排除的目录（训练数据/增强数据）
EXCLUDE_DIRS = {"augmented", "Learning", "Other"}

# ==================== 加载模型 ====================
model = TinyNumberCNN(3)
model.load_state_dict(torch.load(MODEL_PATH, map_location='cpu', weights_only=True))
model.eval()

transform = transforms.Compose([
    transforms.Resize((IMG_H, IMG_W)),
    transforms.ToTensor(),
])

LABELS = ["3", "4", "negative"]

def predict_image(path):
    """对单张图推理"""
    img = Image.open(path).convert('L')
    x = transform(img).unsqueeze(0)
    with torch.no_grad():
        out = model(x)
        probs = torch.softmax(out, dim=1)
    pred = torch.argmax(probs).item()
    conf = probs[0, pred].item()
    return LABELS[pred], conf, probs[0].tolist()

# ==================== 主程序 ====================
if __name__ == "__main__":
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("批量推理结果\n")
        f.write(f"模型: {MODEL_PATH}\n")
        f.write(f"类别: 3, 4, negative\n")
        f.write("=" * 80 + "\n\n")
        
        # 遍历所有子目录
        for subdir in sorted(os.listdir(PICTURE_DIR)):
            if subdir in EXCLUDE_DIRS:
                continue
            
            subpath = os.path.join(PICTURE_DIR, subdir)
            if not os.path.isdir(subpath):
                continue
            
            png_files = sorted([f for f in os.listdir(subpath) if f.endswith('.png')])
            if not png_files:
                continue
            
            print(f"\n处理目录: {subdir} ({len(png_files)} 张)")
            f.write(f"\n{'='*80}\n")
            f.write(f"目录: {subdir} (共 {len(png_files)} 张)\n")
            f.write(f"{'='*80}\n")
            f.write(f"{'文件名':<20} {'预测':<10} {'置信度':<10} {'prob[3]':<12} {'prob[4]':<12} {'prob[neg]':<12}\n")
            f.write("-" * 80 + "\n")
            
            stats = {"3": 0, "4": 0, "negative": 0}
            
            for fname in png_files:
                path = os.path.join(subpath, fname)
                try:
                    pred, conf, prob_list = predict_image(path)
                    stats[pred] += 1
                    f.write(f"{fname:<20} {pred:<10} {conf:<10.4f} {prob_list[0]:<12.6f} {prob_list[1]:<12.6f} {prob_list[2]:<12.6f}\n")
                except Exception as e:
                    f.write(f"{fname:<20} ERROR: {e}\n")
            
            # 目录统计
            f.write("-" * 80 + "\n")
            total = sum(stats.values())
            f.write(f"统计: 3={stats['3']} ({stats['3']/total*100:.1f}%), 4={stats['4']} ({stats['4']/total*100:.1f}%), negative={stats['negative']} ({stats['negative']/total*100:.1f}%)\n")
            print(f"  3: {stats['3']}, 4: {stats['4']}, negative: {stats['negative']}")
        
        f.write(f"\n{'='*80}\n")
        f.write("推理完成\n")
    
    print(f"\n结果已保存: {OUTPUT_FILE}")
