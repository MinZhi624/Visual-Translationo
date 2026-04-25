#!/usr/bin/env python3
"""
生成残缺增强图片，并测试模型鲁棒性
"""
import os
import random
import numpy as np
import cv2
import torch
from torchvision import transforms
from PIL import Image

# ==================== 配置 ====================
SRC_DIR = "/home/minzhi/ws05_fourth_assessment/picture/Learning"
AUG_DIR = "/home/minzhi/ws05_fourth_assessment/picture/augmented"
MODEL_PATH = "/home/minzhi/ws05_fourth_assessment/DeepLearning/best.pth"
IMG_H, IMG_W = 28, 20
SEED = 42

random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)
os.makedirs(os.path.join(AUG_DIR, "3"), exist_ok=True)
os.makedirs(os.path.join(AUG_DIR, "4"), exist_ok=True)

# ==================== 加载模型 ====================
import sys
sys.path.insert(0, '/home/minzhi/ws05_fourth_assessment/DeepLearning')
from train import TinyNumberCNN

model = TinyNumberCNN(3)
model.load_state_dict(torch.load(MODEL_PATH, map_location='cpu', weights_only=True))
model.eval()

transform = transforms.Compose([
    transforms.Resize((IMG_H, IMG_W)),
    transforms.ToTensor(),
])

# ==================== 残缺增强函数 ====================
def augment_broken(img_np, method):
    """
    生成残缺图片
    img_np: numpy array (H, W), uint8
    method: 增强方法编号
    """
    h, w = img_np.shape
    out = img_np.copy()
    
    if method == 0:
        # 随机矩形遮挡（黑色块）
        bw = random.randint(w//4, w//2)
        bh = random.randint(h//4, h//2)
        bx = random.randint(0, w - bw)
        by = random.randint(0, h - bh)
        out[by:by+bh, bx:bx+bw] = 0
    
    elif method == 1:
        # 随机矩形遮挡（灰色块，模拟半透明遮挡）
        bw = random.randint(w//4, w//2)
        bh = random.randint(h//4, h//2)
        bx = random.randint(0, w - bw)
        by = random.randint(0, h - bh)
        out[by:by+bh, bx:bx+bw] = random.randint(20, 60)
    
    elif method == 2:
        # 边缘缺失（模拟ROI提取偏移）
        side = random.choice(['top', 'bottom', 'left', 'right'])
        if side == 'top':
            out[:h//4, :] = 0
        elif side == 'bottom':
            out[-h//4:, :] = 0
        elif side == 'left':
            out[:, :w//4] = 0
        else:
            out[:, -w//4:] = 0
    
    elif method == 3:
        # 水平条带遮挡（模拟灯条或横杆遮挡）
        by = random.randint(0, h - h//4)
        out[by:by+h//4, :] = 0
    
    elif method == 4:
        # 垂直条带遮挡
        bx = random.randint(0, w - w//4)
        out[:, bx:bx+w//4] = 0
    
    elif method == 5:
        # 随机像素腐蚀（10%~30%像素置0）
        mask = np.random.rand(h, w) < random.uniform(0.1, 0.3)
        out[mask] = 0
    
    elif method == 6:
        # 中心区域穿孔
        cx, cy = w // 2, h // 2
        rw, rh = w // 4, h // 4
        out[cy-rh:cy+rh, cx-rw:cx+rw] = random.randint(10, 40)
    
    elif method == 7:
        # 对角线遮挡
        for i in range(h):
            for j in range(w):
                if abs(i - j) < 3 or abs(i + j - w) < 3:
                    if random.random() < 0.7:
                        out[i, j] = 0
    
    elif method == 8:
        # 局部高斯模糊块
        bw = random.randint(w//3, w//2)
        bh = random.randint(h//3, h//2)
        bx = random.randint(0, w - bw)
        by = random.randint(0, h - bh)
        patch = out[by:by+bh, bx:bx+bw]
        patch = cv2.GaussianBlur(patch, (7, 7), 2)
        out[by:by+bh, bx:bx+bw] = patch
    
    elif method == 9:
        # 混合：遮挡 + 噪声
        bw = random.randint(w//4, w//2)
        bh = random.randint(h//4, h//2)
        bx = random.randint(0, w - bw)
        by = random.randint(0, h - bh)
        noise = np.random.randint(0, 50, (bh, bw), dtype=np.uint8)
        out[by:by+bh, bx:bx+bw] = noise
    
    return out

# ==================== 生成并测试 ====================
def test_class(cls_name, num_aug_per_img=5):
    src_files = sorted([f for f in os.listdir(os.path.join(SRC_DIR, cls_name)) if f.endswith('.png')])
    correct = 0
    total = 0
    
    print(f"\n=== 测试类别 [{cls_name}] ===")
    print(f"源图数量: {len(src_files)}, 每张生成 {num_aug_per_img} 张残缺图")
    
    for i, fname in enumerate(src_files):
        src_path = os.path.join(SRC_DIR, cls_name, fname)
        img = cv2.imread(src_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        img = cv2.resize(img, (IMG_W, IMG_H))
        
        for aug_id in range(num_aug_per_img):
            method = random.randint(0, 9)
            aug_img = augment_broken(img, method)
            
            # 保存增强图
            save_name = f"{cls_name}_{fname[:-4]}_aug{aug_id}_m{method}.png"
            save_path = os.path.join(AUG_DIR, cls_name, save_name)
            cv2.imwrite(save_path, aug_img)
            
            # 推理测试
            pil_img = Image.fromarray(aug_img)
            x = transform(pil_img).unsqueeze(0)
            with torch.no_grad():
                out = model(x)
                probs = torch.softmax(out, dim=1)
            pred = torch.argmax(probs).item()
            pred_label = ['3', '4', 'negative'][pred]
            conf = probs[0, pred].item()
            
            if pred_label == cls_name:
                correct += 1
            
            total += 1
            
            # 只打印错误样本
            if pred_label != cls_name:
                print(f"  ❌ 错分: {save_name} -> pred={pred_label} (conf={conf:.3f}), probs=[{probs[0,0]:.3f}, {probs[0,1]:.3f}, {probs[0,2]:.3f}]")
    
    acc = correct / total if total > 0 else 0
    print(f"  总测试: {total}, 正确: {correct}, 准确率: {acc:.2%}")
    return correct, total

# ==================== 主函数 ====================
if __name__ == "__main__":
    c3, t3 = test_class("3", num_aug_per_img=5)
    c4, t4 = test_class("4", num_aug_per_img=5)
    
    total_correct = c3 + c4
    total = t3 + t4
    print(f"\n{'='*50}")
    print(f"总体结果: {total_correct}/{total} = {total_correct/total:.2%}")
    print(f"增强图片保存位置: {AUG_DIR}")
    print(f"{'='*50}")
