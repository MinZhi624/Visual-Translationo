#!/usr/bin/env python3
"""
极简装甲板数字识别训练脚本
3 类: 3, 4, negative
输入: 灰度图 28x20
"""
import os
import random
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from torchvision import transforms
from PIL import Image, ImageFilter
import cv2

# ==================== 配置 ====================
DATA_DIR = "/home/minzhi/ws05_fourth_assessment/picture/Learning"
SAVE_DIR = "/home/minzhi/ws05_fourth_assessment/DeepLearning"
EPOCHS = 80
BATCH_SIZE = 32
LR = 0.001
IMG_H, IMG_W = 28, 20
NUM_CLASSES = 3
CLASS_NAMES = ["3", "4", "negative"]
SEED = 42

random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)

# ==================== 自动生成 negative ====================
def generate_negative_samples(src_dir, dst_dir, num=150):
    """从原图生成各种 noise / blur / blank 作为 negative"""
    os.makedirs(dst_dir, exist_ok=True)
    # 收集所有源图
    src_files = []
    for cls in ["3", "4"]:
        d = os.path.join(src_dir, cls)
        if os.path.isdir(d):
            src_files += [os.path.join(d, f) for f in os.listdir(d) if f.endswith('.png')]
    
    count = 0
    for i in range(num):
        # 随机选一种生成方式
        mode = random.randint(0, 4)
        img = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
        
        if mode == 0:
            # 纯高斯噪声
            img = np.random.normal(64, 40, (IMG_H, IMG_W)).astype(np.uint8)
        elif mode == 1:
            # 空白图 + 少量噪声
            img = np.random.normal(30, 10, (IMG_H, IMG_W)).astype(np.uint8)
        elif mode == 2 and src_files:
            # 取原图极度模糊
            src = cv2.imread(random.choice(src_files), cv2.IMREAD_GRAYSCALE)
            if src is not None:
                src = cv2.resize(src, (IMG_W, IMG_H))
                img = cv2.GaussianBlur(src, (15, 15), 5)
        elif mode == 3 and src_files:
            # 原图随机打乱像素
            src = cv2.imread(random.choice(src_files), cv2.IMREAD_GRAYSCALE)
            if src is not None:
                src = cv2.resize(src, (IMG_W, IMG_H))
                flat = src.flatten()
                np.random.shuffle(flat)
                img = flat.reshape(IMG_H, IMG_W)
        elif mode == 4 and src_files:
            # 原图加大量椒盐噪声
            src = cv2.imread(random.choice(src_files), cv2.IMREAD_GRAYSCALE)
            if src is not None:
                src = cv2.resize(src, (IMG_W, IMG_H))
                noise = np.random.choice([0, 255], size=(IMG_H, IMG_W), p=[0.5, 0.5]).astype(np.uint8)
                mask = (noise > 0) | (noise == 0)
                img = src.copy()
                img[noise == 0] = 0
                img[noise == 255] = 255
        
        img = np.clip(img, 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(dst_dir, f"neg_{i:04d}.png"), img)
        count += 1
    
    print(f"[Negative] 生成 {count} 张 -> {dst_dir}")
    return count

# ==================== Dataset ====================
class ArmorNumberDataset(Dataset):
    def __init__(self, data_dir, class_names, transform=None):
        self.data_dir = data_dir
        self.class_names = class_names
        self.transform = transform
        self.samples = []  # (path, label_idx)
        
        for idx, cls_name in enumerate(class_names):
            cls_dir = os.path.join(data_dir, cls_name)
            if not os.path.isdir(cls_dir):
                continue
            for f in sorted(os.listdir(cls_dir)):
                if f.endswith('.png'):
                    self.samples.append((os.path.join(cls_dir, f), idx))
        
        print(f"Dataset: {len(self.samples)} samples")
        for idx, name in enumerate(class_names):
            cnt = sum(1 for _, l in self.samples if l == idx)
            print(f"  [{name}] {cnt} samples")
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        path, label = self.samples[idx]
        # PIL 打开灰度图
        img = Image.open(path).convert('L')
        if self.transform:
            img = self.transform(img)
        return img, label

# ==================== 数据增强 ====================
train_transform = transforms.Compose([
    transforms.Resize((IMG_H, IMG_W)),
    transforms.RandomAffine(degrees=5, translate=(0.05, 0.05)),
    transforms.ColorJitter(brightness=0.4, contrast=0.4),
    transforms.RandomApply([transforms.GaussianBlur(3, sigma=(0.1, 1.0))], p=0.3),
    transforms.ToTensor(),  # -> [0,1]
])

val_transform = transforms.Compose([
    transforms.Resize((IMG_H, IMG_W)),
    transforms.ToTensor(),
])

# ==================== 极简模型 ====================
class TinyNumberCNN(nn.Module):
    def __init__(self, num_classes=3):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, padding=1),   # 28x20
            nn.ReLU(),
            nn.MaxPool2d(2),                              # 14x10
            nn.Conv2d(8, 16, kernel_size=3, padding=1),   # 14x10
            nn.ReLU(),
            nn.MaxPool2d(2),                              # 7x5
        )
        
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(16 * 7 * 5, 32),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(32, num_classes),
        )
    
    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

# ==================== 训练 ====================
def train():
    device = torch.device("cpu")
    print(f"Device: {device}")
    
    # 1. 生成 negative
    neg_dir = os.path.join(DATA_DIR, "negative")
    if not os.path.isdir(neg_dir) or len(os.listdir(neg_dir)) == 0:
        generate_negative_samples(DATA_DIR, neg_dir, num=150)
    else:
        print(f"[Negative] 已存在 {len(os.listdir(neg_dir))} 张")
    
    # 2. 加载数据
    full_dataset = ArmorNumberDataset(DATA_DIR, CLASS_NAMES, transform=None)
    
    # 划分训练集/验证集 8:2
    n_total = len(full_dataset)
    n_train = int(n_total * 0.8)
    n_val = n_total - n_train
    train_ds, val_ds = random_split(full_dataset, [n_train, n_val])
    
    # 分别设置 transform
    train_ds.dataset.transform = train_transform
    val_ds.dataset.transform = val_transform
    
    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False)
    
    # 3. 模型
    model = TinyNumberCNN(NUM_CLASSES).to(device)
    print(model)
    total = sum(p.numel() for p in model.parameters())
    print(f"Total params: {total:,}")
    
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LR)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=30, gamma=0.5)
    
    best_acc = 0.0
    best_path = os.path.join(SAVE_DIR, "best.pth")
    
    for epoch in range(1, EPOCHS + 1):
        # ---- train ----
        model.train()
        train_loss = 0.0
        train_correct = 0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(imgs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item() * imgs.size(0)
            _, pred = torch.max(outputs, 1)
            train_correct += (pred == labels).sum().item()
        
        train_loss /= len(train_loader.dataset)
        train_acc = train_correct / len(train_loader.dataset)
        
        # ---- val ----
        model.eval()
        val_loss = 0.0
        val_correct = 0
        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(device), labels.to(device)
                outputs = model(imgs)
                loss = criterion(outputs, labels)
                val_loss += loss.item() * imgs.size(0)
                _, pred = torch.max(outputs, 1)
                val_correct += (pred == labels).sum().item()
        
        val_loss /= len(val_loader.dataset)
        val_acc = val_correct / len(val_loader.dataset)
        
        scheduler.step()
        
        if val_acc > best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), best_path)
            tag = "  <-- BEST"
        else:
            tag = ""
        
        if epoch % 10 == 0 or epoch == 1 or tag:
            print(f"Epoch {epoch:03d}: train_loss={train_loss:.4f}, train_acc={train_acc:.4f}, val_loss={val_loss:.4f}, val_acc={val_acc:.4f}{tag}")
    
    print(f"\nBest val acc: {best_acc:.4f}")
    print(f"Model saved: {best_path}")
    
    # 4. 导出 ONNX
    export_onnx(model, os.path.join(SAVE_DIR, "number_cnn.onnx"))

# ==================== 导出 ONNX ====================
def export_onnx(model, path):
    model.eval()
    dummy = torch.randn(1, 1, IMG_H, IMG_W)
    torch.onnx.export(
        model, dummy, path,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
        opset_version=11,
    )
    print(f"ONNX exported: {path}")
    
    # 验证
    import onnx
    onnx_model = onnx.load(path)
    onnx.checker.check_model(onnx_model)
    print("ONNX validation passed.")

if __name__ == "__main__":
    train()
