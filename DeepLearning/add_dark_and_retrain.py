#!/usr/bin/env python3
"""
把暗图加入训练集（取80%），剩余20%保留在原目录做检验
然后重新训练模型
"""
import os
import shutil
import random

random.seed(42)

# 配置
SRC_DIRS = {
    "3": [
        "/home/minzhi/ws05_fourth_assessment/picture/3_mind_4000_8",
        "/home/minzhi/ws05_fourth_assessment/picture/3_mind_5000_2",
    ],
}
LEARN_DIR = "/home/minzhi/ws05_fourth_assessment/picture/Learning"
TRAIN_RATIO = 0.8

# 1. 处理每个源目录：划分训练/检验
for label, src_list in SRC_DIRS.items():
    dst_train = os.path.join(LEARN_DIR, label)
    os.makedirs(dst_train, exist_ok=True)
    
    for src_dir in src_list:
        if not os.path.isdir(src_dir):
            print(f"跳过不存在的目录: {src_dir}")
            continue
        
        files = [f for f in os.listdir(src_dir) if f.endswith('.png')]
        random.shuffle(files)
        
        n_train = int(len(files) * TRAIN_RATIO)
        train_files = files[:n_train]
        test_files = files[n_train:]
        
        # 复制训练集到 Learning
        for f in train_files:
            src_path = os.path.join(src_dir, f)
            # 重命名避免冲突: mind4000_roi_100.png
            prefix = os.path.basename(src_dir).replace('3_', '') + '_'
            dst_name = prefix + f
            dst_path = os.path.join(dst_train, dst_name)
            shutil.copy2(src_path, dst_path)
        
        print(f"[{os.path.basename(src_dir)}] 总{len(files)}张 -> 训练{n_train}张, 检验{len(test_files)}张")

# 2. 清空旧的 negative，让训练脚本重新生成
neg_dir = os.path.join(LEARN_DIR, "negative")
if os.path.isdir(neg_dir):
    for f in os.listdir(neg_dir):
        os.remove(os.path.join(neg_dir, f))
    print("[negative] 已清空，训练时将重新生成150张")

# 3. 统计最终训练集
total_train = 0
for label in ["3", "4"]:
    d = os.path.join(LEARN_DIR, label)
    cnt = len([f for f in os.listdir(d) if f.endswith('.png')])
    print(f"[训练集/{label}] 共 {cnt} 张")
    total_train += cnt

print(f"\n训练集总计: {total_train} 张")

# 4. 重新训练
print("\n开始训练...")
os.system("cd /home/minzhi/ws05_fourth_assessment/DeepLearning && python train.py")

print("\n训练完成！")
print("新模型已保存到: /home/minzhi/ws05_fourth_assessment/DeepLearning/best.pth")
print("ONNX已导出到: /home/minzhi/ws05_fourth_assessment/src/armor_plate_identification/model/number_cnn.onnx")
