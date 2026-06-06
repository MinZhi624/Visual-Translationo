import numpy as np
import torch
from torch import nn as nn
from torch.nn import functional as F

'''
Zenet:自己弄的网络
Lenet:Lenet-5网络
对于数字识别任务,Lenet5已经足够胜任
'''
class Zenet(nn.Module):
  def __init__(self, num_classes):
    super(Zenet, self).__init__()
    self.ConvBNReLU = nn.Sequential(
      nn.Conv2d(1,16, kernel_size=3, stride=1, padding=2),
      nn.BatchNorm2d(16),
      nn.ReLU(),
    )
    self.DWConvBNReLU1 = nn.Sequential(
      nn.Conv2d(16, 16, kernel_size=3),
      nn.BatchNorm2d(16),
      nn.ReLU(),
      nn.Conv2d(16, 64, kernel_size=1),
      nn.BatchNorm2d(64),
      nn.ReLU()
    ) 
    self.DWConvBNReLU2 = nn.Sequential(
      nn.Conv2d(64, 64, kernel_size=3),
      nn.BatchNorm2d(64),
      nn.ReLU(),
      nn.Conv2d(64, 256, kernel_size=1),
      nn.BatchNorm2d(256),
      nn.ReLU()
    )
    self.avgpool = nn.AdaptiveAvgPool2d((1,1))
    self.classifer = nn.Sequential(
      nn.Flatten(),
      nn.Dropout(0.2), 
      nn.Linear(256, num_classes)
    )

  def forward(self, x):
    features = self.ConvBNReLU(x)
    features = self.DWConvBNReLU1(features)
    features = self.DWConvBNReLU2(features)
    output = self.avgpool(features)
    output = self.classifer(output)
    return output

'''
Lenet-5
'''
Lenet5 = nn.Sequential(
    nn.Conv2d(1, 6, kernel_size=5, padding=2), 
    nn.ReLU(),
    nn.MaxPool2d(kernel_size=2, stride=2),
    nn.Conv2d(6, 16, kernel_size=5), 
    nn.ReLU(),
    nn.MaxPool2d(kernel_size=2, stride=2),
    nn.Flatten(),
    nn.Dropout(),
    nn.Linear(16 * 5 * 5, 120), 
    nn.ReLU(),
    nn.Linear(120, 84), 
    nn.ReLU(),
    nn.Linear(84, 11)
    )    