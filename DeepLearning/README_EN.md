# Number classifier for MNIST-like dataset

adapt to RoboMaster autoaim task

## Introduction

Pytorch based Convolutional Neural Network(CNN), which can be used to classify images

Dependencies:
- Pytorch
- numpy
- OpenCV
- ONNX

## Get started

### 0. Install dependencies

```
pip install -r requirements.txt
```

### 1. Prepare dataset. 

unzip the armors.zip in dataset/ and use the following script to generate armors.txt  

```
import os

def mktxt():
  path = "your_path"
  idx = 7000
  txt = open(path+"armors.txt","w")
  while os.path.exists(path+"armors/{}.jpg".format(idx)):
    txt.write(path+"armors/{}.jpg\n".format(idx))
    idx+=1
  txt.close()

mktxt()

```
### 2. Train 

adjust the cfg_path in main() of src/train.py to your path of armor.txt, run

```
python train.py
```

