---
title: "[PyTorch] State_dict란?"
author: knowgyu
description: " "
date: 2023-11-15 12:08:32 +0900
math: true
categories: [AI & CV, PyTorch]
tags: [PyTorch]
---

https://tutorials.pytorch.kr/recipes/recipes/what_is_state_dict.html<br>

## STATE_DICT란?
---
PyTorch에서 `torch.nn.Module` 모델의 학습 가능한 매개변수(가중치와 편향)들은 모델의 배개변수에 포함되어 있습니다.
(`model.parameters()`로 접근) `state_dict`는 간단히 말해 **각 계층을 매개변수 텐서로 매핑되는 Python 사전(dict) 객체**입니다.

### 개요
`state_dict`는 PyTorch에서 모델을 <mark style="background: #ADCCFFA6;">저장하거나 불러오는 데</mark> 필수적인 항목.
`state_dict`객체는 Python dict이기에 쉽게 저장, 업데이트, 변경 및 복원할 수 있으며 이는 모델과 옵티마이저에 **모듈성**을 제공
<mark style="background: #ADCCFFA6;">학습가능한 매개변수를 갖는 계층</mark>(합성곱, 선형 레이어 등) 및 <mark style="background: #ADCCFFA6;">버퍼</mark>들(batchnorm의 running_mean)만 모델의 `state_dict` 항목을 가짐

옵티마이저 객체(`torch.optim`) 또한 상태 뿐만 아니라 사용된 하이퍼 파라미터 정보가 포함된 `state_dict`을 갖습니다.

### 예시
```python
import torch
import torch.nn as nn
import torch.optim as optim

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(3, 6, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(6, 16, 5)
        self.fc1 = nn.Linear(16 * 5 * 5, 120)
        self.fc2 = nn.Linear(120, 84)
        self.fc3 = nn.Linear(84, 10)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = x.view(-1, 16 * 5 * 5)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

net = Net()

optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

# 모델의 state_dict 출력
print("Model's state_dict:")
for param_tensor in net.state_dict():
    print(param_tensor, "\t", net.state_dict()[param_tensor].size())

print()

# 옵티마이저의 state_dict 출력
print("Optimizer's state_dict:")
for var_name in optimizer.state_dict():
    print(var_name, "\t", optimizer.state_dict()[var_name])
```

### 출력 결과
```
Model's state_dict:
conv1.weight 	 torch.Size([6, 3, 5, 5])
conv1.bias 	 torch.Size([6])
conv2.weight 	 torch.Size([16, 6, 5, 5])
conv2.bias 	 torch.Size([16])
fc1.weight 	 torch.Size([120, 400])
fc1.bias 	 torch.Size([120])
fc2.weight 	 torch.Size([84, 120])
fc2.bias 	 torch.Size([84])
fc3.weight 	 torch.Size([10, 84])
fc3.bias 	 torch.Size([10])

Optimizer's state_dict:
state 	 {}
param_groups 	 [{'lr': 0.001, 'momentum': 0.9, 'dampening': 0, 'weight_decay': 0, 'nesterov': False, 'maximize': False, 'foreach': None, 'differentiable': False, 'params': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]}]
```
