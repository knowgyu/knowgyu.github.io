---
title: "[PyTorch] float(), fuse(), eval()"
author: knowgyu
description: " "
date: 2023-11-15 12:09:49 +0900
math: true
categories: [Deep Learning, PyTorch]
tags: [PyTorch]
---

본 문서는 모델을 불러오는 과정에서 사용되는 `float()`, `fuse()`, `eval()`함수에 대해 다룹니다.

## fuse()
***
`fuse()` 함수는 여러개의 PyTorch 모듈을 하나의 모듈로 합칠 때 사용됩니다. 이는 memory access time과 kernel launch time을 줄여 성능을 향상시킬 수 있습니다.<br>
특히 elementwise addition, multiplication, and activation functions에서 유용합니다. 이러한 퓨전 프로세스는 연산들을 하나의 커널로 합치고, 메모리를 읽고 쓰는 횟수를 줄여줍니다.<br>
아래는 `torch.jit.script`를 사용해 함수를 합치는 예시입니다.<br>
```python
import torch

@torch.jit.script
def fused_gelu(x):
    return x * 0.5 * (1.0 + torch.erf(x / 1.41421))

# Usage
input_tensor = torch.tensor([1, 2, 3], dtype=torch.float32)
output_tensor = fused_gelu(input_tensor)
```

## eval()
***
`eval()` 함수는 PyTorch 모듈 혹은 모델을 평가 모드로 설정합니다. 평가 모드에선 배치 정규화 혹은 드롭아웃과 같은 모듈들이 학습 모드일때와 다르게 동작합니다.
평가 모드에선 배치 정규화는 배치 통계 대신 전체 모집단의 통계를 사용합니다. 그리고 드롭아웃 레이어들은 비활성화됩니다.
모델을 평가하거나 추론을 할 때 일정한 행동을 보장하려면 `eval()` 함수를 사용하는 것은 매우 중요합니다. 아래는 예시 입니다
```python
import torch

model = MyModel()
model.load_state_dict(torch.load('model.pth'))
model.eval()

input_tensor = torch.tensor([1, 2, 3], dtype=torch.float32)
output_tensor = model(input_tensor)
```

## float()
***

1. `float()`: The `float()` function is used to convert a PyTorch tensor to a float tensor. It returns a new tensor with the same data but with floating-point data type. This can be useful when you want to perform operations that require floating-point precision. Here's an example:

```python
import torch

int_tensor = torch.tensor([1, 2, 3])
float_tensor = int_tensor.float()

print(float_tensor)
# Output: tensor([1., 2., 3.])
```



https://coffeedjimmy.github.io/pytorch/2019/11/05/pytorch_nograd_vs_train_eval/
