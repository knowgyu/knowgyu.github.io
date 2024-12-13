---
title: "Data Augmentation을 위해 Albumentations 사용해보기"
author: knowgyu
description: " "
date: 2023-11-13 16:01:17 +0900
math: true
categories: [Deep Learning, Dataset]
tags: [Augmentation, Albumentations]
---

## 1. Albumentations란?

**Albumentations**는 **다양한 이미지 변형 기법**을 제공하는 Python 라이브러리입니다.  
주요 특징은 **빠른 속도**, **편리함**, 그리고 **다양한 기능**입니다.

> **주요 이미지 변형 기법**:  
> - **회전**  
> - **이동**  
> - **확대/축소**  
> - **반전**  
> - **색상 변경**  
> - **잡음 추가**  
{: .highlight }

이 변형들은 **데이터셋을 다양화**하여 모델의 **일반화 능력**을 높일 수 있도록 돕습니다.

---

## 2. Albumentations 설치하기

```bash
pip install albumentations
```

---

## 3. 데이터 증강 파이프라인 구성하기

아래의 코드는 **YOLO 형식**의 바운딩 박스를 지원하는 **Albumentations**를 사용해 데이터 증강을 수행합니다.

### 3.1 필요한 라이브러리 불러오기

```python
import os
import cv2
import numpy as np
import albumentations as A
from tqdm import tqdm
```

### 3.2 데이터 증강 함수 정의

```python
def apply_transformations(image, bboxes, class_labels):
    """
    이미지와 바운딩 박스에 변형을 적용하는 함수.
    """
    transformed = transform(image=image, bboxes=bboxes, class_labels=class_labels)
    augmented_image = transformed['image']
    augmented_bboxes = transformed['bboxes']
    augmented_class_labels = transformed['class_labels']
    return augmented_image, augmented_bboxes, augmented_class_labels

def save_augmented_data(image_file, augmented_image, augmented_bboxes, augmented_class_labels, output_dir):
    """
    증강된 데이터를 이미지 파일과 라벨 파일로 저장하는 함수.
    """
    cv2.imwrite(os.path.join(output_dir, image_file), augmented_image)
    label_file = image_file.replace('.jpg', '.txt')
    with open(os.path.join(output_dir, label_file), 'w') as f:
        for i in range(len(augmented_bboxes)):
            f.write(f"{augmented_class_labels[i]} {' '.join(map(str, augmented_bboxes[i]))}\n")
```

---

## 4. 전체 증강 스크립트

아래는 데이터셋의 모든 이미지를 증강하는 스크립트입니다.

```python
def main(input_dir, output_dir, n):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    image_files = [f for f in os.listdir(input_dir) if f.endswith('.jpg')]
    for image_file in tqdm(image_files, desc="Processing Images"):
        image = cv2.imread(os.path.join(input_dir, image_file))
        label_file = image_file.replace('.jpg', '.txt')

        with open(os.path.join(input_dir, label_file), 'r') as f:
            lines = f.readlines()
        
        bboxes = np.array([line.split()[1:5] for line in lines], dtype=float)
        class_labels = np.array([line.split()[0] for line in lines], dtype=int)

        for i in range(n):
            try:
                augmented_image, augmented_bboxes, augmented_class_labels = apply_transformations(image, bboxes, class_labels)
                output_image_file = f"aug_{i}_{image_file}"
                save_augmented_data(output_image_file, augmented_image, augmented_bboxes, augmented_class_labels, output_dir)
            except Exception as e:
                print(f"에러: {e}, 파일 이름: {image_file}")
```

---

## 5. 증강 파라미터 설정하기

다양한 변형 기법을 `A.Compose`를 통해 적용합니다.

```python
transform = A.Compose([
    A.FancyPCA(alpha=0.1, p=0.5),  # 색상 변형
    A.RandomBrightnessContrast(brightness_limit=0.1, contrast_limit=0.1, p=0.5),  # 밝기/대비
    A.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=10, val_shift_limit=15, p=0.5),  # 색상 변형
    A.OneOf([
        A.GaussNoise(var_limit=(1, 7), p=0.5),  # 가우시안 잡음 추가
        A.ISONoise(color_shift=(0.01, 0.05), intensity=(0.1, 0.5), p=0.5),
        A.JpegCompression(quality_lower=90, quality_upper=100, p=0.5)
    ], p=0.7),
    A.OneOf([
        A.Blur(blur_limit=(1, 2), p=0.5),
        A.CLAHE(p=0.5)  # 히스토그램 균등화
    ], p=0.25)
], bbox_params=A.BboxParams(format='yolo', label_fields=['class_labels']))
```

---

## 6. 스크립트 실행 방법

**파라미터**:
- `--input`: 입력 데이터셋 경로
- `--output`: 증강된 데이터를 저장할 경로
- `--n`: 이미지당 생성할 증강 개수

### 실행 예제

```bash
python augment.py --input ./input_data --output ./augmented_data --n 5
```

전체 코드는 아래 저장소에 있습니다.  
[https://github.com/knowgyu/dataset-tools](https://github.com/knowgyu/dataset-tools)


---

## 7. 결론 및 요약

**Albumentations**를 사용하면 다양한 이미지 변형 기법을 빠르게 적용할 수 있습니다.  

---
> [Albumentations 공식 GitHub](https://github.com/albumentations-team/albumentations){: .prompt-tip }
> [Albumentations Documentation](https://albumentations.ai/docs/){: .prompt-tip }
