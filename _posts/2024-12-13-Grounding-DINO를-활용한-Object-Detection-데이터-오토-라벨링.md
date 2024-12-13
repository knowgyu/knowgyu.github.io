---
title: "Grounding DINO를 활용한 Object Detection 데이터 오토 라벨링"
author: knowgyu
description: " "
date: 2023-10-18 15:49:06 +0900
math: true
categories: [Deep Learning, Dataset]
tags: [Auto Annotation, Grounding DINO]
---

**본 문서에서는 Zero-Shot Object Detector 모델인 Grounding DINO를 활용해 이미지 라벨링 작업을 자동화하는 방법을 다룹니다.**   
오토 라벨링을 통해 시간 소모를 최소화하고 효율적으로 데이터셋을 구축할 수 있습니다.

> Grounding DINO를 통해 자동으로 객체를 탐지하고 라벨링한 데이터를 검수할 수 있도록 준비합니다.  
{: .prompt-tip }

---

## 1. 이미지 라벨링 작업의 필요성

이미지 라벨링 작업은 **시간과 비용이 많이 소모되는 반복적 작업**입니다. 특히 **대규모 데이터셋**을 구축하려면 상당한 인력과 시간이 필요합니다. 하지만 Zero-Shot Object Detection 모델인 **Grounding DINO**를 활용하면 특정 카테고리의 객체를 빠르게 탐지하고 라벨링할 수 있습니다. 사용자는 결과만 검수하면 되므로 **생산성을 극대화**할 수 있습니다.

---

## 2. Grounding DINO 소개

Grounding DINO는 **텍스트 프롬프트**를 입력으로 받아 객체를 인식하는 **Zero-Shot Object Detector**입니다. 

### 특징

1. **Open-set Object Detection**: 제한된 객체 클래스 대신 사용자가 원하는 **카테고리 이름**이나 **참조 표현**을 입력하면 탐지할 수 있습니다.
2. **언어와 시각 모달 융합**: 
   - **Backbone**: Feature Extraction
   - **Neck**: Query Initialization
   - **Head**: Detection 결과 출력 (Grounding DINO는 이 과정에서 언어 정보를 융합합니다).

---

## 3. Auto Annotation 준비

### 3.1 GPU 설정 확인

GPU가 정상적으로 인식되는지 확인합니다.

```bash
nvidia-smi
```

### 3.2 환경 설정 및 Grounding DINO 설치

> 주피터 노트북을 활용해 코드를 따라하는 것을 권장합니다.
{: .prompt-tip }

1. **Grounding DINO 저장소 다운로드 및 설치**:
   ```bash
   git clone https://github.com/IDEA-Research/GroundingDINO.git
   cd GroundingDINO
   git checkout feature/more_compact_inference_api
   pip install -q -e .
   ```

2. **Supervision 패키지 설치**:
   ```bash
   pip uninstall -y supervision
   pip install -q supervision==0.6.0
   ```

### 3.3 Weights 및 Configuration 파일 준비

Grounding DINO를 실행하기 위해 필요한 Weight 파일과 Configuration 파일을 설정합니다.

```python
import os

HOME = os.getcwd()
CONFIG_PATH = os.path.join(HOME, "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py")
WEIGHTS_PATH = os.path.join(HOME, "weights", "groundingdino_swint_ogc.pth")

# Weights 파일 다운로드
!mkdir -p {HOME}/weights
!wget -q -P {HOME}/weights https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
```

---

## 4. Auto Annotation 테스트

### 4.1 예시 이미지 다운로드

```python
!mkdir -p {HOME}/data
!wget -q -P {HOME}/data https://media.roboflow.com/notebooks/examples/dog-3.jpeg
```

### 4.2 Grounding DINO 모델 로드

```python
from groundingdino.util.inference import Model

model = Model(model_config_path=CONFIG_PATH, model_checkpoint_path=WEIGHTS_PATH)
```

### 4.3 단일 이미지 오토 라벨링

```python
import cv2
import supervision as sv

# 설정
SOURCE_IMAGE_PATH = f"{HOME}/data/dog-3.jpeg"
CLASSES = ['person', 'dog']
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25

# 이미지 불러오기 및 라벨링
image = cv2.imread(SOURCE_IMAGE_PATH)
detections = model.predict_with_classes(
    image=image,
    classes=[f"all {cls}s" for cls in CLASSES],
    box_threshold=BOX_THRESHOLD,
    text_threshold=TEXT_THRESHOLD
)

# 결과 시각화
annotator = sv.BoxAnnotator()
labels = [f"{CLASSES[class_id]} {confidence:.2f}" for _, _, confidence, class_id, _ in detections]
annotated_frame = annotator.annotate(image.copy(), detections, labels)

sv.plot_image(annotated_frame, (16, 16))
```

---

## 5. 전체 데이터셋 오토 라벨링 및 저장

```python
from tqdm.notebook import tqdm

IMAGES_DIRECTORY = "./data"
ANNOTATIONS_DIRECTORY = "./annotations"

# 이미지 리스트 추출
image_paths = sv.list_files_with_extensions(directory=IMAGES_DIRECTORY, extensions=['jpg', 'jpeg', 'png'])

# 라벨링 및 저장
images, annotations = {}, {}
for image_path in tqdm(image_paths):
    image_name = image_path.name
    image = cv2.imread(str(image_path))
    detections = model.predict_with_classes(
        image=image,
        classes=[f"all {cls}s" for cls in CLASSES],
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD
    )
    images[image_name] = image
    annotations[image_name] = detections

# Pascal VOC 형식으로 저장
sv.Dataset(classes=CLASSES, images=images, annotations=annotations).as_pascal_voc(
    annotations_directory_path=ANNOTATIONS_DIRECTORY
)
```

---

## 6. 결론 및 요약

Grounding DINO를 활용하면 **텍스트 프롬프트**만으로 원하는 카테고리의 객체를 자동으로 라벨링할 수 있습니다.  
이를 통해 대규모 데이터셋 구축 시간을 절약하고 검수 작업만 수행하면 되므로 **효율성과 생산성**을 극대화할 수 있습니다.
