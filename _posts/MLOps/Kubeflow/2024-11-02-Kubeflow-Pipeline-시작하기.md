---
title: "Kubeflow Pipeline을 활용한 YOLO 학습 파이프라인 시작하기"
author: knowgyu
description: " "
date: 2023-11-23 03:52:04 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

## Kubeflow Pipeline 개요

이전 글들에서 언급했듯, Kubeflow를 사용하려면 **컴포넌트**와 **파이프라인**을 함께 작성해야 합니다.

컴포넌트는 함수 형태로 표현되며, 필요한 설정값(Config)을 입력받아 동작합니다.

하나 이상의 컴포넌트를 만들었다면, 이를 어떤 순서로 실행할지 정의하는 파이프라인도 함께 작성해야 합니다.

## YOLOv7 예시

---

YOLOv7 저장소: [https://github.com/WongKinYiu/yolov7](https://github.com/WongKinYiu/yolov7)

YOLOv7을 학습시키기 위해 GitHub에서 저장소를 복제한 후 필요한 라이브러리를 설치하고 아래 명령어를 입력합니다.

```bash
python train.py --workers 8 --device 0 --batch-size 32 --data data/coco.yaml --img 640 640 --cfg cfg/training/yolov7.yaml --weights '' --name yolov7 --hyp data/hyp.scratch.p5.yaml
```

`os` 라이브러리의 `system` 함수나 `subprocess` 를 이용하면 파이썬에서 우분투 셸 명령어를 호출할 수 있습니다.

따라서 아래와 같이 학습용 컴포넌트를 작성할 수 있습니다.

```python
@create_component_from_func
def train():
    import subprocess
    command = ["python", "train.py", "--workers", "8", "--device", "0", "--batch-size", "32", "--data", "data/coco.yaml", "--img", "640", "640", "--cfg", "cfg/training/yolov7.yaml", "--weights", "", "--name", "yolov7", "--hyp", "data/hyp.scratch.p5.yaml"]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    if process.returncode != 0:
        print(f"Error occurred: {stderr.decode('utf-8')}")
    else:
        print(f"Output: {stdout.decode('utf-8')}")
```

다만 이 방식은 학습 후 아티팩트를 저장하거나, 학습 전 데이터셋 준비 단계와 자연스럽게 연결하는 데 한계가 있습니다.

## YOLOv8

YOLOv8 문서: [https://docs.ultralytics.com/](https://docs.ultralytics.com/)

YOLOv8은 다른 버전에 비해 비교적 수월하게 설치하고 학습할 수 있습니다.

`pip` 로 `ultralytics` 패키지를 설치하면 의존성 문제도 비교적 적어 빠르게 실험할 수 있습니다.

따라서 먼저 사용하기 쉬운 YOLOv8 기반 학습 파이프라인을 구성한 뒤, 이후 YOLOv7 학습 파이프라인으로 확장하려고 합니다.

![Untitled](/assets/img/kubeflow/kubepipe000.png)
