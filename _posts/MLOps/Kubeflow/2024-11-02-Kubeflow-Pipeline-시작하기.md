---
title: "Kubeflow Pipeline을 활용한 Yolo 학습 파이프라인 시작하기"
author: knowgyu
description: " "
date: 2023-11-23 03:52:04 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

# Kubeflow

---

이전의 문서들에서 언급했듯 Kubeflow를 사용하기 위해서는 **컴포넌트**와 **파이프라인**을 작성해야 합니다.

컴포넌트의 경우 함수의 형태로 표현되며, 필요한 인자(Config)들을 받아 동작합니다.

하나 이상의 컴포넌트를 작성했다면, 이를 실행하기 위해 파이프라인을 작성해야합니다.

# YOLOv7

---

YOLOv7 repo : [https://github.com/WongKinYiu/yolov7](https://github.com/WongKinYiu/yolov7)

YOLOv7을 학습시키기 위해 Github에서 저장소를 복제한 후 필요한 라이브러리를 설치하고 아래 명령어를 입력합니다.

```bash
python train.py --workers 8 --device 0 --batch-size 32 --data data/coco.yaml --img 640 640 --cfg cfg/training/yolov7.yaml --weights '' --name yolov7 --hyp data/hyp.scratch.p5.yaml
```

`os` 라이브러리의 `system` 함수 혹은 `subprocess` 를 이용해 파이썬에서 우분투 셸 명령어를 호출할 수 있기에, 

아래와 같이 컴포넌트를 작성할 수 있습니다.

```bash
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

하지만, 학습 후 아티팩트를 저장하거나 학습 전 데이터셋을 조작 등 학습 전, 후 컴포넌트와 연결하는 것이 어려울 것이라 예상됩니다.

## YOLOv8

---

YOLOv8 docs : [https://docs.ultralytics.com/](https://docs.ultralytics.com/)

YOLOv8의 경우 비교적 다른 버전에 비해 수월하게 설치 및 학습이 가능합니다.

pip를 이용해 `ultralytics` 패키지를 설치하면 의존성에 관한 문제도 크게 신경 쓸 필요 없습니다.

따라서 먼저 사용하기 쉬운 YOLOv8을 이용해 학습 파이프라인을 구성해본 후 YOLOv7 학습 파이프라인을 작성하려 합니다.

![Untitled](/assets/img/kubeflow/kubepipe000.png)
