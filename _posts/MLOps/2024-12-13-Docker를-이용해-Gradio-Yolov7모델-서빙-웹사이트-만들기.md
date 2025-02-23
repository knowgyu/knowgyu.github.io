---
title: "Docker를 이용해 Gradio Yolov7모델 서빙 웹사이트 만들기"
author: knowgyu
description: " "
date: 2023-10-13 10:34:32 +0900
math: true
categories: [AI & CV, Gradio를 활용한 웹 기반 서빙]
tags: [MLOps, Yolo, Gradio, Docker]
---

# Docker를 활용한 Gradio 기반 YOLOv7 서빙 배포

이전 글에서는 Gradio를 활용해 YOLOv7 모델을 웹페이지 형태로 서빙하는 방법을 알아보았습니다.  
이번 글에서는 **Docker**를 활용해 환경을 패키징하고, 더 간편하게 배포하는 방법을 설명합니다.  

Docker를 활용하면 **로컬 환경의 의존성 문제**를 해결할 수 있고, **GPU**와 같은 하드웨어 리소스도 손쉽게 설정할 수 있습니다.

> 단, 이 게시글에서 사용하는 이미지의 경우 약 14GB 정도의 용량을 사용합니다.  
> 만약, 로컬에 CUDA가 설치되어 있고, 용량이 부족한 환경에서는 잘 고려해보시길 바랍니다!
{: .prompt-warning }

---

## 프로젝트 구조

먼저 프로젝트의 기본 디렉토리 구조를 확인해봅시다.

```plaintext
├── Dockerfile
├── README.md
├── docker-compose.yml
├── hubconf.py
├── main.py
├── models/
├── requirements.txt
└── utils/
```
### Github Repo
[https://github.com/knowgyu/gradio-yolov7](https://github.com/knowgyu/gradio-yolov7)

---

## Dockerfile: 컨테이너 환경 설정

**Dockerfile**은 컨테이너 이미지를 정의하는 파일입니다.  
PyTorch와 CUDA를 기반으로 필요한 라이브러리와 Python 환경을 설정합니다.

### Dockerfile 내용

```dockerfile
FROM pytorch/pytorch:2.0.1-cuda11.7-cudnn8-devel

# 패키지 설치 및 기본 설정
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    vim \
    wget \
    curl \
    libglib2.0-0 \
    libgl1-mesa-glx \  # OpenCV 의존성
    libsm6 \          
    libxext6 \        
    libxrender1 \     
    python3-pip \
    language-pack-ko \
    && rm -rf /var/lib/apt/lists/*

# 타임존 설정
ENV TZ=Asia/Seoul
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime

# Python 라이브러리 설치
COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# 작업 디렉토리 설정
WORKDIR /gradio

# 컨테이너 시작 시 기본 명령어
CMD ["python", "main.py"]
```

### 코드 설명
1. **PyTorch 이미지**: `pytorch/pytorch:2.0.1-cuda11.7-cudnn8-devel`를 사용하여 GPU와 CUDA 환경을 설정합니다.  
   - GPU가 필요 없다면 `pytorch/pytorch:2.0.1-cpu-py310` 이미지로 대체할 수 있습니다.  
2. **필요 패키지 설치**: `libgl1-mesa-glx`와 같은 OpenCV 의존성 라이브러리를 설치합니다.  
3. **Python 라이브러리**: `requirements.txt`에 명시된 패키지를 설치합니다.  
4. **기본 작업 디렉토리**: `/gradio`를 설정하고, 이곳에서 `main.py`를 실행합니다.

---

## docker-compose.yml: 컨테이너 실행 설정

**docker-compose.yml**은 Docker 컨테이너를 실행하고 설정을 관리하는 파일입니다.

### docker-compose.yml 내용

```yaml
version: "3"

services:
  gradio_v1:
    build: .  # 현재 디렉토리의 Dockerfile 사용
    container_name: gradio_yolov7
    runtime: nvidia  # GPU 사용 설정 (필요 없으면 제거 가능)
    volumes:
      - .:/gradio  # 로컬 폴더를 컨테이너와 공유
    ports:
      - "9999:9999"  # 호스트와 컨테이너의 포트를 매핑
    tty: true
    restart: always
    command: python main.py  # 컨테이너 시작 시 실행할 명령어
```

### 코드 설명
1. **GPU 설정**: `runtime: nvidia`를 통해 GPU를 사용할 수 있도록 설정합니다.  
   - **CPU만 사용하려면** 이 옵션을 제거하면 됩니다.  
2. **볼륨 마운트**: `- .:/gradio`를 통해 로컬 프로젝트 폴더를 컨테이너에 연결합니다.  
3. **포트 매핑**: `9999:9999`를 통해 컨테이너의 Gradio 웹 서버에 접근할 수 있습니다.  
   - 포트를 바꾸고 싶다면 `8080:9999`와 같이 수정합니다.  
4. **재시작 설정**: `restart: always`는 컨테이너가 종료되더라도 자동으로 재시작합니다.

---

## requirements.txt: Python 라이브러리

`requirements.txt`에는 Gradio와 YOLOv7 모델 실행에 필요한 패키지들이 나열되어 있습니다.

```plaintext
gradio==3.50.2
opencv-python>=4.1.2
scipy
seaborn>=0.11.0
torch>=1.7.0
torchvision>=0.8.1
```

> PyTorch와 Gradio 버전이 맞지 않을 경우, 필요에 따라 버전을 조정할 수 있습니다.

---

## 실행 방법

### 1. Docker 이미지 빌드
다음 명령어를 통해 Docker 이미지를 빌드합니다.

```bash
docker compose build
```

### 2. 컨테이너 실행
Docker Compose를 통해 컨테이너를 실행합니다.

```bash
docker compose up
```

### 3. 백그라운드 실행
`-d` 옵션을 추가하면 백그라운드에서 실행됩니다.

```bash
docker compose up -d
```

### 4. 컨테이너 종료
실행 중인 컨테이너를 종료하려면 다음 명령어를 사용합니다.

```bash
docker compose down
```

---

## 시연 영상

아래는 Docker를 활용해 YOLOv7 모델을 서빙하는 시연 영상입니다.

<iframe width="840" height="473" src="https://www.youtube-nocookie.com/embed/87pI8rkKzoA?si=r-licJ3JNZrOPzb9" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

## 결론

이 글에서는 **Docker**를 활용해 Gradio 기반의 **YOLOv7 모델 서빙 환경**을 배포하는 방법을 설명했습니다.  
- Dockerfile과 Compose를 사용해 **환경 설정과 실행을 자동화**할 수 있습니다.  
- GPU/CPU에 따라 유연하게 설정을 변경할 수 있습니다.  
