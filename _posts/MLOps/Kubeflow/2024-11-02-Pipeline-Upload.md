---
title: "Pipeline-Upload"
author: knowgyu
description: " "
date: 2023-11-27 07:14:47 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

# Kubeflow

---

> Kubeflow를 사용하기 위해서는 **컴포넌트**와 **파이프라인**을 작성해야 합니다.


컴포넌트는 독립적으로 실행되지 않고 파이프라인의 구성요소로써 실행

→ 컴포넌트를 실행해 보려면 파이프라인을 작성해야 합니다.

## Pipeline

> 파이프라인은 컴포넌트의 집합과 컴포넌트를 실행시키는 순서도로 구성되어 있습니다.
순서도는 방향 순환이 없는 그래프, 간단한 조건문 포함 가능


## Upload Pipeline

이전에 만든 파이프라인을 Kubeflow에 업로드

kubeflow 대시보드 UI를 통해 진행할 수 있음.

```yaml
kubectl port-forward svc/istio-ingressgateway -n istio-system 8080:80
```

http://localhost:8080/ 접속

### 1. Pipelines 탭 클릭

![Untitled](/assets/img/kubeflow/kubepipe601.png)

### 2. 우측 상단 Upload Pipeline 클릭

![Untitled](/assets/img/kubeflow/kubepipe602.png)

### 3. yaml파일 업로드

![Untitled](/assets/img/kubeflow/kubepipe603.png)

> Pipeline Name과 Pipeline Description 작성 후 **`Create`** 버튼 클릭
 

### 4. 생성

![Untitled](/assets/img/kubeflow/kubepipe604.png)

### +) Upload Pipeline Version

![Untitled](/assets/img/kubeflow/kubepipe605.png)

> 3번 과정에서 **`Create a new pipeline version under an existing pipeline`** 클릭
