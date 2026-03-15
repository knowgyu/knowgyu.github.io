---
title: "Pipeline-Upload"
author: knowgyu
description: " "
date: 2023-11-27 07:14:47 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

## Kubeflow Pipeline 업로드

> Kubeflow를 사용하려면 **컴포넌트**와 **파이프라인**을 함께 작성해야 합니다.

컴포넌트는 독립적으로 실행되기보다 파이프라인의 구성 요소로 묶여 실행됩니다.

즉 컴포넌트를 실제로 실행하려면 먼저 파이프라인을 정의해야 합니다.

## Pipeline

> 파이프라인은 여러 컴포넌트의 집합과, 이를 실행시키는 순서도로 구성됩니다.
> 이 순서도는 방향 순환이 없는 그래프(DAG)이며, 간단한 조건문도 포함할 수 있습니다.


## Upload Pipeline

이전에 만든 파이프라인은 Kubeflow 대시보드 UI를 통해 업로드할 수 있습니다.

```yaml
kubectl port-forward svc/istio-ingressgateway -n istio-system 8080:80
```

`http://localhost:8080/` 에 접속합니다.

### 1. Pipelines 탭 클릭

![Untitled](/assets/img/kubeflow/kubepipe601.png)

### 2. 우측 상단의 `Upload Pipeline` 클릭

![Untitled](/assets/img/kubeflow/kubepipe602.png)

### 3. YAML 파일 업로드

![Untitled](/assets/img/kubeflow/kubepipe603.png)

> Pipeline Name과 Pipeline Description을 작성한 뒤 **`Create`** 버튼을 클릭합니다.
 

### 4. 생성

![Untitled](/assets/img/kubeflow/kubepipe604.png)

### 추가: Upload Pipeline Version

![Untitled](/assets/img/kubeflow/kubepipe605.png)

> 3번 과정에서 **`Create a new pipeline version under an existing pipeline`** 을 클릭하면 기존 파이프라인 아래에 버전을 추가할 수 있습니다.
