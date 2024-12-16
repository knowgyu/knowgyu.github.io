---
title: "Pipeline-Run"
author: knowgyu
description: " "
date: 2023-12-02 08:05:44 +0900
math: true
categories: [MLOps, 3-Kubeflow-Pipeline-Run]
tags: [MLOps, Kubeflow]
---

# Kubeflow

---

> Kubeflow를 사용하기 위해서는 **컴포넌트**와 **파이프라인**을 작성해야 합니다.
> 

컴포넌트는 독립적으로 실행되지 않고 파이프라인의 구성요소로써 실행

→ 컴포넌트를 실행해 보려면 파이프라인을 작성해야 합니다.

## Pipeline

> 파이프라인은 컴포넌트의 집합과 컴포넌트를 실행시키는 순서도로 구성되어 있습니다.
순서도는 방향 순환이 없는 그래프, 간단한 조건문 포함 가능
> 

## Before Run

업로드한 파이프라인을 실행시키기 전 Experiment를 생성해야 합니다.

> Experiment란 Kubeflow에서 실행되는 Run을 논리적으로 관리하는 단위입니다.
> 

### Experiments (KFP)페이지에서 우측 상단의 `Create experiment` 버튼 클릭

![Untitled](/assets/img/kubeflow/kubepipe701.png)

> Experiments (AutoML) 페이지는 Hyperparameter Tuning과 Neural Architecture Search를 담당하는 Katib관리 페이지
> 

### Create Experiment

![Untitled](/assets/img/kubeflow/kubepipe702.png)

<aside>
💡 Next 버튼 클릭 시 자동으로 Create Run화면으로 이동됩니다.

</aside>

## Run Pipeline

### 실행하려는 파이프라인 선택 후 우측 상단의 `Create run` 버튼 클릭

(New Experiment로 생성했다면 이 단계는 건너뜀)

![Untitled](/assets/img/kubeflow/kubepipe703.png)

### 내용 입력 후 `Start` 버튼 클릭

![Untitled](/assets/img/kubeflow/kubepipe704.png)

> **`Run parameters`**에 Pipeline Config 값들을 입력해야합니다.
> 

### 실행 완료

![Untitled](/assets/img/kubeflow/kubepipe705.png)

> 컴포넌트 실행이 완료되면 초록색 체크 표시가 나옵니다.

가장 마지막 컴포넌트를 보면 입력한 Config값인 3과 5의 합인 8이 출력된 것을 확인할 수 있습니다.
>
