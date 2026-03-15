---
title: "Pipeline-Run"
author: knowgyu
description: " "
date: 2023-12-02 08:05:44 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

## Kubeflow Pipeline 실행

> Kubeflow를 사용하려면 **컴포넌트**와 **파이프라인**을 함께 작성해야 합니다.

컴포넌트는 독립적으로 실행되기보다 파이프라인의 구성 요소로 묶여 실행됩니다.

즉 실제 실행 단계로 넘어가려면 먼저 파이프라인이 준비되어 있어야 합니다.

## Pipeline

> 파이프라인은 여러 컴포넌트의 집합과, 이를 실행시키는 순서도로 구성됩니다.
> 순서도는 방향 순환이 없는 그래프(DAG)이며, 간단한 조건문도 포함할 수 있습니다.

## Before Run

업로드한 파이프라인을 실행하기 전에 먼저 Experiment를 생성해야 합니다.

> Experiment는 Kubeflow에서 실행되는 여러 Run을 논리적으로 묶어 관리하는 단위입니다.

### Experiments (KFP) 페이지에서 우측 상단의 `Create experiment` 버튼 클릭

![Untitled](/assets/img/kubeflow/kubepipe701.png)

> Experiments (AutoML) 페이지는 Hyperparameter Tuning과 Neural Architecture Search를 담당하는 Katib 관리 페이지입니다.

### Create Experiment

![Untitled](/assets/img/kubeflow/kubepipe702.png)

<aside>
💡 `Next` 버튼을 누르면 자동으로 Create Run 화면으로 이동합니다.

</aside>

## Run Pipeline

### 실행하려는 파이프라인을 선택한 뒤 우측 상단의 `Create run` 버튼 클릭

(New Experiment로 생성했다면 이 단계는 건너뜀)

![Untitled](/assets/img/kubeflow/kubepipe703.png)

### 내용 입력 후 `Start` 버튼 클릭

![Untitled](/assets/img/kubeflow/kubepipe704.png)

> **`Run parameters`** 에 파이프라인 Config 값을 입력해야 합니다.

### 실행 완료

![Untitled](/assets/img/kubeflow/kubepipe705.png)

> 컴포넌트 실행이 완료되면 초록색 체크 표시가 나타납니다.

가장 마지막 컴포넌트를 보면 입력한 Config 값인 3과 5의 합인 8이 출력된 것을 확인할 수 있습니다.
