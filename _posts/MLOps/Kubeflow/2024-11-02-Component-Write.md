---
title: "Component-Write"
author: knowgyu
description: "컴포넌트 작성"
date: 2023-12-01 08:05:24 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

# Kubeflow

---

> Kubeflow를 사용하기 위해서는 **컴포넌트**와 **파이프라인**을 작성해야 합니다.


컴포넌트와 파이프라인을 작성하기 전, 필요한 패키지들과 버전을 설치합니다.

## Install Requirements

- Python version ≥ 3.7, ≤3.9 (3.10버전부터 `scikit-learn==1.0.1` 설치 에러 발생)
- requirements.txt
    
    ```python
    kfp==1.8.9
    scikit-learn==1.0.1
    mlflow==1.21.0
    pandas==1.3.4
    dill==0.3.4
    urllib3<2.0
    numpy<1.20
    ```
    


> (23.12.11. 작성) 가상환경을 생성해 테스트하길 권장하며, `kfp==1.8.22` 버전 사용 가능
{: .prompt-tip }

## Component

컴포넌트를 작성하기 위해서는 아래와 같은 내용을 작성해야 합니다.

1. 컴포넌트 콘텐츠(Component Contents) 작성
2. 컴포넌트 래퍼(Component Wrapper) 작성

### Component Contents

컴포넌트 콘텐츠는 흔히 작성하는 파이썬 코드와 동일함.

ex) 숫자를 입력으로 받고 입력받은 숫자를 출력한 뒤 반환하는 컴포넌트 작성

```python
print(number)
```

위 코드에서 `number` 가 정의되어 있지 않아 에러가 발생함.

Kubeflow에서 `number`와 같이 컴포넌트 콘텐츠에 필요한 값들은 **Config**로 정의함.

→ 컴포넌트 콘텐츠를 실행시키기 위해 필요한 Config들은 컴포넌트 래퍼에서 전달

### Component Wrapper

**필요한 Config를 전달할 수 있도록 컴포넌트 래퍼 작성**

```python
def print_and_return_number(number: int) -> int:
	print(number)
	return number
```

콘텐츠에 필요한 Config를 래퍼의 argument로 추가.

**💡argument 타입 힌트 작성 이유**
  → 파이프라인을 Kubeflow Format으로 변환할 때, 컴포넌트 간 입출력 타입이 일치하는지 체크함

**Convert to Kubeflow Format**

```python
from kfp.components import create_component_from_func

@create_component_from_func
def print_and_return_number(number: int) -> int:
	print(number)
	return number
```
