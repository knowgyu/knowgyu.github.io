---
title: "Component-Environment"
author: knowgyu
description: " "
date: 2023-12-02 08:05:40 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

## Kubeflow 컴포넌트 실행 환경

> Kubeflow를 사용하려면 **컴포넌트**와 **파이프라인**을 함께 작성해야 합니다.

컴포넌트와 파이프라인을 작성하기 전에, 먼저 필요한 패키지와 버전을 어떻게 준비할지 이해해야 합니다.

## Component

> **컴포넌트**는 **컴포넌트 콘텐츠**와 **컴포넌트 래퍼**로 구성됩니다.<br>
>
> 하나의 컴포넌트는 래퍼를 통해 Kubeflow에 전달되며, 전달된 컴포넌트는 정의된 콘텐츠를 실행하고 아티팩트를 생성합니다.

## Kubernetes에서 컴포넌트가 실행되는 방식

---

Kubeflow에서 컴포넌트가 실행되는 순서는 아래와 같습니다.

1. `docker pull <image>` : 정의된 컴포넌트의 실행 환경 정보가 담긴 이미지를 가져옵니다.
2. run `command` : pull한 이미지에서 컴포넌트 콘텐츠를 실행합니다.

이전에 작성한 YAML파일을 확인해보면 `image: python:3.7` 입니다.

즉, 파이프라인 실행 시

1. `docker pull python:3.7`
2. run `command`


> 따라서 `torch`, `numpy`와 같은 라이브러리를 사용하려면 **패키지를 추가**해야 합니다.
{: .prompt-tip }

## 패키지 추가 방법

1. `base_image` 사용
2. `packages_to_install` 사용

컴포넌트를 컴파일할 때 사용했던 함수가 어떤 arg를 받는지 확인합니다.[Component-Write](https://www.notion.so/Component-Write-3e4cfaff96dc472b8c669c784adf4edd?pvs=21)

```python
def create_component_from_func(
    func: Callable,
    output_component_file: Optional[str] = None,
    base_image: Optional[str] = None,
    packages_to_install: List[str] = None,
    annotations: Optional[Mapping[str, str]] = None,
):
```

- `func`: 컴포넌트로 만들 컴포넌트 래퍼 함수
- `base_image`: 컴포넌트 래퍼가 실행할 이미지
- `packages_to_install`: 컴포넌트를 위해 추가로 설치해야 하는 패키지

### `base_image` 사용

```python
from functools import partial
from kfp.components import InputPath, OutputPath, create_component_from_func

@partial(
    create_component_from_func,
    base_image="ghcr.io/mlops-for-all/base-image:latest",
)
def train_from_csv(
    train_data_path: InputPath("csv"),
...생략
    kernel: str,
):
    import dill
...생략
    with open(model_path, mode="wb") as file_writer:
        dill.dump(clf, file_writer)

if __name__ == "__main__":
    train_from_csv.component_spec.save("train_from_csv.yaml")
```

### `packages_to_install` 사용

```python
from functools import partial
from kfp.components import InputPath, OutputPath, create_component_from_func

@partial(
    create_component_from_func,
    packages_to_install=["dill==0.3.4", "pandas==1.3.4", "scikit-learn==1.0.1"],
)
def train_from_csv(
    train_data_path: InputPath("csv"),
...생략
    kernel: str,
):
    import dill
...생략
    with open(model_path, mode="wb") as file_writer:
        dill.dump(clf, file_writer)

if __name__ == "__main__":
    train_from_csv.component_spec.save("train_from_csv.yaml")
```


> 컴포넌트는 실행 시 컨테이너 안에서 동작하므로, **각 Base Image** 에 필요한 **라이브러리**가 설치되어 있어야 합니다.<br>
> 반면 파이프라인을 **컴파일하는 환경**에서는 `kfp` 라이브러리만 있어도 되고, 실제 컴포넌트 실행용 라이브러리까지 모두 설치할 필요는 없습니다.<br>
> 예를 들어 YOLOv8을 학습하거나 추론하려면 실행 이미지에는 `torch`, `ultralytics` 패키지가 있어야 합니다.<br>
> 하지만 YAML 파일을 컴파일하는 환경에 이 패키지들이 없다고 해서 바로 `No module named ...` 에러가 발생하지는 않습니다.
{: .prompt-info }
