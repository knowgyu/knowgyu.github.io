---
title: "PV Mount & HyperParameter Tuning"
author: knowgyu
description: " "
date: 2023-12-05 08:14:07 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

이 글에서는 파이프라인에서 PV를 마운트하는 방법과, YOLOv8에서 기본 제공하는 Ray Tune 기반 하이퍼파라미터 튜닝을 함께 정리합니다.<br>

[https://happycloud-lee.tistory.com/256](https://happycloud-lee.tistory.com/256) ← PV/PVC 참고<br>

[https://docs.ultralytics.com/integrations/ray-tune/?h=hyper](https://docs.ultralytics.com/integrations/ray-tune/?h=hyper) ← HP Tuning 공식 문서<br>

# PV / PVC

---

## k8s 환경에서의 볼륨

### 외부 볼륨의 필요성

쿠버네티스 환경에서는 모든 애플리케이션이 Pod 안의 컨테이너 형태로 실행됩니다.

보통 **컨테이너**에서 데이터를 읽고 저장할 때는 자체 내부 볼륨보다 **외부 볼륨**을 사용하는 편이 안전합니다.

→ 컨테이너는 언제든 사라질 수 있으므로, 데이터를 잃지 않기 위해서입니다.

### 사용 목적별 볼륨 유형

Pod 안에 컨테이너 형태로 실행되는 것은 크게 **애플리케이션**과 **데이터베이스**입니다.

볼륨을 크게 나누면 Pod 로컬 볼륨, Node 로컬 볼륨, 네트워크 볼륨으로 구분해 생각할 수 있습니다. 물론 이는 공식 분류라기보다 이해를 돕기 위한 구분입니다.

볼륨 유형을 정리하면 아래와 같습니다.

| 구분         | 사용 목적                            | 파드 삭제시 볼륨 폐기 | 볼륨 유형      |
| ------------ | ------------------------------------ | --------------------- | -------------- |
| 어플리케이션 | 어떤 처리를 위한 임시 공간           | **Y**                 | 파드 로컬 볼륨 |
|              | 컨피그맵, 시크릿, 파드 정보 참조     | **Y**                 | 파드 로컬 볼륨 |
|              | 파드가 실행된 노드의 파일 접근       | N                     | 노드 로컬 볼륨 |
|              | 특정 노드의 파일 접근                | N                     | 노드 로컬 볼륨 |
|              | 클러스터 외부 스토리지의 파일 접근   | N                     | 네트워크 볼륨  |
| 데이터베이스 | 컨피그맵, 시크릿, 파드 정보 참조     | **Y**                 | 파드 로컬 볼륨 |
|              | 파드가 실행된 노드에만 데이터 저장   | N                     | 노드 로컬 볼륨 |
|              | 특정 노드에만 데이터 저장            | N                     | 노드 로컬 볼륨 |
|              | 클러스터 외부 스토리지에 데이터 저장 | N                     | 노드 로컬 볼륨 |

### Pod의 볼륨 접근 아키텍처와 PV 라이프 사이클

![https://happycloud-lee.tistory.com/256](/assets/img/kubeflow/kubeyolo101.png)

https://happycloud-lee.tistory.com/256

Pod가 볼륨을 사용하는 방법은 ‘**마운트**’입니다.

Pod 내에 볼륨을 ‘마운트’함으로써 어떤 유형의 볼륨이든 Pod의 내부 파일 시스템처럼 사용할 수 있습니다.

하지만 이 아키텍처는 쿠버네티스와 인프라스트럭처가 너무 강하게 결합된(**Tightly Coupled**) 상태가 되기 쉽습니다.

이로 인해 스토리지 제품의 변화나 스토리지 서버의 변화가 Pod에 직접적인 영향을 미칩니다.

- **예시**
    
    `nfs` 라는 네트워크 볼륨을 Pod에 마운트하려면 `nfs` 서버의 IP나 호스트를 지정해야 하는데, `nfs` 서버의 IP가 변경되면 서비스에 문제가 발생할 수 있습니다.
    

이를 해결하려면 쿠버네티스와 인프라스트럭처를 느슨하게 결합(**Loosely Coupled**)시키는 방식이 필요합니다.

이를 위해 중간에 중계자 역할을 하는 **PV/PVC** 를 만들게 됩니다.


> 네트워크 볼륨을 위해 CSI(Container Storage Interface) 같은 계층도 사용할 수 있습니다.
> 다만 딥러닝 학습에서는 네트워크 볼륨이 병목이 되는 경우가 있어, 여기서는 로컬 볼륨 기준으로만 정리합니다.
{: .prompt-tip }

![https://happycloud-lee.tistory.com/256](/assets/img/kubeflow/kubeyolo102.png)
https://happycloud-lee.tistory.com/256

1. Pod 정보, ConfigMap, Secret은 파드 명세에 정의하여 마운트합니다.

2. 노드 로컬 볼륨과 네트워크 볼륨은 PV 리소스로 정의하고 PVC에 바운드(연결)합니다.
    파드 명세에서는 PVC만 지정하면 연결된 볼륨이 마운트됩니다.

3. 스토리지 제품별로 PV를 정의해 볼륨에 접근할 수 있습니다.<br>

이렇게 구성하면 PV를 정의하고 PVC에 연결한 뒤, Pod에는 PVC만 지정하면 됩니다.

## 파이프라인 볼륨

파이프라인은 1개 이상의 컴포넌트로 이루어져 있고, 각 컴포넌트는 Pod 단위로 실행됩니다.
즉 각 컴포넌트는 서로 다른 로컬 볼륨을 사용하므로, 별도 설정 없이는 다른 컴포넌트의 데이터를 바로 읽거나 쓸 수 없습니다.<br>

이를 해결하기 위해 PV를 만들고, 각 컴포넌트(또는 Pod)에서 PVC를 통해 같은 볼륨을 마운트하겠습니다.<br>


> PV를 생성하고 PVC를 통해 Pod에 연결하는 방법은 다양합니다.
> 이 글에서는 가장 단순하게 YAML 파일로 Persistent Volume을 만들고, PVC로 PV를 연결한 뒤, Pod에서 해당 PVC를 지정하는 방식으로 설명합니다.
{: .prompt-tip }

### PV 생성

- `pv.yaml` 를 통해 PV 정의

```yaml
apiVersion: v1
kind: PersistentVolume
metadata:
  name: pv-test
spec:
  capacity:
    storage: 5Gi
  volumeMode: Filesystem
  accessModes:
    - ReadWriteOnce
  persistentVolumeReclaimPolicy: Retain
  storageClassName: 'blog-pv-test'
  hostPath:
    path: "/data/datasets/coco8"
  nodeAffinity:
    required:
      nodeSelectorTerms:
      - matchExpressions:
        - key: kubernetes.io/hostname
          operator: In
          values:
          - blog-space
```

> `accessModes` 는 여러 옵션이 있지만, 여기서는 minikube 기반 단일 노드 환경이므로 RWO(`ReadWriteOnce`) 모드로 지정합니다.


> 변경해야 하는 값들 
{: .prompt-info }

- `name: pv-test`
- `storage: 1Gi`
- `storageClassName: 'blog-pv-test'`
- `path: "/data/datasets/coco8"`
- `values : blog-space`

<br>

- PV 생성

```yaml
kubectl apply -f pv.yaml
# 정상적으로 생성되면 아래 메세지 출력
persistentvolume/pv-test created
```

- 확인

```bash
kubectl get pv -A
# 모든 PV가 표시됩니다. 그 중 생성한 pv를 확인합니다. 아직 마운트되지 않았기에 CLAIM은 비어있어야합니다.
NAME                                       CAPACITY   ACCESS MODES   RECLAIM POLICY   STATUS   CLAIM                                                         STORAGECLASS   REASON   AGE
pv-test                                    1Gi        RWO            Retain           Available                                                                 local-path              22s
```

### PVC 생성

`pvc.yaml`을 직접 작성하지 않아도 Kubeflow 대시보드의 **Volume** 탭이나, 파이프라인 작성 시 `VolumeOp` 를 통해 생성할 수 있습니다.
다만 이 글에서는 동적 할당이 아니라 수동 할당 흐름을 설명하기 위해 직접 명세를 작성해 PVC를 생성합니다.

![Untitled](/assets/img/kubeflow/kubeyolo109.png)

    
- `pvc.yaml`

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: pvc-test
  namespace: blog-space
spec:
  accessModes:
  - ReadWriteOnce
  volumeMode: Filesystem
  resources:
    requests:
      storage: 5Gi
  storageClassName: "scn-test"
```

- PVC 생성

```bash
kubectl apply -f pvc.yaml
# 정상적으로 생성되면 아래 메세지 출력
persistentvolumeclaim/pvc-test created
```

- 확인

```bash
kubectl get pvc -A
# 모든 PVC가 표시됩니다. 네임스페이스를 `blog-space` 로 지정해주었고, `pv-test` 와 제대로 Bound 되었는지 확인합니다.
NAMESPACE                   NAME                                            STATUS   VOLUME                                     CAPACITY   ACCESS MODES   STORAGECLASS   AGE
blog-space                  pvc-test                                        Bound    pv-test                                    5Gi        RWO            scn-test       100m
```

### PV/PVC 생성 확인

제대로 지정되었는지 확인해보겠습니다.

- PV 확인

```bash
kubectl get pv -A |grep pv-test
```

위 명령어를 통해 비어있던 CLAIM이 `blog-space/pvc-test` 로 변경되었는지 확인합니다.

```bash
pv-test                                    5Gi        RWO            Retain           Bound    blog-space/pvc-test                                        scn-test                103m
```

또한, Kubeflow 대시보드를 통해 재차 확인할 수 있습니다.

`blog-space` 네임스페이스를 사용하는 사용자 계정으로 로그인 후 좌측의 Volume 탭을 클릭합니다.

![만약 위의 과정을 똑같이 따라했다면, Name은 pvc-test로 나와야합니다.](/assets/img/kubeflow/kubeyolo103.png)


만약 위의 과정을 똑같이 따라했다면, Name은 pvc-test로 나와야합니다.

### Pod → PV 마운트

PV/PVC를 생성했으니, 이제 이 볼륨을 사용할 Pod에 실제로 마운트해야 합니다.

이를 위해 Pipeline 작성 시 각 컴포넌트에 PV를 마운트하겠습니다.

- 파이프라인 작성 코드 스니펫

```python
from kfp import dsl

@dsl.pipeline(name="pipeline_name",
          description="MLpipline Description",
          )
def train_pipeline(
    model_name: str,
    epochs: int,
    imgsz: int,
    batchsize: int,
    ):
    import os
    
    vop = dsl.VolumeOp(
        name="create-pvc",
        resource_name="pvc-test",
        storage_class='scn-test',
        modes=dsl.VOLUME_MODE_RWO,
        size="5Gi",
        generate_unique_name=False,
        action='apply',
    )
    
    check = verify_training(model_name, epochs, imgsz, batchsize).add_pvolumes({"/data":vop.volume})
    
		trained = train(
        check.output, model_name, epochs, imgsz, batchsize
        ).add_pvolumes({"/data":vop.volume})
```

**`dsl.VolumeOp(...)`**: Kubernetes PVC를 생성하는 파이프라인 태스크입니다.

- 변경 사항

```python
    vop = dsl.VolumeOp(
        name="create-pvc",  <- 파이프라인 UI에서 표시될 이름
        resource_name="pvc-test", <- 생성 또는 참조할 PVC 이름. 기존 PVC를 쓴다면 동일해야 함
        storage_class='scn-test', <- 동적 생성 시 사용할 스토리지 클래스
        modes=dsl.VOLUME_MODE_RWO, <- 단일 노드 환경이므로 ReadWriteOnce
        size="5Gi",         <- 생성될 PVC 크기. 기존 명세보다 작으면 오류가 발생할 수 있음
        generate_unique_name=False, <- True면 pvc-test-1sfd.. 같은 형태로 생성됩니다. 기존 PVC와 이름을 맞추기 위해 False로 둡니다.
        action='apply', <- 적용 시 필요했던 옵션이라 함께 명시했습니다.
    )
```

```python
    check = verify_training(..).add_pvolumes({"/data":vop.volume}) <- 해당 컴포넌트에 마운트
    
		trained = train(
        ..
        ).add_pvolumes({"/data":vop.volume}) <- 해당 컴포넌트에 마운트
# 파드 로컬 볼륨인 /data에 vop.volume을 마운트함.
# vop.volume은 PV.yaml에서 지정한 /data/dataset/coco8
```

## 정리

YOLOv8의 경우 모델과 데이터셋이 없더라도, 기본적으로 `coco128` 데이터셋과 `yolov8n.pt` 모델을 다운로드해 학습을 실행합니다. 하지만, 커스텀 데이터셋을 학습시키기 위해선 파이프라인의 각 컴포넌트들이 실행될 때 내 클러스터 노드의 볼륨에 있는 데이터에 접근해 읽고, 아티팩트를 저장할 수 있어야합니다.

이를 위해, `pv.yaml`을 통해 내 클러스터 노드의 경로를 지정해 Persistent Volume을 생성하였고, 각 컴포넌트에서 PV에 마운트하기위해 `pvc.yaml`을 생성했고, `dsl.VolumeOp` 에서 PVC를 생성할 수 있지만, 미리 생성한 PVC를 이용해 PV와 PVC를 바인딩하여 각 컴포넌트에서 내 클러스터 노드 로컬 볼륨에 접근할 수 있습니다.

이를 통해 Kubeflow 대시보드의 Jupyter Notebook와 학습 파이프라인 모두 내 PV에 접근할 수 있습니다.

그렇다면, 정말로 내 클러스터 노드의 로컬 볼륨에 제대로 마운트 되었는지 확인을 위해 **하이퍼 파라미터 튜닝** 기능을 추가하며 확인하겠습니다.

# Hyperparameter Tuning(231215수정예정)

---

> 하이퍼 파라미터 튜닝은 최적의 하이퍼 파라미터 세트를 찾아 모델의 최대 성능을 이끌어내는데 필수적입니다.
https://docs.ultralytics.com/integrations/ray-tune/?h=hyper
> 


> **(23.12.14)** ray-tune 사용 시 train_args와 데이터셋 경로 설정 등 문제가 발생합니다.
> ultralytics 패키지와 raytune Integration이 제대로 이뤄지지않아 생기는 버그같습니다.
>
> raytune이 아닌, ultralytics에서 제공하는 하이퍼파라미터 튜닝 기능을 사용하는 것으로 수정할 예정입니다.
{: .prompt-warning}

~~YOLOv8은 Ray Tune 패키지와 통합해 `model.tune` 으로 쉽게 하이퍼 파라미터 기능을 사용할 수 있습니다.~~

~~예제)~~

```python
model = YOLO("yolov8n.pt")

result_grid = model.tune(data='coco8.yaml', use_ray=True)
```

~~학습 컴포넌트를 거쳐 생성된 `run-output` 를 불러와 하이퍼파라미터 튜닝을 하는 컴포넌트를 추가해보겠습니다.~~

- ~~tune 컴포넌트~~

```python
@partial(
    create_component_from_func,
    base_image="nohgyu/test:v1.1",
    packages_to_install=["ultralytics","ray[tune]","opencv-python==4.8.0.74","mlflow", "boto3"],
)
def tune(
    model_path: str,
    ):
    from ultralytics import YOLO
    from ray import tune
    import os

    os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
    os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
    os.environ["AWS_ACCESS_KEY_ID"] = "minio"
    os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"
    
    model = YOLO(model_path)
    
    result = model.tune(
                data="coco8.yaml",
                space={"lr0": tune.uniform(1e-5, 1e-1)},
                epochs=2,
                grace_period=2,
                gpu_per_trial=1,
                iterations=1,
                batch=4,
                use_ray=True
                )
```

~~기본적으로 학습 컴포넌트와 유사하게 작성되며 모델의 경로를 문자열로 받아 모델을 불러옵니다.~~ 

~~⚠️ 만약, 볼륨이 마운트 되어있지 않다면, 서로 다른 로컬 볼륨을 사용하므로 `No such file or directory` 에러가 발생할 것입니다.~~

<aside>
💡 ~~문자열로 받아오는게 아닌, `kfp.components.InputPath` `kfp.components.OutputPath` 를 통해 모델을 주고 받을 수 있습니다. 본 페이지에서는 다루지 않겠습니다.~~

</aside>

### 전체 코드

```python
from functools import partial

import kfp
from kfp.components import create_component_from_func, InputPath, OutputPath
from kfp import dsl

@partial(
    create_component_from_func,
    base_image="nohgyu/test:v1.1",
)
def verify_training(
    model_name: str,
    epochs: int,
    imgsz: int,
    batchsize: int,
    ) -> bool:
    import torch

    assert isinstance(model_name, str), "model_name must be a string"
    assert isinstance(epochs, int), "epochs must be an integer"
    assert isinstance(imgsz, int), "imgsz must be an integer"
    assert isinstance(batchsize, int), "batchsize must be an integer"
        
    print(torch.cuda.is_available())
    print(torch.cuda.get_device_name())
    print(torch.cuda.device_count())
    
    return torch.cuda.is_available()
    
    
@partial(
    create_component_from_func,
    base_image="nohgyu/test:v1.1",
    packages_to_install=["ultralytics","opencv-python==4.8.0.74","mlflow", "boto3"],
)
def train(
    checker: bool,
    model_name: str,
    epochs: int,
    imgsz: int,
    batchsize: int,
        ) -> str:
    import os
    from ultralytics import YOLO
    import mlflow
    
    if not checker:
        print("CUDA is not available.\nPlease Check GPU Device")
        return None

    os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
    os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
    os.environ["AWS_ACCESS_KEY_ID"] = "minio"
    os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"

    model = YOLO(model_name)    
    
    results = model.train(data='coco8.yaml', epochs=epochs, imgsz=imgsz, batch=batchsize,
                          project='testprj', name='testexp', exist_ok=True,)

    return os.path.join('testprj', 'testexp', 'weights', 'run-output')
    
@partial(
    create_component_from_func,
    base_image="nohgyu/test:v1.1",
    packages_to_install=["ultralytics","ray[tune]","opencv-python==4.8.0.74","mlflow", "boto3"],
)
def tune(
    model_path: str,
    ):
    from ultralytics import YOLO
    import os

    os.environ["MLFLOW_TRACKING_URI"] = "http://mlflow-server-service.mlflow-system.svc:5000"
    os.environ["MLFLOW_S3_ENDPOINT_URL"] = "http://minio-service.kubeflow.svc:9000"
    os.environ["AWS_ACCESS_KEY_ID"] = "minio"
    os.environ["AWS_SECRET_ACCESS_KEY"] = "minio123"
    
    model = YOLO(model_path)
    
    result = model.tune(
                data="coco8.yaml",
                space={"lr0": tune.uniform(1e-5, 1e-1)},
                epochs=2,
                grace_period=2,
                gpu_per_trial=1,
                iterations=1,
                batch=4,
                use_ray=True
                )
        
        
@dsl.pipeline(name="Thisisplname",
          description="MLpipline Description",
          )
def train_pipeline(
    model_name: str,
    epochs: int,
    imgsz: int,
    batchsize: int,
    ):
    import os
    
    vop = dsl.VolumeOp(
        name="create-pvc",
        resource_name="this-is-pvc",
        storage_class='scn-test',
        modes=dsl.VOLUME_MODE_RWO,
        size="5Gi",
        generate_unique_name=False,
        action='apply',
    )
    
    check = verify_training(model_name, epochs, imgsz, batchsize).add_pvolumes({"/data":vop.volume})
    trained = train(
        check.output, model_name, epochs, imgsz, batchsize
        ).add_pvolumes({"/data":vop.volume})
    tune(trained.output).add_pvolumes({"/data":vop.volume})

if __name__ == "__main__":
    kfp.compiler.Compiler().compile(train_pipeline, "pipeline-mount-test.yaml")
```

- `pipeline-mount-test.yaml` *~~# 전체코드와 yaml파일은 예제와 다를 수 있습니다.~~*
    
    ```python
    apiVersion: argoproj.io/v1alpha1
    kind: Workflow
    metadata:
      generateName: thisisplname-
      annotations: {pipelines.kubeflow.org/kfp_sdk_version: 1.8.9, pipelines.kubeflow.org/pipeline_compilation_time: '2023-12-01T16:49:44.795741',
        pipelines.kubeflow.org/pipeline_spec: '{"description": "MLpipline Description",
          "inputs": [{"name": "model_name", "type": "String"}, {"name": "epochs", "type":
          "Integer"}, {"name": "imgsz", "type": "Integer"}, {"name": "batchsize", "type":
          "Integer"}], "name": "Thisisplname"}'}
      labels: {pipelines.kubeflow.org/kfp_sdk_version: 1.8.9}
    spec:
      entrypoint: thisisplname
      templates:
      - name: create-pvc
        resource:
          action: apply
          manifest: |
            apiVersion: v1
            kind: PersistentVolumeClaim
            metadata:
              name: this-is-pvc
...
...
...
생략
...
...
...
        volumes:
        - name: create-pvc
          persistentVolumeClaim: {claimName: '{{inputs.parameters.create-pvc-name}}'}
      arguments:
        parameters:
        - {name: model_name}
        - {name: epochs}
        - {name: imgsz}
        - {name: batchsize}
      serviceAccountName: pipeline-runner
    ```
    

## 파이프라인 업로드 및 실행

---

- **파이프라인 업로드**

![Untitled](/assets/img/kubeflow/kubeyolo104.png)


- **파이프라인 실행 결과**

![Untitled](/assets/img/kubeflow/kubeyolo105.png)


- **주피터 노트북 확인 결과**
    
    ![노트북 생성 중 볼륨 생성이 아닌 기존 볼륨을 사용해 생성해야 합니다.](/assets/img/kubeflow/kubeyolo106.png)
    
    노트북 생성 중 볼륨 생성이 아닌 기존 볼륨을 사용해 생성해야 합니다.
    
    ![Jupyter Notebook 접속 사진](/assets/img/kubeflow/kubeyolo107.png)
    
    Jupyter Notebook 접속 사진
    
    ![클러스터 노드 로컬 볼륨](/assets/img/kubeflow/kubeyolo108.png)
    
    클러스터 노드 로컬 볼륨
