---
title: "PV Mount & HyperParameter Tuning"
author: knowgyu
description: " "
date: 2023-12-05 08:14:07 +0900
math: true
categories: [MLOps, 4-Kubeflow-Pipeline-YOLOv8]
tags: [MLOps, Kubeflow]
---

본 문서는 파이프라인을 PV에 마운트하는 것과 YOLOv8에서 기본으로 제공하는 raytune을 이용해 하이퍼파라미터 튜닝을 하는 것에 대해 다룹니다.<br>

[https://happycloud-lee.tistory.com/256](https://happycloud-lee.tistory.com/256) ← PV/PVC 참고<br>

[https://docs.ultralytics.com/integrations/ray-tune/?h=hyper](https://docs.ultralytics.com/integrations/ray-tune/?h=hyper) ← HP Tuning 공식문서<br>

# PV / PVC

---

## k8s 환경에서의 볼륨

### 외부 볼륨의 필요성

쿠버네티스 환경에서는 모든 어플리케이션이 Pod안에 컨테이너로 실행됩니다.

보통 **컨테이너**에서 데이터를 읽고 저장할 때는 자신의 내부 볼륨이 아닌 **외부 볼륨**을 사용합니다.

→ 컨테이너는 언제든 사라질 수 있기에, 데이터를 잃는 상황을 막기 위해!

### 사용 목적별 볼륨 유형

Pod안에 컨테이너화되어 실행되는 것은 크게 **어플리케이션**과 **데이터베이스**입니다.

볼륨을 크게 나누면 Pod 로컬 볼륨, Node 로컬 볼륨, 네트워크 볼륨이 있습니다.(공식적인건 아님)

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

Pod가 볼륨을 사용하는 방법은 ‘**마운트**’ 입니다.

Pod내에 볼륨을 ‘마운트’ 함으로써 어떤 유형의 볼륨이든 Pod의 내부 파일 시스템처럼 사용할 수 있습니다.

하지만, 이 아키텍처는 쿠버네티스와 인프라스트럭처가 너무 강한 결합(**Tightly Coupled**)되어버립니다.

이로 인해 스토리지 제품의 변화나 스토리지 서버의 변화가 Pod에 직접적인 영향을 미칩니다.

- **예시**
    
    `nfs` 라는 네트워크 볼륨을 Pod에 마운트하려면 `nfs` 서버의 IP나 호스트를 지정해야 하는데, `nfs` 서버의 IP가 변경되면 서비스에 문제가 발생할 수 있습니다.
    

이를 해결하기 위해 쿠버네티스와 인프라스트럭처를 느슨하게 결합(**Loosly Coupled**)시키면 됩니다.

이를 위해 중간에 중계자 역할을 하는 **PV/PVC** 를 만들게 됩니다.


> 네트워크 볼륨을 위해 CSIContainer Storage Interface라는 중계자 역할도 있습니다.
> 하지만, 딥러닝 학습 시 네트워크 볼륨을 사용하게 된다면 학습이 느려진다는 단점이 있어 로컬 볼륨을 사용하기에 CSI에 대해서는 다루지 않겠습니다.
{: .prompt-tip }

![https://happycloud-lee.tistory.com/256](/assets/img/kubeflow/kubeyolo102.png)
https://happycloud-lee.tistory.com/256

1. Pod정보, Config-map, Secret은 파드 명세에 정의하여 마운트합니다.

2. 노드 로컬 볼륨과 네트워크 볼륨은 PV리소스로 정의하고 PVC에 바운드(연결)합니다.
    파드 명세에서는 PVC만 지정하면 연결된 볼륨이 마운트 됩니다.

3. 스토리지 제품별로 PV을 정의하여 볼륨을 접근할 수 있습니다.<br>

이렇게 되면 PV를 정의하고, PVC에 연결한 후 Pod에 PVC만 지정해주면 됩니다.

## 파이프라인 볼륨

파이프라인은 1개 이상의 컴포넌트로 이루어져 있습니다. 각각의 컴포넌트는 Pod로 구성되어 파이프라인을 통해 실행됩니다. 
즉, 각각의 컴포넌트는 서로 다른 로컬 볼륨을 사용하므로 다른 컴포넌트의 데이터를 읽거나 쓰는 등의 작업이 불가합니다.<br>

이를 위해, PV를 만들고, 각각의 컴포넌트(혹은 Pod)에서 PVC를 통해 볼륨을 마운트 하겠습니다.<br>


> PV를 생성하고 PVC를 통해 Pod에 연결하는 방법은 다양한 것 같지만, 본 페이지에서는 다른 방법을 잘 모르기에, yaml파일을 통해 Persistent Volume을 생성하고, PVC로 PV를 연결한 후, Pod에서 PVC를 지정하는 방법으로 하겠습니다.
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
  storageClassName: 'knowgyu-pv-test'
  hostPath:
    path: "/data/datasets/coco8"
  nodeAffinity:
    required:
      nodeSelectorTerms:
      - matchExpressions:
        - key: kubernetes.io/hostname
          operator: In
          values:
          - knowgyu
```

> `accessModes` 의 경우 Many와 Once가 있는데, Many로 설정하게 된다면 여러 개의 노드에서 동시에 사용할 수 있도록 지정하는 것입니다. 하지만, 현재 minikube를 이용해 단일 노드로 생성했기에, RWO모드로 지정합니다.


> 변경해야 하는 값들 
{: .prompt-info }

- `name: pv-test`
- `storage: 1Gi`
- `storageClassName: 'knowgyu-pv-test'`
- `path: "/data/datasets/coco8"`
- `values : knowgyu`

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

`pvc.yaml`을 작성하지 않아도, Kubeflow 대시보드의 **Volume**탭을 이용하거나, Pipeline 작성 시 `VolumeOp`를 통해 생성할 수 있습니다. 하지만, 본 페이지에선 동적 할당이 아닌 수동 할당을 위해 직접 명세를 작성해 PVC를 생성하겠습니다.

- *~~+) 위 내용이 정확한 사실인지는 잘 모르겠습니다. 제가 공부하고 실험해 본 결과 그랬던 것 같습니다.~~*
    
    ![Untitled](/assets/img/kubeflow/kubeyolo109.png)

    
- `pvc.yaml`

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: pvc-test
  namespace: knowgyu 
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
# 모든 PVC가 표시됩니다. 네임스페이스를 knowgyu로 지정해주었고, pv-test와 제대로 Bound 되었는지 확인합니다.
NAMESPACE                   NAME                                            STATUS   VOLUME                                     CAPACITY   ACCESS MODES   STORAGECLASS   AGE
knowgyu                     pvc-test                                        Bound    pv-test                                    5Gi        RWO            scn-test       100m
```

### PV/PVC 생성 확인

제대로 지정되었는지 확인해보겠습니다.

- PV 확인

```bash
kubectl get pv -A |grep pv-test
```

위 명령어를 통해 비어있던 CLAIM이 `knowgyu/pvc-test` 로 변경되었는지 확인합니다.

```bash
pv-test                                    5Gi        RWO            Retain           Bound    knowgyu/pvc-test                                           scn-test                103m
```

또한, Kubeflow 대시보드를 통해 재차 확인할 수 있습니다.

`knowgyu` 네임스페이스를 사용하는 사용자 계정으로 로그인 후 좌측의 Volume탭을 클릭합니다.

![만약 위의 과정을 똑같이 따라했다면, Name은 pvc-test로 나와야합니다.](/assets/img/kubeflow/kubeyolo103.png)


만약 위의 과정을 똑같이 따라했다면, Name은 pvc-test로 나와야합니다.

### Pod → PV 마운트

PV/PVC를 생성했으니, 이 볼륨을 사용할 Pod에 마운트를 시켜야 합니다.

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

**`dsl.VolumeOp(...)`** : k8s PVC를 생성하는 파이프라인 태스크

- 변경 사항

```python
    vop = dsl.VolumeOp(
        name="create-pvc",  <- Pipeline에서 표시될 이름인 것 같습니다.
        resource_name="pvc-test", <- 생성될 PVC 이름. 기존의 PVC를 사용하기에 똑같이 해야함
        storage_class='scn-test', <- *동적으로 생성되는 PVC에 사용할 스토리지 클래스를 지정// 잘 모르는 기능*
        modes=dsl.VOLUME_MODE_RWO, <- 모드. Single Node이므로 ReadWriteOnce
        size="5Gi",         <- 생성될 PVC 크기. 기존PVC 명세보다 크게는 가능하나 작게 지정할 시 에러가 발생합니다.
        generate_unique_name=False, <- True시 pvc-test-1sfd.. 이런식으로 생성될 것 같습니다. 기존 생성한 PVC와 이름이 같아야하기에 False로 합니다.
        action='apply', <- 깃 이슈에서 참고했는데, 추가해야 적용됐습니다.
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

~~학습 컴포넌트를 거쳐 생성된 `knowgyu`를 불러와 하이퍼파라미터 튜닝을 하는 컴포넌트를 추가해보겠습니다.~~

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

    return os.path.join('testprj','testexp','weights','knowgyu')
    
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
    kfp.compiler.Compiler().compile(train_pipeline, "knowgyu_MountTest.yaml")
```

- `knowgyu_MountTest.yaml` *~~# 전체코드와 yaml파일은 예제와 다를 수 있습니다.~~*
    
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
