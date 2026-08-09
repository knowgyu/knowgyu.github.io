---
title: "Setup Components(MLflow Tracking Server)"
author: knowgyu
description: "MLflow 서버 설정"
kind: technical-note
date: 2023-11-16 14:03:20 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

# Setup Components
---
MLflow는 오픈소스 ML 실험 관리 도구입니다. MLflow는 실험 관리 용도 외에도 ML Model 패키징, ML 모델 배포 관리, ML 모델 저장과 같은 기능도 제공함.

MLflow를 실험 관리 용도로 사용할 것.

MLflow에서 관리하는 데이터를 저장하고 UI를 제공하는 MLflow Tracking Server를 쿠버네티스 클러스터에 배포해 사용 예정

## Before Install MLflow Tracking Server

### PostgreSQL DB 설치

MLflow Tracking Server가 Backend Store로 사용할 용도의 DB를 쿠버네티스 클러스터에 배포합니다.

```bash
# mlflow-system namespace 생성
kubectl create ns mlflow-system

# postgresql DB를 mlflow-system namespace에 생성
kubectl -n mlflow-system apply -f <https://raw.githubusercontent.com/mlops-for-all/helm-charts/b94b5fe4133f769c04b25068b98ccfa7a505aa60/mlflow/manifests/postgres.yaml>

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get pod -n mlflow-system -w |grep postgresql
```

- 정상적으로 실행되면 아래와 같은 메세지가 출력됨

```bash
postgresql-mlflow-7b9bc8c79f-srkh7   1/1     Running   0          38s
```

### Minio 설정

MLflow Tracking Server가 Artifacts Store로 사용할 용도의 MInio는 이전 Kubeflow 설치 시 설치된 Minio를 활용함. 단, kubeflow와 분리하기 위해, 전용 버킷을 생성

```bash
# minio에 접속해 버킷을 생성하기 위해 포트포워딩
kubectl port-forward svc/minio-service -n kubeflow 9000:9000
```

[http://localhost:9000/](http://localhost:9000/) 에 접속합니다.

![Untitled](/assets/img/kubeflow/kube101.png)

- Username : `minio`
- Password : `minio123`

우측 하단의 `+` 버튼을 클릭 → `Create Bucket` 클릭

`Bucket Name`에 `mlflow`를 입력하여 버킷을 생성합니다.

![Untitled](/assets/img/kubeflow/kube102.png)

## Install MLflow Tracking Server

### Helm Repository 추가

```bash
helm repo add mlops-for-all <https://mlops-for-all.github.io/helm-charts>
```

- 출력

```bash
"mlops-for-all" has been added to your repositories
```

### Helm Repository 업데이트

```bash
helm repo update
```

- 출력

```bash
Hang tight while we grab the latest from your chart repositories...
...Successfully got an update from the "mlops-for-all" chart repository
Update Complete. ⎈Happy Helming!⎈
```

### Helm Install

```bash
# mlflow-server Helm Chart 0.2.0 버전을 설치합니다
helm install mlflow-server mlops-for-all/mlflow-server \\
  --namespace mlflow-system \\
  --version 0.2.0

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get pod -n mlflow-system -w | grep mlflow-server
```

<aside> 💡 **주의:** 위의 helm chart는 MLflow의 backend store와 artifacts store의 접속 정보를 minio와 postgresql 정보를 default로 하여 설치합니다.

별개로 생성한 DB 혹은 Object Storage를 활용하고 싶은 경우, 아래 링크를 참고해 helm install 시 value를 따로 설정해 설치합니다. [https://github.com/mlops-for-all/helm-charts/tree/main/mlflow/chart](https://github.com/mlops-for-all/helm-charts/tree/main/mlflow/chart)

</aside>

- 정상적으로 설치되었다면 아래와 같은 메세지 출력

```bash
mlflow-server-ffd66d858-6hm62        1/1     Running   0          74s
```

## 정상 설치 확인

MLflow Server에 정상적으로 접속되는지 확인

우선 클라이언트 노드에서 접속하기 위해, 포트포워딩을 수행

```bash
kubectl port-forward svc/mlflow-server-service -n mlflow-system 5000:5000
```

[http://localhost:5000/](http://localhost:5000/) 에 접속

![Untitled](/assets/img/kubeflow/kube103.png)
