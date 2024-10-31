---
title: "Setup Components(Kubeflow)"
author: knowgyu
description: " "
date: 2023-11-16 14:03:07 +0900
math: true
categories: [MLOps, Kubeflow, Setup]
tags: [MLOps, Kubeflow]
---

#kubeflow #MLOps 
https://mlops-for-all.github.io/docs/introduction/intro
# Setup Component
---
## **Kubeflow v1.4.0**

설치 파일 준비

```bash
git clone -b v1.4.0 <https://github.com/kubeflow/manifests.git>
cd manifests
```

### 각 구성 요소별 설치

git repo에 일괄 설치 커맨드가 있으나, **권장하지 않음.**

```bash
while ! kustomize build example | kubectl apply -f -; do echo "Retrying to apply resources"; sleep 10; done
```

> 23.11.21) 일괄 설치의 경우 mysql pod가 생성되지않고 pending 상태에 머물러 이로 인해 의존성이 걸려있는 pod들이 생성되지 못해 Kubeflow 설치가 정상적으로 되지 않았음.

설치하며 발생할 수 있는 이슈를 확인하는 등 **구성 요소별 설치 권장**

### **Cert-manager**

```bash
# cert-manager 설치
kustomize build common/cert-manager/cert-manager/base | kubectl apply -f -

# cert-manager namespace의 3개의 pod가 모두 Running이 될 때 까지 기다리기.
kubectl get pod -n cert-manager -w
```

- 모두 Running이 되었다면 아래와 같은 결과가 출력됨

```bash
NAME                                       READY   STATUS    RESTARTS   AGE
cert-manager-7dd5854bb4-7nmpd              1/1     Running   0          2m10s
cert-manager-cainjector-64c949654c-2scxr   1/1     Running   0          2m10s
cert-manager-webhook-6b57b9b886-7q6g2      1/1     Running   0          2m10s
```

```bash
# kubeflow-issuer 설치
kustomize build common/cert-manager/kubeflow-issuer/base | kubectl apply -f -
```

⚠️ cert-manager-webhook 이슈

cert-manager-webhook deployment가 Running이 아닌 경우에 발생합니다.

cert-manager의 pod **3개 모두 running**임을 확인한 후 다시 명령어 실행

`kustomize build common/cert-manager/kubeflow-issuer/base | kubectl apply -f -`

### **Istio**

```bash
# istio관련 Custom Resource Definition을 설치합니다.
kustomize build common/istio-1-9/istio-crds/base | kubectl apply -f -

# istio namespace를 설치합니다
kustomize build common/istio-1-9/istio-namespace/base | kubectl apply -f -

# istio를 설치합니다
kustomize build common/istio-1-9/istio-install/base | kubectl apply -f -

# istio-system namespace의 2개의 pod가 모두 Running이 될 때까지 기다리기
kubectl get po -n istio-system -w
```

- 모두 Running이 되었다면 아래와 같은 결과가 출력됨

```bash
NAME                                   READY   STATUS    RESTARTS   AGE
istio-ingressgateway-79b665c95-xm22l   1/1     Running   0          16s
istiod-86457659bb-5h58w                1/1     Running   0          16s
```

### Dex

```bash
# Dex 설치
kustomize build common/dex/overlays/istio | kubectl apply -f -

# auth namespace의 1개의 pod가 모두 Running이 될 때까지 기다리기
kubectl get po -n auth -w
```

- 모두 Running이 되었다면 아래와 같은 결과가 출력됨

```bash
NAME                   READY   STATUS    RESTARTS   AGE
dex-5ddf47d88d-458cs   1/1     Running   1          12s
```

### OIDC AuthService

```bash
# OIDC AuthService를 설치합니다
kustomize build common/oidc-authservice/base | kubectl apply -f -

# istio-system namespace의 3개의 pod가 모두 Running이 될 때까지 기다리기
kubectl get po -n istio-system -w
```

- 모두 Running이 되었다면 아래와 같은 결과가 출력됨

```bash
NAME                                   READY   STATUS    RESTARTS   AGE
authservice-0                          1/1     Running   0          14s
istio-ingressgateway-79b665c95-xm22l   1/1     Running   0          2m37s
istiod-86457659bb-5h58w                1/1     Running   0          2m37s
```

⚠️ 만약 authservice-0 pod가 Pending에서 5분 이상 머물러있다면, 아래 링크 참고

[](https://velog.io/@moey920/Kubeflow-authservice-0-permission-denied-%EC%97%90%EB%9F%AC-%ED%95%B4%EA%B2%B0)[https://velog.io/@moey920/Kubeflow-authservice-0-permission-denied-에러-해결](https://velog.io/@moey920/Kubeflow-authservice-0-permission-denied-%EC%97%90%EB%9F%AC-%ED%95%B4%EA%B2%B0)

### Kubeflow Namespace

```bash
# kubeflow namespace를 생성합니다
kustomize build common/kubeflow-namespace/base | kubectl apply -f -

# kubeflow namespace를 조회합니다.
kubectl get ns kubeflow
```

- 정상적으로 생성되면 아래와 같은 결과가 출력됨

```bash
NAME       STATUS   AGE
kubeflow   Active   8s
```

### Kubeflow Roles

```bash
# kubeflow-roles를 설치합니다
kustomize build common/kubeflow-roles/base | kubectl apply -f -

# 생성한 kubeflow-roles를 조회합니다. 6개의 cluster role이 출력되어야함.
kubectl get clusterrole | grep kubeflow
```

### Kubeflow Istio Resources

```bash
# kubeflow-istio-resources를 설치합니다
kustomize build common/istio-1-9/kubeflow-istio-resources/base | kubectl apply -f -

# 생성한 kubeflow roles를 조회합니다. 3개의 cluster role이 출력되어야함.
kubectl get clusterrole | grep kubeflow-istio
```

Kubeflow namespace에 gateway가 정상적으로 설치되었는지 확인합니다.

```bash
kubectl get gateway -n kubeflow
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
NAME               AGE
kubeflow-gateway   31s
```

### Kubeflow Pipelines

```bash
# kubeflow pipelines를 설치합니다
kustomize build apps/pipeline/upstream/env/platform-agnostic-multi-user | kubectl apply -f -
```

설치 순서의 의존성이 있는 리소스 존재. 아래와 같은 에러가 발생할 수 있음.

```bash
"error: unable to recognize "STDIN": no matches for kind "CompositeController" in version "metacontroller.k8s.io/v1alpha1""
```

위 에러 발생 시 10초 기다린 후 다시 설치 명령 실행

`kustomize build apps/pipeline/upstream/env/platform-agnostic-multi-user | kubectl apply -f -`

정상적으로 설치되었는지 확인 16개의 pod가 모두 Running이 될 때까지 기다림

```bash
kubectl get po -n kubeflow -w
```

```bash
NAME                                                     READY   STATUS    RESTARTS   AGE
cache-deployer-deployment-79fdf9c5c9-bjnbg               2/2     Running   1          5m3s
cache-server-5bdf4f4457-48gbp                            2/2     Running   0          5m3s
kubeflow-pipelines-profile-controller-7b947f4748-8d26b   1/1     Running   0          5m3s
metacontroller-0                                         1/1     Running   0          5m3s
metadata-envoy-deployment-5b4856dd5-xtlkd                1/1     Running   0          5m3s
metadata-grpc-deployment-6b5685488-kwvv7                 2/2     Running   3          5m3s
metadata-writer-548bd879bb-zjkcn                         2/2     Running   1          5m3s
minio-5b65df66c9-k5gzg                                   2/2     Running   0          5m3s
ml-pipeline-8c4b99589-85jw6                              2/2     Running   1          5m3s
ml-pipeline-persistenceagent-d6bdc77bd-ssxrv             2/2     Running   0          5m3s
ml-pipeline-scheduledworkflow-5db54d75c5-zk2cw           2/2     Running   0          5m2s
ml-pipeline-ui-5bd8d6dc84-j7wqr                          2/2     Running   0          5m2s
ml-pipeline-viewer-crd-68fb5f4d58-mbcbg                  2/2     Running   1          5m2s
ml-pipeline-visualizationserver-8476b5c645-wljfm         2/2     Running   0          5m2s
mysql-f7b9b7dd4-xfnw4                                    2/2     Running   0          5m2s
workflow-controller-5cbbb49bd8-5zrwx                     2/2     Running   1          5m2s
```

- 추가로 ml-pipeline UI가 정상적으로 접속되는지 확인

```bash
kubectl port-forward svc/ml-pipeline-ui -n kubeflow 8888:80
```

[http://localhost:8888/#/pipelines/](http://localhost:8888/#/pipelines/) 경로에 접속합니다.

![Untitled](/assets/img/kubeflow/kube001.png)

### Katib

```bash
# Katib을 설치합니다.
kustomize build apps/katib/upstream/installs/katib-with-kubeflow | kubectl apply -f -

# 4개의 pod가 Running이 될 때까지 기다립니다.
kubectl get po -n kubeflow | grep katib
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
katib-controller-68c47fbf8b-b985z                        1/1     Running   0          82s
katib-db-manager-6c948b6b76-2d9gr                        1/1     Running   0          82s
katib-mysql-7894994f88-scs62                             1/1     Running   0          82s
katib-ui-64bb96d5bf-d89kp                                1/1     Running   0          82s
```

- 추가로 katib UI가 정상적으로 접속되는지 확인

```bash
kubectl port-forward svc/katib-ui -n kubeflow 8081:80
```

[http://localhost:8081/katib/](http://localhost:8081/katib/) 경로에 접속합니다.

![Untitled](/assets/img/kubeflow/kube002.png)

### Central Dashboard

```bash
# Central Dashboard를 설치합니다
kustomize build apps/centraldashboard/upstream/overlays/istio | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow -w | grep centraldashboard
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
centraldashboard-8fc7d8cc-xl7ts                          1/1     Running   0          52s
```

- 추가로 Central Dashboard UI가 정상적으로 접속되는지 확인

```bash
kubectl port-forward svc/centraldashboard -n kubeflow 8082:80
```

[http://localhost:8082/](http://localhost:8082/) 경로에 접속합니다

![Untitled](/assets/img/kubeflow/kube003.png)

### Admission Webhook

```bash
# Admission Webhook을 설치합니다
kustomize build apps/admission-webhook/upstream/overlays/cert-manager | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow -w | grep admission-webhook
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
admission-webhook-deployment-667bd68d94-2hhrx            1/1     Running   0          11s
```

### Notebooks & Jupyter Web APP

```bash
# Notebook controller를 설치합니다
kustomize build apps/jupyter/notebook-controller/upstream/overlays/kubeflow | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow -w | grep notebook-controller
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
notebook-controller-deployment-75b4f7b578-w4d4l          1/1     Running   0          105s
```

```bash
# Jupyter Web App를 설치합니다
kustomize build apps/jupyter/jupyter-web-app/upstream/overlays/istio | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow | grep jupyter-web-app
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
jupyter-web-app-deployment-6f744fbc54-p27ts              1/1     Running   0          2m
```

### Profiles + KFAM

```bash
# Profile controller를 설치합니다
kustomize build apps/profiles/upstream/overlays/kubeflow | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow -w | grep profiles-deployment
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
profiles-deployment-89f7d88b-qsnrd                       2/2     Running   0          42s
```

### Volumes Web App

```bash
# Volumes Web APP를 설치합니다
kustomize build apps/volumes-web-app/upstream/overlays/istio | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow -w | grep volumes-web-app
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
volumes-web-app-deployment-8589d664cc-62svl              1/1     Running   0          27s
```

### Tensorboard & Tensorboard Web App

```bash
# Tensorboard Web App를 설치합니다
kustomize build apps/tensorboard/tensorboards-web-app/upstream/overlays/istio | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow | grep tensorboards-web-app
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
tensorboards-web-app-deployment-6ff79b7f44-qbzmw            1/1     Running             0          22s
```

```bash
# Tensorboard Controller를 설치합니다
kustomize build apps/tensorboard/tensorboard-controller/upstream/overlays/kubeflow | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow -w | grep tensorboard-controller
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
tensorboard-controller-controller-manager-954b7c544-vjpzj   3/3     Running   1          73s
```

### Training Operator

```bash
# Training Operator를 설치합니다
kustomize build apps/training-operator/upstream/overlays/kubeflow | kubectl apply -f -

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get po -n kubeflow -w | grep training-operator
```

- 정상적으로 설치되었다면 아래와 같은 메세지가 출력됨

```bash
training-operator-7d98f9dd88-6887f                          1/1     Running   0          28s
```

### User Namespace

Kubeflow의 사용을 위해, 사용할 User의 Ubeflow Profile을 생성합니다.

```bash
kustomize build common/user-namespace/base | kubectl apply -f -
```

- 정상적으로 수행되면 아래와 같은 메세지가 출력됨

```bash
configmap/default-install-config-9h2h2b6hbk created
profile.kubeflow.org/kubeflow-user-example-com created
```

## 정상 설치 확인

```bash
kubectl port-forward svc/istio-ingressgateway -n istio-system 8080:80
```

[http://localhost:8080/에](http://localhost:8080/%EC%97%90) 접속합니다.

![Untitled](/assets/img/kubeflow/kube004.png)

Email Address : `user@example.com`

Password : `12341234`

![Untitled](/assets/img/kubeflow/kube005.png)
