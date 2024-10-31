---
title: "Setup Kubernetes"
author: knowgyu
description: " "
date: 2023-11-19 14:02:58 +0900
math: true
categories: [MLOps, Kubeflow, Setup]
tags: [MLOps, Kubeflow]
---

# Setup Kubernetes
---
## Setup Kubernetes Cluster

**k3s** : 쿠버네티스 클러스터를 쉽게 구축할 수 있음
**kubeadm** : 프로덕션 레벨의 클러스터 구축. 공식적으로 지원하는 도구
**minikube** : 다른 쿠버네티스를 add-on 형식으로 쉽게 설치할 수 있음

## Install Prerequisite
**apt 패키지 설치**
클라이언트와 클러스터의 원활한 통신을 위해 포트포워딩 사용.

```bash
sudo apt-get update
sudo apt-get install -y socat
```

**도커 설치**
(본 문서에서는 다루지 않음)

**Swap 메모리 끄기**
kubelet의 정상 동작을 위해 클러스터 노드에서 swap을 꺼야함.

```bash
sudo sed -i '/ swap / s/^\\(.*\\)$/#\\1/g' /etc/fstab
sudo swapoff -a
```

**Kubectl 설치(클라이언트 노드)**
kubectl은 쿠버네티스 클러스터에 API를 요청할 때 사용하는 클라이언트 툴.

```bash
# kubectl v1.21.7 Download
curl -LO <https://dl.k8s.io/release/v1.21.7/bin/linux/amd64/kubectl>

# kubectl 사용을 위해 권한과 위치 변경
sudo install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl

# 정상 설치 확인
kubectl version --client

# 아래와 같은 메시지가 보이면 정상 설치됨
Client Version: version.Info{Major:"1", Minor:"21", GitVersion:"v1.21.7", GitCommit:"1f86634ff08f37e54e8bfcd86bc90b61c98f84d4", GitTreeState:"clean", BuildDate:"2021-11-17T14:41:19Z", GoVersion:"go1.16.10", Compiler:"gc", Platform:"linux/amd64"}
```

## Minikube
**Minikube 설치**
```bash
# 설치
wget <https://github.com/kubernetes/minikube/releases/download/v1.24.0/minikube-linux-amd64>
sudo install minikube-linux-amd64 /usr/local/bin/minikube

# 정상 설치 확인
minikube version

# 아래와 같은 메시지가 보이면 정상 설치됨
minikube version: v1.24.0
commit: 76b94fb3c4e8ac5062daf70d60cf03ddcc0a741b
```

### 쿠버네티스 클러스터 셋업

Minikube를 활용해 클러스터 구축. GPU의 원활한 사용과 클러스터-클라이언트 간 통신을 편하게 수행하기 위해, `driver=none`
`driver=none` 옵션은 root user로 실행해야 함.

```bash
sudo su

# 클러스터 생성
minikube start --driver=none \\
  --kubernetes-version=v1.21.7 \\
  --extra-config=apiserver.service-account-signing-key-file=/var/lib/minikube/certs/sa.key \\
  --extra-config=apiserver.service-account-issuer=kubernetes.default.svc

# 미사용 add-on 비활성화
minikube addons disable storage-provisioner
minikube addons disable default-storageclass
```

### 쿠버네티스 클라이언트 셋업
**클라이언트**와 **클러스터** 노드가 분리되지 않은 경우 root user로 모든 작업을 진행해야 함.
**클라이언트**와 **클러스터** 노드가 분리된 경우, k8s의 관리자 인증 정보를 **클라이언트**로 가져옵니다.

```bash
# 클러스터 노드
minikube kubectl -- config view --flatten
```

- 출력 예시
    
    ```bash
    apiVersion: v1
    clusters:
    - cluster:
        certificate-authority-data: LS0...생략==
        extensions:
        - extension:
            last-update: Tue, 21 Nov 2023 16:49:33 KST
            provider: minikube.sigs.k8s.io
            version: v1.24.0
          name: cluster_info
        server: <https://192.168.1.87:8443>
      name: minikube
    contexts:
    - context:
        cluster: minikube
        extensions:
        - extension:
            last-update: Tue, 21 Nov 2023 16:49:33 KST
            provider: minikube.sigs.k8s.io
            version: v1.24.0
          name: context_info
        namespace: default
        user: minikube
      name: minikube
    current-context: minikube
    kind: Config
    preferences: {}
    users:
    - name: minikube
      user:
        client-certificate-data: LS0...생략==
        client-key-data: LS0t...생략==
    ```
    

```bash
# 클라이언트 노드
mkdir -p /home/$USER/.kube

# 클러스터 노드에서 출력한 정보 붙여넣기
vi /home/$USER/.kube/config
```

**정상 설치 확인**

```bash
kubectl get nodes -o wide

# 아래와 같은 출력이 보이면 정상 설치
NAME        STATUS   ROLES                  AGE   VERSION   INTERNAL-IP    EXTERNAL-IP   OS-IMAGE             KERNEL-VERSION     CONTAINER-RUNTIME
stans-dev   Ready    control-plane,master   18h   v1.21.7   192.168.1.87   <none>        Ubuntu 22.04.3 LTS   6.2.0-36-generic   docker://24.0.7
```

## Install Kubernetes Modules(클라이언트 노드)

아래 모듈들은 모두 **클라이언트 노드**에서 진행

### Helm
Helm은 k8s 패키지와 관련된 자원을 한 번에 배포하고 관리할 수 있게 도와주는 패키지 매니징 도구

```bash
# 다운로드
wget <https://get.helm.sh/helm-v3.7.1-linux-amd64.tar.gz>

# 설치
tar -zxvf helm-v3.7.1-linux-amd64.tar.gz
sudo mv linux-amd64/helm /usr/local/bin/helm

# 정상 설치 확인
helm help
```

### Kustomize
kustomize 또한 여러 k8s 리소스를 한 번에 배포하고 관리할 수 있게 도와주는 패키지 매니징 도구

```bash
# 다운로드(Linux amd64)
wget <https://github.com/kubernetes-sigs/kustomize/releases/download/kustomize%2Fv3.10.0/kustomize_v3.10.0_linux_amd64.tar.gz>

# 설치
tar -zxvf kustomize_v3.10.0_linux_amd64.tar.gz
sudo mv kustomize /usr/local/bin/kustomize

# 정상 설치 확인
kustomize help
```

### CSI Plugin : Local Path Provisioner
CSI Plugin은 k8s 내의 스토리지를 담당하는 모듈.

```bash
# 설치
kubectl apply -f <https://raw.githubusercontent.com/rancher/local-path-provisioner/v0.0.20/deploy/local-path-storage.yaml>
```

```bash
# pod이 Running인지 확인
kubectl -n local-path-storage get pod

# Running이 되었다면 default storage class로 변경
kubectl patch storageclass local-path  -p '{"metadata": {"annotations":{"storageclass.kubernetes.io/is-default-class":"true"}}}'

# default storage class로 설정되었는지 확인
kubectl get sc
```

## Setup GPU

쿠버네티스 및 Kubeflow 등 클러스터에서 GPU를 사용하기 위해

**Nvidia Driver 설치**
(본 문서에서는 다루지 않음)

**Nvidia-Docker 설치**
(본 문서에서는 다루지 않음)

### **Nvidia-Docker를 Default Container Runtime으로 설정**
`/etc/docker/daemon.json` 파일 수정

```bash
sudo vi /etc/docker/daemon.json
```

```json
{
  "default-runtime": "nvidia",
  "runtimes": {
      "nvidia": {
          "path": "nvidia-container-runtime",
          "runtimeArgs": []
  }
  }
}
```

```bash
# 파일 수정 후 Docker 재시작
sudo systemctl daemon-reload
sudo service docker start

# 변경 사항 반영 확인
sudo docker info |grep nvidia
```

### Nvidia-Device-Plugin

```bash
# nvidia-device-plugin daemonet을 생성합니다.
kubectl create -f <https://raw.githubusercontent.com/NVIDIA/k8s-device-plugin/v0.10.0/nvidia-device-plugin.yml>

# pod이 Running 상태인지 확인
kubectl get pod -n kube-system | grep nvidia

# Running 상태가 되었다면 GPU가 사용가능하도록 설정되었는지 확인
kubectl get nodes "-o=custom-columns=NAME:.metadata.name,GPU:.status.allocatable.nvidia\\.com/gpu"
```

정상적으로 설정 되었다면 아래와 같은 메시지가 출력됩니다.

```bash
NAME        GPU
stans-dev   1
```
