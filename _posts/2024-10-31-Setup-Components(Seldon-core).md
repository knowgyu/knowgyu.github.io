---
title: "Setup Components(Seldon-core)"
author: knowgyu
description: " "
date: 2023-11-16 14:03:28 +0900
math: true
categories: [MLOps, Kubeflow, Setup]
tags: [MLOps, Kubeflow]
---

# Setup Components
---
Seldon-Core는 쿠버네티스 환경에서 머신러닝 모델을 배포하고 관리할 수 있는 오픈소스 프레임워크

## Install Seldon-Core

Seldon-Core를 사용하기 위해서는 쿠버네티스의 인그레스(Ingress)를 담당하는 Ambassador혹은 Istio 모듈이 필요함.

본 문서에서는 Ambassador를 사용해 Seldon-Core를 사용

### Ambassador - Helm Repository 추가

```bash
helm repo add datawire <https://www.getambassador.io>
```

- 출력

```bash
"datawire" has been added to your repositories
```

### Helm Repository 업데이트

```bash
helm repo update
```

- 출력

```bash
Hang tight while we grab the latest from your chart repositories...
...Successfully got an update from the "datawire" chart repository
Update Complete. ⎈Happy Helming!⎈
```

### Ambassador - Helm Install

```bash
# ambassador Chart 6.9.3 버전을 설치합니다
helm install ambassador datawire/ambassador \\
  --namespace seldon-system \\
  --create-namespace \\
  --set image.repository=quay.io/datawire/ambassador \\
  --set enableAES=false \\
  --set crds.keep=false \\
  --version 6.9.3

# 4개의 pod가 Running이 될 때까지 기다림
kubectl get pod -n seldon-system -w
```

- 다음과 같은 메세지가 출력되어야 합니다.

```bash
생략...

W1206 17:01:36.026326   26635 warnings.go:70] rbac.authorization.k8s.io/v1beta1 Role is deprecated in v1.17+, unavailable in v1.22+; use rbac.authorization.k8s.io/v1 Role
W1206 17:01:36.029764   26635 warnings.go:70] rbac.authorization.k8s.io/v1beta1 RoleBinding is deprecated in v1.17+, unavailable in v1.22+; use rbac.authorization.k8s.io/v1 RoleBinding
NAME: ambassador
LAST DEPLOYED: Mon Dec  6 17:01:34 2021
NAMESPACE: seldon-system
STATUS: deployed
REVISION: 1
NOTES:
-------------------------------------------------------------------------------
  Congratulations! You've successfully installed Ambassador!

-------------------------------------------------------------------------------
To get the IP address of Ambassador, run the following commands:
NOTE: It may take a few minutes for the LoadBalancer IP to be available.
     You can watch the status of by running 'kubectl get svc -w  --namespace seldon-system ambassador'

  On GKE/Azure:
  export SERVICE_IP=$(kubectl get svc --namespace seldon-system ambassador -o jsonpath='{.status.loadBalancer.ingress[0].ip}')

  On AWS:
  export SERVICE_IP=$(kubectl get svc --namespace seldon-system ambassador -o jsonpath='{.status.loadBalancer.ingress[0].hostname}')

  echo http://$SERVICE_IP:

For help, visit our Slack at <http://a8r.io/Slack> or view the documentation online at <https://www.getambassador.io>.
```

```bash
ambassador-7f596c8b57-4s9xh                  1/1     Running   0          7m15s
ambassador-7f596c8b57-dt6lr                  1/1     Running   0          7m15s
ambassador-7f596c8b57-h5l6f                  1/1     Running   0          7m15s
ambassador-agent-77bccdfcd5-d5jxj            1/1     Running   0          7m15s
```

### Seldon-Core - Helm Install

```bash
# seldon-core operator chart 1.11.2버전을 설치합니다
helm install seldon-core seldon-core-operator \\
    --repo <https://storage.googleapis.com/seldon-charts> \\
    --namespace seldon-system \\
    --set usageMetrics.enabled=true \\
    --set ambassador.enabled=true \\
    --version 1.11.2

# 1개의 pod가 Running이 될 때까지 기다림
kubectl get pod -n seldon-system -w | grep seldon-controller
```

- 다음와 같은 메세지가 출력되어야 합니다

```bash
생략...

W1206 17:05:38.336391   28181 warnings.go:70] admissionregistration.k8s.io/v1beta1 ValidatingWebhookConfiguration is deprecated in v1.16+, unavailable in v1.22+; use admissionregistration.k8s.io/v1 ValidatingWebhookConfiguration
NAME: seldon-core
LAST DEPLOYED: Mon Dec  6 17:05:34 2021
NAMESPACE: seldon-system
STATUS: deployed
REVISION: 1
TEST SUITE: None
```

```bash
seldon-controller-manager-8457b8b5c7-r2frm   1/1     Running   0          2m22s
```
