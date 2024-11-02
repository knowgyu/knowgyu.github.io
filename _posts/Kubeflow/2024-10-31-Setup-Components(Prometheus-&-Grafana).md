---
title: "Setup Components(Prometheus & Grafana)"
author: knowgyu
description: " "
date: 2023-11-16 14:03:38 +0900
math: true
categories: [MLOps, 1-Kubeflow-Setup]
tags: [MLOps, Kubeflow]
---

# Setup Components
---
프로메테우스는 다양한 대상으로부터 Metric을 수집하는 도구
그라파나는 모인 데이터를 시각화하는 것을 도와주는 도구

이번 페이지에서는 쿠버네티스 클러스터에 프로메테우스와 그라파나를 설치한 뒤, Seldon-Core로 생성한 SeldonDeployment로 API 요청을 보내 정상적으로 Metrics가 수집되는지 확인

## Install Prometheus & Grafana

### Helm Repository 추가

```bash
helm repo add seldonio <https://storage.googleapis.com/seldon-charts>
```

- 출력

```bash
"seldonio" has been added to your repositories
```

### Helm Repository 업데이트

```bash
helm repo update
```

- 출력

```bash
Hang tight while we grab the latest from your chart repositories...
...Successfully got an update from the "seldonio" chart repository
...Successfully got an update from the "datawire" chart repository
Update Complete. ⎈Happy Helming!⎈
```

### Helm Install

```bash
# seldon-core-analytics Helm Chart 1.12.0버전을 설치합니다
helm install seldon-core-analytics seldonio/seldon-core-analytics \\
  --namespace seldon-system \\
  --version 1.12.0

# 6개의 pod가 Running이 될 때까지 기다림
kubectl get pod -n seldon-system -w | grep seldon-core-analytics
```

- 출력

```bash
생략...
NAME: seldon-core-analytics
LAST DEPLOYED: Tue Dec 14 18:29:38 2021
NAMESPACE: seldon-system
STATUS: deployed
REVISION: 1
```

```bash
seldon-core-analytics-grafana-657c956c88-ng8wn                  2/2     Running   0          114s
seldon-core-analytics-kube-state-metrics-94bb6cb9-svs82         1/1     Running   0          114s
seldon-core-analytics-prometheus-alertmanager-64cf7b8f5-nxbl8   2/2     Running   0          114s
seldon-core-analytics-prometheus-node-exporter-5rrj5            1/1     Running   0          114s
seldon-core-analytics-prometheus-pushgateway-8476474cff-sr4n6   1/1     Running   0          114s
seldon-core-analytics-prometheus-seldon-685c664894-7cr45        2/2     Running   0          114s
```

## 정상 설치 확인

그라파나에 정상적으로 접속이 되는지 확인.

```bash
# 클라이언트 노드에서 접속하기 위해, 포트포워딩
kubectl port-forward svc/seldon-core-analytics-grafana -n seldon-system 8090:80
```

[http://localhost:8090/](http://localhost:8090/) 접속

![Untitled](/assets/img/kubeflow/kube201.png)

- Email or Username : `admin`
- Password : `password`

![Untitled](/assets/img/kubeflow/kube202.png)

- 좌측의 대시보드 아이콘 클릭 → `Manage` 버튼 클릭

**Kubernetes cluster monitoring (via Prometheus) v2**

![Untitled](/assets/img/kubeflow/kube203.png)

**Prediction Analytics**

![Untitled](/assets/img/kubeflow/kube204.png)
