---
title: "Misc. MetalLB Settings"
author: knowgyu
description: " "
date: 2023-11-26 05:46:15 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

이 글에서는 베어메탈 클러스터용 로드 밸런서인 MetalLB의 설치와 설정 과정을 정리합니다.

# Load Balancer

---

로드 밸런서는 네트워크 트래픽을 여러 서버에 분산시키는 역할을 합니다.

> *부하 분산*은 백 엔드 서버 또는 리소스의 그룹에서 들어오는 네트워크 트래픽을 효율적으로 분산하는 것을 의미합니다.<br>
[https://learn.microsoft.com/ko-kr/azure/load-balancer/load-balancer-overview](https://learn.microsoft.com/ko-kr/azure/load-balancer/load-balancer-overview)

쿠버네티스를 사용할 때 AWS, GCP, Azure 같은 클라우드 플랫폼은 자체 로드 밸런서를 제공하지만,

온프레미스 클러스터에서는 로드 밸런싱 기능을 제공하는 모듈을 별도로 설치해야 합니다.

**MetalLB**는 베어메탈 환경에서 사용할 수 있는 로드 밸런서를 제공하는 오픈소스입니다.

## MetalLB 설치 요구사항

---

| 요구 사항                                                                           | 버전 및 내용                                                     |
| ----------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| Kubernetes                                                                          | 로드 밸런싱 기능이 없는 >= v1.13.0                               |
| [호환가능한 네트워크 CNI](https://metallb.universe.tf/installation/network-addons/) | Calico, Canal, Cilium, Flannel, Kube-ovn, Kube-router, Weave Net |
| IPv4 주소                                                                           | MetalLB 배포에 사용                                              |
| BGP 모드를 사용할 경우                                                              | BGP 기능을 지원하는 하나 이상의 라우터                           |
| 노드 간 포트 TCP/UDP 7946 오픈                                                      | memberlist 요구 사항                                             |

💡 여기서는 BGP 모드가 아니라 Layer 2 모드를 사용할 예정이며, 추가로 준비할 항목은 없었습니다.

## MetalLB 설치

---

### Preparation

IPVS 모드에서 `kube-proxy` 를 사용하는 경우 K8s v1.14.2 이후부터는 strict ARP 모드를 사용해야 합니다.

여기서 사용한 환경은 v1.21.7 이므로 strict ARP 설정을 적용합니다.

```bash
# 현재 모드 확인
kubectl get configmap kube-proxy -n kube-system -o yaml | \
grep strictARP
```

```bash
strictARP: false
```

`false` 로 되어 있다면 `true` 로 변경합니다.

```bash
# actually apply the changes, returns nonzero returncode on errors only
kubectl get configmap kube-proxy -n kube-system -o yaml | \
sed -e "s/strictARP: false/strictARP: true/" | \
kubectl apply -f - -n kube-system
```

```bash
Warning: resource configmaps/kube-proxy is missing the kubectl.kubernetes.io/last-applied-configuration annotation which is required by kubectl apply. kubectl apply should only be used on resources created declaratively by either kubectl create --save-config or kubectl apply. The missing annotation will be patched automatically.
configmap/kube-proxy configured
```

### 설치 - Manifest

```bash
kubectl apply -f https://raw.githubusercontent.com/metallb/metallb/v0.11.0/manifests/namespace.yaml
kubectl apply -f https://raw.githubusercontent.com/metallb/metallb/v0.11.0/manifests/metallb.yaml

# 정상적으로 설치되었는지 확인합니다. 두 Pod가 모두 `Running` 상태가 될 때까지 기다립니다.
kubectl get pod -n metallb-system
```

```bash
NAME                          READY   STATUS    RESTARTS   AGE
controller-7dcc8764f4-8n92q   1/1     Running   1          1m
speaker-fnf8l                 1/1     Running   1          1m
```

- **controller** : Deployment로 배포되며, 로드 밸런싱에 사용할 외부 IP 주소 할당을 담당합니다.
- **speaker** : DaemonSet 형태로 배포되며, 외부 트래픽과 서비스를 연결해 네트워크 통신이 가능하도록 동작합니다.

<aside>
💡 설치 리소스에는 controller 와 speaker 가 동작하는 데 필요한 RBAC 권한도 함께 포함됩니다.

</aside>

## Configuration

---

MetalLB의 로드 밸런싱 정책은 ConfigMap을 배포해 설정할 수 있습니다.

MetalLB에서 구성할 수 있는 모드는 `Layer 2 모드` 와 `BGP 모드` 입니다.

### Layer 2 Configuration

Layer 2 모드는 사용할 IP 주소 대역만 지정하면 됩니다.

Layer 2 모드를 사용할 경우 워커 노드의 네트워크 인터페이스에 IP를 직접 바인딩하지 않아도 됩니다.

→ 로컬 네트워크의 ARP 요청에 직접 응답해 컴퓨터의 MAC주소를 클라이언트에 제공

- `metallb_config.yaml`

```bash
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: metallb-system
  name: config
data:
  config: |
    address-pools:
    - name: default
      protocol: layer2
      addresses:
      - 192.168.1.87-192.168.1.97 # IP 대역폭
```

> 위 설정은 `192.168.1.87 ~ 192.168.1.97` 구간의 IP에 대한 제어 권한을 부여하고, Layer 2 모드를 구성합니다.
> 클러스터 노드와 클라이언트 노드가 분리돼 있다면, 이 IP 대역은 두 노드 모두에서 접근 가능한 네트워크여야 합니다.

```bash
kubectl apply -f metallb_config.yaml
```

```bash
# 정상적으로 배포되면 아래와 같은 메세지가 출력됩니다.
configmap/config created
```

## MetalLB 사용

---

### Kubeflow Dashboard

Kubeflow 대시보드를 제공하는 `istio-system` 네임스페이스의 `istio-ingressgateway` 서비스 타입을 변경합니다.

```bash
# 현재 서비스타입 확인
kubectl get svc/istio-ingressgateway -n istio-system
```

```bash
NAME                   TYPE        CLUSTER-IP    EXTERNAL-IP   PORT(S)                                        AGE
istio-ingressgateway   ClusterIP   10.98.123.79   <none>        15021/TCP,80/TCP,443/TCP,31400/TCP,15443/TCP   4h21m
```

> 서비스 타입은 `ClusterIP` 이며, 외부 IP 값은 `<none>` 상태입니다.

```bash
# 타입을 `LoadBalancer` 로 변경하고 IP 주소를 입력합니다.
# IP 주소를 직접 입력하지 않으면 위에서 설정한 IP 풀에서 자동 배정됩니다.
kubectl edit svc/istio-ingressgateway -n istio-system
```

```yaml
# Please edit the object below. Lines beginning with a '#' will be ignored,
# and an empty file will abort the edit. If an error occurs while saving this file will be
# reopened with the relevant failures.
#
apiVersion: v1
kind: Service
metadata:
  annotations:
    kubectl.kubernetes.io/last-applied-configuration: |
      {"apiVersion":"v1","kind":"Service","metadata":{"annotations":{},"labels":{"app":"istio-ingressgateway","install.operator.istio.io/owning-resource":"unknown","istio":"ingressgateway","istio.io/rev":"default","operator.istio.io/component":"IngressGateways","release":"istio"},"name":"istio-ingressgateway","namespace":"istio-system"},"spec":{"ports":[{"name":"status-port","port":15021,"protocol":"TCP","targetPort":15021},{"name":"http2","port":80,"protocol":"TCP","targetPort":8080},{"name":"https","port":443,"protocol":"TCP","targetPort":8443},{"name":"tcp","port":31400,"protocol":"TCP","targetPort":31400},{"name":"tls","port":15443,"protocol":"TCP","targetPort":15443}],"selector":{"app":"istio-ingressgateway","istio":"ingressgateway"},"type":"NodePort"}}
  creationTimestamp: "2023-11-29T08:38:45Z"
  labels:
    app: istio-ingressgateway
    install.operator.istio.io/owning-resource: unknown
    istio: ingressgateway
    istio.io/rev: default
    operator.istio.io/component: IngressGateways
    release: istio
  name: istio-ingressgateway
  namespace: istio-system
  resourceVersion: "560305"
  uid: 4ba0fce1-69ed-4ef6-ae52-07c8fc2d9b3d
spec:
  clusterIP: 10.98.123.79
  clusterIPs:
  - 10.98.123.79
  externalTrafficPolicy: Cluster
  ipFamilies:
  - IPv4
  ipFamilyPolicy: SingleStack
  ports:
	- name: status-port
    nodePort: 30824
    port: 15021
    protocol: TCP
    targetPort: 15021
  - name: http2
    nodePort: 31277
    port: 80
    protocol: TCP
    targetPort: 8080
  - name: https
    nodePort: 30779
    port: 443
    protocol: TCP
    targetPort: 8443
  - name: tcp
    nodePort: 31259
    port: 31400
    protocol: TCP
    targetPort: 31400
  - name: tls
    nodePort: 31998
    port: 15443
    protocol: TCP
    targetPort: 15443
  selector:
    app: istio-ingressgateway
    istio: ingressgateway
  sessionAffinity: None
  type: LoadBalancer               <------- LoadBalancer로 변경
  loadBalancerIP: 192.168.1.88     <------- 원하는 IP주소 입력
status:
  loadBalancer: {}
```

```bash
# 적용되었는지 확인
kubectl get svc/istio-ingressgateway -n istio-system
```

```bash
NAME                   TYPE           CLUSTER-IP     EXTERNAL-IP    PORT(S)                                                                      AGE
istio-ingressgateway   LoadBalancer   10.98.123.79   192.168.1.88   15021:30824/TCP,80:31277/TCP,443:30779/TCP,31400:31259/TCP,15443:31998/TCP   44h
```

> 만약 외부 IP 주소가 `<none>` 으로 출력된다면, 다른 서비스가 해당 IP 주소를 이미 사용 중일 수 있습니다.
{: .prompt-warning }

```bash
kubectl get svc -A
```

위 명령어로 모든 서비스를 출력한 뒤, 사용 가능한 IP 대역 중 아직 할당되지 않은 IP 주소로 변경해 저장합니다.

저장 후 `kubectl get svc/istio-ingressgateway -n istio-system` 으로 외부 IP 값이 지정한 주소인지 확인합니다.

마지막으로 웹 브라우저에서 `http://192.168.1.88` 로 접속이 되는지 확인합니다.

### MinIO Dashboard

위와 동일한 방법으로 진행합니다.

```bash
# 현재 서비스 상태 확인
kubectl get svc/minio-service -n kubeflow
```

```bash
NAME            TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)    AGE
minio-service   ClusterIP   10.99.88.190   <none>        9000/TCP   5h14m
```

Kubeflow 대시보드와 동일한 방식으로 수정하되, IP 주소는 다른 값으로 지정해야 합니다.

```bash
kubectl edit svc/minio-service -n kubeflow
```

```yaml
# Please edit the object below. Lines beginning with a '#' will be ignored,
# and an empty file will abort the edit. If an error occurs while saving this file will be
# reopened with the relevant failures.
#
apiVersion: v1
kind: Service
metadata:
  annotations:
    kubectl.kubernetes.io/last-applied-configuration: |
      {"apiVersion":"v1","kind":"Service","metadata":{"annotations":{},"labels":{"application-crd-id":"kubeflow-pipelines"},"name":"minio-service","namespace":"kubeflow"},"spec":{"ports":[{"name":"http","port":9000,"protocol":"TCP","targetPort":9000}],"selector":{"app":"minio","application-crd-id":"kubeflow-pipelines"}}}
  creationTimestamp: "2023-11-21T07:55:40Z"
  labels:
    application-crd-id: kubeflow-pipelines
  name: minio-service
  namespace: kubeflow
  resourceVersion: "554411"
  uid: 1c6fd99e-6ee5-4baf-935b-d69a5cf9634c
spec:
  clusterIP: 10.99.88.190
  clusterIPs:
  - 10.99.88.190
  externalTrafficPolicy: Cluster
  ipFamilies:
  - IPv4
  ipFamilyPolicy: SingleStack
  ports:
  - name: http
    nodePort: 31772
    port: 9000
    protocol: TCP
    targetPort: 9000
  selector:
    app: minio
    application-crd-id: kubeflow-pipelines
  sessionAffinity: None
  type: LoadBalancer               <------- LoadBalancer로 변경
  loadBalancerIP: 192.168.1.89     <------- 원하는 IP주소 입력
status:
  loadBalancer: {}
```

```bash
# 적용되었는지 확인
kubectl get svc/minio-service -n kubeflow
```

```bash
NAME            TYPE           CLUSTER-IP     EXTERNAL-IP    PORT(S)          AGE
minio-service   LoadBalancer   10.99.88.190   192.168.1.89   9000:31772/TCP   9d
```

웹 브라우저로 접속이 되는지 확인합니다. `http://192.168.1.89:9000/` ~~(minio, minio123)~~

### Grafana Dashboard

위 과정들과 동일합니다.

```yaml
# 현재 서비스 상태 확인
kubectl get svc/seldon-core-analytics-grafana -n seldon-system
```

```yaml
NAME                            TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)   AGE
seldon-core-analytics-grafana   ClusterIP   10.99.201.117   <none>        80/TCP    94s
```

위 과정들과 동일하게 수정합니다.(IP주소는 변경)

```yaml
kubectl edit svc/seldon-core-analytics-grafana -n seldon-system
```

```yaml
# Please edit the object below. Lines beginning with a '#' will be ignored,
# and an empty file will abort the edit. If an error occurs while saving this file will be
# reopened with the relevant failures.
#
apiVersion: v1
kind: Service
metadata:
  annotations:
    meta.helm.sh/release-name: seldon-core-analytics
    meta.helm.sh/release-namespace: seldon-system
  creationTimestamp: "2023-11-21T08:12:29Z"
  labels:
    app.kubernetes.io/instance: seldon-core-analytics
    app.kubernetes.io/managed-by: Helm
    app.kubernetes.io/name: grafana
    app.kubernetes.io/version: 7.0.3
    helm.sh/chart: grafana-5.1.4
  name: seldon-core-analytics-grafana
  namespace: seldon-system
  resourceVersion: "554665"
  uid: e3e52bd2-3cc6-4cd0-8566-76a22f120547
spec:
  clusterIP: 10.99.201.117
  clusterIPs:
  - 10.99.201.117
  externalTrafficPolicy: Cluster
  ipFamilies:
  - IPv4
  ipFamilyPolicy: SingleStack
  loadBalancerIP: 192.168.1.90
  ports:
  - name: service
    nodePort: 30315
    port: 80
    protocol: TCP
    targetPort: 3000
  selector:
    app.kubernetes.io/instance: seldon-core-analytics
    app.kubernetes.io/name: grafana
  sessionAffinity: None
  type: LoadBalancer
status:
  loadBalancer:
    ingress:
    - ip: 192.168.1.90
```

```yaml
kubectl get svc/seldon-core-analytics-grafana -n seldon-system
```

```bash
NAME                            TYPE           CLUSTER-IP      EXTERNAL-IP    PORT(S)        AGE
seldon-core-analytics-grafana   LoadBalancer   10.99.201.117   192.168.1.90   80:30315/TCP   9d
```

웹 브라우저로 접속이 되는지 확인합니다. `http://192.168.1.90/`  ~~(admin, password)~~
