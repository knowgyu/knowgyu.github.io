---
title: "Misc. Kubeflow Change password & Add User"
author: knowgyu
description: " "
date: 2023-11-26 05:46:09 +0900
math: true
categories: [AI & CV, Kubeflow]
tags: [MLOps, Kubeflow]
---

이 글에서는 Kubeflow 대시보드에 접속하기 위한 사용자 비밀번호를 변경하고, 새 사용자를 추가하는 방법을 정리합니다.

# Change User Password

---

Kubeflow를 처음 구축했을 때 기본 ID와 비밀번호는 아래와 같습니다.

- ID : `user@example.com`
- 비밀번호 : `12341234`

Kubeflow에서는 해시 값을 사용해 비밀번호를 ConfigMap에 저장합니다.

먼저 사용할 비밀번호 해시를 생성합니다.

### Hash password

```bash
# `passlib` 라이브러리가 필요합니다.
# 설치되어 있지 않다면 `pip install passlib` 로 먼저 설치합니다.
python3 -c 'from passlib.hash import bcrypt; import getpass; print(bcrypt.using(rounds=12, ident="2y").hash(getpass.getpass()))'
```

- 예시

```bash
gyu@gyu:~$ python3 -c 'from passlib.hash import bcrypt; import getpass; print(bcrypt.using(rounds=12, ident="2y").hash(getpass.getpass()))'
Password: # 입력한 비밀번호는 표시되지 않습니다. 예시에서는 'Hi'를 입력했습니다.
$2y$12$7JDOZDuVSop3nsg4QduEoOPi46DJl2A8phUKEbbtawWmPhaE1x1CG
```

### ConfigMap 수정

`manifests/common/dex/base/config-map.yaml` 수정

```bash
...
    enablePasswordDB: true
    staticPasswords:
    - email: user@example.com
			# 아래 hash 값을 변경합니다.
      hash: $2y$12$7JDOZDuVSop3nsg4QduEoOPi46DJl2A8phUKEbbtawWmPhaE1x1CG
      # https://github.com/dexidp/dex/pull/1601/commits
      # FIXME: Use hashFromEnv instead
      username: user
      userID: "15841185641784"
```

### Rebuild and Restart Pod

ConfigMap 변경 사항을 적용합니다.

```bash
kustomize build common/dex/base | kubectl apply -f -
kubectl rollout restart deployment dex -n auth
```

# Add User

---

다른 사용자를 추가하려면 두 가지 과정이 필요합니다.

- Dex에서 ID/비밀번호 생성
- 자원을 사용할 네임스페이스 생성(또는 Profile 생성)

### Create ID/비밀번호

먼저 사용자를 추가합니다. 절차는 비밀번호 변경과 거의 동일합니다.

`manifests/common/dex/base/config-map.yaml` 수정

```yaml
...
    enablePasswordDB: true
    staticPasswords:
    - email: user@example.com
      hash: $2y$12$7JDOZDuVSop3nsg4QduEoOPi46DJl2A8phUKEbbtawWmPhaE1x1CG
      # https://github.com/dexidp/dex/pull/1601/commits
      # FIXME: Use hashFromEnv instead
      username: user
      userID: "15841185641784"
      # 이 아래 부분 추가
    - email: insert_your_email.com
      hash: $2y$12$7JDOZDuVSop3nsg4QduEoOPi46DJl2A8phUKEbbtawWmPhaE1x1CG
      username: gyu
      userID: "demo-user"
```

### Rebuild and Restart Pod

ConfigMap 변경 사항을 적용합니다.

```yaml
kustomize build common/dex/base | kubectl apply -f -
kubectl rollout restart deployment dex -n auth
```

- 접속 확인

![Untitled.png](/assets/img/kubeflow/kubepipe402.png)

⚠️ 위 이미지의 좌측 상단을 보면, 아직 네임스페이스가 할당되지 않은 상태임을 확인할 수 있습니다.

네임스페이스가 없으면 자원을 생성할 수 없으므로, 추가한 사용자를 위해 네임스페이스(Profile)를 생성합니다.

### Create Profile

`profile.yaml` 작성 (`manifests/apps/profiles/upstream/samples` 참고)

```yaml
apiVersion: kubeflow.org/v1beta1
kind: Profile
metadata:
  name: insert_username
spec:
  owner:
    kind: User
    name: insert_email_here.com
#  resourceQuotaSpec:
#    hard:
#      cpu: "8"
#      memory: 8Gi
```

> 이름과 사용할 리소스 값은 환경에 맞게 수정합니다.


> `resourceQuotaSpec` 을 지정한 뒤 네임스페이스를 생성하면 Pod 생성마다 리소스 제한이 적용됩니다.
> 하지만 여기서는 사용자별 자원 제한보다 로컬 리소스를 모두 활용하는 것이 목적이어서 주석 처리했습니다.
>
> 만약 리소스를 지정한 뒤 제거하고 싶다면 `kubectl describe quota -n 유저네임` 으로 `유저네임` 네임스페이스의 `ResourceQuota` 를 확인한 뒤, `kubectl delete quota kf-resource-quota -n 유저네임` 으로 삭제하면 됩니다.
{: .prompt-warning }

```bash
kubectl apply -f profile.yaml

# 정상적으로 실행되면 아래 메시지가 출력됩니다.
profile.kubeflow.org/유저네임 created
```

### Refresh Dashboard

Kubeflow 대시보드를 새로고침(F5) 합니다.

![Untitled.png](/assets/img/kubeflow/kubepipe401.png)

정상적으로 네임스페이스가 할당된 것을 확인할 수 있습니다.

> 사용자 정보를 ConfigMap으로 관리하면 보안상 불리할 수 있습니다.
> 운영 환경이라면 별도 DB 기반 인증 저장소로 바꾸는 것도 고려할 만합니다.
{: .prompt-tip }
