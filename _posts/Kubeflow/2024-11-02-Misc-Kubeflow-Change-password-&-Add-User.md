---
title: "Misc. Kubeflow Change password & Add User"
author: knowgyu
description: " "
date: 2023-11-26 05:46:09 +0900
math: true
categories: [MLOps, 2-Kubeflow-Pipeline-Setting]
tags: [MLOps, Kubeflow]
---

본 페이지에서는 Kubeflow 대시보드에 접속하기 위한 사용자의 비밀번호를 변경하고, 사용자를 추가하는 법에 대해 다루겠습니다.

# Change User Password

---

우선, Kubeflow 구축시 기본 ID와 PW는 아래와 같습니다.

- ID : `user@example.com`
- PW : `12341234`

Kubeflow에선 Hash를 사용해 PW를 configmap에 저장합니다.

우선, 사용할 비밀번호를 생성하겠습니다.

### Hash password

```bash
# passlib 라이브러리가 필요합니다.
# 만약 설치되어있지 않다면 pip install passlib을 통해 설치합니다.
python3 -c 'from passlib.hash import bcrypt; import getpass; print(bcrypt.using(rounds=12, ident="2y").hash(getpass.getpass()))'
```

- 예시

```bash
gyu@gyu:~$ python3 -c 'from passlib.hash import bcrypt; import getpass; print(bcrypt.using(rounds=12, ident="2y").hash(getpass.getpass()))'
Password: # 입력한 비밀번호는 표시되지 않습니다. 예시에서는 'Hi'를 입력했습니다.
$2y$12$7JDOZDuVSop3nsg4QduEoOPi46DJl2A8phUKEbbtawWmPhaE1x1CG
```

### Edit config-map

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

### Rebuild and Restart pod

config-map 변경 사항을 적용하겠습니다.

```bash
kustomize build common/dex/base | kubectl apply -f -
kubectl rollout restart deployment dex -n auth
```

# Add User

---

다른 사용자를 추가하기 위해선 두 가지 과정이 필요합니다.

- Dex에서 ID/PW 생성
- 자원을 사용하기 위한 네임스페이스 생성(혹은 profile 생성)

### Create ID/PW

우선, 사용자를 추가하겠습니다. 비밀번호 변경 방법과 유사합니다.

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
      userID: "knowgyu"
```

### Rebuild and Restart pod

config-map 변경 사항을 적용하겠습니다.

```yaml
kustomize build common/dex/base | kubectl apply -f -
kubectl rollout restart deployment dex -n auth
```

- 접속 확인

![Untitled.png](/assets/img/kubeflow/kubepipe402.png)

⚠️ 위 이미지에서 좌측 상단을 보면, 네임스페이스가 할당되지 않은 것을 확인할 수 없습니다.

네임스페이스가 없을 경우 자원을 생성할 수 없기에, 추가한 사용자를 위해 네임스페이스(Profile)을 생성하겠습니다.

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

> 이름과 사용할 리소스 등을 수정합니다.
> 


> resourceQuotaSpec 지정 후 네임스페이스 생성 시 pod 생성마다 resource 제한을 걸어줘야합니다.
>하지만, 사용자마다 자원을 제한할게아닌, 로컬 리소스를 모두 사용하려하기에 주석처리하였습니다.
>
>만약, resource를 지정하였는데, 이를 제거하고싶다면 `kubectl describe quota -n 유저네임` 명령어를 통해 `유저네임` 네임스페이스의 ResourceQuota 확인 후 `kubectl delete quota >kf-resource-quota -n 유저네임` 명령어를 통해 제거합니다.
{: .prompt-warning }

```bash
kubectl apply -f profile.yaml

# 정상적으로 실행 시 아래 메세지 출력됨.
profile.kubeflow.org/유저네임 created
```

### Refresh Dashboard

Kubeflow 대시보드를 새로고침(F5) 합니다.

![Untitled.png](/assets/img/kubeflow/kubepipe401.png)

정상적으로 네임스페이스가 할당된 것을 확인할 수 있습니다.

> 사용자 정보를 Config-map을 이용해 관리하게 된다면, 보안 문제가 발생할 수 있습니다.
이를 위해 DB로 저장하는 방식으로 변경하는 것을 고려해볼 수 있습니다.
{: .prompt-tip }
