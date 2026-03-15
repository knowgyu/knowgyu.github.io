---
title: "[Error] Docker CLI Context 에러"
author: knowgyu
description: " "
date: 2023-09-05 13:49:13 +0900
math: true
categories: [Computer Science, Docker]
tags: [Docker, Trouble Shooting]
---

### 현상
Docker 설정 중 아래와 같이 CLI에서 에러 발생
```bash
Unable to resolve the current Docker CLI context "default": context "default" does not exist
```

### 원인
Docker CLI context "default"를 가져올 수 없어 발생하는 에러입니다.
"default"라는 이름의 Docker Context가 존재하지 않거나, 현재 작업 중인 context가 "default"로 설정되어 있지 않은 경우 발생합니다.

### 해결 방안
```bash
docker context ls # 현재 사용 가능한 컨텍스트 확인
docker context create default # "default" 없을 경우
docker context use default # 현재 작업중인 컨텍스트를 "default"로 설정
```

---
---
---
