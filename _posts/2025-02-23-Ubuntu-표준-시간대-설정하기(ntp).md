---
title: "Ubuntu 표준 시간대 설정하기(ntp)"
author: knowgyu
description: " "
date: 2025-01-23 11:43:05 +0900
math: true
categories: [Computer Science, Ubuntu]
tags: [Ubuntu, ntp]
---

## 개요

Ubuntu는 기본적으로 인터넷에 연결되어 있으면 타임 서버와 자동 동기화하여 시스템 시간을 설정합니다.  
하지만 네트워크 보안 정책 등으로 인해 타임 서버 동기화가 제대로 이루어지지 않을 경우, `ntp` 패키지를 사용하여 문제를 해결할 수 있습니다.  
이번 글에서는 `ntp` 패키지의 설치, 설정 파일 수정, 그리고 동기화 상태 확인 방법을 살펴봅니다.

---

## 1. ntp 패키지 설치

터미널에서 아래 명령어를 입력하여 `ntp` 패키지를 설치합니다.

```bash
sudo apt install ntp
```

설치가 완료되면, ntp 서비스가 자동으로 시작됩니다.

---

## 2. ntp 설정 파일 수정

네트워크 보안 등의 이유로 기본 타임 서버와의 동기화에 문제가 발생할 경우, 설정 파일을 수정하여 다른 타임 서버를 지정할 수 있습니다.

1. ntp 설정 파일을 편집합니다.

   ```bash
   sudo vi /etc/ntp.conf
   ```

2. 파일 내 약 20번째 줄 부근에 있는 기본 타임 서버 설정 부분은 아래와 같이 주석 처리되어 있습니다.

   ```bash
   #pool 0.ubuntu.pool.ntp.org iburst
   #pool 1.ubuntu.pool.ntp.org iburst
   #pool 2.ubuntu.pool.ntp.org iburst
   #pool 3.ubuntu.pool.ntp.org iburst
   ```

3. 위 4줄 대신 아래와 같이 새로운 타임 서버를 추가합니다.

   ```bash
   server 0.kr.pool.ntp.org iburst
   server time.bora.net iburst
   server time.windows.com iburst
   server time.google.com iburst
   ```


> `iburst` 옵션은 초기 동기화 속도를 높여주는 역할을 합니다.
{: .prompt-tip }

---

## 3. ntp 서비스 재시작

설정 파일 수정이 완료되면, ntp 서비스를 재시작하여 변경 사항을 적용합니다.

```bash
service ntp restart
```

> 명령어 실행 시 아래와 같이 **`AUTHENTICATING FOR org.freedesktop.systemd1.manage-units`** 메시지가 출력될 수 있습니다.  
> 이는 리눅스 시스템에서 서비스를 관리할 때 발생하는 정상적인 인증 요청이며, 사용자 비밀번호를 입력하면 정상적으로 동작합니다.
{: .prompt-warning }

---

## 4. 동기화 상태 확인

설정이 완료된 후, 아래 명령어로 타임 서버와의 동기화 상태를 확인할 수 있습니다.

```bash
ntpq -p
```

명령어를 실행하면 다음과 같은 출력이 나타납니다:

```bash
hero@hero-desktop:~$ ntpq -p
     remote           refid      st t when poll reach   delay   offset  jitter
==============================================================================
 ntp.ubuntu.com  .POOL.          16 p    -   64    0    0.000    0.000   0.000
 121.174.142.82  .INIT.          16 u    -   64    0    0.000    0.000   0.000
*time.bora.net   90.1.14.51       2 u    9   64    1   93.057  -19.483  16.766
 52.231.114.183  .STEP.          16 u  477   64    0    0.000    0.000   0.000
 time4.google.co .STEP.          16 u    -   64    0    0.000    0.000   0.000
```

출력 결과에서 특수문자는 다음과 같은 의미를 갖습니다:

| 기호   | 의미                                          |
| ------ | --------------------------------------------- |
| `*`    | 현재 동기화 중인 서버                         |
| `+`    | 접속은 가능하지만 아직 동기화 중이 아님       |
| `-`    | 접속은 가능하나 동기화 리스트에서 제외된 서버 |
| (없음) | 접속 실패                                     |

동기화 중인 서버 옆에 `*` 기호가 표시되면, 해당 서버와 정상적으로 시간 동기화가 이루어지고 있음을 의미합니다.
