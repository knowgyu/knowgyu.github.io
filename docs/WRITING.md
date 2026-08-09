# Writing for knowgyu.github.io

## 목적

이 블로그는 공부 기록, 프로젝트 진행 기록, 그리고 작업에서 얻은 재사용 가능한 판단을 함께 보관한다. 카테고리는 **주제**를, 글 종류는 **읽는 목적**을 표현한다.

## 콘텐츠 모델

### Categories: 무엇에 관한 글인가

카테고리는 기술 주제만 표현하며 최대 두 단계다. 먼저 `_data/taxonomy.yml`에 있는 경로를 사용한다.

```yaml
categories: [AI & CV, PyTorch]
```

새 분류는 실제 글이 두세 편 이상 쌓여 묶음이 필요할 때만 `_data/taxonomy.yml`과 함께 추가한다. `Projects`, 작업 상태, 감정, `Etc`는 카테고리가 아니다.

### Post role: 왜 읽는 글인가

새 글은 아래 `kind` 값을 권장한다. Jekyll은 이 메타데이터를 보존하며, 현재 목록 화면은 기존 글과의 호환성을 위해 `kind`로 필터링하지 않는다.

| 역할 | `kind` 값 | 용도 |
| --- | --- | --- |
| 학습 노트 | `technical-note` | 개념, 실험, 재현 가능한 구현 |
| 프로젝트 기록 | `project-log` | 설계 결정, 문제 해결, 마일스톤 |
| 인사이트/회고 | `reflection` | 여러 작업에 적용할 판단, 암묵지 |

### Project와 series

```yaml
project: contextwhere
series: ros-navigation
tags: [SQLite, Agent Memory]
```

`project`는 같은 프로젝트의 기록을 연결하는 축이지 카테고리를 복제하는 수단이 아니다. 여러 프로젝트를 가로지르는 판단은 `project`를 비워 두고 `reflection`과 적절한 주제 카테고리를 사용한다. `series`는 순서가 있는 연재에만 쓴다.

## Projects

현재 `/projects/`는 `project:`가 있는 모든 글을 프로젝트별로 묶는다. 프로젝트 대표 요약 전용 `project-summary` 글은 아직 쓰지 않는다. 나중에 실제 대표 요약 글을 만들기로 결정한 뒤에만 별도 계약과 목록 필터를 추가한다. 그 전에는 기존 프로젝트 기록을 숨기거나 Projects를 비우지 않는다.

## 새 글 front matter

```yaml
---
title: "글의 질문이 드러나는 제목"
author: knowgyu
description: "독자가 얻을 답을 한 문장으로"
date: 2026-08-09 12:00:00 +0900
categories: [AI & CV, PyTorch]
tags: [PyTorch]
kind: technical-note
# project: contextwhere
# series: ros-navigation
---
```

`math: true`는 수식이 있을 때만 넣는다. 날짜·제목·설명·링크·수치·배포 상태는 공개 전에 다시 확인한다.

## 글 템플릿

- **학습 노트**: 문제/개념 → 최소 재현 또는 예제 → 헷갈렸던 지점 → 적용 조건
- **프로젝트 기록**: 상황 → 선택지와 제약 → 결정 → 구현/검증 결과 → 다음 연결
- **인사이트·회고**: 관찰 → 반복해서 나타난 이유 → 적용 가능한 판단 기준 → 예외 또는 한계

## 작성 전 점검

- 제목과 설명이 글의 실제 질문을 말하는가
- 공개 가능한 코드·스크린샷·경험만 포함했는가
- 카테고리가 주제이고 `project`가 프로젝트 연결인지 구분했는가
- 기존 글을 반복하지 않고 새 판단 또는 재현 가능한 정보를 남기는가

글을 새로 만들거나 크게 고치기 전에는 [DESIGN.md](../DESIGN.md)와 [AGENTS.md](../AGENTS.md)도 함께 읽는다.
