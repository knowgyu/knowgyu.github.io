---
title: "ROS1 Hybrid A* Global Planner 사용하기"
author: knowgyu
description: " "
date: 2025-02-12 13:46:04 +0900
math: true
categories: [Embedded System, ROS]
tags: [ROS, 자율주행, SLAM]
---

## 개요

ROS1 네비게이션 스택에서는 Global Planner로 다양한 알고리즘(A\*, Dijkstra 등)을 제공합니다.  
하지만, 이들 대부분은 그리드 기반의 경로 탐색에 초점을 맞추고 있어 실제 차량의 동역학 제약을 충분히 반영하지 못합니다.  
  
Hybrid A\* 알고리즘을 활용하면, 기존의 A\*와 달리 차량의 회전 반경 및 비홀로노믹 제약을 고려한 경로 생성이 가능해집니다.  
이번 포스트에서는 ROS1 환경에서 Hybrid A\* 플러그인을 적용하여 글로벌 플래너로 사용하는 방법에 대해 다룹니다.  

> ROS1 move_base의 기본 Global Planner는 [https://wiki.ros.org/global_planner?distro=noetic](https://wiki.ros.org/global_planner?distro=noetic)를 참고바랍니다.
{: .prompt-tip}

---

## 1. Hybrid A* vs. A* 비교


- **A\*:** 단순한 경로 탐색, 차량의 동역학 무시  
- **Hybrid A\*:** 차량 제약을 반영한 경로 생성으로, 애커만 차량에 적합  

ROS2의 Nav2에서는 Hybrid A*를 기본 제공하지만, ROS1에서는 별도로 직접 구현하거나 외부 플러그인을 활용해야 합니다.  
이번 예제에서는 [https://github.com/dengpw/hybrid_astar_planner](https://github.com/dengpw/hybrid_astar_planner) 레포를 기반으로 플러그인을 설치하고 활용하는 방법을 다룹니다.

---

## 2. 플러그인 설치

### 2.1. 레포 클론 및 빌드

터미널에서 아래 명령어를 실행하여 레포를 클론하고 빌드 환경을 구성합니다.

```bash
sudo apt install libompl-dev \
&& cd ~/catkin_ws/src \
&& git clone https://github.com/dengpw/hybrid_astar_planner.git  \
&& cd .. \
&& catkin_make \
&& source devel/setup.bash \
&& rospack profile
```

 
> OMPL 관련 문제 발생 시, OMPL의 설치 경로(예: `/usr/lib` 또는 `/usr/local/lib`)를 확인하고 환경변수 `OMPL_DIR`를 올바르게 설정해야 합니다.    
> 예를 들어, OMPL이 `/usr/local/lib`에 설치된 경우 아래와 같이 환경변수를 추가할 수 있습니다.
> 
> ```bash
> export OMPL_DIR=/usr/local/lib
> ```
{: .prompt-warning}

### 2.2. 플러그인 테스트

아래 명령어를 실행하여 플러그인이 정상적으로 설치되었는지 확인합니다.

```bash
roslaunch hybrid_astar_planner test.launch
```

테스트가 성공적으로 실행되면, Hybrid A* 알고리즘을 사용하여 경로를 생성하는 것을 확인할 수 있습니다.

---

## 3. 기존 패키지에 플러그인 설정

### 3.1 런치 파일 내 글로벌 플래너 변경

기존의 글로벌 플래너 설정을 Hybrid A* 플러그인으로 변경합니다.  
런치 파일에서 다음과 같이 수정합니다.

```xml
<!-- 기존 설정 -->
<!-- <param name="base_global_planner" value="navfn/NavfnROS"/> -->

<!-- Hybrid A* 플러그인 설정 -->
<param name="base_global_planner" value="hybrid_astar_planner/HybridAStarPlanner"/>
```

### 3.2 package.xml 수정

패키지 의존성에도 해당 플러그인을 추가해줘야 합니다.  
아래와 같이 `package.xml` 파일에 의존성을 추가합니다.

```xml
...
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>tf</exec_depend>
  <depend>backoff_recovery</depend>
  <depend>hybrid_astar_planner</depend>
...
```

> 모든 의존성이 올바르게 설정되어 있지 않으면, 빌드 시 오류가 발생할 수 있습니다.
{: .prompt-warning}

![alt text](/assets/img/ros/astar.gif)
---


## 마무리

이번 글에서는 ROS1 환경에서 Hybrid A\*를 사용하기 위한 방법에 대해 다뤘습니다.  

이제, 기본적인 기능은 모두 추가했으니 다음부터는 파라미터 튜닝에 대해 알아보겠습니다.  
