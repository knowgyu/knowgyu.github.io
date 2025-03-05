---
title: "ROS Recovery Behavior 설정(Backoff 추가)"
author: knowgyu
description: " "
date: 2025-02-05 12:04:57 +0900
math: true
categories: [Embedded System, ROS]
tags: [ROS, 자율주행, SLAM]
---

## 개요

ROS1 네비게이션 스택은 기본적으로 제자리 회전과 코스트맵 초기화를 recovery behavior로 제공합니다.  
하지만 애커만 조향 차량은 제자리 회전이 불가능하므로, 후진과 같은 다른 방안이 필요합니다.  
  
    
이번 글에서는 [https://github.com/maker-ATOM/backoff_recovery](https://github.com/maker-ATOM/backoff_recovery/) 레포를 활용하여 후진 리커버리 동작을 추가하는 방법을 다룹니다.  
이를 통해 ROS1 환경에서도 애커만 차량에 적합한 리커버리 동작을 구현할 수 있습니다.

---

## 1. Repo Clone

먼저, ROS 워크스페이스 내에 backoff_recovery 레포를 클론합니다.

```bash
cd ~/catkin_ws/src
git clone https://github.com/maker-ATOM/backoff_recovery/
```

---

## 2. 네임스페이스 설정

backoff_recovery 패키지의 클래스 초기화 시, recovery 파라미터가 올바르게 반영되도록 네임스페이스를 설정해주어야 합니다.  
아래와 같이 코드를 수정합니다.

```cpp
namespace backoff_recovery
{
    BackoffRecovery::BackoffRecovery() : initialized_(false)
    {
    }

    void BackoffRecovery::initialize(std::string name, tf2_ros::Buffer *,
                                     costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap)
    {
        if (!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);  // 여기 수정

            private_nh.param("backoff_distance", backoff_distance_, 0.2);
            private_nh.param("frequency", frequency_, 20.0);
            private_nh.param("vel", vel_, 0.1);
```

> 저 부분을 수정하지 않으면, recovery 파라미터 파일(recovery_params.yaml)에서 후진 속도나 후진 거리를 조정해도 반영되지 않습니다.
{: .prompt-tip}

---

## 3. 설치 확인

아래 명령어를 실행하여 backoff_recovery 플러그인이 정상적으로 설치되었는지 확인합니다.

```bash
rospack plugins --attrib=plugin nav_core
```

출력에 아래와 같이 backoff_recovery 관련 항목이 표시되면 성공입니다.

```yaml
backoff_recovery /<path_to_ws>/src/backoff_recovery/backoff_plugin.xml
```

---

## 4. 리커버리 파라미터 설정

네비게이션 리커버리 동작에 사용할 파라미터 파일(recovery_params.yaml)을 작성합니다.  
여기서는 기본 코스트맵 초기화와 두 단계의 후진 동작을 정의합니다.

```yaml
recovery_behaviors:
  - name: 'conservative_reset'    
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
  
  - name: 'backoff_slow'           # 첫 번째 후진 시도 (천천히)
    type: 'backoff_recovery/BackoffRecovery'
  
  - name: 'aggressive_reset'      
    type: 'clear_costmap_recovery/ClearCostmapRecovery'

  - name: 'backoff_fast'           # 두 번째 후진 시도 (빠르게)
    type: 'backoff_recovery/BackoffRecovery'

# recovery 세부 설정
conservative_reset:
  reset_distance: 3.0
  layer_names: ["laser_scan"]

backoff_slow:                      # 첫 번째 후진 파라미터
  backoff_distance: 0.2    
  vel: 0.3

backoff_fast:                      # 두 번째 후진 파라미터
  backoff_distance: 0.5    
  vel: 0.3

aggressive_reset:
  reset_distance: 0.0            
  layer_names: ["laser_scan"]
```

> 위 파라미터들 또한 추후 튜닝할 예정입니다.
{: .prompt-tip}

---

## 마무리

이번 글에서는 ROS1 네비게이션 스택에 기본적으로 제공되는 recovery behavior에 추가로 후진 리커버리 동작을 추가했습니다.  

이와 같이 ROS1에서 기본 제공되지 않지만, 필요한 기능들을 Github에서 찾아보거나 직접 플러그인을 만들어 사용할 수 있습니다.
