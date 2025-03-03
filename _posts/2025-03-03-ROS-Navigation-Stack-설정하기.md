---
title: "ROS Navigation Stack 설정하기"
author: knowgyu
description: " "
date: 2025-02-03 10:30:37 +0900
math: true
categories: [Embedded System, ROS]
tags: [ROS, 자율주행, SLAM]
---

## 개요

이전 글에서 모터 드라이버를 ROS 패키지화 했습니다.  
이번 글에서는 **my_slam** 패키지에 네비게이션 스택 설정을 추가하여, SLAM으로 생성된 지도 위에서 경로를 계획하고 이동할 수 있도록 합니다.  

---

## 1. 현재 디렉터리 구조

먼저, **my_slam** 패키지의 디렉터리 구조입니다.  
여기에는 SLAM 관련 파일과 네비게이션 설정 파일이 추가될 예정입니다.

```bash
my_slam
├── CMakeLists.txt
├── include
│   └── my_slam
├── launch
│   ├── my_slam.launch
│   └── teleop_test.launch
├── package.xml
├── rviz
│   └── slam.rviz
└── src
```

---

## 2. 패키지 설치

네비게이션 스택을 사용하기 위해 필요한 ROS 패키지를 설치합니다.  

```bash
sudo apt-get install ros-noetic-move-base ros-noetic-navigation
```

---

## 3. 파라미터 파일 생성

네비게이션 스택의 동작을 제어하는 여러 파라미터 파일들을 생성합니다.  
아래 명령어를 통해 파라미터 파일을 저장할 디렉터리와 필요한 파일들을 생성합니다.

```bash
cd ~/catkin_ws/src/my_slam
mkdir -p config/move_base

# 파라미터 파일 생성
touch config/move_base/costmap_common_params.yaml
touch config/move_base/local_costmap_params.yaml
touch config/move_base/global_costmap_params.yaml
touch config/move_base/base_local_planner_params.yaml
```

---

## 4. 파라미터 파일 내용

### 4.1 costmap_common_params.yaml

모든 costmap이 공통으로 사용하는 기본 파라미터를 정의합니다.  
로봇의 물리적 크기, 센서 설정, 장애물 감지 및 안전 거리 등을 포함합니다.

```yaml
# 로봇의 물리적 특성
footprint: [[-0.15, -0.06], [-0.15, 0.06], [0.15, 0.06], [0.15, -0.06]]
robot_radius: 0.20

# 장애물 감지 및 회피 설정
obstacle_layer:
  observation_sources: laser_scan_sensor
  laser_scan_sensor:
    sensor_frame: laser_frame
    data_type: LaserScan
    topic: scan
    marking: true
    clearing: true
    inf_is_valid: true

# 장애물 팽창 설정
inflation_layer:
  inflation_radius: 0.50
  cost_scaling_factor: 2.0

# 속도 제한 설정
max_vel_x: 0.3
max_vel_theta: 0.8
min_vel_x: -0.3
min_vel_theta: -0.8
min_in_place_vel_theta: 0.4
```

### 4.2 local_costmap_params.yaml

로봇 주변의 단기 경로 계획을 위한 지역(costmap) 설정 파일입니다.  
실시간으로 업데이트되는 작은 영역을 대상으로 하며, 장애물 회피에 주로 활용됩니다.

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_footprint
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 3.0
  height: 3.0
  resolution: 0.05
  transform_tolerance: 0.5

  obstacle_layer:
    observation_persistence: 0.2
    mark_threshold: 0.3
    clearing_threshold: 0.7

  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

### 4.3 global_costmap_params.yaml

전체 환경에 대한 경로 계획을 위한 글로벌 costmap 설정 파일입니다.  
SLAM으로 생성된 전체 지도를 활용하여 출발지부터 목적지까지의 전반적인 경로를 계획합니다.

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_footprint
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: true
  transform_tolerance: 0.5
  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

### 4.4 base_local_planner_params.yaml

로봇의 실제 움직임을 제어하는 파라미터들을 설정합니다.  
최대 속도, 가속도, 목표 도달 허용 오차 등 로봇이 계획된 경로를 어떻게 따라갈지 결정합니다.

```yaml
TrajectoryPlannerROS:
  max_vel_x: 0.5
  min_vel_x: 0.2
  max_vel_theta: 0.8
  min_vel_theta: -0.8
  min_in_place_vel_theta: 0.4

  acc_lim_theta: 0.6
  acc_lim_x: 0.2
  acc_lim_y: 0.0

  holonomic_robot: false
  
  # 목표 도달 허용 오차
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.05

  # Forward Simulation Parameters
  sim_time: 2.0
  vx_samples: 10
  vtheta_samples: 20

  # Trajectory Scoring Parameters
  meter_scoring: true
  path_distance_bias: 0.8
  goal_distance_bias: 0.6
  occdist_scale: 0.02
  heading_lookahead: 0.325
```

> 추후 성능 고도화를 위해 파라미터 튜닝을 진행할 예정이기에, 우선 이렇게 설정하고 넘어갑니다.
{: .prompt-tip }

---

## 5. 런치 파일 작성

네비게이션 스택과 SLAM, 그리고 모터 컨트롤러 노드를 동시에 실행하기 위한 런치 파일을 작성합니다.

### 5.1 navigation.launch

`launch/navigation.launch` 파일은 SLAM, 모터 컨트롤러, Move Base, RViz를 모두 포함하여 전체 시스템을 실행합니다.

```xml
<?xml version="1.0"?>
<launch>
    <!-- SLAM 실행 -->
    <include file="$(find my_slam)/launch/my_slam.launch" />
    
    <!-- 모터 컨트롤러 실행 -->
    <include file="$(find robot_motor_controller)/launch/motor_controller.launch" />

    <!-- Move Base 노드 실행 -->
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <rosparam file="$(find my_slam)/config/move_base/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find my_slam)/config/move_base/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find my_slam)/config/move_base/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find my_slam)/config/move_base/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find my_slam)/config/move_base/base_local_planner_params.yaml" command="load" />
    </node>

    <!-- RViz 실행 -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_slam)/rviz/navigation.rviz"/>
</launch>
```

> RViz 파일은 코드 길이가 길어 생략하겠습니다.  
> 실행 후, 직접 Add로 토픽들을 추가한 후 저장하여 사용할 수 있습니다.
{: .prompt-tip }

### 5.2 my_slam.launch 수정

**my_slam.launch** 파일에서는 네비게이션과 SLAM 둘 다 RViz를 사용하기 때문에, SLAM 테스트를 위한 RViz 실행 부분은 주석 처리합니다.

```xml
...
	<node pkg="tf" type="static_transform_publisher" name="map_to_odom" args="0 0 0 0 0 0 map nav 100"/>
    <!-- RViz -->
    <!--<node pkg="rviz" type="rviz" name="rviz" 
          args="-d $(find my_slam)/rviz/slam.rviz" respawn="true"/>-->
...
```

---

## 6. 실행 결과

모든 설정이 완료되면, 다음 명령어로 전체 시스템을 실행합니다.

```bash
roslaunch my_slam navigation.launch
```

실행 결과로는 SLAM을 통한 지도 생성, 네비게이션 스택에 의한 경로 계획, 그리고 RViz를 통한 시각화가 동시에 이루어지며, 아래와 같은 결과 화면을 확인할 수 있습니다.

- **지도 생성 및 네비게이션 경로:**  
  SLAM으로 생성된 지도 위에 로봇의 경로 및 장애물 정보가 표시됩니다.  
![rviz](/assets/img/ros/rvizresult.png)


- **TF 프레임 및 rosgraph:**  
  로봇의 좌표계 정보와 노드 간의 연결 관계를 확인할 수 있습니다.  
![TF Tree](/assets/img/ros/TFtree.png)
![ros graph](/assets/img/ros/rosgraph.png)  

> 현재 TF Tree에 바퀴와 관련된 Frame들이 보이는데, 이는 URDF에 의한 것으로, 추후 제거 예정입니다.  
> TF tree가 다른 점은 추후 수정할 예정이며, 정상적으로 경로가 생성된다면 괜찮습니다.  
{: .prompt-warning }

---

## 마무리

이번 글에서는 기존의 모터 드라이버 기반 ROS 패키지와 SLAM 시스템에 네비게이션 스택을 통합하는 과정을 다뤘습니다.  

이후 글로벌 플래너와 로컬 플래너를 변경할 예정이며, 각종 파라미터 튜닝을 진행할 계획입니다.  

현재는 정상적으로 경로가 생성되는 지, 그리고 `rostopic echo cmd_vel`을 통해 정상적으로 명령이 출력되는 지 확인하면 됩니다.  
