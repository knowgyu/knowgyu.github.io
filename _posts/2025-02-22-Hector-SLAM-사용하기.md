---
title: "Hector SLAM 사용하기"
author: knowgyu
description: " "
date: 2025-01-22 19:39:09 +0900
math: true
categories: [Embedded System, ROS]
tags: [ROS, 자율주행, SLAM]
---

## Hector SLAM 개요

Hector SLAM은 **주로 2D LiDAR 센서**를 기반으로 동작하는 SLAM 기법으로, IMU나 휠 오도메트리 없이도 실시간으로 맵을 생성할 수 있는 장점이 있습니다. <br>
타 SLAM 기법(예: GMapping, Cartographer 등)은 보통 IMU나 휠 오도메트리 등 다양한 센서 데이터를 활용해 보다 복잡한 환경에서도 정밀한 맵 생성을 지원하지만, 이 프로젝트에서 만드는 차량은 급격한 회전이나 큰 변화가 없으며, IMU와 같이 odom을 생성할 수 있는 센서를 사용하지 않기 때문에 Hector SLAM을 사용합니다.  

Hector SLAM은 다음과 같이 동작합니다:

- **스캔 매칭:** 각 LiDAR 스캔을 기존 맵과 비교하여 로봇의 현재 위치를 추정합니다.
- **실시간 맵 업데이트:** 추정된 위치를 바탕으로 환경 지도를 업데이트하며, 다중 해상도 맵을 활용해 정밀도를 높입니다.
- **센서 독립성:** 오직 LiDAR 데이터만 사용하기 때문에 센서 간의 동기화나  캘리브레이션 과정이 줄어듭니다.

---

## ROS TF (Transform)

ROS의 TF(Transform) 패키지는 서로 다른 좌표계(프레임) 간의 변환 정보를 실시간으로 관리하고 제공하는 역할을 합니다.  
이를 통해 여러 센서나 모듈이 서로 다른 좌표계에서 데이터를 주고받을 때, 정확한 위치 관계를 유지할 수 있습니다.

### 주요 TF 프레임

Hector SLAM에서는 아래와 같은 좌표계가 사용됩니다.  

- **map**  
  - **설명:** 전역 좌표계로, 생성된 지도 전체를 나타냅니다.  
  - **용도:** SLAM 알고리즘에서 최종 생성되는 지도의 기준 좌표계로 사용됩니다.

- **odom (또는 nav)**  
  - **설명:** odom 좌표계로, 로봇의 짧은 시간 내 움직임을 나타냅니다.  
  - **용도:** 로봇의 이동 정보(예: 휠 odom)가 반영되며, 단기적인 위치 추정에 사용됩니다.  
  - Hector SLAM에서는 별도의 odom 센서가 없더라도, 기본적인 움직임 추정을 위해 사용될 수 있습니다.

- **base_link**  
  - **설명:** 로봇 본체의 중심 좌표계입니다.  
  - **용도:** 로봇의 주요 동작이나 센서가 부착되는 기준 좌표로, `base_link`를 기준으로 주변 센서 데이터가 변환됩니다.

- **base_footprint**  
  - **설명:** 로봇의 바닥면을 나타내는 좌표계로, 주로 로봇의 실제 접지면과 관련된 데이터를 다룰 때 사용됩니다.  
  - **용도:** `base_link`와 비슷하지만, 로봇의 2D 평면상의 위치에 초점을 맞춥니다.

- **laser_frame (또는 scan, 또는 lidar_frame)**  
  - **설명:** LiDAR 센서가 부착된 좌표계입니다.  
  - **용도:** LiDAR 스캔 데이터가 이 프레임을 기준으로 측정됩니다.

- **scanmatcher_frame**  
  - **설명:** Hector SLAM에서 스캔 매칭 시 사용하는 추가 프레임으로, 데이터 변환의 보조 역할을 합니다.  
  - **용도:** 주로 TF를 이용해 스캔 데이터와 지도 사이의 정밀한 변환을 수행할 때 활용됩니다.

### TF의 역할과 중요성

TF는 위에서 언급한 다양한 좌표계를 서로 연결하는 역할을 합니다.  
예를 들어, LiDAR 센서에서 수집된 데이터(`laser_frame`)를 로봇의 기준 좌표계(`base_link` 또는 `base_footprint`)로 변환하고, 이를 다시 전역 지도 좌표계(`map`)와 연결하는 작업이 필요합니다.  
이러한 변환이 **정확해야** SLAM이 올바르게 동작하며, RViz 등에서 **올바른 위치**에 로봇과 장애물을 표시할 수 있습니다.

올바른 TF 구성은 Hector SLAM 뿐 아니라, 전체 자율주행 시스템의 성능에 매우 중요한 영향을 미칩니다.

---

## Hector SLAM 설치 및 테스트

1. **Hector SLAM 레포 클론 및 빌드**

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
   cd ~/catkin_ws
   catkin_make
   ```

2. **기본 테스트 실행**

   ```bash
   roslaunch hector_slam tutorial.launch
   ```

### 런치 파일(`mapping_default.launch`) 주요 파라미터

이 Hector SLAM 패키지에서 제일 중요하다고 생각하는 런치 파일입니다.

추후 프로젝트에서도 이 런치파일을 조금씩 수정해 사용하기에, 간단하게 설명을 하겠습니다.  

```xml
<?xml version="1.0"?>

<launch>
  <arg name="tf_map_scanmatch_transform_frame_name" default="scanmatcher_frame"/>
  <arg name="base_frame" default="base_footprint"/>
  <arg name="odom_frame" default="nav"/>
  <arg name="pub_map_odom_transform" default="true"/>
  <arg name="scan_subscriber_queue_size" default="5"/>
  <arg name="scan_topic" default="scan"/>
  <arg name="map_size" default="2048"/>
  
  <node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">
    
    <!-- Frame names -->
    <param name="map_frame" value="map" />
    <param name="base_frame" value="$(arg base_frame)" />
    <param name="odom_frame" value="$(arg odom_frame)" />
    
    <!-- Tf use -->
    <param name="use_tf_scan_transformation" value="true"/>
    <param name="use_tf_pose_start_estimate" value="false"/>
    <param name="pub_map_odom_transform" value="$(arg pub_map_odom_transform)"/>
    
    <!-- Map size / start point -->
    <param name="map_resolution" value="0.050"/>
    <param name="map_size" value="$(arg map_size)"/>
    <param name="map_start_x" value="0.5"/>
    <param name="map_start_y" value="0.5" />
    <param name="map_multi_res_levels" value="2" />
    
    <!-- Map update parameters -->
    <param name="update_factor_free" value="0.4"/>
    <param name="update_factor_occupied" value="0.9" />    
    <param name="map_update_distance_thresh" value="0.4"/>
    <param name="map_update_angle_thresh" value="0.06" />
    <param name="laser_z_min_value" value = "-1.0" />
    <param name="laser_z_max_value" value = "1.0" />
    
    <!-- Advertising config --> 
    <param name="advertise_map_service" value="true"/>
    
    <param name="scan_subscriber_queue_size" value="$(arg scan_subscriber_queue_size)"/>
    <param name="scan_topic" value="$(arg scan_topic)"/>
    
    <!-- Debug parameters -->
    <!--
      <param name="output_timing" value="false"/>
      <param name="pub_drawings" value="true"/>
      <param name="pub_debug_output" value="true"/>
    -->
    <param name="tf_map_scanmatch_transform_frame_name" value="$(arg tf_map_scanmatch_transform_frame_name)" />
  </node>
    
  <!--<node pkg="tf" type="static_transform_publisher" name="map_nav_broadcaster" args="0 0 0 0 0 0 map nav 100"/>-->
</launch>
```

### 주요 인자 및 파라미터 설명

- **런치 파일 인자**
  - `tf_map_scanmatch_transform_frame_name`: 스캔 매칭을 위한 TF 변환 시 사용하는 프레임 이름 (기본값: `scanmatcher_frame`)
  - `base_frame`: 로봇의 기준 프레임, 일반적으로 로봇의 중심을 의미 (기본값: `base_footprint`)
  - `odom_frame`: odom 프레임, 로봇의 이동 정보를 나타냄 (기본값: `nav`)
  - `pub_map_odom_transform`: 맵과 odom 사이의 TF를 퍼블리시할지 여부 (기본값: `true`)
  - `scan_subscriber_queue_size`: LiDAR 스캔 데이터를 수신할 때의 큐 크기 (기본값: `5`)
  - `scan_topic`: LiDAR 스캔 데이터가 발행되는 토픽 이름 (기본값: `scan`)
  - `map_size`: 생성될 지도 크기 (셀 단위, 기본값: `2048`)

- **노드 내부 파라미터**
  - **Frame 설정**
    - `map_frame`: 생성된 지도의 기준 프레임 (값: `map`)
    - `base_frame`와 `odom_frame`: 위 인자에서 정의된 값이 적용됨
  - **TF 관련 설정**
    - `use_tf_scan_transformation`: TF를 통해 스캔 데이터를 변환할지 여부 (값: `true`)
    - `use_tf_pose_start_estimate`: 초기 자세 추정을 TF를 통해 할지 여부 (값: `false`)
    - `pub_map_odom_transform`: 맵과 odom 사이의 변환을 퍼블리시할지 여부
  - **맵 관련 설정**
    - `map_resolution`: 지도의 해상도 (한 셀당 크기, 기본값: `0.050`m)
    - `map_size`: 전체 지도 크기  
    - `map_start_x`, `map_start_y`: 지도 생성 시작 위치의 오프셋 (기본값: `0.5`)
    - `map_multi_res_levels`: 다중 해상도 맵의 레벨 수 (기본값: `2`)
  - **맵 업데이트 파라미터**
    - `update_factor_free`: 자유 공간 업데이트 가중치 (값: `0.4`)
    - `update_factor_occupied`: 장애물로 인식되는 공간 업데이트 가중치 (값: `0.9`)
    - `map_update_distance_thresh`: 지도 업데이트를 위한 거리 임계값 (값: `0.4`)
    - `map_update_angle_thresh`: 지도 업데이트를 위한 각도 임계값 (값: `0.06`)
    - `laser_z_min_value` 및 `laser_z_max_value`: LiDAR 스캔의 Z 축 범위 설정 (값: `-1.0` ~ `1.0`)
  - **기타**
    - `advertise_map_service`: 외부에서 지도 서비스를 요청할 수 있도록 퍼블리시 여부 (값: `true`)
    - `tf_map_scanmatch_transform_frame_name`: 스캔 매칭에 사용되는 TF 프레임 이름

> 이번 페이지에서는 단순히 튜토리얼 런치파일을 실행해 동작했지만, 프로젝트를 수행하며 매우 중요한 부분이기에 각 인자와 파라미터를 알아두는 것이 중요합니다.   
{: .prompt-tip }

---

## 마무리

![alt text](/assets/img/ros/ros11.png)

이렇게 해서 Hector SLAM을 빌드하고 직접 테스트까지 수행했습니다.  

급격한 환경 변화가 생길 경우, 현재 위치를 정상적으로 추정하지 못하여 이상하게 맵이 그려질 수 있습니다.  
이를 위해 간단한 플랫폼을 제작하거나, 혹은 로봇을 모두 완성한 후 수동으로 움직이며 테스트하는 것을 권장드립니다.  

