---
title: "ROS Motor Controller 패키지 생성"
author: knowgyu
description: " "
date: 2025-01-28 10:00:36 +0900
math: true
categories: [Embedded System, ROS]
tags: [ROS, 자율주행, SLAM]
---

## 개요

이번 글에서는 기존에 제작한 모터 드라이버를 활용하여, ROS 환경에서 모터와 서보를 제어할 수 있는 **`robot_motor_controller`** 패키지를 만드는 과정을 다룹니다.

---

## 1. 패키지 생성 및 디렉터리 구조

먼저, 새로운 ROS 패키지를 생성합니다. 아래 명령어를 통해 **`robot_motor_controller`** 패키지를 생성하고, 기본 디렉터리 구조를 구성합니다.

```bash
cd ~/catkin_ws/src
catkin_create_pkg robot_motor_controller roscpp geometry_msgs

cd robot_motor_controller
mkdir -p include/robot_motor_controller config
```

패키지 생성 후 디렉터리 구조는 아래와 같이 구성됩니다.

> 모터 드라이버에 대한 내용은 추후 작성 후, 링크 첨부 예정입니다.
{: .prompt-info }

```bash
$ tree .
.
├── CMakeLists.txt
├── README.md
├── config
├── include
│   └── robot_motor_controller
│       ├── DC_motor.hpp
│       ├── PCA9685.hpp
│       ├── Servo_motor.hpp
│       ├── i2c.hpp
│       └── motor_controller.h
├── launch
├── package.xml
└── src
    ├── DC_motor.cpp
    ├── PCA9685.cpp
    ├── Servo_motor.cpp
    ├── i2c.cpp
    ├── motor_controller.cpp
    └── motor_controller_node.cpp
```

---

## 2. 헤더 파일 작성

`include/robot_motor_controller/motor_controller.h` 파일에서는 모터 제어에 필요한 각종 드라이버 헤더와 ROS 인터페이스를 포함합니다.  
여기서 **ROS Subscriber**를 이용해 `cmd_vel` 토픽을 구독하고, 이를 통해 모터의 throttle과 서보의 각도를 제어하는 인터페이스를 정의합니다.

```cpp
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "DC_motor.hpp"
#include "Servo_motor.hpp"
#include "PCA9685.hpp"
#include "i2c.hpp"

class MotorController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    
    // I2C 디바이스 (모터와 서보를 위한 각각의 I2C 인터페이스)
    I2CDevice i2c_dc_;
    I2CDevice i2c_servo_;
    
    // PCA9685 컨트롤러 (각 I2C 디바이스에 연결)
    PCA9685 pca_dc_;
    PCA9685 pca_servo_;
    
    // 모터 및 서보 제어 객체
    PWMThrottleHat motor_;
    Servo servo_;
    
    // ROS 파라미터 (예: 서보 채널 번호)
    int servo_channel_;

public:
    MotorController(ros::NodeHandle& nh);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

#endif // MOTOR_CONTROLLER_H
```

---

## 3. 소스 코드 작성

### 모터 컨트롤러 클래스 구현

`src/motor_controller.cpp` 파일에서는 생성자 내 초기화 작업과 `cmdVelCallback` 함수에서 받은 속도 명령을 모터 제어 명령으로 변환하는 과정을 다룹니다.  
여기서는 `geometry_msgs/Twist` 메시지의 `linear.x` 값을 throttle로, `angular.z` 값을 서보 각도로 변환합니다.

> 제작하는 차량의 경우 애커만 조향인 점을 고려해 `linear.x`와 `angular.z`만 사용합니다.
{: .prompt-info }

```cpp
#include "robot_motor_controller/motor_controller.h"
#include <algorithm>

MotorController::MotorController(ros::NodeHandle& nh) : 
    nh_(nh),
    i2c_dc_("/dev/i2c-7", 0x40),
    i2c_servo_("/dev/i2c-7", 0x60),
    pca_dc_(i2c_dc_),
    pca_servo_(i2c_servo_),
    motor_(pca_dc_, 0),
    servo_(pca_servo_)
{
    // ROS 파라미터 초기화 (필요 시 rosparam을 이용하여 동적 설정 가능)
    servo_channel_ = 0;  // 기본값 설정
    
    // cmd_vel 토픽 구독: 로봇의 이동 명령을 수신
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MotorController::cmdVelCallback, this);
    
    ROS_INFO("Motor Controller initialized");
}

void MotorController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // linear.x를 throttle로 변환 (-1.0 ~ 1.0 범위)
    float throttle = msg->linear.x;  // 필요한 경우 추가 스케일링 적용 가능
    
    // angular.z를 서보의 회전각으로 변환 (-45° ~ 45°)
    float angle = msg->angular.z * (45.0 / M_PI);  // rad/s -> degree 변환
    
    // 값의 범위를 제한
    throttle = std::max(-1.0f, std::min(1.0f, throttle));
    angle = std::max(-45.0f, std::min(45.0f, angle));
    
    // 모터 및 서보 제어 호출
    motor_.setThrottle(throttle);
    servo_.setAngle(servo_channel_, angle);
}
```

### 노드 파일 작성

`src/motor_controller_node.cpp` 파일에서는 ROS 노드를 초기화하고, 모터 컨트롤러 객체를 생성한 후 `ros::spin()`을 호출하여 콜백 함수가 계속 실행되도록 합니다.  

> ROS **노드(Node)**란 하나의 실행 단위로, 서로 독립적으로 동작하며 메시지를 주고받는 기본 단위입니다.  
> 비유하자면 개별 프로세스 느낌?
{: .prompt-tip }

```cpp
#include <ros/ros.h>
#include "robot_motor_controller/motor_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller_node");
    ros::NodeHandle nh;
    
    MotorController controller(nh);
    
    ros::spin();
    
    return 0;
}
```

---

## 4. 런치 파일 작성

ROS 패키지를 실행할 때 사용하는 런치(launch) 파일을 작성합니다.  
이 파일을 통해, 노드 실행 시 필요한 파라미터 설정과 실행 옵션을 쉽게 관리할 수 있습니다.

```xml
<launch>
    <node pkg="robot_motor_controller" type="motor_controller_node" name="motor_controller" output="screen">
    </node>
</launch>
```

---

## 5. CMakeLists.txt 수정 내용 설명

ROS 패키지를 빌드하기 위해 CMakeLists.txt 파일을 수정합니다.  
여기서 수정 사항은 아래와 같습니다.

### 라이브러리와 실행 파일 분리

- **라이브러리 추가:**  
  소스 파일들(`DC_motor.cpp`, `PCA9685.cpp`, `Servo_motor.cpp`, `i2c.cpp`)을 하나의 라이브러리로 묶어 다른 소스 파일에서 재사용할 수 있도록 합니다.
  
  ```cmake
  add_library(${PROJECT_NAME}
    src/DC_motor.cpp
    src/PCA9685.cpp
    src/Servo_motor.cpp
    src/i2c.cpp
  )
  ```

- **실행 파일 추가:**  
  노드 실행을 위한 `motor_controller_node.cpp`와 모터 컨트롤러 구현 파일(`motor_controller.cpp`)을 하나의 실행 파일로 빌드합니다.
  
  ```cmake
  add_executable(motor_controller_node 
    src/motor_controller_node.cpp
    src/motor_controller.cpp
  )
  ```

### 타겟 간 의존성 및 링크 설정

- 라이브러리와 실행 파일이 올바르게 연결될 수 있도록 `add_dependencies`와 `target_link_libraries`를 사용합니다.  
- 실행 파일은 먼저 라이브러리를 링크한 후, ROS 의존성을 해결하기 위해 `${catkin_LIBRARIES}`를 추가합니다.

  ```cmake
  add_dependencies(motor_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  
  target_link_libraries(motor_controller_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  ```

---

## 6. 빌드 및 실행

모든 파일 작성이 완료되면, catkin 워크스페이스 최상위 디렉터리에서 빌드를 진행합니다.

```bash
cd ~/catkin_ws
catkin_make
```

빌드가 완료된 후, 다음 명령어로 노드를 실행하여 모터 및 서보 제어가 정상 동작하는지 확인합니다.

```bash
roslaunch robot_motor_controller motor_controller.launch
```

---

## 마무리

이번 글에서는 모터 드라이버를 기반으로 한 ROS 패키지 **`robot_motor_controller`**를 제작하는 과정을 다뤘습니다.  

간단한 테스트 후, 정상 작동하는 것을 확인했다면 ROS Navigation Stack을 연동하여 실제로 주행해보겠습니다.  
