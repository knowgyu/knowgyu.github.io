---
title: "YDLidar X4 설정 및 ROS 테스트"
author: knowgyu
description: " "
date: 2025-01-13 17:03:58 +0900
math: true
categories: [Embedded System, ROS]
tags: [ROS, 자율주행, SLAM]
---

## 개요

이번 글에서는 라이다 센서를 사용하기 위한 준비를 합니다.  

YDLidar X4의 경우 12cm ~ 10m 까지 감지가 가능하며, 자세한 내용은 아래 링크를 참고하시면 됩니다.  
[YDlidar X4 디바이스마트](https://www.devicemart.co.kr/goods/view?no=12170775&srsltid=AfmBOoqmNYC1eyAD3EFEgxfDrUDDTENT1vc7EU-PF8s6uXhjf4wEGZMZ)

X4의 경우 단종되었으며 X4 pro는 약 85,000원으로 비슷한 스펙의 RPlidar사 제품보다 저렴하다는 장점이 있습니다.  

> 저는 중고로 구매하여 약 5만원에 구입했습니다.  
{: .prompt-tip}

## YDLidar SDK 설치

YDLidar X4를 사용하기 위해 먼저 공식 SDK를 설치합니다.  

```bash
mkdir -p sdk_ws/src
cd sdk_ws/src
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
```

SDK 디렉터리로 이동한 후, 빌드 디렉터리를 생성하고 컴파일합니다.

```bash
cd YDLidar-SDK/
mkdir build
cd build
cmake ..
make
sudo make install
```

---

## YDLidar ROS Driver 설치

ROS 환경에서 YDLidar X4를 사용하기 위해 ROS 드라이버를 설치합니다. 기존의 `catkin_ws` 워크스페이스에서 아래의 과정을 진행합니다.

### 드라이버 패키지 클론

먼저, `catkin_ws/src` 디렉터리로 이동한 후 드라이버 패키지를 클론합니다.

```bash
cd ~/catkin_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
```

### 종속성 설치

워크스페이스 루트 디렉터리로 이동하여, 필요한 종속성들을 설치합니다.

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 빌드 및 환경 설정

드라이버 설치 후, `catkin_make`를 이용해 워크스페이스를 빌드하고, 환경 설정 파일을 적용합니다.

```bash
catkin_make
source devel/setup.bash
```

---

## 디바이스 및 udev 규칙 설정

LiDAR 장치와의 통신을 위해 `/dev/ttyUSB0` 권한을 설정합니다.

```bash
sudo chmod 666 /dev/ttyUSB0

# 지속적으로 권한 설정을 유지하려면 아래 설정을 추가합니다.
sudo usermod -aG dialout $USER
```

### udev 규칙 설정 (심볼릭 링크)

아래 명령어로 udev 규칙 파일을 생성하여, LiDAR 장치에 심볼릭 링크를 설정합니다.

```bash
sudo vi /etc/udev/rules.d/ydlidar.rules
```

파일에 아래 내용을 추가합니다.

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
```

> 이 심볼릭링크 설정을 통해 실제 디바이스는 `/dev/ttyUSB0`과 같은 형태로 표시되지만, 동시에 `/dev/ydlidar`로 사용할 수 있게 됩니다.
{: .prompt-info }

규칙 적용을 위해 udev 규칙을 새로고침합니다.

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## LiDAR 노드 실행

이제 ROS 환경에서 YDLidar X4 드라이버가 정상적으로 동작하는지 테스트합니다. 아래 명령어로 LiDAR 노드를 실행합니다.

```bash
roslaunch ydlidar_ros_driver ydlidar.launch
```

RViz 혹은 `rostopic list`와 `rostopic echo` 명령어를 통해 데이터를 정상적으로 입력받고 있는지 확인하면 됩니다.  

### 트러블 슈팅

만약 라이다센서의 스캔 데이터가 정상적으로 publish되지 않고 있다면 심볼릭 링크 혹은 권한문제, 혹은 케이블의 문제일 수 있습니다.

다시 한번 `ls /dev/ydlidar` 혹은 `ls /dev/ttyUSB*` 명령어를 통해 장치가 제대로 연결되었는지 확인합니다.  

이후, `sudo chmod 666 <장치명>` 을 통해 권한을 부여하고 다시 테스트하면 됩니다.  


만약 위 해결 방법을 수행했음에도 USB 어댑터 보드에 LED는 켜져있지만 데이터가 제대로 송수신되지 않는다면, 케이블 문제일 수 있습니다.  

충전용 케이블이 아닌, 데이터 송수신 케이블인지 다시 한번 확인 후 연결하면 정상적으로 작동할 것입니다.

> 저는 데이터 송수신 케이블을 사용하지 않아 제대로 동작하지 않았던 문제가 있었습니다.
{: .prompt-warning }

---

## 마무리

이렇게해서 LiDAR 센서가 잘 동작하는 것을 확인했다면, 프로젝트의 절반은 했다고 생각합니다.  (시작이 절반?)

다음 글부터는 LiDAR센서만을 이용해 자세추정까지 가능한 Hector SLAM을 설치하고 테스트해보겠습니다.  
