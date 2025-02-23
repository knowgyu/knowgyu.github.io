---
title: "PID Controller"
author: knowgyu
description: " "
date: 2022-10-25 07:24:20 +0900
math: true
categories: [Embedded System, Control Engineering]
tags: [Embedded System, Control]
---

# ⭐ 고전제어 PID Control

> **PID 제어는 매우 심플하지만 강력함!**  
> 새로운 제어기와의 성능을 비교할 때 기준이 되는 제어기!
{: .highlight}

### PID는 플랜트 모델을 정확히 모르더라도 P, I, D gain값을 휴리스틱하게 변경하여 제어할 수 있음!

> **보통 Ziegler-Nichols Method**를 통해 값을 정함.
{: .prompt-info }
---

## 1. Concept of PID, Heuristic Tuning
![alt text](/assets/img/control/pid1.png)
> **제어는** 시스템의 상태를 원하는 목표치로 도달하게끔 만드는 과정.

- **$x$**: 시스템의 상태  
- **$x_d$**: 원하는 목표치  
- **Plant**: 우리가 관심 있는 시스템 (드론, 로봇팔 등)

<br><br>

### **PID 제어의 개념**

---

1. **비례 제어 (Proportional Gain)**  
   에러값에 비례해서 제어 값을 조정.

   → 초반 에러가 클 경우 **빠른 시간 내에 에러를 줄이는 역할!!**

   > 하지만, 에러가 줄더라도 에러에 비례해 제어하므로 **오버슈팅** 및 **잔류 에러** 발생 가능.
   {: .highlight }


2. **적분 제어 (Integral Gain)**  
   비례 제어의 단점을 보완.  
   에러의 적분값에 비례하여 시스템에 입력을 주어 **잔류 에러를 줄임**.

   > 하지만, 갑작스러운 변화 혹은 에러가 누적되면 적분 값이 커져 시스템이 **발산**할 위험이 있음.  
   → **Anti-windup 기법 사용**을 고려해야 함.
   {: .highlight }


3. **미분 제어 (Derivative Gain)**  
   변화량에 비례한 입력을 줌으로써 안정성을 높임.

   > 미분 값은 미래의 값을 알 수 없어 물리적으로 구현할 수 없음.  
   대신 **현재 에러와 과거 에러**를 활용하여 미분 값을 계산.
   {: .highlight }

   
짧은 샘플링 시간이나 갑작스러운 변화로 인해 튀는 현상을 방지하기 위해 **Low-pass Filter**를 사용.
{: .prompt-tip }
![alt text](/assets/img/control/pid2.png)

### Ziegler-Nichols Method를 통한 PID 튜닝

---

### 모델 정보가 없을 때 사용할 수 있는 방법!  
- $$K_p, T_i, T_d$$ 값을 계산할 수 있음.
 
![alt text](/assets/img/control/pid3.png)

만약 모터를 제어한다고 했을 때, 모터의 엔코더 값이 위의 그림과 같다면
**P Gain** ($K_{cr}$): 진동이 발생하기 시작하는 P gain.  
**진동 주기** ($P_{cr}$): 진동 주기의 시간.

**아래의 표를 통해 PID 값 튜닝 가능**:  
![alt text](/assets/img/control/pid4.png)

> 하지만, Ziegler-Nichols Method는 항상 잘 동작하는 것은 아님.<br>
> 성능과 외란 안정성을 고려해야 함.
{: .prompt-warning }

---

## ⭐모델 기반 PID 튜닝

### 고전 제어 vs 현대 제어

---

**칼만 필터 이후 두 제어 방식이 나뉘게 됨.**

> **칼만 필터란?**  
> 시스템 모델과 측정치를 결합하여 실제 상태를 추정하는 필터.
{: .prompt-info }

#### 고전 제어
- 주파수 관점에서 **입출력 전달함수**를 기반으로 설계.
- **SISO (Single Input Single Output)** 시스템에 적합.

#### 현대 제어
- 시간 관점에서 **상태 방정식**을 기반으로 설계.
- **MIMO (Multi Input Multi Output)** 시스템 설계에 적합.


### 플랜트 모델 찾기 (System Identification)
---
PID 제어를 설계하기 전, 다루고자 하는 플랜트를 정의해야 합니다.  
아래는 모터의 간단한 방정식 예제입니다:

#### 모터 방정식

$$
J\ddot{\theta}(t) + B\dot{\theta}(t) = T(t)
$$

- **$J$**: 관성 모멘트 (Moment of Inertia)  
- **$B$**: 점성 마찰 계수 (Viscous Friction)

라플라스 변환을 통해 주파수 영역으로 변환:
\$$
J \theta(s) s^2 + B \theta(s) s = T(s)
$$
![alt text](/assets/img/control/pid5.png)
![alt text](/assets/img/control/pid6.png)

> 라플라스 식에서 전달함수를 구하면 P(s)를 위 이미지처럼 바꿀 수 있습니다.
{: .prompt-tip }

#### ex) J = 0.02 , b = 0.1 로 설정

Input 은 desired trajectory인 위치 명령, Output은 모터가 움직인 각도

시스템의 특성을 파악하기 좋은 방법으로

**1) 루트 로커스(root locus) 와 2) 나이퀴스트(Nyquist)** 

root loucs는 matlab이나 python의 python control 모듈에서 쉽게 확인 가능.

`pip install control` 을 통해 쉽게 설치!


## 4. Pole과 Zero의 특성

---

Open loop 전달함수를 인수분해하여 Pole과 Zero를 구함:
$$
L(s) = C(s) P(s) = K \frac{(s + z_1)(s + z_2) \cdots (s + z_m)}{(s + p_1)(s + p_2) \cdots (s + p_n)}
$$

### 설명:
- $$ L(s) $$: Open loop 전달함수
- $$ C(s) $$: 제어기 전달함수
- $$ P(s) $$: 플랜트 전달함수
- $$ K $$: Gain
- $$ z_1, z_2, \ldots, z_m $$: 시스템의 Zero
- $$ p_1, p_2, \ldots, p_n $$: 시스템의 Pole

> **Pole**: 전달함수의 분모를 0으로 만드는 근.  
> **Zero**: 전달함수의 분자를 0으로 만드는 근.

> 시스템의 안정성은 **Pole의 위치**로 판단.  
> Pole이 s-plane의 왼쪽에 위치할 때 안정적.
{: .highlight}

### 제어기 설계 시 포인트

1. LHP (Left Half Plane)에 위치한 Pole은 안정성을 나타냄.
2. 허수부가 클수록 시스템의 진동이 커짐.
3. I Gain은 **루트 궤적을 오른쪽으로**, D Gain은 **왼쪽으로** 당기는 역할.
