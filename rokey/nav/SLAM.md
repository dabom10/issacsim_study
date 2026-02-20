
- turtlebot3.usd : 최종 usd 파일

- 맵 생성 시 회전, 정지 최소화 : 바퀴 흔들림, 슬립 등 때문에 Localization이 정확하게 안됨
- 2D Pose estimate에서는 로봇 위치, 보는 방향 최대한 정확하게 맞춰 설정하기# 걍 웬만하면 프로세스 다 kill 때리면 해결된다는 내용

(IsaacSim + TurtleBot3 + Nav2 환경 기준)

---

## 🔴 1. 주요 에러 유형

### 1) 패키지 미인식

```
Package 'turtlebot3_navigation2' not found
```

### 2) RViz 메시지 드롭

```
Message Filter dropping message:
frame 'base_scan'
reason 'discarding message because the queue is full'
```

### 3) FastDDS SHM 에러

```
Failed init_port fastrtps_portXXXX:
open_and_lock_file failed
RTPS_TRANSPORT_SHM Error
```

### 4) pkill 실패

```
pkill: PID XXXX번 강제로 끝내기 실패: 명령을 허용하지 않음
```

---

# 🧠 2. 원인 분석

## 📦 패키지 not found

- workspace build 안됨
- source 안함
- install 공간에 패키지 미존재

## 📡 RViz queue full

- LiDAR publish rate 과다
- Nav2 처리 주기보다 센서 입력이 빠름
- RViz Queue Size 기본값 부족

## 🔥 FastDDS SHM 충돌

- ROS 비정상 종료
- sudo로 ros 실행
- /dev/shm에 lock 파일 잔존
- IsaacSim + Nav2 동시 실행 시 DDS 포트 충돌

## 🛑 pkill 실패

- root 소유 프로세스
- sudo로 실행했던 이력 존재

---

# 🛠 3. 복구 절차 (운영 표준 프로세스)

## 1️⃣ ROS 프로세스 종료

```bash
pkill -9 ros2
pkill -9 rviz2
pkill -9 python3
```

## 2️⃣ 남은 PID 수동 종료

```bash
ps -aux | grep ros
sudo kill -9 <PID>
```

## 3️⃣ Shared Memory 초기화

```bash
sudo rm -rf /dev/shm/fastrtps*
sudo rm -rf /dev/shm/ros*
```

## 4️⃣ Workspace 재빌드

```bash
cd ~/IsaacSim-ros_workspaces/humble_ws
colcon build --symlink-install
source install/setup.bash
```

---

# ⚙️ 4. 안정화 세팅 가이드

## IsaacSim

- LiDAR Frequency: 10Hz

## Nav2

- controller_frequency: 10.0
- planner_frequency: 5~10
- use_sim_time: true

## RViz

- Queue Size: 50 이상

---

# 🚫 5. 절대 금지 사항

❌ sudo ros2 launch

❌ sudo python3 node

❌ sudo IsaacSim 실행

✔ apt install 시에만 sudo 사용

✔ ROS 실행은 일반 사용자 권한 유지

---

# 📌 6. 운영 체크리스트

- build → source → launch 순서 준수
- 비정상 종료 후 SHM 정리
- IsaacSim + Nav2 동시 구동 시 CPU 확인 (htop)
- 센서 publish rate ≤ controller 처리율 유지
- odom, tf, scan 토픽 정상 확인

---

# 🎯 결론

이 문제는 Nav2 알고리즘 문제가 아니라

DDS 통신 계층 및 실행 권한 관리 문제다.

환경을 깨끗하게 유지하는 것이

가장 중요한 안정화 전략이다.