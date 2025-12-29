# Robot Bringup Package

로봇 실행을 위한 통합 패키지 (설정 파일 + 런치 파일)

## 📁 폴더 구조

```
robot_bringup/
├── config/                                    # 설정 파일
│   ├── ekf.yaml                              # robot_localization EKF 설정
│   ├── md_4wd_controllers_no_odom_tf.yaml    # ★ EKF용 컨트롤러 (odom TF 비활성화)
│   ├── frbot_controllers_sim.yaml            # 시뮬레이션용 컨트롤러
│   ├── gz_ros_bridge.yaml                    # Gazebo 브릿지 설정
│   ├── imu_config.yaml                       # IMU 설정
│   ├── realsense_config.yaml                 # RealSense 카메라 설정
│   └── velodyne_config.yaml                  # Velodyne LiDAR 설정
│
├── launch/                                    # 런치 파일
│   ├── robot_bringup.launch.py               # ★ 모바일 베이스 하드웨어 런치
│   ├── arm_moveit_hardware.launch.py         # ★ Arm + Gripper + MoveIt 런치
│   └── mobile_manipulator_moveit_hardware.launch.py  # ★★ 통합 런치 (Arm+Gripper+Mobile+MoveIt)
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🚀 사용법

### 모바일 베이스만 실행 (EKF 포함)

```bash
# 기본 실행 (robot_localization EKF 포함)
ros2 launch robot_bringup robot_bringup.launch.py

# EKF 없이 실행 (diff_drive_controller가 odom TF 발행)
ros2 launch robot_bringup robot_bringup.launch.py use_ekf:=false
```

### Arm + Gripper + MoveIt 실행

```bash
ros2 launch robot_bringup arm_moveit_hardware.launch.py
```

### 통합 실행 (Arm + Gripper + Mobile Base + MoveIt)

```bash
# 전체 하드웨어 통합 실행
ros2 launch robot_bringup mobile_manipulator_moveit_hardware.launch.py

# 파라미터 지정
ros2 launch robot_bringup mobile_manipulator_moveit_hardware.launch.py \
    can_interface:=can0 \
    gripper_port:=/dev/ttyACM1 \
    port_front:=/dev/ttyUSB0 \
    port_rear:=/dev/ttyUSB1
```

### 파라미터 오버라이드

```bash
ros2 launch robot_bringup robot_bringup.launch.py \
    port:=/dev/ttyUSB1 \
    wheel_radius:=0.06 \
    wheel_separation:=0.35
```

### Livox LiDAR + RTAB-Map SLAM 실행

```bash
# Prerequisites: slam_ws와 ros2_ws 모두 소싱 필요
source ~/slam_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

# Livox MID360 + RTAB-Map SLAM 실행 (RViz 포함)
ros2 launch robot_bringup livox_rtabmap_slam.launch.py

# RViz 없이 실행
ros2 launch robot_bringup livox_rtabmap_slam.launch.py rviz:=false

# Localization 모드 (기존 맵 사용)
ros2 launch robot_bringup livox_rtabmap_slam.launch.py localization:=true

# Voxel size 조정
ros2 launch robot_bringup livox_rtabmap_slam.launch.py voxel_size:=0.2
```

**주의사항:**
- `slam_ws`가 소싱되지 않으면 Livox 드라이버만 실행됩니다
- LiDAR IP는 `192.168.1.121`로 설정되어 있습니다 ([livox_ros_driver2/config/MID360_config.json](../livox_ros_driver2/config/MID360_config.json:28))
- RTAB-Map은 2D occupancy grid map을 자동으로 생성합니다

---

## 🔄 TF 구조

### EKF 사용 시 (`use_ekf:=true`, 기본값)

```
map
 └── odom                    ← (SLAM/AMCL이 발행, 또는 static)
      └── base_footprint     ← robot_localization (EKF)가 발행 ★
           └── base_link     ← robot_state_publisher가 발행
                └── wheels, sensors...
```

**diff_drive_controller**: `enable_odom_tf: false` (TF 발행 안 함)  
**robot_localization**: `publish_tf: true` (odom→base_footprint TF 발행)

### EKF 미사용 시 (`use_ekf:=false`)

```
map
 └── odom
      └── base_footprint     ← diff_drive_controller가 발행 ★
           └── base_link
                └── wheels, sensors...
```

---

## 📊 토픽 흐름

```
/diff_drive_controller/cmd_vel_unstamped (Twist)
                ↓
        MD4WDHardware (4륜 모터 제어)
                ↓
/diff_drive_controller/odom (Odometry)
                ↓
        robot_localization (EKF)
                ↓
        ┌───────────────────┐
        │                   │
/odometry/filtered    odom→base_footprint TF
```

---

## ⚙️ 설정 파일 설명

### ekf.yaml

robot_localization EKF 노드 설정

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `frequency` | 30.0 | EKF 업데이트 주기 (Hz) |
| `two_d_mode` | true | 2D 로봇 모드 |
| `publish_tf` | true | odom→base_footprint TF 발행 |
| `odom0` | /diff_drive_controller/odom | 휠 오도메트리 입력 |
| `imu0` | /imu/data | IMU 데이터 입력 |

### md_4wd_controllers_no_odom_tf.yaml

EKF와 함께 사용할 때의 diff_drive_controller 설정

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `enable_odom_tf` | **false** | odom TF 비활성화 (EKF가 발행) |
| `wheel_separation` | 0.3 | 좌우 휠 간격 |
| `wheel_radius` | 0.05 | 휠 반지름 |

---

## ⚙️ 시스템 설정

### USB/TTY 포트 권한 설정

시리얼 포트(/dev/ttyUSB*, /dev/ttyACM*)를 사용하려면 `dialout` 그룹에 사용자를 추가해야 합니다.

```bash
# dialout 그룹에 사용자 추가
sudo usermod -a -G dialout $USER

# 로그아웃 후 재로그인 필요 (또는 재부팅)
# 그룹 확인
groups
# 출력에 'dialout'이 포함되어야 함
```

### 원격 접속 설정 (SSH)

다른 컴퓨터에서 Jetson으로 원격 접속하는 방법:

#### 1. SSH 서버 설치 및 활성화 (Jetson에서 실행)

```bash
# SSH 서버 설치 (이미 설치되어 있을 수 있음)
sudo apt update
sudo apt install openssh-server -y

# SSH 서비스 시작
sudo systemctl start ssh
sudo systemctl enable ssh

# SSH 상태 확인
sudo systemctl status ssh

# Jetson IP 주소 확인
ip addr show | grep inet
# 또는
hostname -I
```

#### 2. 원격 PC에서 접속

```bash
# 기본 SSH 접속
ssh frlab@<JETSON_IP_ADDRESS>
# 예: ssh frlab@192.168.1.100

# VSCode Remote SSH로 접속
# 1. VSCode에서 Remote-SSH 확장 설치
# 2. F1 > "Remote-SSH: Connect to Host"
# 3. ssh frlab@<JETSON_IP_ADDRESS> 입력
```

#### 3. SSH Key 기반 인증 (비밀번호 없이 접속)

```bash
# 원격 PC에서 SSH 키 생성
ssh-keygen -t rsa -b 4096

# 공개키를 Jetson으로 복사
ssh-copy-id frlab@<JETSON_IP_ADDRESS>

# 이후 비밀번호 없이 접속 가능
ssh frlab@<JETSON_IP_ADDRESS>
```

#### 4. 방화벽 설정 (필요시)

```bash
# UFW 방화벽에서 SSH 허용
sudo ufw allow ssh
sudo ufw enable
sudo ufw status
```

#### 5. ROS2 원격 통신 설정

같은 네트워크에서 ROS2 노드 간 통신을 위한 설정:

```bash
# .bashrc에 추가 (Jetson과 원격 PC 모두)
export ROS_DOMAIN_ID=0  # 같은 값으로 설정
export ROS_LOCALHOST_ONLY=0  # 네트워크 통신 허용

# 적용
source ~/.bashrc
```

원격 PC에서 Jetson의 ROS2 토픽 확인:
```bash
# 원격 PC에서
ros2 topic list
ros2 topic echo /livox/lidar
```

---

## 🐛 문제 해결

### TF 중복 에러

```
Transform from odom to base_footprint is already being published
```

**원인**: diff_drive_controller와 robot_localization 둘 다 TF를 발행

**해결**: 
- `use_ekf:=true`로 실행 (기본값) - 자동으로 diff_drive의 TF 비활성화
- 또는 수동으로 `enable_odom_tf: false` 설정

### odom 토픽 없음

```
[WARN] Waiting for odom0 data...
```

**원인**: diff_drive_controller가 아직 활성화되지 않음

**해결**: 컨트롤러 상태 확인
```bash
ros2 control list_controllers
```

---

## 📝 빌드

```bash
cd ~/frbot_ws
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

---

## 🔗 관련 패키지

- `robot_hardware`: MD Motor Hardware Interface
- `robot_description`: 로봇 URDF
- `robot_gazebo`: Gazebo 시뮬레이션
- `robot_slam`: SLAM (Cartographer)
- `robot_nav2`: Navigation2 설정
