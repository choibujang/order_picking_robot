# 🤖 오더 피킹 로봇

주문부터 피킹까지 자동화된 로봇 물류 시스템

## ⚙️ Features
|기능|기능 상세|
|------|------------|
|주문 인식 기능|외부 시스템으로부터 주문 목록 수신|
|객체 탐지 및 3D 위치 추정 기능|피킹 대상 물품을 정확히 인식하고 3차원 위치를 산출|
|로봇 팔 경로 계획 및 피킹 기능|추정된 물체 위치를 기반으로 경로를 생성해 피킹 동작을 수행|

## 💠 Environments
### Hardware
|||
|------|------------|
|RaspberryPi 4B|<img width="100" height="100" alt="Image" src="https://github.com/user-attachments/assets/c138dfb2-b360-43a7-83da-691d4196fe9c" />|
|Orbbec Astra Stereo S U3 3D Depth Camera|<img width="100" height="100" alt="Image" src="https://github.com/user-attachments/assets/3028eb4f-3212-4d1c-a254-c4a41771ccb0" />|
|RC9 HV20Kg서보 * 6|<img width="100" height="100" alt="Image" src="https://github.com/user-attachments/assets/3038fdb7-300f-4b42-a6de-4473217a43ae" />
|PCA9685 PWM 서보 드라이버|<img width="100" height="100" alt="Image" src="https://github.com/user-attachments/assets/a51f8d4e-8652-4dd3-b44d-d07913169591" />|
### Software
- Ubuntu 20.04
- ROS2 Humble

## 🗺️ 시스템 구성도
<img width="1287" height="768" alt="Image" src="https://github.com/user-attachments/assets/4537b622-9864-42eb-97d6-caaedff21c87" />

## 🚀 Getting Started
### 🦾 Robot Arm
#### Orbbec Astra SDK 설치
https://github.com/orbbec/OrbbecSDK   
- 환경변수 설정:
```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:{path_to_orbbecSDK}
export LD_LIBRARY_PATH={path_to_OrbbecSDK}/lib/arm64:$LD_LIBRARY_PATH
```

#### I2C 라이브러리 설치
```bash
sudo apt-get install -y libi2c-dev
sudo usermod -aG i2c $USER
```

#### 프로젝트 다운로드 및 빌드
```bash
git clone https://github.com/choibujang/ros_vision_arm.git
cd robot_arm
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

#### 실행
```bash
ros2 run robot_arm_ros robot_arm_node
```

### 🧠 AI Server
- **운영체제**: Ubuntu 20.04
- **ROS 버전**: ROS2 Humble
#### 프로젝트 다운로드 및 빌드
```bash
git clone https://github.com/choibujang/ros_vision_arm.git
cd ai_server
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
#### 실행
```bash
cd cd src/ai_server_pkg/ai_server_pkg
python3 ./ai_server_node.py
```
