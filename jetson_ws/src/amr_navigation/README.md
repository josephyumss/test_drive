# AMR Navigation

엔코더·IMU 융합 위치 추정, SLAM 지도 작성, Nav2 경로 계획 및 경유점 임무를 담당합니다.

## 토픽과 TF

- 입력: `/mcu/status`, `/imu/data`, `/scan`
- 휠 오도메트리: `/wheel/odometry`
- EKF 결과: `/odometry/filtered`
- TF 책임: EKF가 `odom -> base_link`, SLAM/AMCL이 `map -> odom`을 발행합니다.
- Nav2 속도 명령은 `/cmd_vel_raw`로 중계되어 기존 안전 노드 검사를 거칩니다.

## 설치

```bash
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-topic-tools

cd ~/guide-amr/jetson_ws
colcon build --symlink-install
source install/setup.bash
```

## 지도 작성

하드웨어 시스템과 센서 드라이버를 먼저 실행한 후 다음을 실행합니다.

```bash
ros2 launch amr_navigation mapping.launch.py
rviz2
```

주행해서 지도를 만든 뒤 저장합니다.

```bash
mkdir -p ~/guide-amr/maps
ros2 run nav2_map_server map_saver_cli -f ~/guide-amr/maps/site
```

## 저장한 지도에서 자율주행

```bash
ros2 launch amr_navigation navigation.launch.py \
  map:=$HOME/guide-amr/maps/site.yaml
```

RViz의 `2D Pose Estimate`로 초기 위치를 지정하고 `Nav2 Goal`로 목적지를 지정합니다.
경유점 파일은 `config/waypoints.yaml`의 좌표를 실제 지도에 맞게 수정한 뒤 사용합니다.

```bash
ros2 run amr_navigation waypoint_mission_node --ros-args \
  -p autostart:=true \
  -p waypoint_file:=$HOME/guide-amr/jetson_ws/src/amr_navigation/config/waypoints.yaml
```

실차에서는 `wheel_base_m`, 로봇 반경, 라이다·IMU 장착 위치, 센서 공분산을 반드시 실측하여 조정해야 합니다.
