# ROS2 Dummy 통합시험

## 데이터 흐름

```text
amr_dummy/dummy_sensor_node
  ├─ /scan, /imu/data, /wheel/odometry
  ├─ /obstacle/info, /cliff/state, /battery/state
  ├─ /cmd_vel_raw
  └─ /dummy/active=true, /hardware/motor_enable=false

amr_safety_node/safety_controller_node
  ├─ 기존 IntegratedController 호출
  ├─ /cmd_vel_safe
  └─ /safety/state

amr_dummy/dummy_mcu_node
  ├─ 기존 FakeSTM32 호출
  └─ /mcu/status

amr_monitoring_adapter
  └─ ROS2 상태를 http://127.0.0.1:8080/api/status로 전달
```

## 실행

```bash
python3 -m pip install -e .
source /opt/ros/<ROS_DISTRO>/setup.bash
cd jetson_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_bringup dummy_system.launch.py scenario:=automatic
```

관제 화면: `http://127.0.0.1:8080`

## 시나리오

- `normal`
- `obstacle_slow`
- `obstacle_stop`
- `cliff_stop`
- `communication_loss`
- `imu_timeout`
- `imu_drift`
- `wheel_slip`
- `motor_fault`
- `automatic`

## 안전 제한

- Dummy 실행 중 `/hardware/motor_enable`은 항상 `false`이다.
- Dummy와 실제 센서 드라이버를 같은 토픽 이름으로 동시에 실행하지 않는다.
- `hardware_system.launch.py`의 주행 허가는 기본적으로 비활성화되어 있다.
- 실제 모터 시험은 별도의 하드웨어 점검과 명시적인 활성화 절차를 추가한 후 수행한다.
