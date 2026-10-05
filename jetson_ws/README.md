Jetson ROS2 workspace

ROS2 연결 코드가 들어 있는 colcon 작업공간이다. 루트의 하드웨어 독립 코드를 먼저
설치한 뒤 빌드한다.

```bash
python3 -m pip install -e .
source /opt/ros/<ROS_DISTRO>/setup.bash
cd jetson_ws
colcon build --symlink-install
source install/setup.bash
```

Dummy 전체 실행:

```bash
ros2 launch amr_bringup dummy_system.launch.py scenario:=automatic
```

Dummy 모드에서는 실제 모터 하드웨어 출력을 허용하지 않는다.
