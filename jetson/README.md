```markdown
# Jetson 상위 인지·판단 시스템

이 폴더는 Jetson Orin Nano에서 실행되는 ROS2 기반 상위 소프트웨어를 관리한다.

## 주요 역할

- LiDAR와 RGB-D 카메라 데이터 수신
- 2D LiDAR SLAM 및 위치 추정
- YOLO 기반 객체 인식
- Depth 기반 객체 거리 계산
- 장애물 거리와 TTC 계산
- 상위 안전 상태 판단
- 안전 속도 제한
- Jetson–STM32 통신
- 햅틱 및 TTS 이벤트 생성
- 상태 모니터링과 로그 저장

## 구조

```text
jetson/
├── README.md
└── src/
    ├── amr_interfaces/
    ├── amr_bringup/
    ├── amr_serial_bridge/
    ├── amr_safety_manager/
    ├── amr_perception/
    ├── amr_navigation/
    ├── amr_hri/
    └── amr_monitoring/
