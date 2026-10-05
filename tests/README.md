테스트

하드웨어 독립 핵심 로직을 Python 표준 라이브러리로 시험한다.

```bash
python3 -m unittest discover -s tests -v
```

필수 검증 대상:

- 표준 CRC 값
- 패킷 왕복 및 손상 거부
- 스트림 재동기화
- 거리와 TTC 중 더 위험한 판단 선택
- 안전 상태 우선순위
- 정지 후 자동 재출발 방지
- 상태별 속도 제한
- 통합 컨트롤러 명령 생성
- 메모리 전송 및 통신 브리지 timeout
- 가상 STM32 독립 안전정지
- 정상→감속→정지→리셋→Cliff 통합 시나리오
- ROS2 Dummy 시나리오·LiDAR·IMU 생성기
- 센서 timeout 및 ROS2 입력 변환
- ROS2 상태의 관제 API 변환
