# 하드웨어 독립 통합 시뮬레이션

Jetson 통합 컨트롤러, 통신 브리지, 가상 STM32를 메모리 통신으로 연결한다.

```bash
python3 -m simulation.scenario_runner
```

기본 시나리오:

```text
정상 주행
→ 장애물 접근 감속
→ 장애물 일반정지
→ 수동 리셋
→ 재출발
→ Cliff 긴급정지
```

`FakeSTM32`는 다음 기능을 모의한다.

- WHEEL_COMMAND 좌·우 목표 RPM 수신 (기존 DRIVE_COMMAND도 호환)
- 차동 구동 목표속도 변환
- 모터 가감속
- 500ms 명령 타임아웃
- Cliff·E-Stop·모터 고장
- 정지 후 수동 리셋
- ROBOT_STATUS 반환

시뮬레이터는 실제 STM32 펌웨어를 대체하지 않는다. 실제 하드웨어 연결 전 통신과
상태 전이 오류를 조기에 발견하기 위한 개발 도구이다.
