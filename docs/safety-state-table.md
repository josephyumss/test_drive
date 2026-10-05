안전 상태표 초안

| 상태 | 진입 조건 | 속도 | 복귀 |
|---|---|---:|---|
| INIT | 자체 점검 중 | 0 | 점검 통과 시 READY |
| READY | 정상, 주행 허가 없음 | 0 | 별도 주행 허가 |
| RUN | 필수 점검 정상 | 설정 한도 | 위험 발생 시 전환 |
| SLOW | 장애물 주의, 센서 저하, 배터리 경고 | 감속 한도 | 조건 해소 시 자동 |
| CONTROLLED_STOP | 장애물 정지 구역, 정지 요청 | 0까지 감속 | 수동 리셋 |
| EMERGENCY_STOP | Cliff, E-Stop, 통신 단절, 손 놓침 | 0 | 위험 해제 후 수동 리셋 |
| FAULT | 모터 또는 필수 안전 센서 고장 | 0 | 점검 후 수동 리셋 |

우선순위:

```text
FAULT / 물리 E-Stop
> EMERGENCY_STOP
> CONTROLLED_STOP
> SLOW
> RUN
```

`CONTROLLED_STOP`, `EMERGENCY_STOP`, `FAULT`에서 RUN으로 직접 전환하지 않는다.
수동 리셋은 먼저 READY로만 복귀시킨다.


