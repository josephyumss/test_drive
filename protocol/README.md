```markdown
# 통신 프로토콜

이 폴더는 Jetson과 STM32가 공통으로 사용하는 통신 규격과 상수, 시험 데이터를 관리한다.

## 목적

Jetson과 STM32 사이에서 다음 정보를 안전하게 교환한다.

- Heartbeat
- 목표 선속도 및 각속도
- 햅틱 명령
- 시스템 제어 명령
- 실제 모터 속도
- Cliff·ToF 및 E-Stop 상태
- 배터리와 모터 오류
- 안전 상태와 고장 원인

## 파일 구성

```text
protocol/
├── README.md
├── protocol_constants.py
├── protocol_constants.h
└── test_vectors.json
