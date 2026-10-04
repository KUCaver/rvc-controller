# 02. 입력·출력 계약과 결정 기록

갱신: 2026-10-03. 아래는 현재 구현 계약이며 실행 검증의 성공 선언이 아니다. 세부 전이는 [06](06_implemented_design.md), 함수 원형은 [05](05_function_specifications.md)를 따른다.

## 요구와 설계 선택의 구분

| 구분 | 내용 |
|---|---|
| PDF 255p | 완전한 Structured Charts와 DFD를 바탕으로 C 구현. 장치 라이브러리 모의 구현 허용 |
| 교수 피드백 | FSM 명령, 빠진 DFD 계층, 초기화 출력 경로, Controller 2.1.1 상세화의 명시 |
| 공유 PPT 기준 | STOP_OFF, 오른쪽 우선, 회전 5 Tick·후진 3 Tick, 미감지 시 일반 청소 복귀 |
| 일관성 보완 | Controller만 타이머 관리. OFF 후 회피 명령. 초기화 실패와 종료의 상태 계약 |
| 도구 선택 | 제품 C17, GoogleTest 테스트 실행기 C++17. GoogleTest가 수업 지정 도구라는 의미는 아님 |

미감지 후 추가 5 Tick 강화 유지라는 이전 제안은 채택하지 않았다. 강의의 “for a while”와 공유 PPT의 감지 기반 정책 해석이 완전히 동일하다고 보증하지 않는다. 강의 고정값과 팀 선택값을 구별하여 이후 피드백에 따라 한 곳에서 변경한다.

## 모듈 입출력

| 모듈 | 입력 | 반환·효과 |
|---|---|---|
| Front Sensor | 초기 읽기 / 전방 Boolean 보고 | 인스턴스별 전방 캐시. Tick마다 하드웨어 재조회하지 않음 |
| Left/Right/Dust Sensor | 샘플 요청 | Boolean 캐시·유효 여부. False는 정상 미감지 |
| Obstacle Detector | Rvc, 결과 저장 공간 | 센서 인터페이스를 거쳐 F/L/R을 모두 보존한 결과 |
| Dust Detector | Rvc, 결과 저장 공간 | 먼지 센서 인터페이스를 거친 D |
| Controller | Rvc, 호출 한 번=Tick | 입력 수집 → 상태 판단 → 하위 동작 호출, RvcStatus 반환 |
| Move Forward | ENABLE/DISABLE | ENABLE은 Forward, DISABLE은 활성 해제만 수행 |
| Turn/Backward/Stop | 호출 한 번=Trigger | 각각 Left/Right/Backward/Stop 명령 전달 |
| Set Cleaning | OFF/ON/POWER1 | OFF/ON/UP 장치 명령 |
| Motor/Cleaner Interface | 명령 enum | 장치 콜백 호출, 결과와 출력 유효성 기록 |
| Mock Adapter | 센서 설정, 장애 주입, 장치 명령 | 고정된 센서 응답, 순서가 있는 호출 기록 |

Trigger를 함수 호출 자체로 구현한다. Enable/Disable은 값이 다른 명령이므로 enum 매개변수로 구별한다. 데이터 결과는 출력 포인터로, 처리 성공/실패는 반환값으로 분리한다. 구조도의 “출력”을 항상 C return으로 해석하지 않는다. Controller는 CommandList를 반환하지 않고 동작 모듈을 정해진 순서로 호출한다.

## 수명주기와 오류

1. `rvc_create`: 필수 콜백 확인, 표 복사, 메모리 생성. 센서/장치 호출 없음.
2. `rvc_initialize`: STOP을 시도하고 OFF도 시도한다. 어느 하나가 실패해도 두 초기 출력은 모두 시도한다. 출력 성공 후 센서값을 준비하여 STOP_OFF를 확정한다. 초기화 중에는 외부 Tick을 받지 않는다.
3. `rvc_tick`: 초기화된 객체의 Controller를 한 번 실행한다. sleep이나 추가 Tick을 만들지 않는다.
4. `rvc_report_front`: 캐시만 갱신한다. 즉시 FSM 전이나 장치 출력은 발생하지 않는다.
5. `rvc_shutdown`: OFF 후 STOP을 각각 시도하고 성공/실패와 무관하게 미준비 상태로 전환한다. 이후 Tick에는 재초기화가 필요하다.
6. `rvc_destroy`: 메모리만 해제한다. NULL 허용. 콜백 context를 해제하지 않으며 장치 출력을 대신하지 않는다.

context는 호출자 소유이며, 마지막 장치 호출이 끝날 때까지 살아 있어야 한다. `RvcDevice`의 센서 함수는 성공할 때만 `*out`을 갱신한다. 잘못된 enum이나 NULL 인자는 `RVC_INVALID_ARGUMENT`, 미준비 상태의 실행은 `RVC_NOT_READY`, 장치 실패는 오류 상태로 반환한다.

Tick 중 I/O 실패 시 OFF와 STOP을 각각 최선으로 시도한 뒤 잠근다. 장치 출력은 원자적 트랜잭션이 아니며 이미 실행한 명령을 단순히 상태를 되돌려 취소할 수 없다. 실패한 출력은 유효하지 않은 것으로 표시하며 이전 성공값이 현재 장치 상태라고 단정하지 않는다. 재초기화 성공 전에는 정상 실행을 재개하지 않는다.

## 입력 스냅샷과 공개 조회

- 전방 보고는 비공개 캐시에 반영된다. 아직 처리되지 않은 보고를 마지막 완료 Tick의 입력인 것처럼 공개하지 않는다.
- 각 Tick에서 좌·우·먼지 센서를 읽고 한 번의 판단에 사용할 입력을 고정한다.
- 입력의 일부를 읽은 뒤 다른 읽기가 실패해도 불완전한 결과를 공개 스냅샷으로 확정하지 않는다.
- 완료된 Tick의 입력·상태는 `rvc_get_telemetry`로 복사해서 조회한다. 호출자는 내부 상태를 쓰지 않는다.
- 초기화 전, fault 후, shutdown 후 조회는 `RVC_NOT_READY`이고 `*out`은 그대로 유지된다. 실패 상황은 반환 상태와 모의 장치 호출 로그로 확인한다.

## 시간·출력 정책

회전/후진 진입 시 `elapsed_ticks=0`. 각각 5/3번째 후속 Tick에 완료 판단한다. 만료 전에도 입력은 갱신하지만 일반 장애물 변화로 동작을 중단하지 않는다. 단, 장치 I/O 실패는 별도 fault 경로로 중단한다.

전진에서 회피 진입은 `Move Forward(DISABLE) → Set Cleaning(OFF) → 회피 Trigger` 순이다. 초기 상태나 이전 회피처럼 청소 출력이 이미 유효한 OFF라면 OFF는 중복 발행하지 않고 회피 Trigger만 보낸다. DISABLE은 모터 STOP이 아니므로 OFF와 회피 명령 사이까지 이전 모터 출력이 남을 수 있다. 목표는 회피 명령 전에 청소 OFF가 성립하도록 하는 것이다. 물리 장치의 청소 모터가 실제로 멈추는 시간을 확인하는 프로토콜은 범위 밖이다.

전진 복귀는 `Forward → ON/UP`. 전진 상태를 유지할 때 모터를 반복 명령하지 않는다. 먼지 모드가 바뀔 때만 ON/UP이 바뀐다. 회피 만료 후 같은 회피 구간을 다시 선택하면 Trigger를 새로 호출하고 경과를 0으로 재설정한다. Up은 누적 증가 연산이 아닌 절대 강화 단계다.
