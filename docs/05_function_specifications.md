# 05. C 모듈 구성과 함수별 명세

갱신: 2026-10-03. 현재 구현 계약을 설명한다. 함수 선언만으로 구현 완료를 주장하지 않으며, 실행 검증 결과는 별도의 테스트 기록으로 확인한다. 상세 FSM과 명령 묶음은 [06](06_implemented_design.md)에 있다.

## C 모듈과 함수 입출력

제품은 C17이다. `.h`는 외부에서 사용할 타입과 함수 선언을, `.c`는 구현을 담는다. 소스가 헤더를 include하고 빌드가 해당 구현을 컴파일·링크한다. 여러 함수가 한 헤더를 공유할 수 있고 내부 보조 함수는 `.c` 안의 `static` 함수로 숨길 수 있다. C++ class로 제품을 구현하지 않으며 `Rvc*`의 내부 정의를 감추어 상태를 캡슐화한다.

| 종류 | 의미 | 예 |
|---|---|---|
| 입력 매개변수 | 호출자가 전달하는 업무 값/상태 객체 | `RvcCleaningMode mode`, `Rvc *self` |
| 출력 매개변수 | 함수가 결과를 써 줄 저장 공간 | `RvcObstacles *out` |
| 반환값 | 성공/실패 또는 생성한 객체 | `RvcStatus`, `Rvc*` |
| 내부 상태 변경 | 해당 인스턴스의 캐시/상태/카운터 갱신 | `elapsed_ticks` |
| 장치 출력 | 하위 인터페이스·콜백으로 명령 전달 | `write_motor(context, RIGHT)` |

입력이 있는 함수도 값을 반환할 수 있고, 업무 입력이 없는 센서 조회도 `self/context`를 받을 수 있다. 이 프로젝트에서 대부분의 함수는 결과를 `out`에 기록하고 상태 코드를 반환한다. Trigger는 함수 호출 한 번으로 표현하고 Enable/Disable은 enum 값으로 표현한다. 이는 C 구현 대응이며 교수님의 구두 필기에 대한 확정 해석은 아니다.

## 공통 계약

- 모든 `self`, `out` 및 context는 유효해야 한다. 공개 함수는 NULL 등 잘못된 인자를 거부한다.
- 센서 False는 정상 미감지이며 I/O 실패가 아니다. 실패한 조회는 호출자의 `*out`을 변경하지 않는다.
- 초기화된 로봇의 이벤트는 직렬 처리한다. ISR에서 다른 Tick과 동시에 직접 호출하지 않는다.
- 상태와 타이머는 Controller가 소유한다. 동작 모듈은 기다리거나 다음 동작을 결정하지 않는다.
- 이미 장치에 전달한 명령은 반환 실패만으로 취소되지 않는다. I/O 실패 시 최선의 OFF/STOP 후 미준비 상태로 잠그며 재초기화가 필요하다.
- 내부 모듈 함수는 공개 API에서 정한 순서로 호출한다. 테스트용으로 직접 호출할 경우 필요한 캐시/장치 준비 조건을 만족해야 한다.

## A. 공개 API — include/rvc/rvc.h / src/rvc.c

| 원형 | 입력·반환 | 처리·부작용 |
|---|---|---|
| `Rvc *rvc_create(const RvcDevice *device)` | 필수 콜백 표. 성공 객체/실패 NULL | 표 복사와 메모리 생성. 장치 호출 없음. context의 소유권은 호출자에게 남음 |
| `void rvc_destroy(Rvc *self)` | 로봇. NULL 허용 | 메모리만 해제. 장치 정지/콜백 context 해제는 하지 않음 |
| `RvcStatus rvc_initialize(Rvc *self)` | 생성한 로봇 | Controller 초기화 위임. STOP/OFF와 센서가 준비되면 STOP_OFF |
| `RvcStatus rvc_shutdown(Rvc *self)` | 생성한 로봇 | Controller 종료 위임. OFF 후 STOP 모두 시도, 미준비 전환 |
| `RvcStatus rvc_tick(Rvc *self)` | 초기화된 로봇. 호출 1회=논리 Tick | Controller 호출 위임. 자체 입력/전이 정책이나 sleep 없음 |
| `RvcStatus rvc_report_front(Rvc *self, bool blocked)` | 초기화된 로봇, 전방 보고 | Front Sensor에 전달. 캐시만 변경, 상태/명령/경과 시간 변경 없음 |
| `RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out)` | 결과 저장 공간 | 마지막 완료 판단의 읽기 전용 복사본. 미준비/fault/shutdown 후 NOT_READY, 실패 시 out 유지 |

호출 순서는 `create → initialize → report_front/tick/get_telemetry → shutdown → destroy`다. 실패 후에는 재초기화하거나 종료한다. `shutdown`도 실패할 수 있으며 메모리 해제가 실제 장치 정지를 보증하지 않는다.

## B. 센서 — src/sensing/sensing.h

| 원형 / DFD | 입력 | 결과·처리 |
|---|---|---|
| `RvcStatus rvc_front_sensor_initialize(Rvc *self)` / 1.1 | read_front와 context | 호출되면 전방을 읽고 성공 시 캐시 확정. Determine가 캐시가 유효하지 않을 때만 호출하므로 정상 Tick마다 재읽지 않음 |
| `RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked)` / 1.1 | 보고받은 Boolean | 전방 캐시·유효 여부 갱신. 타이머/출력 변경 없음 |
| `RvcStatus rvc_left_sensor_sample(Rvc *self)` / 1.2 | read_left와 context | Boolean을 읽고 성공 시 왼쪽 캐시 갱신 |
| `RvcStatus rvc_right_sensor_sample(Rvc *self)` / 1.3 | read_right와 context | Boolean을 읽고 성공 시 오른쪽 캐시 갱신 |
| `RvcStatus rvc_dust_sensor_sample(Rvc *self)` / 1.4 | read_dust와 context | Boolean을 읽고 성공 시 먼지 캐시 갱신. 강화 정책을 결정하지 않음 |

각 구현 파일은 `src/sensing/{front,left,right,dust}_sensor.c`다. 하드웨어 ADC 상세 대신 PDF의 Boolean 장치 인터페이스를 사용한다. 실제 아날로그 센서를 연결하면 어댑터가 Boolean으로 변환해야 한다.

## C. 입력 판단 — src/perception/perception.h

| 원형 / DFD | 입력 | 결과·처리 |
|---|---|---|
| `RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out)` / 1.5 | 로봇, 결과 공간 | Front Interface를 통한 초기 준비/캐시와 Left/Right Interface 샘플을 이용. 모든 값 성공 시 F/L/R을 함께 반환 |
| `RvcStatus rvc_determine_dust(Rvc *self, bool *out)` / 1.6 | 로봇, 결과 공간 | Dust Interface를 호출하여 성공한 D 반환 |

구현은 `obstacle_detector.c`, `dust_detector.c`다. 장애물 정보를 단일 방향 enum으로 축약하지 않으며 회피 우선순위도 판단하지 않는다. 이전의 `rvc_detection_initialize/tick`과 별도 `detection.c`는 제거한다. 센서를 호출하므로 `const Rvc*`가 아니라 `Rvc*`를 받는다.

## D. Controller — src/control/controller.h / controller.c

| 원형 / DFD | 입력 | 결과·처리 |
|---|---|---|
| `RvcStatus rvc_controller_initialize(Rvc *self)` / 2.1.1 | 로봇·장치 | Stop Motor → Set Cleaning(OFF) 모두 시도 → Determine 두 모듈로 초기 입력 준비 → STOP_OFF 확정 |
| `RvcStatus rvc_controller_tick(Rvc *self)` / 2.1.1 | 현재 상태/시간과 로봇 장치 | Determine Obstacle/Dust 호출 → 입력 고정 → 경과 갱신 → 전이 하나 선택 → 정해진 동작 호출 → 성공 상태/스냅샷 확정 |
| `RvcStatus rvc_controller_shutdown(Rvc *self)` / 2.1.1 수명주기 | 로봇·장치 | Cleaning OFF 후 Stop Motor 모두 시도. UNINITIALIZED, initialized=false. 오류가 있어도 재사용을 잠금 |

Controller가 CommandList를 반환하지 않는다. 반환값은 처리 상태이며, Command는 하위 동작 호출로 전달된다. Tick의 인자로 외부 스냅샷을 받던 이전 원형도 제거했다. 회전·후진 타이머는 이 모듈에서만 계산한다. 초기화/종료/오류의 UNINITIALIZED는 API 준비 상태이며 6개 업무 FSM에 추가한 청소 동작 상태가 아니다.

## E. 동작 — src/actions/actions.h

| 원형 / DFD | 입력 의미 | 결과·처리 |
|---|---|---|
| `RvcStatus rvc_move_forward(Rvc *self, RvcForwardControl control)` / 2.1.2 | ENABLE/DISABLE | ENABLE은 Motor Interface(FORWARD), DISABLE은 활성 해제만. Stop 출력 아님 |
| `RvcStatus rvc_turn_left(Rvc *self)` / 2.1.3 | 호출=Trigger | Motor Interface(LEFT). 자체 시간 대기 없음 |
| `RvcStatus rvc_turn_right(Rvc *self)` / 2.1.4 | 호출=Trigger | Motor Interface(RIGHT). 자체 시간 대기 없음 |
| `RvcStatus rvc_move_backward(Rvc *self)` / 2.1.5 | 호출=Trigger | Motor Interface(BACKWARD). 후속 회전 선택은 Controller 책임 |
| `RvcStatus rvc_stop_motor(Rvc *self)` / 2.1.6 | 호출=Trigger | Motor Interface(STOP), 전진 활성 해제 |
| `RvcStatus rvc_set_cleaning(Rvc *self, RvcCleaningMode mode)` / 2.1.7 | OFF/ON/POWER1 | Cleaner Interface(OFF/ON/UP). UP은 절대 강화 단계 |

각 함수는 동명 동작의 별도 `.c`에 구현한다. 하위 인터페이스 호출을 생략하고 직접 device 콜백을 호출하지 않는다. 인자 enum을 검사하고 하위 오류를 전파한다. Controller가 이 결과를 보고 fault 처리한다.

## F. 출력 인터페이스 — src/interfaces/interfaces.h

| 원형 / DFD | 입력 | 결과·처리 |
|---|---|---|
| `RvcStatus rvc_motor_apply(Rvc *self, RvcMotorCommand command)` / 2.2 | STOP/FORWARD/LEFT/RIGHT/BACKWARD | 검증 → write_motor → 성공 명령 기록. 쓰기 실패 시 출력 유효성 해제 |
| `RvcStatus rvc_cleaner_apply(Rvc *self, RvcCleanerCommand command)` / 2.3 | OFF/ON/UP | 검증 → write_cleaner → 성공 명령 기록. 쓰기 실패 시 출력 유효성 해제 |

파일은 `motor_interface.c`, `cleaner_interface.c`다. 방향, 먼지 정책, 다음 상태와 타이머는 여기서 결정하지 않는다.

## G. 장치 콜백 타입 — include/rvc/device.h

| 함수 포인터 타입 | 입력 | 결과 |
|---|---|---|
| `RvcReadSensor(void *context, bool *out)` | 해당 장치, 출력 저장 공간 | RvcStatus와 성공한 Boolean |
| `RvcWriteMotor(void *context, RvcMotorCommand command)` | 해당 장치, 모터 명령 | RvcStatus, 장치 명령 적용 |
| `RvcWriteCleaner(void *context, RvcCleanerCommand command)` | 해당 장치, 청소 명령 | RvcStatus, 장치 명령 적용 |

세 항목은 콜백의 형식을 나타내며 과제 모듈의 새 프로세스 번호가 아니다. 네 개의 센서 읽기 콜백과 두 출력 콜백을 `RvcDevice`에 연결한다.

## H. 모의 장치 — adapters/mock/mock_device.h

| 원형 | 처리 |
|---|---|
| `void rvc_mock_init(RvcMockDevice *mock)` | 호출자 소유 모의 장치 초기화 |
| `RvcDevice rvc_mock_device(RvcMockDevice *mock)` | 해당 mock을 context로 연결한 독립 콜백 표 반환 |
| `void rvc_mock_clear_events(RvcMockDevice *mock)` | 출력 이벤트 로그를 비워 다음 구간 검증 준비 |
| `void rvc_mock_set_inputs(RvcMockDevice *mock, RvcSensorSnapshot inputs)` | 센서 입력 설정. 전방 변경을 Controller에 알리려면 별도 report_front 호출 필요 |
| `void rvc_mock_fail_next(RvcMockDevice *mock, RvcMockOperation operation, size_t count)` | 해당 장치 연산의 다음 count회 실패 주입. 0으로 취소 |

모의 장치는 FSM 정책을 계산하지 않는다. 출력 이벤트에는 성공한 쓰기만 들어가며 read/write 횟수는 실패한 시도도 포함한다. 로그 용량은 헤더의 `RVC_MOCK_MAX_EVENTS`를 따른다. 테스트는 로그와 카운터의 의미를 혼동하지 않아야 한다. 실패 주입·입력 설정 함수는 테스트 지원용이며 과제의 추가 제어 모듈이 아니다.

## I. Main과 함수 설명의 관리

진입점은 `int main(int argc, char **argv)`다. `--scenario <CSV>`로 입력 파일을, `--realtime-ms <1..60000>`으로 요청 Tick 간격을 지정한다. 옵션을 생략하면 내장 시나리오를 실제 대기 없이 실행한다. 입력 오류는 0이 아닌 종료 코드로 보고하며, 로봇을 만든 이후의 종료 경로에서는 shutdown과 destroy를 수행한다.

Main은 장치 context를 만들고 API 수명주기 및 Tick/전방 보고를 전달한다. FSM 판단을 복제하지 않는다. 실시간 Tick의 실제 대기와 테스트의 직접 Tick 호출은 동일한 Controller를 사용한다.

새 함수를 추가할 때에는 책임, 매개변수 방향/값 범위, 반환 코드, 상태 변경, 하위 호출, 실패 결과, 관련 전이/테스트를 함께 기록한다. `.h` 주석은 호출 계약을 요약하고, 긴 알고리즘 설명은 이 문서와 상태표에서 관리한다. 문서 수정만으로 원격 PPT나 제출본이 바뀌지 않는다.
