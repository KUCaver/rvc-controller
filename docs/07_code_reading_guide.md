# 07. 코드를 하나씩 읽고 확인하는 순서

이 안내는 C 파일을 처음부터 따라가며 설계와 구현을 대조하려는 사람을 위한 것이다. 제품은 C17이며, `Rvc` 내부 데이터를 숨기고 책임별 함수와 파일로 나누었다. 각 함수 앞의 한국어 주석에서 역할·입력·출력·오류를 먼저 읽고, 함수 본문에서 그 순서를 확인하면 된다.

## 1. 먼저 읽을 파일

| 순서 | 파일 | 읽으면서 답할 질문 |
|---:|---|---|
| 1 | [types.h](../include/rvc/types.h) | 센서값, FSM 상태, 모터·청소 명령은 각각 어떤 타입인가? |
| 2 | [rvc.h](../include/rvc/rvc.h), [device.h](../include/rvc/device.h) | 외부에서 어떤 함수를 호출하며 장치 함수를 어떻게 연결하는가? |
| 3 | [rvc.c](../src/rvc.c), [rvc_internal.h](../src/internal/rvc_internal.h) | 로봇별 상태가 어디에 있고, 공개 함수가 어느 모듈에 위임하는가? |
| 4 | [controller.c](../src/control/controller.c) | 입력을 읽고 전이·Command를 결정하는 순서와 Tick 경계는 무엇인가? |
| 5 | [obstacle_detector.c](../src/perception/obstacle_detector.c), [dust_detector.c](../src/perception/dust_detector.c) | 센서를 어떻게 호출하고 결과를 출력 매개변수에 전달하는가? |
| 6 | [sensing.h](../src/sensing/sensing.h)와 같은 폴더의 `.c` | Front 보고 캐시와 나머지 주기 센서의 차이는 무엇인가? |
| 7 | [actions.h](../src/actions/actions.h), [interfaces.h](../src/interfaces/interfaces.h)와 각 `.c` | 동작 함수가 어떤 장치 명령으로 이어지는가? |
| 8 | [mock_device.h](../adapters/mock/mock_device.h), [mock_device.c](../adapters/mock/mock_device.c) | 실제 장치 대신 입력·출력·실패를 어떻게 재현하는가? |
| 9 | [main.c](../app/main.c) | 장치 연결, 입력 행, Tick 발생, 결과 출력, 종료를 어떤 순서로 실행하는가? |
| 10 | [rvc_tests.cpp](../tests/rvc_tests.cpp) | 입력을 준비한 후 어떤 실제 결과를 어떤 기대값과 비교하는가? |

4번은 먼저 `rvc_controller_tick`의 전체 흐름을 보고, `capture_inputs → select_next_state → apply_transition → dispatch_motion`을 따라 읽는다. 그다음 초기화·종료·오류 처리를 확인한다. 모듈 전체와 DFD 번호의 대응은 [구조 문서](01_structure.md)에 있다.

## 2. C 문법과 모듈 계약을 함께 읽기

`.h`는 타입과 함수 사용법을 공개하고 `.c`는 동작을 구현한다. `#include`는 헤더의 선언을 읽게 하며, 서로 다른 `.c`에 있는 함수는 빌드·링크 과정에서 연결된다. 이 프로젝트는 `.c`를 다른 `.c`에서 include하지 않는다.

| 코드 표현 | 이 프로젝트에서의 의미 |
|---|---|
| `Rvc *self` | 처리할 로봇 객체의 주소. C++의 `this`와 비슷한 역할을 함수 인자로 명시한다. |
| `const Rvc *self` | 그 포인터를 통해 객체를 수정하지 않는 조회 계약 |
| `RvcObstacles *out` | 호출자가 결과를 받을 저장 공간의 주소 |
| `&value` / `*out = value` | 저장 공간의 주소 전달 / 그 공간에 결과 기록 |
| `self->telemetry` | self가 가리키는 객체의 진단·상태 정보 |
| `RvcStatus` | 성공/실패 코드. 장애물 존재 여부와는 별개 |
| `static` 함수 | 해당 `.c`에서만 사용하는 내부 보조 함수 |
| 장치 함수 포인터 | `read_left`, `write_motor`처럼 연결된 장치 구현을 호출하는 통로 |
| `extern "C"` | C++ 테스트가 C로 컴파일한 함수를 올바른 이름으로 연결하도록 하는 선언 |

예를 들어 내부 Determine 함수는 다음과 같이 사용된다.

```c
RvcObstacles observed;
RvcStatus status = rvc_determine_obstacles(self, &observed);
if (status != RVC_OK) {
    return status;
}
/* 여기서부터 observed의 F/L/R을 정상적인 측정 결과로 사용한다. */
```

`status`는 읽기 성공 여부, `observed`는 센서 결과다. `false`는 정상적인 미감지이며 실패와 다르다. `out`을 받는 조회/판단 함수는 실패 시 호출자의 기존 값을 보존한다. 이 예시는 내부 모듈 호출 방식이며, Main은 공개 API를 사용한다.

## 3. 실제 입력 한 번을 끝까지 따라가기

[기본 실행 로그](../examples/demo_output.csv)의 Tick 4를 예로 든다. 직전에는 `F_ON`으로 전진·일반 청소 중이다. 새 입력은 `F=1, L=0, R=0, D=1`이다.

1. Main이 모의 센서값을 설정한다. 전방이 변경되었으므로 `rvc_report_front(robot, true)`도 호출한다. 이때는 캐시만 바뀐다.
2. Main의 `rvc_tick(robot)`이 공개 진입점을 거쳐 `rvc_controller_tick`을 호출한다.
3. `capture_inputs`가 Determine 모듈들을 호출한다. Front 캐시와 새 Left/Right/Dust 읽기 결과를 이번 판단에 사용한다.
4. `select_next_state`가 전방 장애물과 열린 오른쪽을 보고 `RIGHT_OFF`를 선택한다. 먼지가 있어도 이 입력에서는 회피한다.
5. `apply_transition`이 전진을 Disable한다. 이는 활성 기록의 해제이며 STOP 명령은 아니다.
6. `rvc_set_cleaning(OFF) → rvc_cleaner_apply(OFF) → write_cleaner` 순서로 청소를 끈다.
7. `rvc_turn_right → rvc_motor_apply(RIGHT) → write_motor` 순서로 우회전 명령을 전달한다.
8. 모두 성공하면 상태를 `RIGHT_OFF`, 경과 Tick을 `0`으로 확정한다. Main이 진단 복사본과 로그를 출력한다.

따라서 해당 행의 명령 순서는 `C:OFF|M:RIGHT`다. 이는 FSM에 condition뿐 아니라 실행되는 Command가 있다는 것을 코드와 로그 양쪽에서 확인하는 예다. 이미 청소가 OFF인 상태에서 다시 회피하면 중복 OFF는 생략한다.

## 4. Tick을 셀 때 주의할 점

기본 로그에서 Tick 4는 회전 **진입**이며 `elapsed=0`이다. Tick 5~8은 후속 1~4 Tick으로 회전을 유지한다. Tick 9에서 후속 5 Tick이 되어 새 입력으로 다음 상태를 고른다. 그동안 앞이 열려 있어도 유지 구간에서는 회전 출력이 바뀌지 않는다.

Tick 10의 후진 진입도 `elapsed=0`이다. 후속 3번째 Tick에서 다시 판단하며, 그때 앞이 열려 있어도 우회전 또는 좌회전, 둘 다 막히면 재후진을 선택한다. 후진에서 즉시 전진하지 않는다.

`rvc_tick()` 안에는 `sleep`이 없다. 테스트는 이 함수를 연속 호출하여 시간을 논리적으로 진행한다. Main의 `--realtime-ms` 옵션만 실제 대기를 추가한다. 200ms를 선택했다면 5 Tick은 요청 간격 기준 약 1초지만, 운영체제의 정확한 주기 보장을 뜻하지 않는다.

## 5. 같은 숫자처럼 보여도 구별할 정보

| 구별 | 이유 |
|---|---|
| `sampled`와 `telemetry.sensors` | 앞쪽은 읽는 중인 개별 센서 캐시, 뒤쪽은 마지막 성공한 초기화/Tick의 확정 입력이다. |
| `state`와 `motor`/`cleaner` | 상태는 판단 결과, 모터·청소 값은 인터페이스의 쓰기 결과다. 실제 Command는 하위 함수를 통해 전달한다. |
| 출력 값과 `*_valid` | 쓰기에 실패하면 과거 명령 값이 남아 있어도 현재 성공 출력처럼 해석하면 안 된다. |
| Enable/Disable과 Trigger | 전진은 인자로 활성/해제를 받는다. 좌·우·후진·정지는 함수 한 번 호출하는 것이 Trigger다. |
| `UP`과 출력 증가 | UP은 정해진 강화 단계 설정이다. 반복 호출마다 계속 증가하지 않는다. |
| `shutdown`과 `destroy` | 앞은 OFF/STOP 명령과 준비 해제, 뒤는 메모리 해제다. |
| 모의 장치 호출 횟수와 이벤트 로그 | 횟수에는 실패 시도가 포함되고, 로그에는 성공한 출력만 기록된다. |

## 6. 오류 경로도 따라 읽기

Tick의 센서 읽기나 명령 쓰기가 실패하면 Controller는 `fail_closed`에서 청소 OFF와 모터 STOP을 각각 시도한다. 원래 오류를 반환하며, 그 뒤에는 재초기화 전까지 `tick`과 `get_telemetry`가 `NOT_READY`를 반환한다. 앞서 성공한 장치 쓰기가 자동 취소되는 것은 아니다.

초기화는 STOP → OFF, 종료·Tick 오류 처리는 OFF → STOP 순서다. 코드가 이 두 순서를 구별하는지, 첫 쓰기가 실패해도 다른 종료 쓰기를 시도하는지 확인한다. `get_telemetry`로 오류 이후 상태를 읽는 대신 테스트에서는 반환 코드와 모의 장치의 출력 로그·시도 횟수를 검사한다.

## 7. 테스트로 직접 확인하기

프로젝트 폴더에서 `./build.ps1`을 실행하면 제품·테스트를 빌드하고 실제 GoogleTest를 실행한다. 상세 실행법과 개별 시나리오 선택은 [테스트 안내](../tests/README.md)를 따른다.

테스트 파일에서는 준비(Arrange: 입력·장치·상태 만들기), 실행(Act: Tick/이벤트 호출), 검증(Assert: `EXPECT_*`/`ASSERT_*`)을 구별해 읽는다. 상태를 내부 변수에 직접 대입하지 않고 실제 입력 이력으로 만든다. 기대값은 테스트의 상수 표에 있으며 제품의 판단 함수를 호출해서 정답을 만들지 않는다.

한 모듈씩 확인할 때는 다음을 설명할 수 있으면 된다.

- 누가 이 함수를 호출하며, 다음에는 어떤 하위 모듈을 호출하는가?
- 입력 값과 출력 저장 공간은 무엇이며 반환값은 무엇을 뜻하는가?
- 어떤 로봇 내부 값이나 장치 출력이 바뀌는가?
- 실패하면 어떤 값이 유지되고 어떤 오류가 전달되는가?
- 연결된 FSM 전이와 테스트 사례를 찾을 수 있는가?

세부 함수 계약은 [함수 명세](05_function_specifications.md), 전이별 조건·Command는 [구현 설계](06_implemented_design.md)를 참고한다. 먼지 유지시간 등 자료 해석의 미확정 부분은 [작업 상태](STATUS.md)에 남아 있으며, 코드 주석은 현재 구현된 정책을 설명한다.
