/* RVC 통합 헤더: controller.c와 같은 폴더에 놓는다.
 * 공통 타입, 장치 콜백, 함수 선언, 모의 장치 계약을 한 곳에 모았다.
 * Rvc 내부 상태는 controller.c에 숨기며, 객체별 상태와 모듈별 함수는 유지한다.
 * 일반 앱은 3절의 공개 API를 사용한다. 4절은 구조도 대응과 모듈 시험용이다.
 * 파일을 나눈 개발본에서 생성했으며, 원래 헤더를 함께 include하지 않는다.
 */
#ifndef RVC_SINGLE_FILE_CONTROLLER_H
#define RVC_SINGLE_FILE_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 1. 공통 타입과 데이터 사전
 * ======================================================================== */

/* 원래 모듈: include/rvc/types.h */
/* 공통 데이터 사전: 모든 모듈이 공유하는 센서값, 상태, 명령, 처리 결과.
 * DFD의 데이터 화살표는 아래 구조체/enum으로 표현하며, 판단 로직은 넣지 않는다.
 */
/* 함수 반환값은 처리 성공 여부다. 센서값/상태 정보는 별도 out 인자로 받는다. */
typedef enum {
    RVC_OK = 0,
    RVC_INVALID_ARGUMENT,
    RVC_NOT_READY,
    RVC_NOT_IMPLEMENTED, /* 이전 골격의 예약 값. 현재 제품 실행 경로에서는 반환하지 않는다. */
    RVC_IO_ERROR
} RvcStatus;

/* true=장애물 감지, false=미감지. 읽기 실패를 false로 표현하지 않는다. */
typedef struct {
    bool front_blocked;
    bool left_blocked;
    bool right_blocked;
} RvcObstacles;

/* 한 번의 판단에서 사용할 F/L/R/D 묶음. 실제 센서가 동시에 측정됐다는 뜻은 아니다. */
typedef struct {
    RvcObstacles obstacles;
    bool dust_detected;
} RvcSensorSnapshot;

/* Motor Interface가 장치 콜백에 전달하는 구체적인 출력 명령. */
typedef enum {
    RVC_MOTOR_STOP, RVC_MOTOR_FORWARD, RVC_MOTOR_LEFT,
    RVC_MOTOR_RIGHT, RVC_MOTOR_BACKWARD
} RvcMotorCommand;

/* UP은 강화 단계 설정이다. 호출할 때마다 출력을 계속 높이는 증가 연산이 아니다. */
typedef enum {
    RVC_CLEANER_OFF, RVC_CLEANER_ON, RVC_CLEANER_UP
} RvcCleanerCommand;

/* Structured Chart의 제어 신호. DISABLE은 전진 활성 해제이며 STOP 명령과 다르다. */
typedef enum { RVC_FORWARD_DISABLE, RVC_FORWARD_ENABLE } RvcForwardControl;
/* Controller→Set Cleaning의 의미 값. 하위 Interface에서 쓸 명령과 타입을 구분한다. */
typedef enum { RVC_CLEANING_OFF, RVC_CLEANING_ON, RVC_CLEANING_POWER1 } RvcCleaningMode;

/* 공유 PPT의 6개 동작 상태. UNINITIALIZED는 초기화 전/종료/오류 후의 API 준비 상태다.
 * F_ON/F_POWER1: 전진+일반/강화 청소. LEFT/RIGHT/BACK_OFF: 회피+청소 꺼짐.
 */
typedef enum {
    RVC_STATE_UNINITIALIZED,
    RVC_STATE_STOP_OFF, RVC_STATE_F_ON, RVC_STATE_F_POWER1,
    RVC_STATE_LEFT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_BACK_OFF
} RvcState;

/* 사용자·테스트가 읽는 진단 복사본. 내부 객체의 상태를 직접 수정하는 통로가 아니다. */
typedef struct {
    RvcState state;
    uint32_t elapsed_ticks; /* 현재 회피 구간의 경과값. 진입=0, 회전 5/후진 3에서 재판단. */
    RvcSensorSnapshot sensors; /* 마지막 성공한 초기화/Tick에서 확정한 입력. */
    RvcMotorCommand motor;
    RvcCleanerCommand cleaner;
    bool initialized;
    bool motor_valid; /* 성공한 쓰기로 확인된 명령인지 표시. 모터의 물리 동작 측정값은 아님. */
    bool cleaner_valid; /* 쓰기 실패 시 false. 그때 남아 있는 명령 값을 성공으로 해석하면 안 됨. */
} RvcTelemetry;


/* ========================================================================
 * 2. 장치 콜백 계약
 * ======================================================================== */

/* 원래 모듈: include/rvc/device.h */
/* 장치 교체 지점: FSM은 실제 센서/모터 대신 이 함수 포인터 표를 사용한다.
 * 현재는 controller.c의 8절 모의 장치가 연결되며, 같은 계약을 지키는 실제 장치 어댑터로 교체할 수 있다.
 */
/* context는 호출자가 소유한 장치 데이터 주소로, 로봇 해제까지 유효하게 유지한다.
 * Rvc는 포인터를 보관할 뿐 context를 복사하거나 해제하지 않는다.
 * 읽기: 성공 시에만 *out에 Boolean을 저장하고 RVC_OK를 반환한다.
 * 쓰기: command를 적용하고 결과를 반환한다. 이미 적용한 다른 명령의 취소는 보장하지 않는다.
 * Tick의 실패 처리는 Controller가 담당한다(OFF/STOP 시도 후 재초기화 필요).
 */
typedef RvcStatus (*RvcReadSensor)(void *context, bool *out);
typedef RvcStatus (*RvcWriteMotor)(void *context, RvcMotorCommand command);
typedef RvcStatus (*RvcWriteCleaner)(void *context, RvcCleanerCommand command);
/* 콜백 6개는 모두 필수. context에 필요한 유효성 조건은 연결한 어댑터가 정한다. */
typedef struct {
    void *context;
    RvcReadSensor read_front; /* 초기 읽기. 이후 변경은 rvc_report_front로 전달한다. */
    RvcReadSensor read_left;
    RvcReadSensor read_right;
    RvcReadSensor read_dust;
    RvcWriteMotor write_motor;
    RvcWriteCleaner write_cleaner;
} RvcDevice;


/* ========================================================================
 * 3. 공개 API와 객체 수명주기
 * ======================================================================== */

/* 원래 모듈: include/rvc/rvc.h */
/* 앱/테스트가 사용하는 공개 API. 먼저 이 파일을 읽으면 전체 수명주기를 파악할 수 있다.
 * create → initialize → (report_front / tick / get_telemetry) 반복 → shutdown → destroy
 * 호출은 한 실행 흐름에서 직렬 처리한다. ISR에서 Tick과 동시에 직접 호출하지 않는다.
 */
/* 내부 필드를 감춘 객체 핸들. 실제 구조는 controller.c의 1절에만 있다. */
typedef struct Rvc Rvc;
/* [입력] 필수 콜백 6개가 채워진 device. [반환] 생성한 객체, 실패 시 NULL.
 * 함수 포인터 표를 복사한다. 장치 읽기/출력은 아직 없으며 메모리만 준비한다.
 * context의 수명 관리는 호출자가 맡는다. device 자체의 주소를 계속 보관하지는 않는다.
 */
Rvc *rvc_create(const RvcDevice *device);
/* 메모리 해제만 수행한다(NULL 허용). 장치 정지가 필요하면 먼저 shutdown을 호출한다. */
void rvc_destroy(Rvc *self);

/* [입력] create로 만든 self. [반환] 성공 RVC_OK / 잘못된 인자 / 장치 오류.
 * STOP → OFF를 모두 시도하고 초기 센서값을 읽는다. 모두 성공하면 STOP_OFF로 준비된다.
 * 실패 시 미준비 상태를 유지한다. 재호출하면 캐시·시간을 초기화하여 다시 시작한다.
 */
RvcStatus rvc_initialize(Rvc *self);
/* OFF → STOP을 각각 시도하고 미준비 상태로 만든다. 하나가 실패해도 다른 하나를 시도한다.
 * 성공이면 RVC_OK, 실패이면 첫 실패 코드를 반환한다. 실제 장치 정지 성공은 보장하지 않는다.
 * 다시 Tick을 처리하려면 initialize가 필요하다.
 */
RvcStatus rvc_shutdown(Rvc *self);
/* 호출 한 번=논리 Tick 하나. 센서 읽기→FSM 판단→명령 실행을 Controller에 위임한다.
 * 성공 RVC_OK, 미초기화/오류 후 NOT_READY. I/O 실패 시 종료 명령을 시도하고 원래 오류 반환.
 * 실제 시간 대기나 밀리초 인자는 없다. Main/테스트가 호출 시점을 정한다.
 */
RvcStatus rvc_tick(Rvc *self);
/* [입력] blocked=true는 전방 감지. 초기화된 객체의 캐시만 갱신한다.
 * 이 호출만으로 전이/명령/Tick은 발생하지 않으며 다음 Tick에서 최신 보고를 사용한다.
 * 성공 RVC_OK, NULL 인자 INVALID_ARGUMENT, 미준비 NOT_READY.
 */
RvcStatus rvc_report_front(Rvc *self, bool blocked);

/* [출력] 성공 시 *out에 진단 정보를 복사한다. 호출자는 그 복사본을 읽으면 된다.
 * 초기화 전/오류 후/종료 후 NOT_READY, NULL 인자는 INVALID_ARGUMENT. 실패 시 *out 유지.
 * 출력 명령 값은 해당 valid가 true일 때만 확인된 쓰기로 해석한다.
 */
RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out);


/* ========================================================================
 * 4. Structured Chart의 모듈별 함수 선언
 * ======================================================================== */

/* 원래 모듈: src/control/controller.h */
/* Controller 2.1.1의 내부 진입점. 세부 FSM과 명령 순서는 controller.c에 있다. */

/* controller.c의 2절 공개 API가 호출한다. 앱은 공개 controller.h의 공개 API를 사용한다.
 * 모든 함수는 자기 객체 self를 받고 RvcStatus로 성공/실패를 반환한다.
 * initialize: STOP/OFF → 센서 준비 → STOP_OFF. 실패하면 Tick 처리 불가.
 */
RvcStatus rvc_controller_initialize(Rvc *self);

/* tick: 상태·시간을 소유하며, 입력 한 묶음으로 최대 한 번 전이하고 필요한 명령을 호출한다. */
RvcStatus rvc_controller_tick(Rvc *self);
/* shutdown: OFF와 STOP 모두 시도 → 준비 해제. 장치 오류가 있어도 준비를 해제한다. */
RvcStatus rvc_controller_shutdown(Rvc *self);

/* 원래 모듈: src/perception/perception.h */
/* 판단 모듈의 내부 계약. Controller가 호출하고, 앱은 controller.h의 공개 API를 사용한다.
 * self는 로봇 인스턴스, out은 결과를 받을 유효한 주소여야 한다.
 * 반환 RvcStatus는 처리 성공/실패이고 실제 센서 결과는 *out으로 전달한다.
 * self/out 누락은 INVALID_ARGUMENT, 센서 오류는 그대로 전달, 성공은 OK.
 * 실패 시 *out은 변경하지 않는다. 이 모듈은 장치 출력이나 FSM 전이를 수행하지 않는다.
 */
/* DFD 1.5: Front 캐시(없으면 초기 읽기) + Left/Right 읽기를 성공한 뒤 F/L/R을 전달한다. */
RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out);

/* DFD 1.6: Dust Sensor Interface를 호출한 뒤 감지 여부를 *out에 쓴다(false도 정상값). */
RvcStatus rvc_determine_dust(Rvc *self, bool *out);

/* 원래 모듈: src/sensing/sensing.h */
/* 센서 인터페이스의 내부 계약. 앱은 이 헤더 대신 controller.h의 공개 API를 사용한다.
 * self는 로봇별 상태를 가리키며 NULL이면 INVALID_ARGUMENT을 반환한다.
 * sample/initialize는 장치 콜백을 호출하고, 성공 시 self->sampled와 valid를 갱신한다.
 * 콜백 누락은 INVALID_ARGUMENT, 장치 오류는 그대로 전달한다. 실패 시 valid=false.
 * bool 측정값(false 포함)과 RvcStatus 성공/실패는 서로 다른 정보다.
 * 이 함수들은 센서 값을 준비할 뿐, Controller의 FSM이나 출력을 결정하지 않는다.
 */
/* DFD 1.1: Front 초기값을 read_front로 읽는다. 이후 Tick은 유효한 캐시를 사용한다. */
RvcStatus rvc_front_sensor_initialize(Rvc *self);

/* DFD 1.1: blocked(true=막힘) 이벤트로 Front 캐시를 갱신한다. Tick과 직렬화해 호출한다.
 * 장치 콜백 없이 캐시만 갱신하며, self가 유효하면 OK. FSM 전이는 다음 Tick에서 수행한다.
 */
RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked);

/* DFD 1.2: 초기화 및 매 Tick에 read_left를 호출하여 좌측 장애물 유무를 저장한다. */
RvcStatus rvc_left_sensor_sample(Rvc *self);

/* DFD 1.3: 초기화 및 매 Tick에 read_right를 호출하여 우측 장애물 유무를 저장한다. */
RvcStatus rvc_right_sensor_sample(Rvc *self);

/* DFD 1.4: 초기화 및 매 Tick에 read_dust를 호출하여 먼지 감지 여부를 저장한다. */
RvcStatus rvc_dust_sensor_sample(Rvc *self);

/* 원래 모듈: src/actions/actions.h */
/* Structured Chart의 동작 모듈 계약. Controller가 호출하며 앱은 controller.h의 공개 API를 사용한다.
 * self는 로봇 인스턴스이며 NULL이면 INVALID_ARGUMENT. 출력 매개변수는 없고,
 * 하위 Motor/Cleaner Interface 호출이 장치 출력에 해당한다. RvcStatus는 성공/실패다.
 * 장치 출력의 오류는 호출자에게 그대로 전달한다. Timer, 센서 판단, FSM은 Controller 소유다.
 * Turn/Backward/Stop의 Trigger는 함수 호출 자체로 표현하며 별도 bool을 받지 않는다.
 */
/* DFD 2.1.2: Enable은 FORWARD 출력, Disable은 활성 해제만 수행한다(정지 출력 없음).
 * control이 두 값 이외이면 INVALID_ARGUMENT. Enable 성공 시에만 활성 플래그를 설정한다.
 */
RvcStatus rvc_move_forward(Rvc *self, RvcForwardControl control);

/* DFD 2.1.3: Trigger 한 번에 LEFT 명령 한 번. 유지시간은 Controller가 관리한다. */
RvcStatus rvc_turn_left(Rvc *self);

/* DFD 2.1.4: Trigger 한 번에 RIGHT 명령 한 번. 유지시간은 Controller가 관리한다. */
RvcStatus rvc_turn_right(Rvc *self);

/* DFD 2.1.5: Trigger 한 번에 BACKWARD 명령 한 번. 완료 후 동작은 Controller가 결정한다. */
RvcStatus rvc_move_backward(Rvc *self);

/* DFD 2.1.6: STOP을 출력하고 성공 시 전진 활성 플래그를 해제한다. 청소 출력은 별개다. */
RvcStatus rvc_stop_motor(Rvc *self);

/* DFD 2.1.7: OFF/ON/POWER1을 OFF/ON/UP으로 변환한다. UP은 강화 단계의 절대 설정이다.
 * mode가 세 값 이외이면 INVALID_ARGUMENT이며 장치 출력은 수행하지 않는다.
 */
RvcStatus rvc_set_cleaning(Rvc *self, RvcCleaningMode mode);

/* 원래 모듈: src/interfaces/interfaces.h */
/* 장치 출력 인터페이스의 내부 계약. 동작 모듈이 호출하며 앱은 controller.h의 공개 API를 사용한다.
 * self: 로봇 인스턴스, command: 장치에 전달할 절대 명령(매개변수가 장치 출력 데이터).
 * RvcStatus 반환값은 출력 명령 자체가 아니라 장치 콜백의 성공/실패를 나타낸다.
 * self/명령/필수 콜백 오류는 INVALID_ARGUMENT이며 장치 호출과 telemetry 변경은 없다.
 * 장치 콜백 실패는 그대로 전달하고 해당 valid=false. 성공은 명령 기록 + valid=true + OK.
 * 콜백은 RvcDevice에서 주입되므로 같은 모듈을 실제 장치와 모의 장치에 연결할 수 있다.
 */
/* DFD 2.2: 모터 명령을 검증해 write_motor로 전달한다. FSM 판단과 대기는 수행하지 않는다. */
RvcStatus rvc_motor_apply(Rvc *self, RvcMotorCommand command);

/* DFD 2.3: OFF/ON/UP을 검증해 write_cleaner로 전달한다. UP은 고정 강화 단계 설정이다. */
RvcStatus rvc_cleaner_apply(Rvc *self, RvcCleanerCommand command);


/* ========================================================================
 * 5. 모의 장치와 테스트 지원
 * ======================================================================== */

/* 원래 모듈: adapters/mock/mock_device.h */
/* 모의 하드웨어의 공개 계약. 센서 입력·명령 기록·고장 주입을 제공하고 FSM 판단은 하지 않는다. */
/* 고정 크기 로그가 가득 차면 추가 쓰기는 IO_ERROR로 실패한다. */
enum { RVC_MOCK_MAX_EVENTS = 1024 };
typedef enum { RVC_MOCK_MOTOR, RVC_MOCK_CLEANER } RvcMockEventKind;
/* 성공한 출력 한 건. command의 실제 열거형은 kind가 MOTOR인지 CLEANER인지로 구분한다. */
typedef struct {
    RvcMockEventKind kind;
    int command;
} RvcMockEvent;
/* 실패를 주입할 콜백의 식별자. READ 네 개는 read_counts의 인덱스와도 대응한다. */
typedef enum {
    RVC_MOCK_READ_FRONT, RVC_MOCK_READ_LEFT, RVC_MOCK_READ_RIGHT,
    RVC_MOCK_READ_DUST, RVC_MOCK_WRITE_MOTOR, RVC_MOCK_WRITE_CLEANER,
    RVC_MOCK_OPERATION_COUNT
} RvcMockOperation;

/* 호출자가 소유하는 독립 장치 한 대. 여러 인스턴스 사이에 입력/기록/고장을 공유하지 않는다.
 * counts에는 유효한 호출의 실패 시도도 포함하지만 events에는 성공한 쓰기만 담긴다.
 * motor/cleaner는 마지막으로 성공한 명령이다. 실패했다고 이전 출력이 자동 복원되는 것은 아니다.
 */
typedef struct {
    RvcSensorSnapshot inputs; /* 다음 센서 읽기에서 돌려줄 원시 입력. */
    RvcMockEvent events[RVC_MOCK_MAX_EVENTS]; /* 성공한 출력의 시간 순서 기록. */
    size_t event_count; /* 현재 로그의 길이. clear_events는 이 길이만 0으로 만든다. */
    size_t read_counts[4]; /* FRONT/LEFT/RIGHT/DUST별 누적 호출 시도 수. */
    size_t write_counts[2]; /* MOTOR/CLEANER별 누적 호출 시도 수. */
    size_t fault_remaining[RVC_MOCK_OPERATION_COUNT]; /* 연속으로 실패시킬 남은 호출 수. */
    RvcMotorCommand motor;
    RvcCleanerCommand cleaner;
    bool motor_valid; /* 적어도 한 번 모터 명령을 성공적으로 기록했는가. */
    bool cleaner_valid; /* 적어도 한 번 청소 명령을 성공적으로 기록했는가. */
} RvcMockDevice;

/* mock 전체를 초기화한다. 초기 출력값은 STOP/OFF지만 valid는 첫 쓰기 전까지 false다. NULL은 무시한다. */
void rvc_mock_init(RvcMockDevice *mock);
/* mock을 context로 참조하는 콜백 묶음을 값으로 반환한다. mock 자체를 복사하거나 새로 할당하지 않는다. */
RvcDevice rvc_mock_device(RvcMockDevice *mock);
/* 로그 길이만 0으로 만든다. 출력값·누적 횟수·입력·예약된 고장은 보존한다. NULL은 무시한다. */
void rvc_mock_clear_events(RvcMockDevice *mock);
/* 다음 읽기용 입력만 교체한다. 전방 변화 통지(rvc_report_front)는 별도로 호출해야 한다. */
void rvc_mock_set_inputs(RvcMockDevice *mock, RvcSensorSnapshot inputs);
/* operation의 다음 count회 호출을 실패시킨다. 기존 횟수를 대체하며 0이면 고장 주입을 취소한다.
 * 범위를 벗어난 operation 또는 NULL은 이 테스트 지원 함수에서 무시한다.
 */
void rvc_mock_fail_next(RvcMockDevice *mock, RvcMockOperation operation, size_t count);

#ifdef __cplusplus
}
#endif
#endif /* RVC_SINGLE_FILE_CONTROLLER_H */
