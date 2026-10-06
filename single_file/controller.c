/* RVC 통합 구현: Main + Controller + 판단/센서 + 동작/출력 + 모의 장치.
 * 준비할 구현 파일은 이 controller.c와 같은 폴더의 controller.h 두 개다.
 * 컴파일: gcc -std=c17 -Wall -Wextra -Wpedantic -Werror controller.c -o rvc.exe
 * 실행:   ./rvc.exe                         (내장 시나리오; 외부 CSV 불필요)
 *         ./rvc.exe --scenario demo.csv     (선택한 CSV로 논리 Tick 실행)
 *         ./rvc.exe --realtime-ms 200        (실제 대기를 넣는 예시)
 *
 * 읽기: 9절 Main → 2절 공개 API → 3절 Controller → 4~7절 모듈 → 8절 모의 장치.
 * Structured Chart의 함수와 호출 관계는 유지하고 파일 배치만 통합했다.
 * Sensor 결과는 out 매개변수, 성공/실패는 RvcStatus 반환값으로 구분한다.
 * Controller가 조건을 판단해 동작 함수를 호출하며 실제 Command는 장치 콜백에 전달한다.
 * 회전 5 Tick/후진 3 Tick, 진입 e=0. Tick 발생과 대기는 Main이 담당한다.
 *
 * GoogleTest에 연결할 때만 -DRVC_NO_MAIN을 지정하여 Main과 그 전용 헬퍼를 제외한다.
 * 제품 자체에는 GoogleTest/C++/원래 모듈 폴더/CMake/Python이 필요하지 않다.
 * 개발본에서 다시 생성: python tools/generate_single_file.py
 */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#include "controller.h"
#include <stdlib.h>
#include <string.h>
#ifndef RVC_NO_MAIN
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <time.h>
#endif
#endif

/* ========================================================================
 * 1. 내부 객체 상태: 인스턴스별 캡슐화
 * ======================================================================== */

/* C에서 내부 상태를 감추는 실제 객체 정의. 외부 앱은 이 헤더를 include하지 않는다.
 * 로봇마다 이 구조체 하나를 가지므로 센서·시간·장치가 다른 인스턴스와 섞이지 않는다.
 * Controller가 FSM/시간을, 센서 모듈이 sampled를, 출력 인터페이스가 출력 유효성을 갱신한다.
 */
struct Rvc {
    RvcDevice device; /* 호출할 센서/출력 함수와 호출자 소유 context. */
    RvcTelemetry telemetry; /* 확정 상태·입력과 장치 쓰기 결과. get_telemetry는 이를 복사한다. */
    RvcSensorSnapshot sampled; /* 읽는 중인 센서 캐시. 아직 확정된 telemetry.sensors와 구별. */
    /* valid 플래그가 false인 미확정/오류 캐시와 정상 미감지(측정값 false)를 구별하는 표시. */
    bool front_valid;
    bool left_valid;
    bool right_valid;
    bool dust_valid;
    bool forward_enabled; /* Enable/Disable 제어의 기록. 다음 상태를 선택하는 FSM 값은 아니다. */
};

/* ========================================================================
 * 2. 공개 API: Main과 Controller의 연결
 * ======================================================================== */

/* 원래 모듈: src/rvc.c */
/* 공개 API 구현: 객체 수명 관리와 내부 모듈로의 호출 연결을 담당한다.
 * Main/GoogleTest → 이 파일의 rvc_* 함수 → Controller/Front Sensor.
 * 여기서 방향·청소 정책을 다시 판단하지 않는다. FSM의 유일한 구현은 Controller다.
 * 인자/반환 계약은 controller.h의 공개 API, 내부 저장 구조는 controller.c의 1절 참조.
 */

/* 필요한 장치 함수를 먼저 확인한 뒤 독립 객체를 할당한다. 초기 장치 출력은 initialize의 일이다. */
Rvc *rvc_create(const RvcDevice *device)
{
    if (!device || !device->read_front || !device->read_left ||
        !device->read_right || !device->read_dust ||
        !device->write_motor || !device->write_cleaner) {
        return NULL;
    }
    /* calloc으로 유효성 표시와 시간 등을 0으로 시작한다. context의 내용은 복제하지 않는다. */
    Rvc *self = calloc(1, sizeof(*self));
    if (self) {
        self->device = *device;
        self->telemetry.state = RVC_STATE_UNINITIALIZED;
    }
    return self;
}

/* 해제만 수행하므로 shutdown의 성공 여부를 숨기지 않는다. free(NULL)은 허용된다. */
void rvc_destroy(Rvc *self)
{
    free(self);
}

/* 공개 초기화 진입점. 초기 명령·센서 읽기의 순서는 Controller 한 곳에서 관리한다. */
RvcStatus rvc_initialize(Rvc *self)
{
    return rvc_controller_initialize(self);
}

/* 장치 종료와 준비 해제 결과를 그대로 호출자에게 전달한다. 객체 메모리는 남아 있다. */
RvcStatus rvc_shutdown(Rvc *self)
{
    return rvc_controller_shutdown(self);
}

/* Structured Chart의 Controller로 들어가는 공개 함수. 전달 자체가 새 FSM 계층은 아니다. */
RvcStatus rvc_tick(Rvc *self)
{
    return rvc_controller_tick(self);
}

/* 전방 이벤트는 센서 캐시만 갱신한다. 공개 진단 입력은 다음 성공 Tick에서 확정된다. */
RvcStatus rvc_report_front(Rvc *self, bool blocked)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;
    return rvc_front_sensor_report(self, blocked);
}

/* 읽기 전용 진단 복사: 조건 검사 이후에만 *out을 써서 실패 시 호출자의 값을 보존한다. */
RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out)
{
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;
    *out = self->telemetry;
    return RVC_OK;
}


/* ========================================================================
 * 3. Controller 2.1.1: 상태, 조건, Command, Tick 관리
 * ======================================================================== */

/* 원래 모듈: src/control/controller.c */
/* Controller 2.1.1: FSM 조건과 출력 Command를 실제 모듈 호출로 연결하는 곳.
 * 입력: Determine 두 모듈의 센서값 + 현재 상태/경과 Tick.
 * 처리: 입력 읽기 → 회피 시간 확인 → 다음 상태 선택 → 명령 실행 → 성공 상태 확정.
 * 출력: 동작 모듈을 통한 모터/청소 명령과 RvcStatus. 명령을 반환값으로 돌려주지 않는다.
 * 이 파일만 방향·먼지 정책과 회전/후진 시간을 결정한다. 실제 대기는 Main의 책임이다.
 * 기존 전이·Command와 함수 호출을 보존했다. 통합본도 동일한 134개 GoogleTest로 검사한다.
 */

/* 단위는 밀리초가 아닌 후속 Tick 수. 회피에 들어간 순간은 e=0이다. */
enum { TURN_TICKS = 5, REVERSE_TICKS = 3 };
/* 전진 모터를 공유하는 두 상태인지 확인한다. 청소 모드만 달라질 때 중복 전진 명령을 줄인다. */
static bool is_forward(RvcState s)
{
    return s == RVC_STATE_F_ON || s == RVC_STATE_F_POWER1;
}
/* 회전/후진 상태에만 구간 타이머를 적용한다. 전진·정지는 이 타이머를 기다리지 않는다. */
static bool is_maneuver(RvcState s)
{
    return s == RVC_STATE_LEFT_OFF || s == RVC_STATE_RIGHT_OFF || s == RVC_STATE_BACK_OFF;
}
/* Controller → Determine → Sensor Interface의 호출 구조를 지킨다.
 * input은 호출자가 만든 임시 저장 공간이다. 장애물 읽기 후 먼지 읽기가 실패하면
 * 일부 값이 들어 있어도 사용하지 않는다. 두 읽기 모두 성공해야 판단에 사용한다.
 * 내부 전용 함수이므로 self/input의 준비는 호출하는 초기화/Tick 함수가 보장한다.
 */
static RvcStatus capture_inputs(Rvc *self, RvcSensorSnapshot *input)
{
    RvcStatus result = rvc_determine_obstacles(self, &input->obstacles);
    if (result != RVC_OK) return result;
    return rvc_determine_dust(self, &input->dust_detected);
}
/* 명시적 종료와 Tick 오류 복구에서 공통 사용한다. OFF/STOP 이후 Tick 처리를 잠근다.
 * 둘 다 성공하면 OK, 청소 실패가 있으면 그 코드, 아니면 모터 결과를 반환한다.
 */
RvcStatus rvc_controller_shutdown(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    /* 청소 출력 실패가 모터 정지 시도까지 막지 않게 두 함수를 독립적으로 호출한다.
     * 이미 전달된 명령을 되돌리는 원자적 작업은 아니다. 반환값은 장치 쓰기 결과를 알린다.
     */
    RvcStatus clean = rvc_set_cleaning(self, RVC_CLEANING_OFF);
    RvcStatus motor = rvc_stop_motor(self);
    self->telemetry.initialized = false;
    self->telemetry.state = RVC_STATE_UNINITIALIZED;
    self->telemetry.elapsed_ticks = 0;
    self->forward_enabled = false;
    return clean != RVC_OK ? clean : motor;
}
/* 종료를 시도하되 호출자에게는 최초 원인 cause를 돌려준다.
 * 복구 중 추가 실패가 생겨도 초기 오류를 덮지 않는다. 성공을 가정하고 계속 운행하지 않는다.
 */
static RvcStatus fail_closed(Rvc *self, RvcStatus cause)
{
    (void)rvc_controller_shutdown(self);
    return cause;
}
/* 최초 실행/재시작: 유효성 초기화 → 정지/청소 꺼짐 → 센서 읽기 → 준비 완료.
 * 생성만 된 객체에도 호출할 수 있다. 실패하면 initialized=false가 남아 Tick이 거부된다.
 */
RvcStatus rvc_controller_initialize(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    self->telemetry.initialized = false;
    self->telemetry.state = RVC_STATE_UNINITIALIZED;
    self->telemetry.elapsed_ticks = 0;
    self->front_valid = self->left_valid = false;
    self->right_valid = self->dust_valid = false;
    self->forward_enabled = false;
    /* 최초 상태 진입의 명령이다. 아직 Tick은 아니며, STOP 실패 시에도 OFF는 시도한다. */
    RvcStatus motor = rvc_stop_motor(self);
    RvcStatus clean = rvc_set_cleaning(self, RVC_CLEANING_OFF);
    if (motor != RVC_OK || clean != RVC_OK)
        return motor != RVC_OK ? motor : clean;
    RvcSensorSnapshot input = {0};
    RvcStatus result = capture_inputs(self, &input);
    if (result != RVC_OK) return fail_closed(self, result);
    self->telemetry.sensors = input;
    self->telemetry.state = RVC_STATE_STOP_OFF;
    self->telemetry.initialized = true;
    return RVC_OK;
}
/* 장치 출력 없이 다음 상태만 고르는 함수. current/input을 읽고 RvcState를 반환한다.
 * 호출 시점은 정지/전진의 매 Tick 또는 회피 구간의 만료 Tick이다.
 * 공유 PPT 19p: 후진 뒤에는 앞이 열렸더라도 우회전→좌회전→재후진 순으로 선택한다.
 */
static RvcState select_next_state(RvcState current, const RvcSensorSnapshot *input)
{
    const RvcObstacles *o = &input->obstacles;
    if (current == RVC_STATE_BACK_OFF || o->front_blocked) {
        if (!o->right_blocked) return RVC_STATE_RIGHT_OFF;
        if (!o->left_blocked) return RVC_STATE_LEFT_OFF;
        return RVC_STATE_BACK_OFF;
    }
    /* 앞이 열린 일반 판단: 먼지값에 따라 전진+강화 또는 전진+일반 청소.
     * 현재 PPT 정책에는 먼지가 사라진 후 추가로 유지하는 타이머가 없다.
     */
    return input->dust_detected ? RVC_STATE_F_POWER1 : RVC_STATE_F_ON;
}
/* 상태를 Structured Chart의 동작 함수 호출로 바꾼다. next 자체를 장치에 보내지 않는다.
 * 좌/우/후진 함수 호출 한 번이 Trigger다. 그 함수들은 기다리거나 시간을 세지 않는다.
 */
static RvcStatus dispatch_motion(Rvc *self, RvcState next)
{
    switch (next) {
    case RVC_STATE_F_ON:
    case RVC_STATE_F_POWER1: return rvc_move_forward(self, RVC_FORWARD_ENABLE);
    case RVC_STATE_LEFT_OFF: return rvc_turn_left(self);
    case RVC_STATE_RIGHT_OFF: return rvc_turn_right(self);
    case RVC_STATE_BACK_OFF: return rvc_move_backward(self);
    default: return RVC_INVALID_ARGUMENT;
    }
}
/* 현재 상태→next 전이에 필요한 출력만 정해진 순서로 보낸다. FSM 상태 확정은 호출자의 일.
 * 회피 구간 만료에서 같은 회피를 선택해도 새 Trigger를 보낸다. 유지 Tick은 이 함수를 안 탄다.
 * 하위 출력 하나라도 실패하면 즉시 반환하며, 호출자가 OFF/STOP 오류 처리를 수행한다.
 */
static RvcStatus apply_transition(Rvc *self, RvcState next)
{
    bool was_forward = is_forward(self->telemetry.state);
    bool now_forward = is_forward(next);
    RvcStatus result;
    if (was_forward && !now_forward) {
        /* 전진 활성 해제는 STOP 명령이 아니다. 이어서 선택한 회피 방향 명령을 보낸다. */
        result = rvc_move_forward(self, RVC_FORWARD_DISABLE);
        if (result != RVC_OK) return result;
    }
    if (!now_forward) {
        /* 청소가 켜져 있거나 출력이 불확실하면 OFF를 먼저 보낸다.
         * OFF 실패 시 방향 명령을 보내지 않는다. 이미 OFF이면 중복 쓰기를 생략한다.
         */
        if (!self->telemetry.cleaner_valid || self->telemetry.cleaner != RVC_CLEANER_OFF) {
            result = rvc_set_cleaning(self, RVC_CLEANING_OFF);
            if (result != RVC_OK) return result;
        }
        return dispatch_motion(self, next);
    }
    if (!was_forward || !self->telemetry.motor_valid) {
        /* 정지/회피에서 전진으로 진입할 때 전진 명령. 기존 전진의 먼지 변화에는 생략한다. */
        result = dispatch_motion(self, next);
        if (result != RVC_OK) return result;
    }
    /* mode는 동작 모듈 입력, command는 장치의 현재 출력과 비교할 값이다.
     * 의미가 대응하더라도 두 계층의 enum을 숫자 캐스팅으로 섞지 않는다.
     */
    RvcCleaningMode mode = next == RVC_STATE_F_POWER1 ? RVC_CLEANING_POWER1 : RVC_CLEANING_ON;
    RvcCleanerCommand command = next == RVC_STATE_F_POWER1 ? RVC_CLEANER_UP : RVC_CLEANER_ON;
    if (!self->telemetry.cleaner_valid || self->telemetry.cleaner != command)
        return rvc_set_cleaning(self, mode);
    return RVC_OK;
}
/* 논리 Tick 하나의 전체 처리. 여러 번 호출해야 여러 Tick이 흐른다.
 * 성공 시 확정한 상태/입력을 telemetry에 남긴다. 미준비는 출력 없이 NOT_READY 반환.
 * 센서/명령 실패는 fail_closed로 처리하므로 다음 Tick 전에 재초기화해야 한다.
 */
RvcStatus rvc_controller_tick(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;
    /* 이번 Tick의 판단에 사용할 지역 입력. 회피 유지 중에도 센서를 새로 읽는다. */
    RvcSensorSnapshot input = {0};
    RvcStatus result = capture_inputs(self, &input);
    if (result != RVC_OK) return fail_closed(self, result);
    RvcState current = self->telemetry.state;
    if (is_maneuver(current)) {
        /* 진입 후 첫 호출에서 0→1. 회전은 1~4 유지, 5에서 재판단; 후진은 1~2 유지, 3에서 재판단.
         * 만료값을 새 상태의 경과값으로 남기지 않고, 아래의 전이 확정에서 다시 0으로 만든다.
         */
        uint32_t elapsed = self->telemetry.elapsed_ticks + 1U;
        uint32_t duration = current == RVC_STATE_BACK_OFF ? REVERSE_TICKS : TURN_TICKS;
        if (elapsed < duration) {
            /* 입력과 시간만 확정한다. 모터/청소 콜백을 다시 호출하지 않아 기존 출력을 유지한다. */
            self->telemetry.elapsed_ticks = elapsed;
            self->telemetry.sensors = input;
            return RVC_OK;
        }
    } else if (current != RVC_STATE_STOP_OFF && !is_forward(current)) {
        return fail_closed(self, RVC_INVALID_ARGUMENT);
    }
    /* 조건 선택과 Command 실행을 구분해 읽는다. 다음 상태만 바꾸고 출력을 빠뜨리면 안 된다. */
    RvcState next = select_next_state(current, &input);
    result = apply_transition(self, next);
    if (result != RVC_OK) return fail_closed(self, result);
    /* 명령이 모두 성공했을 때만 새 상태·입력을 확정한다. Tick 하나에 전이는 최대 한 번.
     * 상태 진입은 추가 Tick이 아니므로 새 회피 구간의 e는 0부터 시작한다.
     * 장치 쓰기는 이미 순서대로 발생했으며, 이 대입들이 하드웨어 원자성을 뜻하지는 않는다.
     */
    self->telemetry.state = next;
    self->telemetry.elapsed_ticks = 0;
    self->telemetry.sensors = input;
    return RVC_OK;
}


/* ========================================================================
 * 4. 입력 판단: Determine Obstacle / Dust
 * ======================================================================== */

/* 원래 모듈: src/perception/obstacle_detector.c */
/* Determine Obstacle Location (DFD 1.5): Controller에 F/L/R 관측값을 한 묶음으로 전달한다.
 * 호출: Controller -> 이 함수 -> Front/Left/Right Sensor Interface.
 * 입력 self: 로봇 상태. 출력 out: 호출자가 준비한 RvcObstacles 저장 공간.
 * 반환: self/out이 NULL이면 INVALID_ARGUMENT, 센서 실패는 그대로 전달, 성공은 OK.
 * out은 모든 필수 읽기가 성공한 뒤에만 쓴다. 중간 실패 시 호출자의 *out은 유지된다.
 * 단, 읽기에 성공한 개별 센서의 내부 캐시는 이미 갱신될 수 있다.
 * Front는 캐시가 없을 때만 최초 읽기를 하고, 이후에는 이벤트 보고 값을 사용한다.
 * Left/Right는 매 호출마다 읽는다. 회피 방향 선택은 Controller가 담당한다.
 */
RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out)
{
    RvcStatus status;
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    /* Front의 이벤트 입력 방식을 보존한다. 매 Tick에 read_front를 호출하지 않는다. */
    if (!self->front_valid) {
        status = rvc_front_sensor_initialize(self);
        if (status != RVC_OK) return status;
    }
    status = rvc_left_sensor_sample(self);
    if (status != RVC_OK) return status;
    status = rvc_right_sensor_sample(self);
    if (status != RVC_OK) return status;
    /* 각 센서가 준비된 뒤 결과를 한 번에 복사한다. 반환값은 데이터가 아닌 상태 코드다. */
    *out = self->sampled.obstacles;
    return RVC_OK;
}

/* 원래 모듈: src/perception/dust_detector.c */
/* Determine Dust Existence (DFD 1.6).
 * 호출: Controller -> 이 함수 -> Dust Sensor Interface -> 장치 읽기 콜백.
 * 입력 self: 로봇 상태. 출력 out: 호출자가 준비한 bool 저장 공간(true=먼지 감지).
 * 반환: self/out 누락은 INVALID_ARGUMENT, 읽기 실패는 센서 상태, 성공은 OK.
 * 읽기에 성공할 때만 *out을 쓴다. false를 반환 상태로 표현하지 않으므로
 * '정상적으로 읽은 먼지 없음'과 '읽기 오류'를 Controller에서 구별할 수 있다.
 * ON/UP 명령 선택은 Controller의 FSM이 수행한다.
 */
RvcStatus rvc_determine_dust(Rvc *self, bool *out)
{
    RvcStatus status;
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    status = rvc_dust_sensor_sample(self);
    if (status != RVC_OK) return status;
    *out = self->sampled.dust_detected;
    return RVC_OK;
}


/* ========================================================================
 * 5. 센서 인터페이스: Front / Left / Right / Dust
 * ======================================================================== */

/* 원래 모듈: src/sensing/front_sensor.c */
/* Front Sensor Interface (DFD 1.1): 최초 읽기와 이후 이벤트 보고를 분리한다.
 * 호출: Determine Obstacle Location -> 이 함수 -> device.read_front 콜백.
 * 입력 self: 해당 로봇의 장치 콜백과 센서 캐시를 가진 인스턴스.
 * 출력: 성공하면 sampled.obstacles.front_blocked와 front_valid를 갱신한다.
 * 반환: self/콜백이 없으면 INVALID_ARGUMENT, 장치 실패는 그 상태를 그대로 반환.
 * false는 '장애물 없음'이라는 정상 데이터이며, 읽기 실패와 다르다.
 * 실패 시 이전 수치는 남지만 valid=false이므로 새 관측값으로 사용하면 안 된다.
 */
RvcStatus rvc_front_sensor_initialize(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    self->front_valid = false;
    if (!self->device.read_front) return RVC_INVALID_ARGUMENT;
    status = self->device.read_front(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.front_blocked = blocked;
    self->front_valid = true;
    return RVC_OK;
}

/* 공개 API rvc_report_front()에서 전달한 전방 센서 이벤트를 캐시에 반영한다.
 * 입력 blocked: true=전방 막힘, false=전방 열림. 출력은 캐시와 valid 플래그뿐이다.
 * self가 없으면 INVALID_ARGUMENT, 갱신하면 OK. 장치 읽기는 수행하지 않는다.
 * FSM 전이·모터 출력·Tick 증가는 다음 Controller 실행에서 처리한다.
 * 호출은 Tick 처리와 직렬화해야 한다. 실제 하드웨어 ISR과의 동기화 기능은 없다.
 */
RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    self->sampled.obstacles.front_blocked = blocked;
    self->front_valid = true;
    return RVC_OK;
}

/* 원래 모듈: src/sensing/left_sensor.c */
/* Left Sensor Interface (DFD 1.2).
 * 호출: Determine Obstacle Location -> 이 함수 -> device.read_left 콜백.
 * 입력 self: 장치와 센서 캐시를 가진 인스턴스. 초기화와 매 Tick에 한 번 읽는다.
 * 출력: 성공한 blocked 값만 sampled.obstacles.left_blocked에 저장한다.
 * 반환: self/콜백 누락은 INVALID_ARGUMENT, 읽기 실패는 장치 상태, 성공은 OK.
 * false는 '장애물 없음'이다. left_valid를 읽기 전에 해제하고 성공 시에만 설정하여
 * 실패 후 남아 있는 이전 값을 정상적인 이번 Tick 입력으로 오인하지 않게 한다.
 */
RvcStatus rvc_left_sensor_sample(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    self->left_valid = false;
    if (!self->device.read_left) return RVC_INVALID_ARGUMENT;
    status = self->device.read_left(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.left_blocked = blocked;
    self->left_valid = true;
    return RVC_OK;
}

/* 원래 모듈: src/sensing/right_sensor.c */
/* Right Sensor Interface (DFD 1.3).
 * 호출: Determine Obstacle Location -> 이 함수 -> device.read_right 콜백.
 * 입력 self: 장치와 센서 캐시를 가진 인스턴스. 초기화와 매 Tick에 한 번 읽는다.
 * 출력: 성공한 blocked 값만 sampled.obstacles.right_blocked에 저장한다.
 * 반환: self/콜백 누락은 INVALID_ARGUMENT, 읽기 실패는 장치 상태, 성공은 OK.
 * false는 '장애물 없음'이다. right_valid를 읽기 전에 해제하고 성공 시에만 설정하여
 * 실패 후 남아 있는 이전 값을 정상적인 이번 Tick 입력으로 오인하지 않게 한다.
 */
RvcStatus rvc_right_sensor_sample(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    self->right_valid = false;
    if (!self->device.read_right) return RVC_INVALID_ARGUMENT;
    status = self->device.read_right(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.right_blocked = blocked;
    self->right_valid = true;
    return RVC_OK;
}

/* 원래 모듈: src/sensing/dust_sensor.c */
/* Dust Sensor Interface (DFD 1.4).
 * 호출: Determine Dust Existence -> 이 함수 -> device.read_dust 콜백.
 * 입력 self: 장치와 센서 캐시를 가진 인스턴스. 초기화와 매 Tick에 한 번 읽는다.
 * 출력: 성공한 detected 값만 sampled.dust_detected에 저장한다.
 * 반환: self/콜백 누락은 INVALID_ARGUMENT, 읽기 실패는 장치 상태, 성공은 OK.
 * detected=false는 '먼지 미감지'이며 I/O 오류가 아니다. dust_valid를 먼저 해제하고
 * 성공 시에만 설정한다. 청소 세기 결정이나 강화 유지시간 관리는 이 모듈의 일이 아니다.
 */
RvcStatus rvc_dust_sensor_sample(Rvc *self)
{
    bool detected = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    self->dust_valid = false;
    if (!self->device.read_dust) return RVC_INVALID_ARGUMENT;
    status = self->device.read_dust(self->device.context, &detected);
    if (status != RVC_OK) return status;
    self->sampled.dust_detected = detected;
    self->dust_valid = true;
    return RVC_OK;
}


/* ========================================================================
 * 6. 동작: Forward / Left / Right / Backward / Stop / Cleaning
 * ======================================================================== */

/* 원래 모듈: src/actions/move_forward.c */
/* Move Forward (DFD 2.1.2): Controller의 Enable/Disable 제어를 처리한다.
 * 입력 self: 로봇 상태, control: RVC_FORWARD_ENABLE 또는 RVC_FORWARD_DISABLE.
 * Enable 출력: Motor Interface에 FORWARD를 보내고, 성공한 경우 활성 플래그를 설정한다.
 * Disable 출력: 활성 플래그만 해제한다. 장치의 기존 모터 명령은 그대로 남는다.
 * 따라서 Disable은 정지 명령이 아니다. Controller는 이어서 회피 동작 또는 Stop Motor를
 * 호출해 실제 모터 명령을 바꾼다. 이 함수는 그 순서나 지속시간을 결정하지 않는다.
 * 반환: self/control 오류는 INVALID_ARGUMENT, Disable 성공은 OK,
 * Enable은 Motor Interface의 성공/오류를 그대로 반환한다.
 */
RvcStatus rvc_move_forward(Rvc *self, RvcForwardControl control)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    if (control == RVC_FORWARD_DISABLE) {
        self->forward_enabled = false;
        return RVC_OK;
    }
    if (control != RVC_FORWARD_ENABLE) return RVC_INVALID_ARGUMENT;
    status = rvc_motor_apply(self, RVC_MOTOR_FORWARD);
    if (status == RVC_OK) self->forward_enabled = true;
    return status;
}

/* 원래 모듈: src/actions/turn_left.c */
/* Turn Left (DFD 2.1.3): Controller가 이 함수를 호출하는 행위가 Trigger 한 번이다.
 * 입력 self: 로봇 상태. 별도의 Trigger bool 매개변수는 필요하지 않다.
 * 출력: 공용 Motor Interface에 LEFT를 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스의 성공/오류를 전달한다.
 * 회전 완료를 기다리거나 여기서 Tick을 세지 않는다. 5 Tick 유지와 다음 동작 결정은
 * Controller가 담당하므로 대기 중에도 매 Tick에 센서를 확인할 수 있다.
 */
RvcStatus rvc_turn_left(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_LEFT);
}

/* 원래 모듈: src/actions/turn_right.c */
/* Turn Right (DFD 2.1.4): Controller가 이 함수를 호출하는 행위가 Trigger 한 번이다.
 * 입력 self: 로봇 상태. 별도의 Trigger bool 매개변수는 필요하지 않다.
 * 출력: 공용 Motor Interface에 RIGHT를 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스의 성공/오류를 전달한다.
 * 5 Tick 회전 시간은 Controller가 센다. 이 함수는 즉시 반환하며, 우회전 우선순위나
 * 센서에 따른 다음 상태를 결정하지 않는다.
 */
RvcStatus rvc_turn_right(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_RIGHT);
}

/* 원래 모듈: src/actions/move_backward.c */
/* Move Backward (DFD 2.1.5): Controller가 호출하면 Trigger 한 번으로 처리한다.
 * 입력 self: 로봇 상태. 출력: 공용 Motor Interface에 BACKWARD를 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스의 성공/오류를 전달한다.
 * 3 Tick 유지, 후진 구간 재시작, 후진 뒤 회전 선택은 모두 Controller의 책임이다.
 * 여기에는 대기나 센서 조건문이 없으므로 동작 모듈과 FSM의 역할이 겹치지 않는다.
 */
RvcStatus rvc_move_backward(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_BACKWARD);
}

/* 원래 모듈: src/actions/stop_motor.c */
/* Stop Motor (DFD 2.1.6): Controller가 초기화·종료·오류 정지에 사용하는 실제 정지 명령.
 * 입력 self: 로봇 상태. 출력: Motor Interface에 STOP을 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스 상태를 그대로 전달한다.
 * 장치 콜백이 성공을 알린 뒤에만 forward_enabled를 해제한다.
 * Forward의 Disable과 달리 하드웨어 명령을 보낸다. 청소 OFF는 별도 모듈의 책임이다.
 */
RvcStatus rvc_stop_motor(Rvc *self)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    status = rvc_motor_apply(self, RVC_MOTOR_STOP);
    if (status == RVC_OK) self->forward_enabled = false;
    return status;
}

/* 원래 모듈: src/actions/set_cleaning.c */
/* Set Cleaning (DFD 2.1.7): FSM의 청소 모드를 Cleaner Interface의 명령으로 변환한다.
 * 입력 self: 로봇 상태, mode: OFF / ON / POWER1.
 * 출력: 각각 장치 명령 OFF / ON / UP을 한 번 전달한다.
 * UP은 '강화 단계로 설정'하는 절대 명령이다. 호출할 때마다 세기를 누적해서 올리지 않는다.
 * 반환: self 또는 mode 오류는 INVALID_ARGUMENT, 그 외에는 인터페이스 상태를 전달한다.
 * 먼지 유무에 따른 모드 선택과 유지 정책은 Controller에 있으며 이 함수에는 없다.
 */
RvcStatus rvc_set_cleaning(Rvc *self, RvcCleaningMode mode)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    switch (mode) {
        case RVC_CLEANING_OFF:
            return rvc_cleaner_apply(self, RVC_CLEANER_OFF);
        case RVC_CLEANING_ON:
            return rvc_cleaner_apply(self, RVC_CLEANER_ON);
        case RVC_CLEANING_POWER1:
            return rvc_cleaner_apply(self, RVC_CLEANER_UP);
        default:
            return RVC_INVALID_ARGUMENT;
    }
}


/* ========================================================================
 * 7. 출력 인터페이스: Motor / Cleaner
 * ======================================================================== */

/* 원래 모듈: src/interfaces/motor_interface.c */
/* Motor Interface (DFD 2.2): 모든 이동 동작이 공유하는 장치 출력 경계.
 * 호출: Forward/Turn/Backward/Stop 모듈 -> 이 함수 -> device.write_motor 콜백.
 * 입력 self: 로봇과 장치 연결, command: STOP/FORWARD/LEFT/RIGHT/BACKWARD 중 하나.
 * 출력: 장치에 명령을 전달하고 성공하면 telemetry.motor와 motor_valid를 갱신한다.
 * 반환: self/명령/콜백 오류는 INVALID_ARGUMENT(장치 호출 없음), 장치 오류는 그대로,
 * 성공은 OK. 데이터 출력은 콜백의 command 인자로, 성공 여부는 반환값으로 표현한다.
 * 콜백 실패 시 장치의 실제 상태를 확정할 수 없으므로 마지막 명령 값은 보존하되
 * motor_valid=false로 표시한다. 장치 복구나 재시도 정책은 Controller가 결정한다.
 */
RvcStatus rvc_motor_apply(Rvc *self, RvcMotorCommand command)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    switch (command) {
        case RVC_MOTOR_STOP:
        case RVC_MOTOR_FORWARD:
        case RVC_MOTOR_LEFT:
        case RVC_MOTOR_RIGHT:
        case RVC_MOTOR_BACKWARD:
            break;
        default:
            return RVC_INVALID_ARGUMENT;
    }
    if (!self->device.write_motor) return RVC_INVALID_ARGUMENT;
    status = self->device.write_motor(self->device.context, command);
    if (status != RVC_OK) {
        /* 실패를 '이전 명령이 확실히 유지됨'으로 해석하지 않는다. */
        self->telemetry.motor_valid = false;
        return status;
    }
    self->telemetry.motor = command;
    self->telemetry.motor_valid = true;
    return RVC_OK;
}

/* 원래 모듈: src/interfaces/cleaner_interface.c */
/* Cleaner Interface (DFD 2.3): Set Cleaning과 실제/모의 청소 장치를 연결한다.
 * 호출: Set Cleaning -> 이 함수 -> device.write_cleaner 콜백.
 * 입력 self: 로봇과 장치 연결, command: OFF/ON/UP 중 하나.
 * 출력: 장치에 명령을 전달하고 성공하면 telemetry.cleaner와 cleaner_valid를 갱신한다.
 * UP은 정해진 강화 단계로 설정하는 절대 명령이다. 반복 호출해도 단계가 누적되지 않는다.
 * 반환: self/명령/콜백 오류는 INVALID_ARGUMENT(장치 호출 없음), 장치 오류는 그대로,
 * 성공은 OK. 콜백 성공은 이 인터페이스의 확인 기준이며 물리적 완료 검출은 별도다.
 * 장치 콜백 실패 시 이전 명령 수치는 남지만 cleaner_valid=false이므로 확정 출력으로 읽지 않는다.
 */
RvcStatus rvc_cleaner_apply(Rvc *self, RvcCleanerCommand command)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    switch (command) {
        case RVC_CLEANER_OFF:
        case RVC_CLEANER_ON:
        case RVC_CLEANER_UP:
            break;
        default:
            return RVC_INVALID_ARGUMENT;
    }
    if (!self->device.write_cleaner) return RVC_INVALID_ARGUMENT;
    status = self->device.write_cleaner(self->device.context, command);
    if (status != RVC_OK) {
        /* 장치가 명령을 일부 수행한 뒤 실패했을 수도 있어 실제 출력을 미확정으로 둔다. */
        self->telemetry.cleaner_valid = false;
        return status;
    }
    self->telemetry.cleaner = command;
    self->telemetry.cleaner_valid = true;
    return RVC_OK;
}


/* ========================================================================
 * 8. 모의 장치: 센서 입력과 명령 기록
 * ======================================================================== */

/* 원래 모듈: adapters/mock/mock_device.c */
/* RvcDevice 콜백의 모의 구현. 실제 하드웨어 대신 구조체를 읽고 성공한 출력 순서를 기록한다. */

/* 내부의 유효한 mock/operation만 받는다. 예약된 실패를 한 번 소비하면 true를 반환한다. */
static bool consume_fault(RvcMockDevice *mock, RvcMockOperation operation)
{
    if (mock->fault_remaining[operation] == 0) return false;
    --mock->fault_remaining[operation];
    return true;
}

/* 네 센서 콜백의 공통 처리. operation은 아래 래퍼가 READ 값으로 제한한다.
 * 유효한 호출은 시도 횟수를 먼저 증가시키고, 고장 주입 시 *out을 바꾸지 않고 IO_ERROR를 반환한다.
 * NULL 인수는 INVALID_ARGUMENT, 정상 읽기는 *out을 채운 뒤 OK다.
 */
static RvcStatus read_input(void *context, bool *out, RvcMockOperation operation)
{
    RvcMockDevice *mock = context;
    if (!mock || !out) return RVC_INVALID_ARGUMENT;
    ++mock->read_counts[operation];
    if (consume_fault(mock, operation)) return RVC_IO_ERROR;
    switch (operation) {
    case RVC_MOCK_READ_FRONT: *out = mock->inputs.obstacles.front_blocked; break;
    case RVC_MOCK_READ_LEFT: *out = mock->inputs.obstacles.left_blocked; break;
    case RVC_MOCK_READ_RIGHT: *out = mock->inputs.obstacles.right_blocked; break;
    case RVC_MOCK_READ_DUST: *out = mock->inputs.dust_detected; break;
    default: return RVC_INVALID_ARGUMENT;
    }
    return RVC_OK;
}
/* 전방 원시 센서 읽기. 초기화 후의 전방 변화 통지는 실행기/테스트가 별도로 전달한다. */
static RvcStatus read_front(void *context, bool *out)
{ return read_input(context, out, RVC_MOCK_READ_FRONT); }
/* 좌측 입력을 읽는다. 성공/실패 계약은 read_input과 같다. */
static RvcStatus read_left(void *context, bool *out)
{ return read_input(context, out, RVC_MOCK_READ_LEFT); }
/* 우측 입력을 읽는다. 성공/실패 계약은 read_input과 같다. */
static RvcStatus read_right(void *context, bool *out)
{ return read_input(context, out, RVC_MOCK_READ_RIGHT); }
/* 먼지 감지 입력을 읽는다. 청소 모드의 결정은 이 콜백의 책임이 아니다. */
static RvcStatus read_dust(void *context, bool *out)
{ return read_input(context, out, RVC_MOCK_READ_DUST); }

/* 모터 명령 적용. 인수 오류는 INVALID_ARGUMENT, 주입된 고장/로그 포화는 IO_ERROR다.
 * 모든 검사를 통과해야 성공 로그와 마지막 출력값을 함께 갱신한다. 실패 시 출력값은 그대로다.
 */
static RvcStatus write_motor(void *context, RvcMotorCommand command)
{
    RvcMockDevice *mock = context;
    if (!mock || command < RVC_MOTOR_STOP || command > RVC_MOTOR_BACKWARD)
        return RVC_INVALID_ARGUMENT;
    ++mock->write_counts[RVC_MOCK_MOTOR];
    if (consume_fault(mock, RVC_MOCK_WRITE_MOTOR)) return RVC_IO_ERROR;
    if (mock->event_count >= RVC_MOCK_MAX_EVENTS) return RVC_IO_ERROR;
    mock->events[mock->event_count++] = (RvcMockEvent){RVC_MOCK_MOTOR, command};
    mock->motor = command;
    mock->motor_valid = true;
    return RVC_OK;
}
/* 청소 명령 적용. write_motor와 같은 기록 규칙이며 OFF/ON/UP을 절대 모드로 저장한다. */
static RvcStatus write_cleaner(void *context, RvcCleanerCommand command)
{
    RvcMockDevice *mock = context;
    if (!mock || command < RVC_CLEANER_OFF || command > RVC_CLEANER_UP)
        return RVC_INVALID_ARGUMENT;
    ++mock->write_counts[RVC_MOCK_CLEANER];
    if (consume_fault(mock, RVC_MOCK_WRITE_CLEANER)) return RVC_IO_ERROR;
    if (mock->event_count >= RVC_MOCK_MAX_EVENTS) return RVC_IO_ERROR;
    mock->events[mock->event_count++] = (RvcMockEvent){RVC_MOCK_CLEANER, command};
    mock->cleaner = command;
    mock->cleaner_valid = true;
    return RVC_OK;
}

/* 장치 전체 초기화. STOP/OFF 값만 미리 두며, 실제 명령 성공 전에는 valid를 false로 유지한다. */
void rvc_mock_init(RvcMockDevice *mock)
{
    if (!mock) return;
    memset(mock, 0, sizeof(*mock));
    mock->motor = RVC_MOTOR_STOP;
    mock->cleaner = RVC_CLEANER_OFF;
}
/* C의 함수 포인터로 장치 인터페이스를 연결한다. context로 전달할 mock의 수명은 호출자가 관리한다. */
RvcDevice rvc_mock_device(RvcMockDevice *mock)
{
    RvcDevice device = {
        .context = mock,
        .read_front = read_front,
        .read_left = read_left,
        .read_right = read_right,
        .read_dust = read_dust,
        .write_motor = write_motor,
        .write_cleaner = write_cleaner
    };
    return device;
}
/* 다음 검사 구간을 위해 로그 길이만 비운다. 누적 시도 횟수와 현재 출력은 초기화하지 않는다. */
void rvc_mock_clear_events(RvcMockDevice *mock)
{
    if (mock) mock->event_count = 0;
}
/* 센서값 주입만 수행한다. Tick을 진행하거나 FSM을 직접 바꾸지 않는다. NULL은 무시한다. */
void rvc_mock_set_inputs(RvcMockDevice *mock, RvcSensorSnapshot inputs)
{
    if (mock) mock->inputs = inputs;
}
/* 지정 콜백의 앞으로 실패할 횟수를 설정한다. 테스트가 일시적/지속적 오류를 재현할 때 사용한다. */
void rvc_mock_fail_next(RvcMockDevice *mock, RvcMockOperation operation, size_t count)
{
    if (mock && operation >= RVC_MOCK_READ_FRONT && operation < RVC_MOCK_OPERATION_COUNT)
        mock->fault_remaining[operation] = count;
}


/* ========================================================================
 * 9. Main: 내장/CSV 입력, Tick 발생, 상태·Command 출력
 * ======================================================================== */

#ifndef RVC_NO_MAIN
/*
 * 실행 예제의 시작점: CSV(또는 내장 시나리오)를 모의 센서 입력으로 전달한다.
 * Main은 Tick을 발생시키고 결과를 출력하며, 상태 전이와 명령 결정은 Controller가 맡는다.
 * 기본 실행은 기다리지 않는 논리 시간 방식이다. --realtime-ms는 Tick 사이 대기만 추가한다.
 * 읽기 순서: main의 인수 해석 -> 첫 입력/초기화 -> Tick 반복 -> shutdown/메모리 해제.
 */

/* 벽시계 변경에 영향받지 않는 경과 시간(ms). POSIX 시계 조회 실패 시 프로세스를 종료한다. */
static uint64_t monotonic_ms(void)
{
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
    return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
#endif
}
/* deadline은 monotonic_ms와 같은 기준의 절대 시각이다. 도달할 때까지 대기한다.
 * 운영체제 스케줄링 때문에 늦게 깨어날 수 있으므로 정확한 실시간 주기를 보장하지 않는다.
 */
static void wait_until(uint64_t deadline)
{
    for (;;) {
        uint64_t now = monotonic_ms();
        if (now >= deadline) return;
        uint64_t remaining = deadline - now;
#ifdef _WIN32
        Sleep((DWORD)remaining);
#else
        struct timespec delay = {(time_t)(remaining / 1000U),
                                 (long)((remaining % 1000U) * 1000000U)};
        while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {}
#endif
    }
}
/* 상태 열거값을 CSV용 이름으로 변환한다. 범위 밖 값은 배열 접근 없이 INVALID로 표시한다. */
static const char *state_name(RvcState s)
{
    static const char *const names[] = {
        "UNINITIALIZED", "STOP_OFF", "F_ON", "F_POWER1",
        "LEFT_OFF", "RIGHT_OFF", "BACK_OFF"
    };
    return s >= RVC_STATE_UNINITIALIZED && s <= RVC_STATE_BACK_OFF
        ? names[(int)s] : "INVALID";
}
/* 모터 명령을 사람이 읽을 이름으로 변환한다. 반환 문자열은 해제할 필요가 없다. */
static const char *motor_name(int command)
{
    static const char *const names[] = {"STOP", "FORWARD", "LEFT", "RIGHT", "BACKWARD"};
    return command >= 0 && command < 5 ? names[command] : "INVALID";
}
/* 청소 명령 OFF/ON/UP을 출력용 문자열로 변환한다. UP은 강화 모드의 절대 설정이다. */
static const char *cleaner_name(int command)
{
    static const char *const names[] = {"OFF", "ON", "UP"};
    return command >= 0 && command < 3 ? names[command] : "INVALID";
}
/* 모의 장치에 성공적으로 기록된 명령을 실행 순서대로 출력한다. 실패한 시도는 포함되지 않는다. */
static void print_events(const RvcMockDevice *mock)
{
    for (size_t i = 0; i < mock->event_count; ++i) {
        const RvcMockEvent *event = &mock->events[i];
        if (i) putchar('|');
        if (event->kind == RVC_MOCK_MOTOR)
            printf("M:%s", motor_name(event->command));
        else
            printf("C:%s", cleaner_name(event->command));
    }
}
/* 성공한 초기화/Tick의 센서·상태·출력과 해당 구간의 명령 로그를 CSV 한 줄로 표시한다. */
static void print_frame(uint64_t tick, const RvcTelemetry *t, const RvcMockDevice *mock)
{
    printf("%" PRIu64 ",%d,%d,%d,%d,%s,%" PRIu32 ",%s,%s,",
           tick, t->sensors.obstacles.front_blocked,
           t->sensors.obstacles.left_blocked, t->sensors.obstacles.right_blocked,
           t->sensors.dust_detected, state_name(t->state), t->elapsed_ticks,
           motor_name(t->motor), cleaner_name(t->cleaner));
    print_events(mock);
    putchar('\n');
}
/* 다음 센서 행을 읽는다. file/out/line은 유효한 포인터를 전달하는 내부 호출용이다.
 * 반환: 1=정상 행, 0=파일 끝, -1=형식/읽기 오류. 정상일 때만 *out을 갱신한다.
 * 공백 행·# 주석·헤더는 건너뛰고, F/L/R/D에는 0 또는 1만 허용한다.
 * *line은 오류 위치 보고를 위해 읽은 실제 줄 수를 누적한다. 정상 데이터 한 행이 한 Tick이다.
 */
static int read_frame(FILE *file, RvcSensorSnapshot *out, unsigned *line)
{
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), file)) {
        ++*line;
        if (!strchr(buffer, '\n') && !feof(file)) return -1;
        buffer[strcspn(buffer, "\r\n")] = '\0';
        char *start = buffer;
        while (isspace((unsigned char)*start)) ++start;
        if (!*start || *start == '#') continue;
        if (strcmp(start, "front,left,right,dust") == 0) continue;
        char f, l, r, d, extra;
        if (sscanf(start, " %c , %c , %c , %c %c", &f, &l, &r, &d, &extra) != 4)
            return -1;
        if ((f != '0' && f != '1') || (l != '0' && l != '1') ||
            (r != '0' && r != '1') || (d != '0' && d != '1')) return -1;
        *out = (RvcSensorSnapshot){{f == '1', l == '1', r == '1'}, d == '1'};
        return 1;
    }
    return ferror(file) ? -1 : 0;
}
/* CLI 진입점. 성공은 EXIT_SUCCESS, 인수·파일·장치 처리 실패는 EXIT_FAILURE로 반환한다. */
int main(int argc, char **argv)
{
    /* 1. 실행 방법 결정: 시나리오를 생략하면 내장 입력, 주기를 생략하면 빠른 논리 Tick 실행. */
    const char *scenario = NULL;
    unsigned period_ms = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario = argv[++i];
        } else if (strcmp(argv[i], "--realtime-ms") == 0 && i + 1 < argc) {
            char *end = NULL;
            errno = 0;
            unsigned long value = strtoul(argv[++i], &end, 10);
            if (errno || !*argv[i] || *end || value == 0 || value > 60000) {
                fputs("Period must be an integer from 1 to 60000 ms.\n", stderr);
                return EXIT_FAILURE;
            }
            period_ms = (unsigned)value;
        } else if (strcmp(argv[i], "--help") == 0) {
            puts("rvc_demo [--scenario file.csv] [--realtime-ms 200]");
            puts("CSV: front,left,right,dust (0/1). Each row advances one Tick.");
            puts("No period: deterministic fast run. Default: built-in scenario.");
            return EXIT_SUCCESS;
        } else {
            fputs("Invalid argument. Use --help.\n", stderr);
            return EXIT_FAILURE;
        }
    }
    /* 전진/강화/정상 복귀/우회전/후진/좌회전/전진을 관찰하는 기본 입력 시퀀스. */
    static const RvcSensorSnapshot builtin[] = {
        {{0,0,0},0}, {{0,0,0},1}, {{0,0,0},0},
        {{1,0,0},1}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0},
        {{0,0,0},0}, {{0,0,0},0}, {{1,1,1},0},
        {{0,0,1},0}, {{0,0,1},0}, {{0,0,1},0},
        {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}
    };
    const size_t builtin_count = sizeof(builtin) / sizeof(builtin[0]);
    /* 2. 첫 행을 미리 읽는다. 이 입력은 초기 센서 읽기와 첫 번째 Tick 양쪽에 사용한다.
     * 초기화 결과는 tick=0으로 표시하지만, 초기화 자체가 논리 Tick을 소비하지는 않는다.
     */
    FILE *file = scenario ? fopen(scenario, "r") : NULL;
    if (scenario && !file) { perror("scenario"); return EXIT_FAILURE; }
    unsigned line = 0;
    size_t index = 0;
    RvcSensorSnapshot input = builtin[0];
    int available = file ? read_frame(file, &input, &line) : 1;
    if (available != 1) {
        fputs("Scenario is empty or its first row is invalid.\n", stderr);
        if (file) fclose(file);
        return EXIT_FAILURE;
    }
    /* 장치 콜백의 context가 이 mock을 가리키므로 robot을 쓰는 동안 mock의 수명이 유지되어야 한다. */
    RvcMockDevice mock;
    rvc_mock_init(&mock);
    rvc_mock_set_inputs(&mock, input);
    RvcDevice device = rvc_mock_device(&mock);
    Rvc *robot = rvc_create(&device);
    if (!robot) { if (file) fclose(file); return EXIT_FAILURE; }
    int exit_code = EXIT_SUCCESS;
    RvcStatus result = rvc_initialize(robot);
    if (result != RVC_OK) {
        fprintf(stderr, "Initialization failed: %d\n", (int)result);
        exit_code = EXIT_FAILURE;
    } else {
        RvcTelemetry telemetry;
        result = rvc_get_telemetry(robot, &telemetry);
        if (result != RVC_OK) {
            exit_code = EXIT_FAILURE;
        } else {
            puts("tick,F,L,R,D,state,elapsed,motor,cleaner,events");
            print_frame(0, &telemetry, &mock);
            uint64_t tick = 0;
            uint64_t deadline = period_ms ? monotonic_ms() + period_ms : 0;
            bool previous_front = input.obstacles.front_blocked;
            /* 3. 입력 한 행 -> 전방 변화 통지 -> Controller 한 번 실행 -> 결과 출력.
             * FSM의 elapsed_ticks는 rvc_tick 호출로만 증가하며, 실제 대기 시간은 판단에 넣지 않는다.
             */
            while (available == 1) {
                if (period_ms) wait_until(deadline);
                rvc_mock_clear_events(&mock);
                rvc_mock_set_inputs(&mock, input);
                /* 전방은 변화 통지 방식, 좌/우/먼지는 Tick 내부의 주기 읽기 방식으로 모사한다. */
                if (input.obstacles.front_blocked != previous_front) {
                    result = rvc_report_front(robot, input.obstacles.front_blocked);
                    previous_front = input.obstacles.front_blocked;
                }
                if (result == RVC_OK) result = rvc_tick(robot);
                if (result == RVC_OK) result = rvc_get_telemetry(robot, &telemetry);
                if (result != RVC_OK) {
                    fprintf(stderr, "Tick failed: %d\n", (int)result);
                    exit_code = EXIT_FAILURE;
                    break;
                }
                print_frame(++tick, &telemetry, &mock);
                if (period_ms) {
                    deadline += period_ms;
                    uint64_t now = monotonic_ms();
                    /* 처리가 지연됐으면 다음 대기 시각을 다시 잡는다. 누락 시간을 몰아서 Tick으로 만들지 않는다. */
                    if (deadline < now) deadline = now + period_ms;
                }
                if (file) available = read_frame(file, &input, &line);
                else if (++index < builtin_count) input = builtin[index];
                else available = 0;
                if (available < 0) {
                    fprintf(stderr, "Invalid scenario at line %u.\n", line);
                    exit_code = EXIT_FAILURE;
                }
            }
        }
    }
    /* 4. 이 공통 종료 경로로 들어온 정상 종료와 입력/장치 오류는 OFF와 STOP을 시도한다.
     * POSIX 시계 조회 자체의 치명적 실패는 monotonic_ms에서 즉시 exit하는 별도 경로다.
     * shutdown은 장치 종료, destroy는 메모리 해제다. destroy만 호출해도 정지되는 것은 아니다.
     */
    rvc_mock_clear_events(&mock);
    if (rvc_shutdown(robot) != RVC_OK) exit_code = EXIT_FAILURE;
    printf("# shutdown,");
    print_events(&mock);
    putchar('\n');
    rvc_destroy(robot);
    if (file) fclose(file);
    return exit_code;
}
#endif /* RVC_NO_MAIN */
