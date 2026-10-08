/*
 * RVC: Structured Design에 따른 로봇 청소기 제어 프로그램 (C17)
 * 구성: controller.c, controller.h. 외부 장치는 센서값과 명령 로그로 모사한다.
 * 빌드: gcc -std=c17 -Wall -Wextra -Wpedantic -Werror controller.c -o rvc.exe
 * 실행: ./rvc.exe                         (내장 입력 18개, 18 Tick)
 *       ./rvc.exe --scenario input.csv    (CSV 입력)
 *       ./rvc.exe --realtime-ms 200       (Tick 사이 200 ms 대기)
 * CSV: front,left,right,dust 순서의 0/1. 1은 장애물 또는 먼지 감지이다.
 * 데이터 한 행마다 rvc_tick을 한 번 호출한다. tick=0 출력은 초기화 결과이며,
 * 첫 데이터 행은 초기 센서 읽기와 tick=1에 함께 사용한다.
 * 결과: tick,F,L,R,D,state,elapsed,motor,cleaner,events. M:/C:는 출력 명령이다.
 * 상태 코드: OK=0, INVALID_ARGUMENT=1, NOT_READY=2, IO_ERROR=4.
 * CLI 종료 코드: 정상 종료 0, 인수/입력/제어 처리 오류 EXIT_FAILURE.
 *
 * Structured Chart의 모듈과 대응 함수:
 * Main -> main -> 공개 API -> Controller: rvc_controller_initialize/tick/shutdown
 * Determine Obstacle Location -> rvc_determine_obstacles
 *   Front Sensor Interface -> rvc_front_sensor_initialize/report
 *   Left/Right Sensor Interface -> rvc_left_sensor_sample/rvc_right_sensor_sample
 * Determine Dust Existence -> rvc_determine_dust -> rvc_dust_sensor_sample
 * Move Forward -> rvc_move_forward; Turn Left/Right -> rvc_turn_left/rvc_turn_right
 * Move Backward -> rvc_move_backward; Stop Motor -> rvc_stop_motor
 * 위 이동 모듈 -> Motor Interface: rvc_motor_apply -> write_motor 콜백
 * Set Cleaning -> rvc_set_cleaning -> Cleaner Interface: rvc_cleaner_apply
 *   -> write_cleaner 콜백. 센서값은 out 인자, 명령은 command 인자로 전달한다.
 *
 * 본 구현에서 채택한 FSM/시간 정책:
 * - 초기 상태 STOP_OFF: STOP, OFF 순서로 출력한 뒤 센서 입력을 확보한다.
 * - 정지/전진 판단: 앞이 열리면 먼지 없음 F_ON(FORWARD, ON),
 *   먼지 있음 F_POWER1(FORWARD, UP). UP은 강화 단계의 절대 설정이다.
 * - 앞이 막히면 우측 열림 RIGHT_OFF, 아니면 좌측 열림 LEFT_OFF,
 *   양쪽 모두 막힘 BACK_OFF. 각 상태의 명령은 RIGHT/LEFT/BACKWARD와 OFF이다.
 * - 회전은 5 Tick, 후진은 3 Tick 후 다시 판단한다. 진입 시 elapsed=0이며
 *   유지 Tick에는 센서를 갱신하되 기존 출력을 유지한다. 만료 시 최대 한 번 전이한다.
 * - 후진 만료 시에는 앞쪽과 무관하게 우회전 -> 좌회전 -> 재후진 순으로 선택한다.
 * - 전진 중 먼지가 사라지면 다음 판단에서 ON으로 복귀한다. 추가 유지 타이머는 없다.
 * - 회피 진입은 필요 시 청소 OFF -> 방향 명령, 전진 진입은 FORWARD -> ON/UP 순서이다.
 *   같은 전진 상태/출력은 재전송하지 않으며 회피 재진입은 새 방향 명령을 보낸다.
 * - Controller가 논리 Tick을 관리하고 Main이 호출 주기와 대기를 담당한다.
 *   Tick은 고정된 실제 초 단위가 아니며 이 정책은 본 프로그램의 구현 선택이다.
 * - 센서/출력 오류 시 OFF와 STOP을 각각 시도하고 재초기화 전까지 Tick을 거부한다.
 *   콜백 성공이 출력 확인 기준이며 실제 하드웨어 정지 완료를 검출하지는 않는다.
 *
 * RVC_NO_MAIN 정의 시 실행기와 모의 장치를 제외한 제어 모듈만 컴파일한다.
 */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#include "controller.h"
#include <stddef.h>
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

/* 1. 내부 모듈 선언과 제어 객체 */
typedef enum { RVC_FORWARD_DISABLE, RVC_FORWARD_ENABLE } RvcForwardControl;
typedef enum { RVC_CLEANING_OFF, RVC_CLEANING_ON, RVC_CLEANING_POWER1 } RvcCleaningMode;

RvcStatus rvc_controller_initialize(Rvc *self);
RvcStatus rvc_controller_tick(Rvc *self);
RvcStatus rvc_controller_shutdown(Rvc *self);

RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out);
RvcStatus rvc_determine_dust(Rvc *self, bool *out);

RvcStatus rvc_front_sensor_initialize(Rvc *self);
RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked);
RvcStatus rvc_left_sensor_sample(Rvc *self);
RvcStatus rvc_right_sensor_sample(Rvc *self);
RvcStatus rvc_dust_sensor_sample(Rvc *self);

RvcStatus rvc_move_forward(Rvc *self, RvcForwardControl control);
RvcStatus rvc_turn_left(Rvc *self);
RvcStatus rvc_turn_right(Rvc *self);
RvcStatus rvc_move_backward(Rvc *self);
RvcStatus rvc_stop_motor(Rvc *self);
RvcStatus rvc_set_cleaning(Rvc *self, RvcCleaningMode mode);

RvcStatus rvc_motor_apply(Rvc *self, RvcMotorCommand command);
RvcStatus rvc_cleaner_apply(Rvc *self, RvcCleanerCommand command);

#ifndef RVC_NO_MAIN
enum { RVC_MOCK_MAX_EVENTS = 1024 };
typedef enum { RVC_MOCK_MOTOR, RVC_MOCK_CLEANER } RvcMockEventKind;
typedef struct {
    RvcMockEventKind kind;
    int command;
} RvcMockEvent;
typedef struct {
    RvcSensorSnapshot inputs;
    RvcMockEvent events[RVC_MOCK_MAX_EVENTS];
    size_t event_count;
} RvcMockDevice;
#endif

/* 인스턴스 상태: Controller는 FSM/시간, 센서는 sampled, 출력 인터페이스는 출력 유효성을 관리한다. */
struct Rvc {
    RvcDevice device; /* 장치 콜백과 호출자 소유 context */
    RvcTelemetry telemetry; /* 성공한 제어 결과와 출력 유효성 */
    RvcSensorSnapshot sampled; /* 개별 센서 읽기 캐시 */

    bool front_valid; /* 전방 이벤트 캐시의 유효성 */
    bool forward_enabled; /* Structured Chart의 Enable/Disable 기록 */
};

/* 2. 공개 API */

/* 공개 API: device의 필수 콜백을 검사하고 독립 제어 객체를 생성한다.
 * 출력/반환: 객체 포인터, 인수 오류 또는 할당 실패 시 NULL. context는 호출자 소유이다. */
Rvc *rvc_create(const RvcDevice *device)
{
    if (!device || !device->read_front || !device->read_left ||
        !device->read_right || !device->read_dust ||
        !device->write_motor || !device->write_cleaner) {
        return NULL;
    }

    Rvc *self = calloc(1, sizeof(*self));
    if (self) {
        self->device = *device;
        self->telemetry.state = RVC_STATE_UNINITIALIZED;
    }
    return self;
}

/* 공개 API: self의 메모리를 해제한다(NULL 허용). 장치 정지는 rvc_shutdown이 담당한다. */
void rvc_destroy(Rvc *self)
{
    free(self);
}

/* 공개 API: self를 초기화한다. 초기 명령과 센서 처리 결과를 RvcStatus로 반환한다. */
RvcStatus rvc_initialize(Rvc *self)
{
    return rvc_controller_initialize(self);
}

/* 공개 API: self에 OFF/STOP을 요청하고 준비 상태를 해제한다. 종료 처리 상태를 반환한다. */
RvcStatus rvc_shutdown(Rvc *self)
{
    return rvc_controller_shutdown(self);
}

/* 공개 API: 초기화된 self에서 Controller를 한 번 실행한다. 한 호출이 한 논리 Tick이다. */
RvcStatus rvc_tick(Rvc *self)
{
    return rvc_controller_tick(self);
}

/* 공개 API: blocked(true=막힘)를 전방 센서 캐시에 반영한다.
 * 미초기화 시 NOT_READY. 전이와 출력은 다음 Tick에서 처리하며 호출은 Tick과 직렬화한다. */
RvcStatus rvc_report_front(Rvc *self, bool blocked)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;
    return rvc_front_sensor_report(self, blocked);
}

/* 공개 API: 초기화된 self의 상태를 *out에 복사한다.
 * 인수 오류는 INVALID_ARGUMENT, 미초기화는 NOT_READY. 실패 시 *out은 변경하지 않는다. */
RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out)
{
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;
    *out = self->telemetry;
    return RVC_OK;
}

/* 3. Controller 2.1.1: 상태 전이와 Command 결정 */
/* 회피 진입은 elapsed=0. 단위는 후속 논리 Tick 수이다. */
enum { TURN_TICKS = 5, REVERSE_TICKS = 3 };

/* 상태 분류: s가 전진 상태(F_ON 또는 F_POWER1)인지 반환한다. */
static bool is_forward(RvcState s)
{
    return s == RVC_STATE_F_ON || s == RVC_STATE_F_POWER1;
}

/* 상태 분류: s가 시간 유지가 필요한 회전/후진 상태인지 반환한다. */
static bool is_maneuver(RvcState s)
{
    return s == RVC_STATE_LEFT_OFF || s == RVC_STATE_RIGHT_OFF || s == RVC_STATE_BACK_OFF;
}

/* 입력 수집: 장애물 -> 먼지 순으로 호출하여 *input을 채운다.
 * 반환은 최초 실패 상태 또는 OK. 일부 읽기만 성공한 입력은 판단에 사용하지 않는다. */
static RvcStatus capture_inputs(Rvc *self, RvcSensorSnapshot *input)
{
    RvcStatus result = rvc_determine_obstacles(self, &input->obstacles);
    if (result != RVC_OK) return result;
    return rvc_determine_dust(self, &input->dust_detected);
}

/* Controller 종료: self에 청소 OFF와 모터 STOP을 각각 요청한다.
 * initialized=false, UNINITIALIZED, elapsed=0으로 변경한다.
 * 청소 실패를 우선 반환하고, 청소 성공 시 모터 결과를 반환한다. */
RvcStatus rvc_controller_shutdown(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;

    /* OFF 실패 시에도 STOP은 독립적으로 시도한다. */
    RvcStatus clean = rvc_set_cleaning(self, RVC_CLEANING_OFF);
    RvcStatus motor = rvc_stop_motor(self);
    self->telemetry.initialized = false;
    self->telemetry.state = RVC_STATE_UNINITIALIZED;
    self->telemetry.elapsed_ticks = 0;
    self->forward_enabled = false;
    return clean != RVC_OK ? clean : motor;
}

/* 오류 처리: OFF/STOP을 시도하고 최초 원인 cause를 반환한다. 추가 종료 오류로 덮지 않는다. */
static RvcStatus fail_closed(Rvc *self, RvcStatus cause)
{
    (void)rvc_controller_shutdown(self);
    return cause;
}

/* Controller 초기화: 캐시 무효화 -> STOP/OFF -> 센서 수집 -> STOP_OFF.
 * self 누락은 INVALID_ARGUMENT. 명령/센서 실패는 그 상태를 반환하고 미준비 상태를 유지한다. */
RvcStatus rvc_controller_initialize(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    self->telemetry.initialized = false;
    self->telemetry.state = RVC_STATE_UNINITIALIZED;
    self->telemetry.elapsed_ticks = 0;
    self->front_valid = false;
    self->forward_enabled = false;

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

/* FSM 조건 판단: current와 input으로 다음 상태를 반환하며 장치 출력은 수행하지 않는다.
 * 회피 유지 구간을 제외한 Tick에서 호출한다. 후진 만료는 우측 -> 좌측 -> 재후진 우선이다. */
static RvcState select_next_state(RvcState current, const RvcSensorSnapshot *input)
{
    const RvcObstacles *o = &input->obstacles;
    if (current == RVC_STATE_BACK_OFF || o->front_blocked) {
        if (!o->right_blocked) return RVC_STATE_RIGHT_OFF;
        if (!o->left_blocked) return RVC_STATE_LEFT_OFF;
        return RVC_STATE_BACK_OFF;
    }

    return input->dust_detected ? RVC_STATE_F_POWER1 : RVC_STATE_F_ON;
}

/* FSM 출력 연결: next에 대응하는 이동 모듈을 호출하고 그 처리 상태를 반환한다.
 * 회전/후진 함수 호출 자체가 Trigger이다. 논리 시간은 Controller, 실제 대기는 Main이 담당한다. */
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

/* 전이 출력: 현재 상태에서 next로 전환할 때 필요한 Command를 순서대로 전달한다.
 * 회피: 전진 Disable -> 필요 시 OFF -> 방향 명령. 전진: 필요 시 FORWARD -> ON/UP.
 * 최초 출력 실패를 반환한다. FSM 상태 확정과 오류 정지는 호출자가 수행한다. */
static RvcStatus apply_transition(Rvc *self, RvcState next)
{
    bool was_forward = is_forward(self->telemetry.state);
    bool now_forward = is_forward(next);
    RvcStatus result;
    if (was_forward && !now_forward) {

        result = rvc_move_forward(self, RVC_FORWARD_DISABLE);
        if (result != RVC_OK) return result;
    }
    if (!now_forward) {

        if (!self->telemetry.cleaner_valid || self->telemetry.cleaner != RVC_CLEANER_OFF) {
            result = rvc_set_cleaning(self, RVC_CLEANING_OFF);
            if (result != RVC_OK) return result;
        }
        return dispatch_motion(self, next);
    }
    if (!was_forward || !self->telemetry.motor_valid) {

        result = dispatch_motion(self, next);
        if (result != RVC_OK) return result;
    }

    RvcCleaningMode mode = next == RVC_STATE_F_POWER1 ? RVC_CLEANING_POWER1 : RVC_CLEANING_ON;
    RvcCleanerCommand command = next == RVC_STATE_F_POWER1 ? RVC_CLEANER_UP : RVC_CLEANER_ON;
    if (!self->telemetry.cleaner_valid || self->telemetry.cleaner != command)
        return rvc_set_cleaning(self, mode);
    return RVC_OK;
}

/* Controller Tick: 센서 수집 -> 유지 시간 검사 -> 조건 판단 -> Command -> 상태 확정.
 * 입력은 self의 상태와 센서값, 출력은 장치 명령과 telemetry이다.
 * 미준비 시 NOT_READY. 센서/명령 실패는 종료를 시도하고 원인 상태를 반환한다. */
RvcStatus rvc_controller_tick(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;

    RvcSensorSnapshot input = {0};
    RvcStatus result = capture_inputs(self, &input);
    if (result != RVC_OK) return fail_closed(self, result);
    RvcState current = self->telemetry.state;
    if (is_maneuver(current)) {

        /* 회전: 1~4 유지, 5에서 재판단. 후진: 1~2 유지, 3에서 재판단. */
        uint32_t elapsed = self->telemetry.elapsed_ticks + 1U;
        uint32_t duration = current == RVC_STATE_BACK_OFF ? REVERSE_TICKS : TURN_TICKS;
        if (elapsed < duration) {

            self->telemetry.elapsed_ticks = elapsed;
            self->telemetry.sensors = input;
            return RVC_OK;
        }
    } else if (current != RVC_STATE_STOP_OFF && !is_forward(current)) {
        return fail_closed(self, RVC_INVALID_ARGUMENT);
    }

    RvcState next = select_next_state(current, &input);
    result = apply_transition(self, next);
    if (result != RVC_OK) return fail_closed(self, result);

    /* 모든 출력이 성공한 뒤에만 상태를 확정하며 새 구간은 elapsed=0으로 시작한다. */
    self->telemetry.state = next;
    self->telemetry.elapsed_ticks = 0;
    self->telemetry.sensors = input;
    return RVC_OK;
}

/* 4. 입력 판단 모듈 */

/* Determine Obstacle Location: self의 Front/Left/Right 관측값을 *out으로 전달한다.
 * Front는 초기 읽기 후 이벤트 캐시, Left/Right는 매 호출마다 읽는다.
 * 인수 오류 또는 센서 오류를 반환하며, 모든 읽기가 성공해야 *out을 갱신한다. */
RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out)
{
    RvcStatus status;
    if (!self || !out) return RVC_INVALID_ARGUMENT;

    if (!self->front_valid) {
        status = rvc_front_sensor_initialize(self);
        if (status != RVC_OK) return status;
    }
    status = rvc_left_sensor_sample(self);
    if (status != RVC_OK) return status;
    status = rvc_right_sensor_sample(self);
    if (status != RVC_OK) return status;

    *out = self->sampled.obstacles;
    return RVC_OK;
}

/* Determine Dust Existence: Dust Sensor Interface를 호출해 *out에 감지 여부를 저장한다.
 * self/out 누락은 INVALID_ARGUMENT, 센서 실패는 해당 상태. 성공 시에만 *out을 변경한다. */
RvcStatus rvc_determine_dust(Rvc *self, bool *out)
{
    RvcStatus status;
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    status = rvc_dust_sensor_sample(self);
    if (status != RVC_OK) return status;
    *out = self->sampled.dust_detected;
    return RVC_OK;
}

/* 5. 센서 인터페이스 */

/* Front Sensor Interface 초기 읽기: read_front의 결과로 전방 캐시와 유효성을 갱신한다.
 * self/콜백 누락은 INVALID_ARGUMENT. 장치 오류 시 캐시는 무효 상태이며 오류를 반환한다. */
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

/* Front Sensor Interface 이벤트: blocked를 캐시에 저장하고 유효 상태로 만든다.
 * 장치를 읽거나 Tick을 진행하지 않는다. self 누락은 INVALID_ARGUMENT, 갱신 성공은 OK. */
RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    self->sampled.obstacles.front_blocked = blocked;
    self->front_valid = true;
    return RVC_OK;
}

/* Left Sensor Interface: self의 read_left를 호출해 성공한 값만 좌측 캐시에 저장한다.
 * self/콜백 누락은 INVALID_ARGUMENT, 반환은 장치 처리 상태이다. */
RvcStatus rvc_left_sensor_sample(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->device.read_left) return RVC_INVALID_ARGUMENT;
    status = self->device.read_left(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.left_blocked = blocked;
    return RVC_OK;
}

/* Right Sensor Interface: self의 read_right를 호출해 성공한 값만 우측 캐시에 저장한다.
 * self/콜백 누락은 INVALID_ARGUMENT, 반환은 장치 처리 상태이다. */
RvcStatus rvc_right_sensor_sample(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->device.read_right) return RVC_INVALID_ARGUMENT;
    status = self->device.read_right(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.right_blocked = blocked;
    return RVC_OK;
}

/* Dust Sensor Interface: self의 read_dust를 호출해 성공한 값만 먼지 캐시에 저장한다.
 * self/콜백 누락은 INVALID_ARGUMENT, 반환은 장치 처리 상태이다. false는 정상적인 미감지이다. */
RvcStatus rvc_dust_sensor_sample(Rvc *self)
{
    bool detected = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->device.read_dust) return RVC_INVALID_ARGUMENT;
    status = self->device.read_dust(self->device.context, &detected);
    if (status != RVC_OK) return status;
    self->sampled.dust_detected = detected;
    return RVC_OK;
}

/* 6. 동작 모듈 */

/* Move Forward: control=Enable이면 Motor Interface에 FORWARD를 전달한다.
 * Disable은 활성 기록만 해제하며 STOP을 출력하지 않는다. 다음 방향은 Controller가 지시한다.
 * self/control 오류는 INVALID_ARGUMENT, 그 외에는 처리 상태를 반환한다. */
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

/* Turn Left: self의 Motor Interface에 LEFT를 한 번 전달하는 Trigger이다.
 * self 누락은 INVALID_ARGUMENT, 반환은 출력 처리 상태. 5 Tick 유지는 Controller가 수행한다. */
RvcStatus rvc_turn_left(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_LEFT);
}

/* Turn Right: self의 Motor Interface에 RIGHT를 한 번 전달하는 Trigger이다.
 * self 누락은 INVALID_ARGUMENT, 반환은 출력 처리 상태. 5 Tick 유지는 Controller가 수행한다. */
RvcStatus rvc_turn_right(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_RIGHT);
}

/* Move Backward: self의 Motor Interface에 BACKWARD를 한 번 전달하는 Trigger이다.
 * self 누락은 INVALID_ARGUMENT, 반환은 출력 처리 상태. 3 Tick 유지는 Controller가 수행한다. */
RvcStatus rvc_move_backward(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_BACKWARD);
}

/* Stop Motor: self의 Motor Interface에 STOP을 전달하고 성공 시 전진 활성 기록을 해제한다.
 * self 누락은 INVALID_ARGUMENT, 반환은 출력 처리 상태. 청소 OFF는 별도 호출로 수행한다. */
RvcStatus rvc_stop_motor(Rvc *self)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    status = rvc_motor_apply(self, RVC_MOTOR_STOP);
    if (status == RVC_OK) self->forward_enabled = false;
    return status;
}

/* Set Cleaning: mode(OFF/ON/POWER1)를 Cleaner Interface의 OFF/ON/UP 명령으로 변환한다.
 * self/mode 오류는 INVALID_ARGUMENT, 반환은 출력 처리 상태. UP은 절대 강화 단계이다. */
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

/* 7. 출력 인터페이스 */

/* Motor Interface: command를 검증하고 self의 write_motor 콜백으로 전달한다.
 * 성공하면 telemetry.motor를 확정하고, 콜백 실패 시 motor_valid=false로 표시한다.
 * 인수/콜백 오류는 INVALID_ARGUMENT, 나머지는 장치 처리 상태를 반환한다. */
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

        self->telemetry.motor_valid = false;
        return status;
    }
    self->telemetry.motor = command;
    self->telemetry.motor_valid = true;
    return RVC_OK;
}

/* Cleaner Interface: command를 검증하고 self의 write_cleaner 콜백으로 전달한다.
 * 성공하면 telemetry.cleaner를 확정하고, 콜백 실패 시 cleaner_valid=false로 표시한다.
 * 인수/콜백 오류는 INVALID_ARGUMENT, 나머지는 장치 처리 상태를 반환한다. */
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

        self->telemetry.cleaner_valid = false;
        return status;
    }
    self->telemetry.cleaner = command;
    self->telemetry.cleaner_valid = true;
    return RVC_OK;
}

/* 8. 하드웨어 에뮬레이션: 센서 입력과 명령 기록 */
#ifndef RVC_NO_MAIN
/* 전방 센서 모사: context의 현재 전방 입력을 *out에 저장한다. 누락 인수는 INVALID_ARGUMENT. */
static RvcStatus read_front(void *context, bool *out)
{
    RvcMockDevice *mock = context;
    if (!mock || !out) return RVC_INVALID_ARGUMENT;
    *out = mock->inputs.obstacles.front_blocked;
    return RVC_OK;
}

/* 좌측 센서 모사: context의 현재 좌측 입력을 *out에 저장한다. 누락 인수는 INVALID_ARGUMENT. */
static RvcStatus read_left(void *context, bool *out)
{
    RvcMockDevice *mock = context;
    if (!mock || !out) return RVC_INVALID_ARGUMENT;
    *out = mock->inputs.obstacles.left_blocked;
    return RVC_OK;
}

/* 우측 센서 모사: context의 현재 우측 입력을 *out에 저장한다. 누락 인수는 INVALID_ARGUMENT. */
static RvcStatus read_right(void *context, bool *out)
{
    RvcMockDevice *mock = context;
    if (!mock || !out) return RVC_INVALID_ARGUMENT;
    *out = mock->inputs.obstacles.right_blocked;
    return RVC_OK;
}

/* 먼지 센서 모사: context의 현재 먼지 입력을 *out에 저장한다. 누락 인수는 INVALID_ARGUMENT. */
static RvcStatus read_dust(void *context, bool *out)
{
    RvcMockDevice *mock = context;
    if (!mock || !out) return RVC_INVALID_ARGUMENT;
    *out = mock->inputs.dust_detected;
    return RVC_OK;
}

/* 모터 출력 모사: command를 순서대로 기록한다. 인수 오류는 INVALID_ARGUMENT, 로그 포화는 IO_ERROR. */
static RvcStatus write_motor(void *context, RvcMotorCommand command)
{
    RvcMockDevice *mock = context;
    if (!mock || command < RVC_MOTOR_STOP || command > RVC_MOTOR_BACKWARD)
        return RVC_INVALID_ARGUMENT;
    if (mock->event_count >= RVC_MOCK_MAX_EVENTS) return RVC_IO_ERROR;
    mock->events[mock->event_count++] = (RvcMockEvent){RVC_MOCK_MOTOR, command};
    return RVC_OK;
}

/* 청소 출력 모사: command를 순서대로 기록한다. 인수 오류는 INVALID_ARGUMENT, 로그 포화는 IO_ERROR. */
static RvcStatus write_cleaner(void *context, RvcCleanerCommand command)
{
    RvcMockDevice *mock = context;
    if (!mock || command < RVC_CLEANER_OFF || command > RVC_CLEANER_UP)
        return RVC_INVALID_ARGUMENT;
    if (mock->event_count >= RVC_MOCK_MAX_EVENTS) return RVC_IO_ERROR;
    mock->events[mock->event_count++] = (RvcMockEvent){RVC_MOCK_CLEANER, command};
    return RVC_OK;
}

/* 모의 장치 초기화: 입력값과 명령 로그를 0으로 초기화한다. */
static void rvc_mock_init(RvcMockDevice *mock)
{
    if (mock) memset(mock, 0, sizeof(*mock));
}

/* 장치 연결: mock을 context로 사용하는 센서/출력 콜백 묶음을 반환한다. */
static RvcDevice rvc_mock_device(RvcMockDevice *mock)
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

/* 명령 로그 초기화: 이전 출력 구간의 로그를 비우며 센서 입력은 유지한다. */
static void rvc_mock_clear_events(RvcMockDevice *mock)
{
    if (mock) mock->event_count = 0;
}

/* 센서 입력 설정: inputs를 저장한다. 전방 변화는 Main이 rvc_report_front로 별도 통지한다. */
static void rvc_mock_set_inputs(RvcMockDevice *mock, RvcSensorSnapshot inputs)
{
    if (mock) mock->inputs = inputs;
}

/* 9. Main 실행기: 시나리오, Tick 발생, 결과 출력 */

/* 실행기 시간: 단조 증가 시계의 밀리초 값을 반환한다. POSIX 조회 실패 시 프로세스를 종료한다. */
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

/* 실행기 대기: monotonic_ms 기준 deadline까지 기다린다. 운영체제 수준의 엄밀한 실시간 보장은 없다. */
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

/* 출력 형식: 상태를 CSV 문자열로 변환하며 범위 밖 값은 INVALID로 표시한다. */
static const char *state_name(RvcState s)
{
    static const char *const names[] = {
        "UNINITIALIZED", "STOP_OFF", "F_ON", "F_POWER1",
        "LEFT_OFF", "RIGHT_OFF", "BACK_OFF"
    };
    return s >= RVC_STATE_UNINITIALIZED && s <= RVC_STATE_BACK_OFF
        ? names[(int)s] : "INVALID";
}

/* 출력 형식: 모터 명령을 문자열로 변환하며 범위 밖 값은 INVALID로 표시한다. */
static const char *motor_name(int command)
{
    static const char *const names[] = {"STOP", "FORWARD", "LEFT", "RIGHT", "BACKWARD"};
    return command >= 0 && command < 5 ? names[command] : "INVALID";
}

/* 출력 형식: 청소 명령을 문자열로 변환하며 범위 밖 값은 INVALID로 표시한다. */
static const char *cleaner_name(int command)
{
    static const char *const names[] = {"OFF", "ON", "UP"};
    return command >= 0 && command < 3 ? names[command] : "INVALID";
}

/* 출력 로그: 기록된 명령을 전달 순서대로 M:명령 또는 C:명령으로 표시한다. */
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

/* 결과 출력: tick, 센서값, 상태, 경과 Tick, 최종 출력, 명령 로그를 CSV 한 행으로 표시한다. */
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

/* CSV 입력: 정상 데이터 행을 *out에 저장하고 실제 줄 수를 *line에 누적한다.
 * 반환 1=데이터, 0=EOF, -1=형식/읽기 오류. 성공 시에만 *out을 변경한다.
 * 공백 행, # 주석, 정확한 헤더는 건너뛰며 네 필드는 각각 0 또는 1이어야 한다. */
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

/* Main: 인수 해석 -> 첫 입력/장치 초기화 -> 행별 Tick/결과 출력 -> OFF/STOP/자원 해제.
 * 정상 EOF는 성공 종료한다. 인수·파일·장치 오류는 EXIT_FAILURE를 반환한다.
 * 기본 실행은 내장 18개 입력을 사용하며 --realtime-ms는 Tick 사이 대기만 추가한다. */
int main(int argc, char **argv)
{

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

    /* 전진, 청소 강화, 우회전, 후진, 좌회전과 복귀를 관찰하는 입력 시나리오. */
    static const RvcSensorSnapshot builtin[] = {
        {{0,0,0},0}, {{0,0,0},1}, {{0,0,0},0},
        {{1,0,0},1}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0},
        {{0,0,0},0}, {{0,0,0},0}, {{1,1,1},0},
        {{0,0,1},0}, {{0,0,1},0}, {{0,0,1},0},
        {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}
    };
    const size_t builtin_count = sizeof(builtin) / sizeof(builtin[0]);

    /* 첫 입력은 초기화와 첫 Tick에서 함께 사용한다. 초기화 자체는 Tick을 소비하지 않는다. */
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

            while (available == 1) {
                if (period_ms) wait_until(deadline);
                rvc_mock_clear_events(&mock);
                rvc_mock_set_inputs(&mock, input);

                /* 전방은 변화 통지, 좌/우/먼지는 Controller의 주기 읽기로 전달한다. */
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

                    /* 지연된 대기 시간을 여러 Tick으로 소급 처리하지 않는다. */
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

    /* 공통 종료 경로: 정상 EOF 및 입력/제어 오류에 대해 OFF/STOP을 요청한다. */
    rvc_mock_clear_events(&mock);
    if (rvc_shutdown(robot) != RVC_OK) exit_code = EXIT_FAILURE;
    printf("# shutdown,");
    print_events(&mock);
    putchar('\n');
    rvc_destroy(robot);
    if (file) fclose(file);
    return exit_code;
}
#endif
