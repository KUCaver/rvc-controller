/* Controller 2.1.1: FSM 조건과 출력 Command를 실제 모듈 호출로 연결하는 곳.
 * 입력: Determine 두 모듈의 센서값 + 현재 상태/경과 Tick.
 * 처리: 입력 읽기 → 회피 시간 확인 → 다음 상태 선택 → 명령 실행 → 성공 상태 확정.
 * 출력: 동작 모듈을 통한 모터/청소 명령과 RvcStatus. 명령을 반환값으로 돌려주지 않는다.
 * 이 파일만 방향·먼지 정책과 회전/후진 시간을 결정한다. 실제 대기는 Main의 책임이다.
 * 전체 전이표: docs/06_implemented_design.md. 검증: tests/rvc_tests.cpp.
 */
#include "control/controller.h"
#include "internal/rvc_internal.h"
#include "perception/perception.h"
#include "actions/actions.h"

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
    self->front_valid = false;
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
