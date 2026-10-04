/* RvcDevice 콜백의 모의 구현. 실제 하드웨어 대신 구조체를 읽고 성공한 출력 순서를 기록한다. */
#include "mock_device.h"
#include <string.h>

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
