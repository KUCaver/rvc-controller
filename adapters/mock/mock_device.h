/* 모의 하드웨어의 공개 계약. 센서 입력·명령 기록·고장 주입을 제공하고 FSM 판단은 하지 않는다. */
#ifndef RVC_MOCK_DEVICE_H
#define RVC_MOCK_DEVICE_H
#include <stddef.h>
#include "rvc/device.h"
#ifdef __cplusplus
extern "C" {
#endif
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
#endif
