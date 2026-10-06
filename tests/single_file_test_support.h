/* 생성된 테스트 전용 선언. 일반 앱·두 파일 독립 실행에는 필요하지 않다.
 * 기존 모듈 직접 검사와 모의 장치 조작에만 사용한다.
 * 타입·선언은 controller.c의 내부 선언과 같은 원본에서 생성한다.
 */
#ifndef RVC_SINGLE_FILE_TEST_SUPPORT_H
#define RVC_SINGLE_FILE_TEST_SUPPORT_H
#include "controller.h"
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* 내부 제어 타입: Structured Chart의 Enable/Disable과 청소 모드. */
typedef enum { RVC_FORWARD_DISABLE, RVC_FORWARD_ENABLE } RvcForwardControl;
typedef enum { RVC_CLEANING_OFF, RVC_CLEANING_ON, RVC_CLEANING_POWER1 } RvcCleaningMode;

/* 내부 모듈: control */
RvcStatus rvc_controller_initialize(Rvc *self);
RvcStatus rvc_controller_tick(Rvc *self);
RvcStatus rvc_controller_shutdown(Rvc *self);

/* 내부 모듈: perception */
RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out);
RvcStatus rvc_determine_dust(Rvc *self, bool *out);

/* 내부 모듈: sensing */
RvcStatus rvc_front_sensor_initialize(Rvc *self);
RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked);
RvcStatus rvc_left_sensor_sample(Rvc *self);
RvcStatus rvc_right_sensor_sample(Rvc *self);
RvcStatus rvc_dust_sensor_sample(Rvc *self);

/* 내부 모듈: actions */
RvcStatus rvc_move_forward(Rvc *self, RvcForwardControl control);
RvcStatus rvc_turn_left(Rvc *self);
RvcStatus rvc_turn_right(Rvc *self);
RvcStatus rvc_move_backward(Rvc *self);
RvcStatus rvc_stop_motor(Rvc *self);
RvcStatus rvc_set_cleaning(Rvc *self, RvcCleaningMode mode);

/* 내부 모듈: interfaces */
RvcStatus rvc_motor_apply(Rvc *self, RvcMotorCommand command);
RvcStatus rvc_cleaner_apply(Rvc *self, RvcCleanerCommand command);

/* 모의 장치: 입력 설정, 명령 로그, 오류 주입. 상태 판단은 Controller가 한다.
 * 로그는 성공한 명령만 기록하고 counts는 실패한 시도도 포함한다. */
enum { RVC_MOCK_MAX_EVENTS = 1024 };
typedef enum { RVC_MOCK_MOTOR, RVC_MOCK_CLEANER } RvcMockEventKind;
typedef struct {
    RvcMockEventKind kind;
    int command;
} RvcMockEvent;
typedef enum {
    RVC_MOCK_READ_FRONT, RVC_MOCK_READ_LEFT, RVC_MOCK_READ_RIGHT,
    RVC_MOCK_READ_DUST, RVC_MOCK_WRITE_MOTOR, RVC_MOCK_WRITE_CLEANER,
    RVC_MOCK_OPERATION_COUNT
} RvcMockOperation;
typedef struct {
    RvcSensorSnapshot inputs;
    RvcMockEvent events[RVC_MOCK_MAX_EVENTS];
    size_t event_count;
    size_t read_counts[4];
    size_t write_counts[2];
    size_t fault_remaining[RVC_MOCK_OPERATION_COUNT];
    RvcMotorCommand motor;
    RvcCleanerCommand cleaner;
    bool motor_valid;
    bool cleaner_valid;
} RvcMockDevice;
void rvc_mock_init(RvcMockDevice *mock);
RvcDevice rvc_mock_device(RvcMockDevice *mock);
void rvc_mock_clear_events(RvcMockDevice *mock);
void rvc_mock_set_inputs(RvcMockDevice *mock, RvcSensorSnapshot inputs);
void rvc_mock_fail_next(RvcMockDevice *mock, RvcMockOperation operation, size_t count);

#ifdef __cplusplus
}
#endif
#endif /* RVC_SINGLE_FILE_TEST_SUPPORT_H */
