/* RVC 공개 헤더: controller.c와 같은 폴더에 놓는다.
 * 센서·상태·명령 타입, 장치 연결, 외부에서 사용하는 함수 7개만 선언한다.
 * 내부 모듈과 모의 장치는 controller.c에서 정의한다.
 * 센서와 조회 결과는 out 인자, 성공/실패는 반환값으로 구분한다.
 * initialize 이후 report_front/tick/get_telemetry를 한 실행 흐름에서 직렬 호출한다.
 * Tick 입출력 오류 후에는 재초기화가 필요하며 실제 장치 정지 성공을 보장하지 않는다.
 */
#ifndef RVC_SINGLE_FILE_CONTROLLER_H
#define RVC_SINGLE_FILE_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* 반환 상태: OK 또는 인자/준비/장치 오류. 센서 true/false는 감지/미감지다.
 * CLEANER_UP은 강화 단계 설정이며, 반복 호출마다 세기를 더하는 명령이 아니다.
 * UNINITIALIZED는 초기화 전·종료·오류 후의 준비 상태를 뜻한다. */
typedef enum {
    RVC_OK = 0,
    RVC_INVALID_ARGUMENT,
    RVC_NOT_READY,
    RVC_IO_ERROR = 4 /* 사용하지 않는 예약 값을 제거하되 기존 오류 번호는 유지한다. */
} RvcStatus;
typedef struct {
    bool front_blocked;
    bool left_blocked;
    bool right_blocked;
} RvcObstacles;
typedef struct {
    RvcObstacles obstacles;
    bool dust_detected;
} RvcSensorSnapshot;
typedef enum {
    RVC_MOTOR_STOP, RVC_MOTOR_FORWARD, RVC_MOTOR_LEFT,
    RVC_MOTOR_RIGHT, RVC_MOTOR_BACKWARD
} RvcMotorCommand;
typedef enum {
    RVC_CLEANER_OFF, RVC_CLEANER_ON, RVC_CLEANER_UP
} RvcCleanerCommand;
typedef enum {
    RVC_STATE_UNINITIALIZED,
    RVC_STATE_STOP_OFF, RVC_STATE_F_ON, RVC_STATE_F_POWER1,
    RVC_STATE_LEFT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_BACK_OFF
} RvcState;
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

/* 장치 연결: 콜백 6개는 필수. context는 호출자가 소유하며 로봇 해제까지 유지한다.
 * 읽기는 성공할 때만 *out을 갱신하고, 쓰기는 명령 적용 결과를 반환한다. */
typedef RvcStatus (*RvcReadSensor)(void *context, bool *out);
typedef RvcStatus (*RvcWriteMotor)(void *context, RvcMotorCommand command);
typedef RvcStatus (*RvcWriteCleaner)(void *context, RvcCleanerCommand command);
typedef struct {
    void *context;
    RvcReadSensor read_front; /* 초기 읽기. 이후 변경은 rvc_report_front로 전달한다. */
    RvcReadSensor read_left;
    RvcReadSensor read_right;
    RvcReadSensor read_dust;
    RvcWriteMotor write_motor;
    RvcWriteCleaner write_cleaner;
} RvcDevice;

/* 내부 상태를 감춘 객체. create → initialize → tick 반복 → shutdown → destroy. */
typedef struct Rvc Rvc;

/* 콜백 표를 복사해 객체를 만든다. 실패 NULL; 아직 장치 입출력은 없다. */
Rvc *rvc_create(const RvcDevice *device);

/* 메모리만 해제한다(NULL 허용). 장치 종료는 먼저 shutdown으로 요청한다. */
void rvc_destroy(Rvc *self);

/* STOP과 OFF를 시도하고 센서를 준비한다. 성공 시 STOP_OFF, 실패 시 미준비. */
RvcStatus rvc_initialize(Rvc *self);

/* OFF와 STOP을 각각 시도한 뒤 미준비 상태로 만든다. 실패하면 첫 오류 반환. */
RvcStatus rvc_shutdown(Rvc *self);

/* 호출 한 번이 논리 Tick 하나. 센서 판단·FSM·명령 실행; 실제 대기는 하지 않는다. */
RvcStatus rvc_tick(Rvc *self);

/* 전방 감지 캐시만 갱신한다. Tick과 직렬 호출하며 명령은 다음 Tick에서 결정한다. */
RvcStatus rvc_report_front(Rvc *self, bool blocked);

/* 마지막 완료 상태를 *out에 복사한다. 오류/종료 후 NOT_READY이며 *out은 유지한다. */
RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out);

#ifdef __cplusplus
}
#endif
#endif /* RVC_SINGLE_FILE_CONTROLLER_H */
