/* Team Project #2: Robotic Vacuum Cleaner — 데이터 및 함수 계약 (C17).
 * 구현: 같은 디렉터리의 controller.c. 외부 하드웨어 라이브러리 없이 실행 가능하다.
 * 입력: 장애물 F/L/R, 먼지 D 및 논리 Tick. 출력: Motor/Cleaner Command.
 * 센서값·조회값은 출력 매개변수로 전달하며, 반환값은 처리 성공·실패를 나타낸다.
 * 사용 순서: create -> initialize -> (report_front, tick, get_telemetry) -> shutdown -> destroy.
 * 한 인스턴스의 함수 호출은 직렬화한다. context의 소유권은 호출자에게 있다.
 */
#ifndef RVC_SINGLE_FILE_CONTROLLER_H
#define RVC_SINGLE_FILE_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* 처리 결과. 정상적인 센서 미감지(false)와 입출력 실패를 구분한다. */
typedef enum {
    RVC_OK = 0,
    RVC_INVALID_ARGUMENT = 1,
    RVC_NOT_READY = 2,
    RVC_IO_ERROR = 4
} RvcStatus;
/* 장애물 위치: true=감지, false=미감지. */
typedef struct {
    bool front_blocked;
    bool left_blocked;
    bool right_blocked;
} RvcObstacles;
/* 한 판단 주기에서 사용하는 센서 입력 묶음. dust_detected=true는 먼지 감지다. */
typedef struct {
    RvcObstacles obstacles;
    bool dust_detected;
} RvcSensorSnapshot;
/* Motor Interface의 장치 출력 명령. */
typedef enum {
    RVC_MOTOR_STOP, RVC_MOTOR_FORWARD, RVC_MOTOR_LEFT,
    RVC_MOTOR_RIGHT, RVC_MOTOR_BACKWARD
} RvcMotorCommand;
/* Cleaner Interface의 출력. UP은 강화 단계의 설정이며 누적 증가 연산이 아니다. */
typedef enum {
    RVC_CLEANER_OFF, RVC_CLEANER_ON, RVC_CLEANER_UP
} RvcCleanerCommand;
/* 동작 상태: F_ON/F_POWER1=전진 및 일반/강화 청소, 회피 상태에서는 청소 OFF.
 * UNINITIALIZED는 초기화 전·종료·장치 오류 후의 API 준비 상태다. */
typedef enum {
    RVC_STATE_UNINITIALIZED,
    RVC_STATE_STOP_OFF, RVC_STATE_F_ON, RVC_STATE_F_POWER1,
    RVC_STATE_LEFT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_BACK_OFF
} RvcState;
/* 확정된 상태와 출력의 조회용 복사본. 내부 상태를 수정하는 입력으로 사용하지 않는다. */
typedef struct {
    RvcState state;
    uint32_t elapsed_ticks; /* 회피 진입=0. 회전은 5 Tick, 후진은 빈 방향 발견 시 즉시 전환. */
    RvcSensorSnapshot sensors; /* 마지막 성공한 초기화/Tick에서 확정한 입력. */
    RvcMotorCommand motor;
    RvcCleanerCommand cleaner;
    bool initialized; /* 상태 조회는 준비된 인스턴스에서만 허용한다. */
    bool motor_valid; /* 마지막 모터 쓰기의 성공 여부. 실제 물리 동작의 측정값은 아니다. */
    bool cleaner_valid; /* 마지막 청소 쓰기의 성공 여부. false이면 남은 command 값은 미확정이다. */
} RvcTelemetry;

/* 장치 인터페이스 계약.
 * context: 연결한 장치의 데이터 주소. 호출자가 로봇 해제까지 유효하게 유지한다.
 * 읽기: 성공 시 *out을 갱신하고 RVC_OK 반환. 쓰기: command 적용 결과를 반환한다.
 * 실패한 장치 작업과 앞서 성공한 다른 작업의 원자적 취소는 보장하지 않는다. */
typedef RvcStatus (*RvcReadSensor)(void *context, bool *out);
typedef RvcStatus (*RvcWriteMotor)(void *context, RvcMotorCommand command);
typedef RvcStatus (*RvcWriteCleaner)(void *context, RvcCleanerCommand command);
/* 여섯 함수 포인터는 모두 필수다. 기본 실행은 C 파일의 모의 장치와 연결한다. */
typedef struct {
    void *context;
    RvcReadSensor read_front; /* 초기 읽기. 이후 변경은 rvc_report_front로 전달한다. */
    RvcReadSensor read_left;
    RvcReadSensor read_right;
    RvcReadSensor read_dust;
    RvcWriteMotor write_motor;
    RvcWriteCleaner write_cleaner;
} RvcDevice;

/* 인스턴스별 제어 상태. 내부 정의는 controller.c에서 관리한다. */
typedef struct Rvc Rvc;

/* 입력 device: 유효한 콜백 표. 표는 복사하되 context 데이터는 복사하지 않는다.
 * 반환: 새 인스턴스. 인자·콜백 오류 또는 할당 실패 시 NULL. 장치 입출력은 수행하지 않는다. */
Rvc *rvc_create(const RvcDevice *device);

/* 입력 self: 해제할 인스턴스(NULL 허용). 메모리만 해제하므로 shutdown을 먼저 호출한다. */
void rvc_destroy(Rvc *self);

/* 입력 self: 생성된 인스턴스. STOP -> OFF -> 센서 초기 입력 순으로 처리한다.
 * 반환: 성공 RVC_OK(STOP_OFF, e=0), NULL은 INVALID_ARGUMENT, 장치 실패는 해당 오류.
 * 초기화 실패 후에는 미준비 상태를 유지한다. */
RvcStatus rvc_initialize(Rvc *self);

/* 입력 self: 생성된 인스턴스. OFF와 STOP을 각각 시도한 뒤 미준비 상태로 전환한다.
 * 반환: 성공 RVC_OK, NULL은 INVALID_ARGUMENT, 출력 실패는 첫 오류. 정지 성공을 가정하지 않는다. */
RvcStatus rvc_shutdown(Rvc *self);

/* 입력 self: 초기화된 인스턴스. 한 호출마다 센서 판단·시간 처리·FSM·명령 출력을 수행한다.
 * 반환: 성공 RVC_OK, 미초기화 NOT_READY, NULL INVALID_ARGUMENT, 장치 실패는 해당 오류.
 * 장치 오류 시 OFF/STOP을 시도하고 재초기화를 요구한다. 함수 내부에서 실제 시간 대기는 하지 않는다. */
RvcStatus rvc_tick(Rvc *self);

/* 입력 self, blocked: 전방 장애물 감지 이벤트. 전방 캐시만 갱신하며 명령은 다음 Tick에서 결정한다.
 * 반환: 성공 RVC_OK, 미초기화 NOT_READY, NULL INVALID_ARGUMENT. */
RvcStatus rvc_report_front(Rvc *self, bool blocked);

/* 입력 self, 출력 out: 마지막 성공한 초기화/Tick의 확정 상태를 복사한다.
 * 반환: 성공 RVC_OK, 미준비 NOT_READY, NULL 인자 INVALID_ARGUMENT. 실패 시 *out은 보존한다. */
RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out);

#ifdef __cplusplus
}
#endif
#endif /* RVC_SINGLE_FILE_CONTROLLER_H */
