/* 공통 데이터 사전: 모든 모듈이 공유하는 센서값, 상태, 명령, 처리 결과.
 * DFD의 데이터 화살표는 아래 구조체/enum으로 표현하며, 판단 로직은 넣지 않는다.
 */
#ifndef RVC_TYPES_H
#define RVC_TYPES_H
#include <stdbool.h>
#include <stdint.h>
/* 함수 반환값은 처리 성공 여부다. 센서값/상태 정보는 별도 out 인자로 받는다. */
typedef enum {
    RVC_OK = 0,
    RVC_INVALID_ARGUMENT,
    RVC_NOT_READY,
    RVC_IO_ERROR = 4 /* 사용하지 않는 예약 값을 제거하되 기존 오류 번호는 유지한다. */
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
#endif
