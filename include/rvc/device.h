/* 장치 교체 지점: FSM은 실제 센서/모터 대신 이 함수 포인터 표를 사용한다.
 * 현재는 adapters/mock이 연결되며, 같은 계약을 지키는 실제 장치 어댑터로 교체할 수 있다.
 */
#ifndef RVC_DEVICE_H
#define RVC_DEVICE_H
#include "rvc/types.h"
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
#endif
