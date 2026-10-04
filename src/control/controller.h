/* Controller 2.1.1의 내부 진입점. 세부 FSM과 명령 순서는 controller.c에 있다. */
#ifndef RVC_CONTROL_CONTROLLER_H
#define RVC_CONTROL_CONTROLLER_H
#include "rvc/rvc.h"
#ifdef __cplusplus
extern "C" {
#endif

/* src/rvc.c가 호출한다. 앱은 공개 rvc/rvc.h를 사용한다.
 * 모든 함수는 자기 객체 self를 받고 RvcStatus로 성공/실패를 반환한다.
 * initialize: STOP/OFF → 센서 준비 → STOP_OFF. 실패하면 Tick 처리 불가.
 */
RvcStatus rvc_controller_initialize(Rvc *self);

/* tick: 상태·시간을 소유하며, 입력 한 묶음으로 최대 한 번 전이하고 필요한 명령을 호출한다. */
RvcStatus rvc_controller_tick(Rvc *self);
/* shutdown: OFF와 STOP 모두 시도 → 준비 해제. 장치 오류가 있어도 준비를 해제한다. */
RvcStatus rvc_controller_shutdown(Rvc *self);
#ifdef __cplusplus
}
#endif
#endif
