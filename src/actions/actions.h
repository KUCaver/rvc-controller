#ifndef RVC_ACTIONS_ACTIONS_H
#define RVC_ACTIONS_ACTIONS_H
#include "rvc/rvc.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Structured Chart의 동작 모듈 계약. Controller가 호출하며 앱은 rvc/rvc.h를 사용한다.
 * self는 로봇 인스턴스이며 NULL이면 INVALID_ARGUMENT. 출력 매개변수는 없고,
 * 하위 Motor/Cleaner Interface 호출이 장치 출력에 해당한다. RvcStatus는 성공/실패다.
 * 장치 출력의 오류는 호출자에게 그대로 전달한다. Timer, 센서 판단, FSM은 Controller 소유다.
 * Turn/Backward/Stop의 Trigger는 함수 호출 자체로 표현하며 별도 bool을 받지 않는다.
 */
/* DFD 2.1.2: Enable은 FORWARD 출력, Disable은 활성 해제만 수행한다(정지 출력 없음).
 * control이 두 값 이외이면 INVALID_ARGUMENT. Enable 성공 시에만 활성 플래그를 설정한다.
 */
RvcStatus rvc_move_forward(Rvc *self, RvcForwardControl control);

/* DFD 2.1.3: Trigger 한 번에 LEFT 명령 한 번. 유지시간은 Controller가 관리한다. */
RvcStatus rvc_turn_left(Rvc *self);

/* DFD 2.1.4: Trigger 한 번에 RIGHT 명령 한 번. 유지시간은 Controller가 관리한다. */
RvcStatus rvc_turn_right(Rvc *self);

/* DFD 2.1.5: Trigger 한 번에 BACKWARD 명령 한 번. 완료 후 동작은 Controller가 결정한다. */
RvcStatus rvc_move_backward(Rvc *self);

/* DFD 2.1.6: STOP을 출력하고 성공 시 전진 활성 플래그를 해제한다. 청소 출력은 별개다. */
RvcStatus rvc_stop_motor(Rvc *self);

/* DFD 2.1.7: OFF/ON/POWER1을 OFF/ON/UP으로 변환한다. UP은 강화 단계의 절대 설정이다.
 * mode가 세 값 이외이면 INVALID_ARGUMENT이며 장치 출력은 수행하지 않는다.
 */
RvcStatus rvc_set_cleaning(Rvc *self, RvcCleaningMode mode);
#ifdef __cplusplus
}
#endif
#endif
