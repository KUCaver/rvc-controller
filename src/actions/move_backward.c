#include "actions/actions.h"
#include "interfaces/interfaces.h"

/* Move Backward (DFD 2.1.5): Controller가 호출하면 Trigger 한 번으로 처리한다.
 * 입력 self: 로봇 상태. 출력: 공용 Motor Interface에 BACKWARD를 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스의 성공/오류를 전달한다.
 * 3 Tick 유지, 후진 구간 재시작, 후진 뒤 회전 선택은 모두 Controller의 책임이다.
 * 여기에는 대기나 센서 조건문이 없으므로 동작 모듈과 FSM의 역할이 겹치지 않는다.
 */
RvcStatus rvc_move_backward(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_BACKWARD);
}
