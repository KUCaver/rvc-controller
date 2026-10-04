#include "actions/actions.h"
#include "interfaces/interfaces.h"

/* Turn Right (DFD 2.1.4): Controller가 이 함수를 호출하는 행위가 Trigger 한 번이다.
 * 입력 self: 로봇 상태. 별도의 Trigger bool 매개변수는 필요하지 않다.
 * 출력: 공용 Motor Interface에 RIGHT를 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스의 성공/오류를 전달한다.
 * 5 Tick 회전 시간은 Controller가 센다. 이 함수는 즉시 반환하며, 우회전 우선순위나
 * 센서에 따른 다음 상태를 결정하지 않는다.
 */
RvcStatus rvc_turn_right(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_RIGHT);
}
