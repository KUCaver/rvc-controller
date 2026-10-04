#include "actions/actions.h"
#include "interfaces/interfaces.h"

/* Turn Left (DFD 2.1.3): Controller가 이 함수를 호출하는 행위가 Trigger 한 번이다.
 * 입력 self: 로봇 상태. 별도의 Trigger bool 매개변수는 필요하지 않다.
 * 출력: 공용 Motor Interface에 LEFT를 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스의 성공/오류를 전달한다.
 * 회전 완료를 기다리거나 여기서 Tick을 세지 않는다. 5 Tick 유지와 다음 동작 결정은
 * Controller가 담당하므로 대기 중에도 매 Tick에 센서를 확인할 수 있다.
 */
RvcStatus rvc_turn_left(Rvc *self)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    return rvc_motor_apply(self, RVC_MOTOR_LEFT);
}
