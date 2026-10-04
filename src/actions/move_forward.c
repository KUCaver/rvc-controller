#include "actions/actions.h"
#include "internal/rvc_internal.h"
#include "interfaces/interfaces.h"

/* Move Forward (DFD 2.1.2): Controller의 Enable/Disable 제어를 처리한다.
 * 입력 self: 로봇 상태, control: RVC_FORWARD_ENABLE 또는 RVC_FORWARD_DISABLE.
 * Enable 출력: Motor Interface에 FORWARD를 보내고, 성공한 경우 활성 플래그를 설정한다.
 * Disable 출력: 활성 플래그만 해제한다. 장치의 기존 모터 명령은 그대로 남는다.
 * 따라서 Disable은 정지 명령이 아니다. Controller는 이어서 회피 동작 또는 Stop Motor를
 * 호출해 실제 모터 명령을 바꾼다. 이 함수는 그 순서나 지속시간을 결정하지 않는다.
 * 반환: self/control 오류는 INVALID_ARGUMENT, Disable 성공은 OK,
 * Enable은 Motor Interface의 성공/오류를 그대로 반환한다.
 */
RvcStatus rvc_move_forward(Rvc *self, RvcForwardControl control)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    if (control == RVC_FORWARD_DISABLE) {
        self->forward_enabled = false;
        return RVC_OK;
    }
    if (control != RVC_FORWARD_ENABLE) return RVC_INVALID_ARGUMENT;
    status = rvc_motor_apply(self, RVC_MOTOR_FORWARD);
    if (status == RVC_OK) self->forward_enabled = true;
    return status;
}
