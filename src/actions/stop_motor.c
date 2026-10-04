#include "actions/actions.h"
#include "internal/rvc_internal.h"
#include "interfaces/interfaces.h"

/* Stop Motor (DFD 2.1.6): Controller가 초기화·종료·오류 정지에 사용하는 실제 정지 명령.
 * 입력 self: 로봇 상태. 출력: Motor Interface에 STOP을 한 번 전달한다.
 * 반환: self가 없으면 INVALID_ARGUMENT, 그 외에는 인터페이스 상태를 그대로 전달한다.
 * 장치 콜백이 성공을 알린 뒤에만 forward_enabled를 해제한다.
 * Forward의 Disable과 달리 하드웨어 명령을 보낸다. 청소 OFF는 별도 모듈의 책임이다.
 */
RvcStatus rvc_stop_motor(Rvc *self)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    status = rvc_motor_apply(self, RVC_MOTOR_STOP);
    if (status == RVC_OK) self->forward_enabled = false;
    return status;
}
