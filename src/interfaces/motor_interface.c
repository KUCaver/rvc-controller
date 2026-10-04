#include "interfaces/interfaces.h"
#include "internal/rvc_internal.h"

/* Motor Interface (DFD 2.2): 모든 이동 동작이 공유하는 장치 출력 경계.
 * 호출: Forward/Turn/Backward/Stop 모듈 -> 이 함수 -> device.write_motor 콜백.
 * 입력 self: 로봇과 장치 연결, command: STOP/FORWARD/LEFT/RIGHT/BACKWARD 중 하나.
 * 출력: 장치에 명령을 전달하고 성공하면 telemetry.motor와 motor_valid를 갱신한다.
 * 반환: self/명령/콜백 오류는 INVALID_ARGUMENT(장치 호출 없음), 장치 오류는 그대로,
 * 성공은 OK. 데이터 출력은 콜백의 command 인자로, 성공 여부는 반환값으로 표현한다.
 * 콜백 실패 시 장치의 실제 상태를 확정할 수 없으므로 마지막 명령 값은 보존하되
 * motor_valid=false로 표시한다. 장치 복구나 재시도 정책은 Controller가 결정한다.
 */
RvcStatus rvc_motor_apply(Rvc *self, RvcMotorCommand command)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    switch (command) {
        case RVC_MOTOR_STOP:
        case RVC_MOTOR_FORWARD:
        case RVC_MOTOR_LEFT:
        case RVC_MOTOR_RIGHT:
        case RVC_MOTOR_BACKWARD:
            break;
        default:
            return RVC_INVALID_ARGUMENT;
    }
    if (!self->device.write_motor) return RVC_INVALID_ARGUMENT;
    status = self->device.write_motor(self->device.context, command);
    if (status != RVC_OK) {
        /* 실패를 '이전 명령이 확실히 유지됨'으로 해석하지 않는다. */
        self->telemetry.motor_valid = false;
        return status;
    }
    self->telemetry.motor = command;
    self->telemetry.motor_valid = true;
    return RVC_OK;
}
