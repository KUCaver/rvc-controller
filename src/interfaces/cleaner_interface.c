#include "interfaces/interfaces.h"
#include "internal/rvc_internal.h"

/* Cleaner Interface (DFD 2.3): Set Cleaning과 실제/모의 청소 장치를 연결한다.
 * 호출: Set Cleaning -> 이 함수 -> device.write_cleaner 콜백.
 * 입력 self: 로봇과 장치 연결, command: OFF/ON/UP 중 하나.
 * 출력: 장치에 명령을 전달하고 성공하면 telemetry.cleaner와 cleaner_valid를 갱신한다.
 * UP은 정해진 강화 단계로 설정하는 절대 명령이다. 반복 호출해도 단계가 누적되지 않는다.
 * 반환: self/명령/콜백 오류는 INVALID_ARGUMENT(장치 호출 없음), 장치 오류는 그대로,
 * 성공은 OK. 콜백 성공은 이 인터페이스의 확인 기준이며 물리적 완료 검출은 별도다.
 * 장치 콜백 실패 시 이전 명령 수치는 남지만 cleaner_valid=false이므로 확정 출력으로 읽지 않는다.
 */
RvcStatus rvc_cleaner_apply(Rvc *self, RvcCleanerCommand command)
{
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    switch (command) {
        case RVC_CLEANER_OFF:
        case RVC_CLEANER_ON:
        case RVC_CLEANER_UP:
            break;
        default:
            return RVC_INVALID_ARGUMENT;
    }
    if (!self->device.write_cleaner) return RVC_INVALID_ARGUMENT;
    status = self->device.write_cleaner(self->device.context, command);
    if (status != RVC_OK) {
        /* 장치가 명령을 일부 수행한 뒤 실패했을 수도 있어 실제 출력을 미확정으로 둔다. */
        self->telemetry.cleaner_valid = false;
        return status;
    }
    self->telemetry.cleaner = command;
    self->telemetry.cleaner_valid = true;
    return RVC_OK;
}
