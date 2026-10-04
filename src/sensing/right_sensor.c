#include "sensing/sensing.h"
#include "internal/rvc_internal.h"

/* Right Sensor Interface (DFD 1.3).
 * 호출: Determine Obstacle Location -> 이 함수 -> device.read_right 콜백.
 * 입력 self: 장치와 센서 캐시를 가진 인스턴스. 초기화와 매 Tick에 한 번 읽는다.
 * 출력: 성공한 blocked 값만 sampled.obstacles.right_blocked에 저장한다.
 * 반환: self/콜백 누락은 INVALID_ARGUMENT, 읽기 실패는 장치 상태, 성공은 OK.
 * false는 '장애물 없음'이다. right_valid를 읽기 전에 해제하고 성공 시에만 설정하여
 * 실패 후 남아 있는 이전 값을 정상적인 이번 Tick 입력으로 오인하지 않게 한다.
 */
RvcStatus rvc_right_sensor_sample(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    self->right_valid = false;
    if (!self->device.read_right) return RVC_INVALID_ARGUMENT;
    status = self->device.read_right(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.right_blocked = blocked;
    self->right_valid = true;
    return RVC_OK;
}
