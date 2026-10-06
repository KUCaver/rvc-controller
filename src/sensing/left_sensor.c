#include "sensing/sensing.h"
#include "internal/rvc_internal.h"

/* Left Sensor Interface (DFD 1.2).
 * 호출: Determine Obstacle Location -> 이 함수 -> device.read_left 콜백.
 * 입력 self: 장치와 센서 캐시를 가진 인스턴스. 초기화와 매 Tick에 한 번 읽는다.
 * 출력: 성공한 blocked 값만 sampled.obstacles.left_blocked에 저장한다.
 * 반환: self/콜백 누락은 INVALID_ARGUMENT, 읽기 실패는 장치 상태, 성공은 OK.
 * false는 '장애물 없음'이다. 호출자는 반환 상태를 검사하고 성공한 값만 판단에 사용한다.
 * 읽기 실패 시 이전 캐시가 남더라도 이번 Tick의 입력으로 확정하지 않는다.
 */
RvcStatus rvc_left_sensor_sample(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->device.read_left) return RVC_INVALID_ARGUMENT;
    status = self->device.read_left(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.left_blocked = blocked;
    return RVC_OK;
}
