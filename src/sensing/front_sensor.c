#include "sensing/sensing.h"
#include "internal/rvc_internal.h"

/* Front Sensor Interface (DFD 1.1): 최초 읽기와 이후 이벤트 보고를 분리한다.
 * 호출: Determine Obstacle Location -> 이 함수 -> device.read_front 콜백.
 * 입력 self: 해당 로봇의 장치 콜백과 센서 캐시를 가진 인스턴스.
 * 출력: 성공하면 sampled.obstacles.front_blocked와 front_valid를 갱신한다.
 * 반환: self/콜백이 없으면 INVALID_ARGUMENT, 장치 실패는 그 상태를 그대로 반환.
 * false는 '장애물 없음'이라는 정상 데이터이며, 읽기 실패와 다르다.
 * 실패 시 이전 수치는 남지만 valid=false이므로 새 관측값으로 사용하면 안 된다.
 */
RvcStatus rvc_front_sensor_initialize(Rvc *self)
{
    bool blocked = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    self->front_valid = false;
    if (!self->device.read_front) return RVC_INVALID_ARGUMENT;
    status = self->device.read_front(self->device.context, &blocked);
    if (status != RVC_OK) return status;
    self->sampled.obstacles.front_blocked = blocked;
    self->front_valid = true;
    return RVC_OK;
}

/* 공개 API rvc_report_front()에서 전달한 전방 센서 이벤트를 캐시에 반영한다.
 * 입력 blocked: true=전방 막힘, false=전방 열림. 출력은 캐시와 valid 플래그뿐이다.
 * self가 없으면 INVALID_ARGUMENT, 갱신하면 OK. 장치 읽기는 수행하지 않는다.
 * FSM 전이·모터 출력·Tick 증가는 다음 Controller 실행에서 처리한다.
 * 호출은 Tick 처리와 직렬화해야 한다. 실제 하드웨어 ISR과의 동기화 기능은 없다.
 */
RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    self->sampled.obstacles.front_blocked = blocked;
    self->front_valid = true;
    return RVC_OK;
}
