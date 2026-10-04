#include "sensing/sensing.h"
#include "internal/rvc_internal.h"

/* Dust Sensor Interface (DFD 1.4).
 * 호출: Determine Dust Existence -> 이 함수 -> device.read_dust 콜백.
 * 입력 self: 장치와 센서 캐시를 가진 인스턴스. 초기화와 매 Tick에 한 번 읽는다.
 * 출력: 성공한 detected 값만 sampled.dust_detected에 저장한다.
 * 반환: self/콜백 누락은 INVALID_ARGUMENT, 읽기 실패는 장치 상태, 성공은 OK.
 * detected=false는 '먼지 미감지'이며 I/O 오류가 아니다. dust_valid를 먼저 해제하고
 * 성공 시에만 설정한다. 청소 세기 결정이나 강화 유지시간 관리는 이 모듈의 일이 아니다.
 */
RvcStatus rvc_dust_sensor_sample(Rvc *self)
{
    bool detected = false;
    RvcStatus status;
    if (!self) return RVC_INVALID_ARGUMENT;
    self->dust_valid = false;
    if (!self->device.read_dust) return RVC_INVALID_ARGUMENT;
    status = self->device.read_dust(self->device.context, &detected);
    if (status != RVC_OK) return status;
    self->sampled.dust_detected = detected;
    self->dust_valid = true;
    return RVC_OK;
}
