#include "perception/perception.h"
#include "internal/rvc_internal.h"
#include "sensing/sensing.h"

/* Determine Dust Existence (DFD 1.6).
 * 호출: Controller -> 이 함수 -> Dust Sensor Interface -> 장치 읽기 콜백.
 * 입력 self: 로봇 상태. 출력 out: 호출자가 준비한 bool 저장 공간(true=먼지 감지).
 * 반환: self/out 누락은 INVALID_ARGUMENT, 읽기 실패는 센서 상태, 성공은 OK.
 * 읽기에 성공할 때만 *out을 쓴다. false를 반환 상태로 표현하지 않으므로
 * '정상적으로 읽은 먼지 없음'과 '읽기 오류'를 Controller에서 구별할 수 있다.
 * ON/UP 명령 선택은 Controller의 FSM이 수행한다.
 */
RvcStatus rvc_determine_dust(Rvc *self, bool *out)
{
    RvcStatus status;
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    status = rvc_dust_sensor_sample(self);
    if (status != RVC_OK) return status;
    *out = self->sampled.dust_detected;
    return RVC_OK;
}
