#include "perception/perception.h"
#include "internal/rvc_internal.h"
#include "sensing/sensing.h"

/* Determine Obstacle Location (DFD 1.5): Controller에 F/L/R 관측값을 한 묶음으로 전달한다.
 * 호출: Controller -> 이 함수 -> Front/Left/Right Sensor Interface.
 * 입력 self: 로봇 상태. 출력 out: 호출자가 준비한 RvcObstacles 저장 공간.
 * 반환: self/out이 NULL이면 INVALID_ARGUMENT, 센서 실패는 그대로 전달, 성공은 OK.
 * out은 모든 필수 읽기가 성공한 뒤에만 쓴다. 중간 실패 시 호출자의 *out은 유지된다.
 * 단, 읽기에 성공한 개별 센서의 내부 캐시는 이미 갱신될 수 있다.
 * Front는 캐시가 없을 때만 최초 읽기를 하고, 이후에는 이벤트 보고 값을 사용한다.
 * Left/Right는 매 호출마다 읽는다. 회피 방향 선택은 Controller가 담당한다.
 */
RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out)
{
    RvcStatus status;
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    /* Front의 이벤트 입력 방식을 보존한다. 매 Tick에 read_front를 호출하지 않는다. */
    if (!self->front_valid) {
        status = rvc_front_sensor_initialize(self);
        if (status != RVC_OK) return status;
    }
    status = rvc_left_sensor_sample(self);
    if (status != RVC_OK) return status;
    status = rvc_right_sensor_sample(self);
    if (status != RVC_OK) return status;
    /* 각 센서가 준비된 뒤 결과를 한 번에 복사한다. 반환값은 데이터가 아닌 상태 코드다. */
    *out = self->sampled.obstacles;
    return RVC_OK;
}
