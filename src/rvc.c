/* 공개 API 구현: 객체 수명 관리와 내부 모듈로의 호출 연결을 담당한다.
 * Main/GoogleTest → 이 파일의 rvc_* 함수 → Controller/Front Sensor.
 * 여기서 방향·청소 정책을 다시 판단하지 않는다. FSM의 유일한 구현은 Controller다.
 * 인자/반환 계약은 include/rvc/rvc.h, 내부 저장 구조는 internal/rvc_internal.h 참조.
 */
#include <stdlib.h>
#include "internal/rvc_internal.h"
#include "control/controller.h"
#include "sensing/sensing.h"

/* 필요한 장치 함수를 먼저 확인한 뒤 독립 객체를 할당한다. 초기 장치 출력은 initialize의 일이다. */
Rvc *rvc_create(const RvcDevice *device)
{
    if (!device || !device->read_front || !device->read_left ||
        !device->read_right || !device->read_dust ||
        !device->write_motor || !device->write_cleaner) {
        return NULL;
    }
    /* calloc으로 유효성 표시와 시간 등을 0으로 시작한다. context의 내용은 복제하지 않는다. */
    Rvc *self = calloc(1, sizeof(*self));
    if (self) {
        self->device = *device;
        self->telemetry.state = RVC_STATE_UNINITIALIZED;
    }
    return self;
}

/* 해제만 수행하므로 shutdown의 성공 여부를 숨기지 않는다. free(NULL)은 허용된다. */
void rvc_destroy(Rvc *self)
{
    free(self);
}

/* 공개 초기화 진입점. 초기 명령·센서 읽기의 순서는 Controller 한 곳에서 관리한다. */
RvcStatus rvc_initialize(Rvc *self)
{
    return rvc_controller_initialize(self);
}

/* 장치 종료와 준비 해제 결과를 그대로 호출자에게 전달한다. 객체 메모리는 남아 있다. */
RvcStatus rvc_shutdown(Rvc *self)
{
    return rvc_controller_shutdown(self);
}

/* Structured Chart의 Controller로 들어가는 공개 함수. 전달 자체가 새 FSM 계층은 아니다. */
RvcStatus rvc_tick(Rvc *self)
{
    return rvc_controller_tick(self);
}

/* 전방 이벤트는 센서 캐시만 갱신한다. 공개 진단 입력은 다음 성공 Tick에서 확정된다. */
RvcStatus rvc_report_front(Rvc *self, bool blocked)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;
    return rvc_front_sensor_report(self, blocked);
}

/* 읽기 전용 진단 복사: 조건 검사 이후에만 *out을 써서 실패 시 호출자의 값을 보존한다. */
RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out)
{
    if (!self || !out) return RVC_INVALID_ARGUMENT;
    if (!self->telemetry.initialized) return RVC_NOT_READY;
    *out = self->telemetry;
    return RVC_OK;
}
