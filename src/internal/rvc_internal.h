#ifndef RVC_INTERNAL_H
#define RVC_INTERNAL_H
#include "rvc/rvc.h"
/* C에서 내부 상태를 감추는 실제 객체 정의. 외부 앱은 이 헤더를 include하지 않는다.
 * 로봇마다 이 구조체 하나를 가지므로 센서·시간·장치가 다른 인스턴스와 섞이지 않는다.
 * Controller가 FSM/시간을, 센서 모듈이 sampled를, 출력 인터페이스가 출력 유효성을 갱신한다.
 */
struct Rvc {
    RvcDevice device; /* 호출할 센서/출력 함수와 호출자 소유 context. */
    RvcTelemetry telemetry; /* 확정 상태·입력과 장치 쓰기 결과. get_telemetry는 이를 복사한다. */
    RvcSensorSnapshot sampled; /* 읽는 중인 센서 캐시. 아직 확정된 telemetry.sensors와 구별. */
    /* valid 플래그가 false인 미확정/오류 캐시와 정상 미감지(측정값 false)를 구별하는 표시. */
    bool front_valid;
    bool left_valid;
    bool right_valid;
    bool dust_valid;
    bool forward_enabled; /* Enable/Disable 제어의 기록. 다음 상태를 선택하는 FSM 값은 아니다. */
};
#endif
