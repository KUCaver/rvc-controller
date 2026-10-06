#ifndef RVC_SENSING_SENSING_H
#define RVC_SENSING_SENSING_H
#include "rvc/rvc.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 센서 인터페이스의 내부 계약. 앱은 이 헤더 대신 rvc/rvc.h를 사용한다.
 * self는 로봇별 상태를 가리키며 NULL이면 INVALID_ARGUMENT을 반환한다.
 * sample/initialize는 장치 콜백을 호출하고, 성공 시 self->sampled를 갱신한다.
 * Front만 이벤트 캐시의 유효성을 front_valid로 기록한다. 나머지는 반환 상태로 성공을 판단한다.
 * 콜백 누락은 INVALID_ARGUMENT, 장치 오류는 그대로 전달한다. 실패 시 측정값을 갱신하지 않는다.
 * bool 측정값(false 포함)과 RvcStatus 성공/실패는 서로 다른 정보다.
 * 이 함수들은 센서 값을 준비할 뿐, Controller의 FSM이나 출력을 결정하지 않는다.
 */
/* DFD 1.1: Front 초기값을 read_front로 읽는다. 이후 Tick은 유효한 캐시를 사용한다. */
RvcStatus rvc_front_sensor_initialize(Rvc *self);

/* DFD 1.1: blocked(true=막힘) 이벤트로 Front 캐시를 갱신한다. Tick과 직렬화해 호출한다.
 * 장치 콜백 없이 캐시만 갱신하며, self가 유효하면 OK. FSM 전이는 다음 Tick에서 수행한다.
 */
RvcStatus rvc_front_sensor_report(Rvc *self, bool blocked);

/* DFD 1.2: 초기화 및 매 Tick에 read_left를 호출하여 좌측 장애물 유무를 저장한다. */
RvcStatus rvc_left_sensor_sample(Rvc *self);

/* DFD 1.3: 초기화 및 매 Tick에 read_right를 호출하여 우측 장애물 유무를 저장한다. */
RvcStatus rvc_right_sensor_sample(Rvc *self);

/* DFD 1.4: 초기화 및 매 Tick에 read_dust를 호출하여 먼지 감지 여부를 저장한다. */
RvcStatus rvc_dust_sensor_sample(Rvc *self);
#ifdef __cplusplus
}
#endif
#endif
