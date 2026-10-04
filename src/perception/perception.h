#ifndef RVC_PERCEPTION_PERCEPTION_H
#define RVC_PERCEPTION_PERCEPTION_H
#include "rvc/rvc.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 판단 모듈의 내부 계약. Controller가 호출하고, 앱은 rvc/rvc.h를 사용한다.
 * self는 로봇 인스턴스, out은 결과를 받을 유효한 주소여야 한다.
 * 반환 RvcStatus는 처리 성공/실패이고 실제 센서 결과는 *out으로 전달한다.
 * self/out 누락은 INVALID_ARGUMENT, 센서 오류는 그대로 전달, 성공은 OK.
 * 실패 시 *out은 변경하지 않는다. 이 모듈은 장치 출력이나 FSM 전이를 수행하지 않는다.
 */
/* DFD 1.5: Front 캐시(없으면 초기 읽기) + Left/Right 읽기를 성공한 뒤 F/L/R을 전달한다. */
RvcStatus rvc_determine_obstacles(Rvc *self, RvcObstacles *out);

/* DFD 1.6: Dust Sensor Interface를 호출한 뒤 감지 여부를 *out에 쓴다(false도 정상값). */
RvcStatus rvc_determine_dust(Rvc *self, bool *out);
#ifdef __cplusplus
}
#endif
#endif
