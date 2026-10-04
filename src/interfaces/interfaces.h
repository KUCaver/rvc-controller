#ifndef RVC_INTERFACES_INTERFACES_H
#define RVC_INTERFACES_INTERFACES_H
#include "rvc/rvc.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 장치 출력 인터페이스의 내부 계약. 동작 모듈이 호출하며 앱은 rvc/rvc.h를 사용한다.
 * self: 로봇 인스턴스, command: 장치에 전달할 절대 명령(매개변수가 장치 출력 데이터).
 * RvcStatus 반환값은 출력 명령 자체가 아니라 장치 콜백의 성공/실패를 나타낸다.
 * self/명령/필수 콜백 오류는 INVALID_ARGUMENT이며 장치 호출과 telemetry 변경은 없다.
 * 장치 콜백 실패는 그대로 전달하고 해당 valid=false. 성공은 명령 기록 + valid=true + OK.
 * 콜백은 RvcDevice에서 주입되므로 같은 모듈을 실제 장치와 모의 장치에 연결할 수 있다.
 */
/* DFD 2.2: 모터 명령을 검증해 write_motor로 전달한다. FSM 판단과 대기는 수행하지 않는다. */
RvcStatus rvc_motor_apply(Rvc *self, RvcMotorCommand command);

/* DFD 2.3: OFF/ON/UP을 검증해 write_cleaner로 전달한다. UP은 고정 강화 단계 설정이다. */
RvcStatus rvc_cleaner_apply(Rvc *self, RvcCleanerCommand command);
#ifdef __cplusplus
}
#endif
#endif 
