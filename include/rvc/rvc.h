/* 앱/테스트가 사용하는 공개 API. 먼저 이 파일을 읽으면 전체 수명주기를 파악할 수 있다.
 * create → initialize → (report_front / tick / get_telemetry) 반복 → shutdown → destroy
 * 호출은 한 실행 흐름에서 직렬 처리한다. ISR에서 Tick과 동시에 직접 호출하지 않는다.
 */
#ifndef RVC_H
#define RVC_H
#include "rvc/device.h"
#ifdef __cplusplus
/* GoogleTest는 C++이므로, C로 빌드한 함수의 이름으로 연결하도록 지정한다. */
extern "C" {
#endif
/* 내부 필드를 감춘 객체 핸들. 실제 구조는 src/internal/rvc_internal.h에만 있다. */
typedef struct Rvc Rvc;
/* [입력] 필수 콜백 6개가 채워진 device. [반환] 생성한 객체, 실패 시 NULL.
 * 함수 포인터 표를 복사한다. 장치 읽기/출력은 아직 없으며 메모리만 준비한다.
 * context의 수명 관리는 호출자가 맡는다. device 자체의 주소를 계속 보관하지는 않는다.
 */
Rvc *rvc_create(const RvcDevice *device);
/* 메모리 해제만 수행한다(NULL 허용). 장치 정지가 필요하면 먼저 shutdown을 호출한다. */
void rvc_destroy(Rvc *self);

/* [입력] create로 만든 self. [반환] 성공 RVC_OK / 잘못된 인자 / 장치 오류.
 * STOP → OFF를 모두 시도하고 초기 센서값을 읽는다. 모두 성공하면 STOP_OFF로 준비된다.
 * 실패 시 미준비 상태를 유지한다. 재호출하면 캐시·시간을 초기화하여 다시 시작한다.
 */
RvcStatus rvc_initialize(Rvc *self);
/* OFF → STOP을 각각 시도하고 미준비 상태로 만든다. 하나가 실패해도 다른 하나를 시도한다.
 * 성공이면 RVC_OK, 실패이면 첫 실패 코드를 반환한다. 실제 장치 정지 성공은 보장하지 않는다.
 * 다시 Tick을 처리하려면 initialize가 필요하다.
 */
RvcStatus rvc_shutdown(Rvc *self);
/* 호출 한 번=논리 Tick 하나. 센서 읽기→FSM 판단→명령 실행을 Controller에 위임한다.
 * 성공 RVC_OK, 미초기화/오류 후 NOT_READY. I/O 실패 시 종료 명령을 시도하고 원래 오류 반환.
 * 실제 시간 대기나 밀리초 인자는 없다. Main/테스트가 호출 시점을 정한다.
 */
RvcStatus rvc_tick(Rvc *self);
/* [입력] blocked=true는 전방 감지. 초기화된 객체의 캐시만 갱신한다.
 * 이 호출만으로 전이/명령/Tick은 발생하지 않으며 다음 Tick에서 최신 보고를 사용한다.
 * 성공 RVC_OK, NULL 인자 INVALID_ARGUMENT, 미준비 NOT_READY.
 */
RvcStatus rvc_report_front(Rvc *self, bool blocked);

/* [출력] 성공 시 *out에 진단 정보를 복사한다. 호출자는 그 복사본을 읽으면 된다.
 * 초기화 전/오류 후/종료 후 NOT_READY, NULL 인자는 INVALID_ARGUMENT. 실패 시 *out 유지.
 * 출력 명령 값은 해당 valid가 true일 때만 확인된 쓰기로 해석한다.
 */
RvcStatus rvc_get_telemetry(const Rvc *self, RvcTelemetry *out);
#ifdef __cplusplus
}
#endif
#endif
