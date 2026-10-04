#include "actions/actions.h"
#include "interfaces/interfaces.h"

/* Set Cleaning (DFD 2.1.7): FSM의 청소 모드를 Cleaner Interface의 명령으로 변환한다.
 * 입력 self: 로봇 상태, mode: OFF / ON / POWER1.
 * 출력: 각각 장치 명령 OFF / ON / UP을 한 번 전달한다.
 * UP은 '강화 단계로 설정'하는 절대 명령이다. 호출할 때마다 세기를 누적해서 올리지 않는다.
 * 반환: self 또는 mode 오류는 INVALID_ARGUMENT, 그 외에는 인터페이스 상태를 전달한다.
 * 먼지 유무에 따른 모드 선택과 유지 정책은 Controller에 있으며 이 함수에는 없다.
 */
RvcStatus rvc_set_cleaning(Rvc *self, RvcCleaningMode mode)
{
    if (!self) return RVC_INVALID_ARGUMENT;
    switch (mode) {
        case RVC_CLEANING_OFF:
            return rvc_cleaner_apply(self, RVC_CLEANER_OFF);
        case RVC_CLEANING_ON:
            return rvc_cleaner_apply(self, RVC_CLEANER_ON);
        case RVC_CLEANING_POWER1:
            return rvc_cleaner_apply(self, RVC_CLEANER_UP);
        default:
            return RVC_INVALID_ARGUMENT;
    }
}
