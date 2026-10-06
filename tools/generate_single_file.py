"""Build a standalone C implementation and one header from the maintained modules.

Only include plumbing, declaration placement and explanatory comments are
transformed. Function bodies remain those of the maintained modular sources.
One small public header serves applications; internal/mock declarations are
shared by the C implementation and a generated test-only support header.
The generated product files are checked in; Python is not needed to compile them.
"""
from pathlib import Path
import argparse
import re

ROOT = Path(__file__).resolve().parents[1]
HEADER_GROUPS = [
    ('1. 공통 타입과 데이터 사전', ['include/rvc/types.h']),
    ('2. 장치 콜백 계약', ['include/rvc/device.h']),
    ('3. 공개 API와 객체 수명주기', ['include/rvc/rvc.h']),
    ('4. Structured Chart의 모듈별 함수 선언', [
        'src/control/controller.h', 'src/perception/perception.h',
        'src/sensing/sensing.h', 'src/actions/actions.h', 'src/interfaces/interfaces.h']),
    ('5. 모의 장치와 테스트 지원', ['adapters/mock/mock_device.h']),
]
SOURCE_GROUPS = [
    ('2. 공개 API: Main과 Controller의 연결', ['src/rvc.c']),
    ('3. Controller 2.1.1: 상태, 조건, Command, Tick 관리', ['src/control/controller.c']),
    ('4. 입력 판단: Determine Obstacle / Dust', [
        'src/perception/obstacle_detector.c', 'src/perception/dust_detector.c']),
    ('5. 센서 인터페이스: Front / Left / Right / Dust', [
        'src/sensing/front_sensor.c', 'src/sensing/left_sensor.c',
        'src/sensing/right_sensor.c', 'src/sensing/dust_sensor.c']),
    ('6. 동작: Forward / Left / Right / Backward / Stop / Cleaning', [
        'src/actions/move_forward.c', 'src/actions/turn_left.c',
        'src/actions/turn_right.c', 'src/actions/move_backward.c',
        'src/actions/stop_motor.c', 'src/actions/set_cleaning.c']),
    ('7. 출력 인터페이스: Motor / Cleaner', [
        'src/interfaces/motor_interface.c', 'src/interfaces/cleaner_interface.c']),
    ('8. 모의 장치: 센서 입력과 명령 기록', ['adapters/mock/mock_device.c']),
]


def read(relative):
    return (ROOT / relative).read_text(encoding='utf-8-sig')


def section(title):
    return '\n/* ' + '=' * 72 + '\n * ' + title + '\n * ' + '=' * 72 + ' */\n\n'


def explain_local_locations(text):
    # Comment-only substitutions: generated code needs no original directory.
    def replace_comment(match):
        comment = match.group(0)
        for before, after in [
            ('src/internal/rvc_internal.h', 'controller.c의 1절'),
            ('internal/rvc_internal.h', 'controller.c의 1절'),
            ('include/rvc/rvc.h', 'controller.h의 공개 API'),
            ('rvc/rvc.h', 'controller.h의 공개 API'),
            ('src/rvc.c', 'controller.c의 2절 공개 API'),
            ('현재는 adapters/mock이 연결되며', '현재는 controller.c의 8절 모의 장치가 연결되며'),
            ('전체 전이표: docs/06_implemented_design.md. 검증: tests/rvc_tests.cpp.',
             '기존 전이·Command와 함수 호출을 보존했다. 통합본도 동일한 134개 GoogleTest로 검사한다.'),
        ]:
            comment = comment.replace(before, after)
        return comment
    return re.sub(r'/\*.*?\*/', replace_comment, text, flags=re.S)


def header_body(relative):
    text = read(relative)
    guard = re.search(r'^#ifndef (\w+)\n#define \1\n', text, flags=re.M)
    if not guard or not re.search(r'#endif\s*\Z', text):
        raise ValueError('Unexpected header guard: ' + relative)
    text = text[:guard.start()] + text[guard.end():]
    text = re.sub(r'#endif\s*\Z', '', text)
    text = re.sub(r'^#include[^\n]*\n', '', text, flags=re.M)
    text = re.sub(r'#ifdef __cplusplus\n(?:/\*.*?\*/\n)?extern "C" \{\n#endif\n', '', text, flags=re.S)
    text = re.sub(r'#ifdef __cplusplus\n\}\n#endif\n?', '', text)
    if re.search(r'^#', text, flags=re.M):
        raise ValueError('Unreviewed header preprocessor directive: ' + relative)
    return explain_local_locations(text).strip() + '\n'


def source_body(relative):
    text = read(relative)
    if relative == 'app/main.c':
        # Feature-test macros must precede even controller.h and its system headers.
        start = text.index('#ifndef _WIN32\n')
        end = text.index('#include "mock_device.h"\n') + len('#include "mock_device.h"\n')
        text = text[:start] + text[end:]
    text = re.sub(r'^#include[^\n]*\n', '', text, flags=re.M)
    return explain_local_locations(text).strip() + '\n'


def declarations_only(text, keep_inline_comments=False):
    """Compact declarations without duplicating the implementation's long comments."""
    def compact_comment(match):
        line_prefix = text[text.rfind('\n', 0, match.start()) + 1:match.start()]
        return match.group(0) if keep_inline_comments and line_prefix.strip() else ''
    text = re.sub(r'/\*.*?\*/', compact_comment, text, flags=re.S)
    text = re.sub(r'\n[ \t]*\n(?:[ \t]*\n)*', '\n', text)
    return '\n'.join(line.rstrip() for line in text.strip().splitlines()) + '\n'


def public_and_private_declarations():
    types = declarations_only(header_body('include/rvc/types.h'), keep_inline_comments=True)
    private_types = []
    for name in ('RvcForwardControl', 'RvcCleaningMode'):
        pattern = r'^typedef enum \{[^\n]+\} ' + name + r';\n'
        matches = re.findall(pattern, types, flags=re.M)
        if len(matches) != 1:
            raise ValueError('Review internal enum extraction: ' + name)
        private_types.append(matches[0])
        types = re.sub(pattern, '', types, flags=re.M)

    # Comments describe complete groups; function contracts remain next to each API.
    public = '/* 반환 상태: OK 또는 인자/준비/장치 오류. 센서 true/false는 감지/미감지다.\n'
    public += ' * CLEANER_UP은 강화 단계 설정이며, 반복 호출마다 세기를 더하는 명령이 아니다.\n'
    public += ' * UNINITIALIZED는 초기화 전·종료·오류 후의 준비 상태를 뜻한다. */\n' + types
    public += '\n/* 장치 연결: 콜백 6개는 필수. context는 호출자가 소유하며 로봇 해제까지 유지한다.\n'
    public += ' * 읽기는 성공할 때만 *out을 갱신하고, 쓰기는 명령 적용 결과를 반환한다. */\n'
    public += declarations_only(header_body('include/rvc/device.h'), keep_inline_comments=True)
    public += '\n/* 내부 상태를 감춘 객체. create → initialize → tick 반복 → shutdown → destroy. */\n'
    api = declarations_only(header_body('include/rvc/rvc.h'))
    contracts = {
        'rvc_create': '콜백 표를 복사해 객체를 만든다. 실패 NULL; 아직 장치 입출력은 없다.',
        'rvc_destroy': '메모리만 해제한다(NULL 허용). 장치 종료는 먼저 shutdown으로 요청한다.',
        'rvc_initialize': 'STOP과 OFF를 시도하고 센서를 준비한다. 성공 시 STOP_OFF, 실패 시 미준비.',
        'rvc_shutdown': 'OFF와 STOP을 각각 시도한 뒤 미준비 상태로 만든다. 실패하면 첫 오류 반환.',
        'rvc_tick': '호출 한 번이 논리 Tick 하나. 센서 판단·FSM·명령 실행; 실제 대기는 하지 않는다.',
        'rvc_report_front': '전방 감지 캐시만 갱신한다. Tick과 직렬 호출하며 명령은 다음 Tick에서 결정한다.',
        'rvc_get_telemetry': '마지막 완료 상태를 *out에 복사한다. 오류/종료 후 NOT_READY이며 *out은 유지한다.',
    }
    found = []
    for line in api.splitlines():
        function = re.search(r'\b(rvc_\w+)\(', line)
        if function:
            name = function.group(1)
            found.append(name)
            public += '\n/* ' + contracts[name] + ' */\n'
        public += line + '\n'
    if set(found) != set(contracts):
        raise ValueError('Review public API declarations before regenerating.')

    private = '/* 내부 제어 타입: Structured Chart의 Enable/Disable과 청소 모드. */\n'
    private += ''.join(private_types)
    for relative in HEADER_GROUPS[3][1]:
        private += '\n/* 내부 모듈: ' + relative.split('/')[-2] + ' */\n'
        private += declarations_only(header_body(relative))
    private += '\n/* 모의 장치: 입력 설정, 명령 로그, 오류 주입. 상태 판단은 Controller가 한다.\n'
    private += ' * 로그는 성공한 명령만 기록하고 counts는 실패한 시도도 포함한다. */\n'
    private += declarations_only(header_body('adapters/mock/mock_device.h'))
    return public, private


def build():
    listed_c = {p for _, files in SOURCE_GROUPS for p in files} | {'app/main.c'}
    actual_c = {p.relative_to(ROOT).as_posix() for top in ['src', 'adapters', 'app'] for p in (ROOT / top).rglob('*.c')}
    listed_h = {p for _, files in HEADER_GROUPS for p in files} | {'src/internal/rvc_internal.h'}
    actual_h = {p.relative_to(ROOT).as_posix() for top in ['src', 'include', 'adapters'] for p in (ROOT / top).rglob('*.h')}
    if listed_c != actual_c or listed_h != actual_h:
        raise ValueError('Module list changed; review additions/removals before regenerating.')

    public, private = public_and_private_declarations()
    header = '''/* RVC 공개 헤더: controller.c와 같은 폴더에 놓는다.
 * 센서·상태·명령 타입, 장치 연결, 외부에서 사용하는 함수 7개만 선언한다.
 * 내부 모듈과 모의 장치는 controller.c에서 정의한다.
 * 센서와 조회 결과는 out 인자, 성공/실패는 반환값으로 구분한다.
 * initialize 이후 report_front/tick/get_telemetry를 한 실행 흐름에서 직렬 호출한다.
 * Tick 입출력 오류 후에는 재초기화가 필요하며 실제 장치 정지 성공을 보장하지 않는다.
 */
#ifndef RVC_SINGLE_FILE_CONTROLLER_H
#define RVC_SINGLE_FILE_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

'''
    header += public + '\n'
    header += '#ifdef __cplusplus\n}\n#endif\n#endif /* RVC_SINGLE_FILE_CONTROLLER_H */\n'

    test_header = '''/* 생성된 테스트 전용 선언. 일반 앱·두 파일 독립 실행에는 필요하지 않다.
 * 기존 모듈 직접 검사와 모의 장치 조작에만 사용한다.
 * 타입·선언은 controller.c의 내부 선언과 같은 원본에서 생성한다.
 */
#ifndef RVC_SINGLE_FILE_TEST_SUPPORT_H
#define RVC_SINGLE_FILE_TEST_SUPPORT_H
#include "controller.h"
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

'''
    test_header += private
    test_header += '\n#ifdef __cplusplus\n}\n#endif\n#endif /* RVC_SINGLE_FILE_TEST_SUPPORT_H */\n'

    source = '''/* RVC 통합 구현: Main + Controller + 판단/센서 + 동작/출력 + 모의 장치.
 * 준비할 구현 파일은 이 controller.c와 같은 폴더의 controller.h 두 개다.
 * 컴파일: gcc -std=c17 -Wall -Wextra -Wpedantic -Werror controller.c -o rvc.exe
 * 실행:   ./rvc.exe                         (내장 시나리오; 외부 CSV 불필요)
 *         ./rvc.exe --scenario demo.csv     (선택한 CSV로 논리 Tick 실행)
 *         ./rvc.exe --realtime-ms 200        (실제 대기를 넣는 예시)
 *
 * 읽기: 9절 Main → 2절 공개 API → 3절 Controller → 4~7절 모듈 → 8절 모의 장치.
 * Structured Chart의 함수와 호출 관계는 유지하고 파일 배치만 통합했다.
 * Sensor 결과는 out 매개변수, 성공/실패는 RvcStatus 반환값으로 구분한다.
 * Controller가 조건을 판단해 동작 함수를 호출하며 실제 Command는 장치 콜백에 전달한다.
 * 회전 5 Tick/후진 3 Tick, 진입 e=0. Tick 발생과 대기는 Main이 담당한다.
 *
 * GoogleTest에 연결할 때만 -DRVC_NO_MAIN을 지정하여 Main과 그 전용 헬퍼를 제외한다.
 * 제품 자체에는 GoogleTest/C++/원래 모듈 폴더/CMake/Python이 필요하지 않다.
 * 개발본에서 다시 생성: python tools/generate_single_file.py
 */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#include "controller.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#ifndef RVC_NO_MAIN
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <time.h>
#endif
#endif
'''
    source += section('1. 내부 선언과 객체 상태: 외부 앱에서 몰라도 되는 구현 상세')
    source += private + '\n'
    source += header_body('src/internal/rvc_internal.h')
    for title, files in SOURCE_GROUPS:
        source += section(title)
        for relative in files:
            source += '/* 원래 모듈: ' + relative + ' */\n' + source_body(relative) + '\n'
    source += section('9. Main: 내장/CSV 입력, Tick 발생, 상태·Command 출력')
    source += '#ifndef RVC_NO_MAIN\n' + source_body('app/main.c') + '#endif /* RVC_NO_MAIN */\n'
    return {'single_file/controller.h': header, 'single_file/controller.c': source,
            'tests/single_file_test_support.h': test_header}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check committed outputs without writing.')
    args = parser.parse_args()
    output = ROOT
    files = build()
    if args.check:
        stale = [name for name, text in files.items() if not (output/name).exists() or (output/name).read_text(encoding='utf-8-sig') != text]
        if stale:
            raise SystemExit('Regenerate single_file: ' + ', '.join(stale))
        print('Single-file outputs match all 18 C sources and 10 headers.')
    else:
        for name, text in files.items():
            (output/name).parent.mkdir(exist_ok=True)
            (output/name).write_text(text, encoding='utf-8', newline='\n')
        print('Generated two product files and the test-only support header.')


if __name__ == '__main__':
    main()
