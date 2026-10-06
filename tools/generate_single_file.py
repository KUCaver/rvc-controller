"""Build a standalone C implementation and one header from the maintained modules.

Only include plumbing, include guards, C++ linkage wrappers and explanatory
comments are transformed. Function bodies remain those of the modular sources.
The generated files are checked in; Python is not needed to compile them.
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


def build():
    listed_c = {p for _, files in SOURCE_GROUPS for p in files} | {'app/main.c'}
    actual_c = {p.relative_to(ROOT).as_posix() for top in ['src', 'adapters', 'app'] for p in (ROOT / top).rglob('*.c')}
    listed_h = {p for _, files in HEADER_GROUPS for p in files} | {'src/internal/rvc_internal.h'}
    actual_h = {p.relative_to(ROOT).as_posix() for top in ['src', 'include', 'adapters'] for p in (ROOT / top).rglob('*.h')}
    if listed_c != actual_c or listed_h != actual_h:
        raise ValueError('Module list changed; review additions/removals before regenerating.')

    header = '''/* RVC 통합 헤더: controller.c와 같은 폴더에 놓는다.
 * 공통 타입, 장치 콜백, 함수 선언, 모의 장치 계약을 한 곳에 모았다.
 * Rvc 내부 상태는 controller.c에 숨기며, 객체별 상태와 모듈별 함수는 유지한다.
 * 일반 앱은 3절의 공개 API를 사용한다. 4절은 구조도 대응과 모듈 시험용이다.
 * 파일을 나눈 개발본에서 생성했으며, 원래 헤더를 함께 include하지 않는다.
 */
#ifndef RVC_SINGLE_FILE_CONTROLLER_H
#define RVC_SINGLE_FILE_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
'''
    for title, files in HEADER_GROUPS:
        header += section(title)
        for relative in files:
            header += '/* 원래 모듈: ' + relative + ' */\n' + header_body(relative) + '\n'
    header += '#ifdef __cplusplus\n}\n#endif\n#endif /* RVC_SINGLE_FILE_CONTROLLER_H */\n'

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
    source += section('1. 내부 객체 상태: 인스턴스별 캡슐화')
    source += header_body('src/internal/rvc_internal.h')
    for title, files in SOURCE_GROUPS:
        source += section(title)
        for relative in files:
            source += '/* 원래 모듈: ' + relative + ' */\n' + source_body(relative) + '\n'
    source += section('9. Main: 내장/CSV 입력, Tick 발생, 상태·Command 출력')
    source += '#ifndef RVC_NO_MAIN\n' + source_body('app/main.c') + '#endif /* RVC_NO_MAIN */\n'
    return {'controller.h': header, 'controller.c': source}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check committed outputs without writing.')
    args = parser.parse_args()
    output = ROOT / 'single_file'
    files = build()
    if args.check:
        stale = [name for name, text in files.items() if not (output/name).exists() or (output/name).read_text(encoding='utf-8-sig') != text]
        if stale:
            raise SystemExit('Regenerate single_file: ' + ', '.join(stale))
        print('Single-file outputs match all 18 C sources and 10 headers.')
    else:
        output.mkdir(exist_ok=True)
        for name, text in files.items():
            (output/name).write_text(text, encoding='utf-8', newline='\n')
        print('Generated single_file/controller.c and controller.h.')


if __name__ == '__main__':
    main()
