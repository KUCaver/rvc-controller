# controller.c + controller.h 통합본

교수님의 파일 통합 안내에 맞춰 모든 제품 C 구현을 **controller.c 한 파일**로, 타입·함수 선언을 **controller.h 한 파일**로 합쳤다. Main, Controller, 센서·판단, 동작·출력 인터페이스, 모의 장치까지 들어 있다. 기존 함수별 역할과 호출 관계는 유지한다.

## 필요한 파일

같은 폴더에 아래 두 파일을 둔다.

```text
controller.c
controller.h
```

이 두 파일만으로 기본 시나리오를 컴파일·실행할 수 있다. 다른 프로젝트 소스, 하위 폴더, GoogleTest, CMake, Python은 기본 실행에 필요하지 않다. C17 컴파일러와 그 표준 라이브러리는 필요하다.

## 컴파일하고 결과 보기

C17 GCC가 PATH에 등록된 터미널에서 이 두 파일이 있는 폴더를 열고 실행한다.

```powershell
gcc -std=c17 -Wall -Wextra -Wpedantic -Werror controller.c -o rvc.exe
.\rvc.exe
```

현재 Windows PC의 GCC가 PATH에 없으면 첫 명령 대신 다음처럼 컴파일러 경로를 지정할 수 있다. 다른 PC에서는 그 PC의 설치 위치를 사용한다.

```powershell
& 'C:\msys64\ucrt64\bin\gcc.exe' -std=c17 -Wall -Wextra -Wpedantic -Werror controller.c -o rvc.exe
```

옵션을 주지 않으면 내장된 18 Tick 시나리오가 실행된다. 외부 CSV는 필요하지 않다. 콘솔에 센서값, 상태, 경과 Tick, 모터·청소 출력과 Command 순서가 표시된다. `tick=0`은 초기화이고 마지막 `# shutdown,C:OFF|M:STOP`은 종료 명령이다.

```powershell
.\rvc.exe > result.csv
.\rvc.exe --realtime-ms 200
```

첫 명령은 결과를 CSV 파일로 저장한다. 두 번째는 Tick 사이에 200ms 대기를 요청한다. 고정 과제 요구값이 아닌 실행 예시이며 OS 스케줄링의 시간 정확도를 보장하지 않는다.

직접 만든 입력으로 실행하려면 같은 폴더에 `demo.csv`를 만들고 다음처럼 실행한다.

```csv
front,left,right,dust
0,0,0,0
0,0,0,1
1,0,0,1
```

```powershell
.\rvc.exe --scenario demo.csv
```

CSV의 한 행이 한 Tick이다. 경로를 주는 경우 현재 실행 폴더를 기준으로 찾는다. 기본 내장 시나리오와 위의 3행 예시는 서로 다른 입력이다.

## Structured Chart와 통합 파일의 대응

`controller.c`는 아래 순서로 절을 구분했고 원래 모듈 이름·함수 설명을 남겼다. Ctrl+F로 절 제목이나 함수 이름을 찾으면 된다.

| 절 | 역할 | 주요 함수 |
|---|---|---|
| 1 | 로봇별 내부 상태 | `struct Rvc` |
| 2 | 공개 진입점·수명주기 | `rvc_create`, `rvc_initialize`, `rvc_tick`, `rvc_shutdown`, `rvc_destroy` |
| 3 | Controller 2.1.1 / FSM | `rvc_controller_tick`, `select_next_state`, `apply_transition` |
| 4 | Determine Obstacle / Dust | `rvc_determine_obstacles`, `rvc_determine_dust` |
| 5 | 네 센서 인터페이스 | `rvc_front_sensor_initialize`, `rvc_front_sensor_report`, 좌·우·먼지 sample 함수 |
| 6 | 여섯 동작 모듈 | `rvc_move_forward`, `rvc_turn_left`, `rvc_turn_right`, `rvc_move_backward`, `rvc_stop_motor`, `rvc_set_cleaning` |
| 7 | Motor / Cleaner Interface | `rvc_motor_apply`, `rvc_cleaner_apply` |
| 8 | 모의 하드웨어 | `rvc_mock_*`와 장치 콜백 |
| 9 | Main | `main`, CSV 입력·Tick 발생·출력 헬퍼 |

호출 순서는 Main → 공개 API → Controller → 판단/동작 → 센서/출력 인터페이스 → 장치 콜백이다. 기존 FSM 조건과 Command 순서는 바꾸지 않았다. 파일 하나에 모았다고 함수별 책임이나 객체별 상태를 합쳐 버린 것은 아니다.

## 테스트와 개발본 관리

통합본 GoogleTest 134개와 두 파일만 복사한 독립 실행 검사 9개가 통과했다. [2026-10-06 검증 기록](VERIFICATION.md)에 환경, 검사 항목, 결과와 한계를 정리했다.

저장소 루트에서 `build.ps1`을 실행하면 모듈 분리본과 통합본에 같은 GoogleTest 134개를 각각 실행한다. 제품 코드는 두 경우 모두 C17이고 테스트 실행기만 C++17이다. 통합본 테스트는 이 폴더의 헤더와 C 구현에만 연결하며 원래 core/mock 라이브러리에 연결하지 않는다.

`RVC_NO_MAIN`은 테스트에 연결할 때 Main과 CLI 헬퍼만 제외하는 컴파일 옵션이다. **위의 일반 실행 명령에는 이 옵션을 넣지 않는다.** 일반 사용자에게 별도의 main.c는 필요하지 않다.

개발본의 모듈 소스를 수정했다면 저장소 루트에서 통합본을 갱신한다. Python은 이 관리 단계에만 필요하다.

```sh
python tools/generate_single_file.py
python tools/generate_single_file.py --check
```

두 파일을 직접 수정했다면 재생성 전에 그 변경을 모듈 소스에도 반영해야 한다. 생성 도구는 통합본을 다시 쓰므로 수정 내용을 확인하고 실행한다. 이전처럼 분리된 C 소스와 통합 C를 동시에 링크하면 같은 함수가 중복 정의되므로 둘 중 하나만 빌드한다.

이 파일 통합은 소스 제출 형식에 대한 준비다. Structured Chart와 시스템 시험 보고서를 담은 PPT/PDF는 별도로 필요하며, 최종 제출물 전체가 완성됐다는 뜻은 아니다.
