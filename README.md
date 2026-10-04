# RVC Controller — C 모듈과 FSM

센서 입력에 따라 전진·회전·후진과 청소 출력을 결정하는 C17 로봇 청소기 제어 프로그램이다. Structured Chart의 모듈 호출 구조를 구현하며, 실제 장치 대신 모의 센서·모터·청소기를 연결한다. GoogleTest 실행기만 C++17이다.

**Team Project #2의 구현 작업본**이다. SD Structured Chart와 시스템 시험 보고서를 담은 최종 PPT/PDF는 이 저장소에 포함되지 않는다. 설계 자료와 코드 사이에 남은 확인 사항은 [작업 상태](docs/STATUS.md)에 기록했다.

## 내려받기

```sh
git clone https://github.com/KUCaver/rvc-controller.git
cd rvc-controller
```

포크해서 수정하려면 GitHub의 **Fork**를 누르고 본인 계정에 생성된 저장소를 clone한다. [협업 방법](CONTRIBUTING.md)을 참고한다.

## Windows에서 빌드·테스트

전제: PowerShell과 MSYS2 UCRT64의 GCC, G++, CMake(3.20 이상), Ninja, windres가 설치되어 있어야 한다. 기본 도구 폴더는 `C:\msys64\ucrt64\bin`이다. GoogleTest 1.15.2 소스는 저장소에 포함되어 있으며 빌드 도중 다운로드하지 않는다.

```powershell
.\build.ps1
# 도구 위치가 다른 경우
.\build.ps1 -ToolchainBin 'D:\msys64\ucrt64\bin'
```

빌드 스크립트는 컴파일, 전체 GoogleTest, 기본 입력 시나리오 실행을 수행한다. 결과는 로컬의 `results/`에 저장된다. 스크립트 실행이 제한된 PC에서는 아래 CMake 명령을 UCRT64 도구가 PATH에 있는 터미널에서 사용한다.

```powershell
cmake -S . -B build/cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build/cmake --parallel 4
ctest --test-dir build/cmake --output-on-failure
```

다른 컴파일러를 사용하던 빌드 폴더는 재사용하지 말고 새 `-B` 경로를 지정한다. Windows GCC 도구 조합에서 검증했으며, 다른 환경은 동일한 CMake 구성을 사용할 수 있지만 별도 확인이 필요하다.

## 시뮬레이션 실행

```powershell
.\build\cmake\rvc_demo.exe --scenario scenarios/demo.csv
.\build\cmake\rvc_demo.exe --scenario scenarios/demo.csv --realtime-ms 200
```

기본은 실제 대기 없는 논리 Tick 실행이다. `--realtime-ms 200`은 Tick 사이에 대기를 넣는 예시이며 요구된 고정 주기는 아니다. 센서·명령을 표로 보여주는 콘솔 시뮬레이션이다.

입력 CSV의 열은 `front,left,right,dust`이며 0은 미감지, 1은 감지다. 한 행이 한 Tick이다. 첫 행은 초기 센서 준비에도 사용한다. 출력의 `tick=0`은 초기화이고, `events`가 비어 있으면 직전 출력을 유지한다.

```csv
front,left,right,dust
0,0,0,0
0,0,0,1
1,0,0,1
```

`scenarios/demo.csv`와 위의 짧은 예시는 서로 다른 입력이다. 기본 시나리오의 [출력 예시](examples/demo_output.csv)에서 `C:OFF|M:RIGHT`는 청소 OFF 다음 우회전 명령을 의미한다. CSV만 수정하면 재컴파일 없이 실행할 수 있다. C 소스·헤더를 수정하면 다시 빌드한다.

## 코드 읽기

| 위치 | 역할 |
|---|---|
| `app/main.c` | 입력 CSV, Tick 발생, 결과 출력, 종료 |
| `include/rvc/` | 공개 타입·함수·장치 콜백 |
| `src/rvc.c` | 객체 수명주기와 공개 진입점 |
| `src/control/controller.c` | 센서 판단 호출, FSM, 경과 Tick, 명령 순서 |
| `src/perception/`, `src/sensing/` | 장애물·먼지 판단과 센서 인터페이스 |
| `src/actions/`, `src/interfaces/` | 동작을 모터·청소 명령으로 전달 |
| `adapters/mock/` | 센서값·오류 주입과 명령 기록 |
| `tests/` | GoogleTest와 시나리오 목록 |

문서: [구조도](docs/01_structure.md) · [입출력 계약](docs/02_contracts_and_decisions.md) · [함수 명세](docs/05_function_specifications.md) · [FSM 조건·Command](docs/06_implemented_design.md) · [읽기 순서](docs/07_code_reading_guide.md) · [테스트](tests/README.md).

## 구현 정책

- Controller가 회전 5 Tick·후진 3 Tick을 관리한다. 진입 순간은 0이다.
- 전방 장애물이 있으면 오른쪽, 왼쪽, 후진 순서로 선택한다.
- 회피 진입 전 청소 OFF를 보낸다. 이미 OFF이면 중복 명령을 생략한다.
- 전진 중 먼지가 감지되면 강화하고, 미감지 시 일반 청소로 복귀한다.
- 센서·출력 오류에는 OFF/STOP을 각각 시도하고 재초기화가 필요하도록 처리한다.
- 입력 보고와 Tick은 직렬 호출한다. 실제 장치·ISR 동시 실행·실시간 성능 보장은 검증 범위 밖이다.

전체 테스트에는 모듈 단위 검사와 제어 동작 검사가 함께 들어 있다. 테스트 개수 전체를 시스템 테스트 개수로 간주하지 않는다. `BUILD_TESTING=OFF`로 설정하면 C++/GoogleTest 없이 C 제품만 빌드할 수 있다.

2026-10-04 공유본을 새로 빌드해 GoogleTest 134개와 기본 시나리오가 통과했다. 환경과 검증 범위는 [공유본 검증 기록](docs/VALIDATION.md)을 참고한다.

GoogleTest의 출처와 라이선스는 [third_party/README.md](third_party/README.md)에 있다. 강의 원본, 팀 PPT/PDF, 로컬 리뷰 백업, 개인 PC 경로를 포함한 실행 로그는 공유하지 않는다.
