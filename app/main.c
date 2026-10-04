/*
 * 실행 예제의 시작점: CSV(또는 내장 시나리오)를 모의 센서 입력으로 전달한다.
 * Main은 Tick을 발생시키고 결과를 출력하며, 상태 전이와 명령 결정은 Controller가 맡는다.
 * 기본 실행은 기다리지 않는 논리 시간 방식이다. --realtime-ms는 Tick 사이 대기만 추가한다.
 * 읽기 순서: main의 인수 해석 -> 첫 입력/초기화 -> Tick 반복 -> shutdown/메모리 해제.
 */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <time.h>
#endif
#include "rvc/rvc.h"
#include "mock_device.h"

/* 벽시계 변경에 영향받지 않는 경과 시간(ms). POSIX 시계 조회 실패 시 프로세스를 종료한다. */
static uint64_t monotonic_ms(void)
{
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
    return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
#endif
}
/* deadline은 monotonic_ms와 같은 기준의 절대 시각이다. 도달할 때까지 대기한다.
 * 운영체제 스케줄링 때문에 늦게 깨어날 수 있으므로 정확한 실시간 주기를 보장하지 않는다.
 */
static void wait_until(uint64_t deadline)
{
    for (;;) {
        uint64_t now = monotonic_ms();
        if (now >= deadline) return;
        uint64_t remaining = deadline - now;
#ifdef _WIN32
        Sleep((DWORD)remaining);
#else
        struct timespec delay = {(time_t)(remaining / 1000U),
                                 (long)((remaining % 1000U) * 1000000U)};
        while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {}
#endif
    }
}
/* 상태 열거값을 CSV용 이름으로 변환한다. 범위 밖 값은 배열 접근 없이 INVALID로 표시한다. */
static const char *state_name(RvcState s)
{
    static const char *const names[] = {
        "UNINITIALIZED", "STOP_OFF", "F_ON", "F_POWER1",
        "LEFT_OFF", "RIGHT_OFF", "BACK_OFF"
    };
    return s >= RVC_STATE_UNINITIALIZED && s <= RVC_STATE_BACK_OFF
        ? names[(int)s] : "INVALID";
}
/* 모터 명령을 사람이 읽을 이름으로 변환한다. 반환 문자열은 해제할 필요가 없다. */
static const char *motor_name(int command)
{
    static const char *const names[] = {"STOP", "FORWARD", "LEFT", "RIGHT", "BACKWARD"};
    return command >= 0 && command < 5 ? names[command] : "INVALID";
}
/* 청소 명령 OFF/ON/UP을 출력용 문자열로 변환한다. UP은 강화 모드의 절대 설정이다. */
static const char *cleaner_name(int command)
{
    static const char *const names[] = {"OFF", "ON", "UP"};
    return command >= 0 && command < 3 ? names[command] : "INVALID";
}
/* 모의 장치에 성공적으로 기록된 명령을 실행 순서대로 출력한다. 실패한 시도는 포함되지 않는다. */
static void print_events(const RvcMockDevice *mock)
{
    for (size_t i = 0; i < mock->event_count; ++i) {
        const RvcMockEvent *event = &mock->events[i];
        if (i) putchar('|');
        if (event->kind == RVC_MOCK_MOTOR)
            printf("M:%s", motor_name(event->command));
        else
            printf("C:%s", cleaner_name(event->command));
    }
}
/* 성공한 초기화/Tick의 센서·상태·출력과 해당 구간의 명령 로그를 CSV 한 줄로 표시한다. */
static void print_frame(uint64_t tick, const RvcTelemetry *t, const RvcMockDevice *mock)
{
    printf("%" PRIu64 ",%d,%d,%d,%d,%s,%" PRIu32 ",%s,%s,",
           tick, t->sensors.obstacles.front_blocked,
           t->sensors.obstacles.left_blocked, t->sensors.obstacles.right_blocked,
           t->sensors.dust_detected, state_name(t->state), t->elapsed_ticks,
           motor_name(t->motor), cleaner_name(t->cleaner));
    print_events(mock);
    putchar('\n');
}
/* 다음 센서 행을 읽는다. file/out/line은 유효한 포인터를 전달하는 내부 호출용이다.
 * 반환: 1=정상 행, 0=파일 끝, -1=형식/읽기 오류. 정상일 때만 *out을 갱신한다.
 * 공백 행·# 주석·헤더는 건너뛰고, F/L/R/D에는 0 또는 1만 허용한다.
 * *line은 오류 위치 보고를 위해 읽은 실제 줄 수를 누적한다. 정상 데이터 한 행이 한 Tick이다.
 */
static int read_frame(FILE *file, RvcSensorSnapshot *out, unsigned *line)
{
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), file)) {
        ++*line;
        if (!strchr(buffer, '\n') && !feof(file)) return -1;
        buffer[strcspn(buffer, "\r\n")] = '\0';
        char *start = buffer;
        while (isspace((unsigned char)*start)) ++start;
        if (!*start || *start == '#') continue;
        if (strcmp(start, "front,left,right,dust") == 0) continue;
        char f, l, r, d, extra;
        if (sscanf(start, " %c , %c , %c , %c %c", &f, &l, &r, &d, &extra) != 4)
            return -1;
        if ((f != '0' && f != '1') || (l != '0' && l != '1') ||
            (r != '0' && r != '1') || (d != '0' && d != '1')) return -1;
        *out = (RvcSensorSnapshot){{f == '1', l == '1', r == '1'}, d == '1'};
        return 1;
    }
    return ferror(file) ? -1 : 0;
}
/* CLI 진입점. 성공은 EXIT_SUCCESS, 인수·파일·장치 처리 실패는 EXIT_FAILURE로 반환한다. */
int main(int argc, char **argv)
{
    /* 1. 실행 방법 결정: 시나리오를 생략하면 내장 입력, 주기를 생략하면 빠른 논리 Tick 실행. */
    const char *scenario = NULL;
    unsigned period_ms = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario = argv[++i];
        } else if (strcmp(argv[i], "--realtime-ms") == 0 && i + 1 < argc) {
            char *end = NULL;
            errno = 0;
            unsigned long value = strtoul(argv[++i], &end, 10);
            if (errno || !*argv[i] || *end || value == 0 || value > 60000) {
                fputs("Period must be an integer from 1 to 60000 ms.\n", stderr);
                return EXIT_FAILURE;
            }
            period_ms = (unsigned)value;
        } else if (strcmp(argv[i], "--help") == 0) {
            puts("rvc_demo [--scenario file.csv] [--realtime-ms 200]");
            puts("CSV: front,left,right,dust (0/1). Each row advances one Tick.");
            puts("No period: deterministic fast run. Default: built-in scenario.");
            return EXIT_SUCCESS;
        } else {
            fputs("Invalid argument. Use --help.\n", stderr);
            return EXIT_FAILURE;
        }
    }
    /* 전진/강화/정상 복귀/우회전/후진/좌회전/전진을 관찰하는 기본 입력 시퀀스. */
    static const RvcSensorSnapshot builtin[] = {
        {{0,0,0},0}, {{0,0,0},1}, {{0,0,0},0},
        {{1,0,0},1}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0},
        {{0,0,0},0}, {{0,0,0},0}, {{1,1,1},0},
        {{0,0,1},0}, {{0,0,1},0}, {{0,0,1},0},
        {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}, {{0,0,0},0}
    };
    const size_t builtin_count = sizeof(builtin) / sizeof(builtin[0]);
    /* 2. 첫 행을 미리 읽는다. 이 입력은 초기 센서 읽기와 첫 번째 Tick 양쪽에 사용한다.
     * 초기화 결과는 tick=0으로 표시하지만, 초기화 자체가 논리 Tick을 소비하지는 않는다.
     */
    FILE *file = scenario ? fopen(scenario, "r") : NULL;
    if (scenario && !file) { perror("scenario"); return EXIT_FAILURE; }
    unsigned line = 0;
    size_t index = 0;
    RvcSensorSnapshot input = builtin[0];
    int available = file ? read_frame(file, &input, &line) : 1;
    if (available != 1) {
        fputs("Scenario is empty or its first row is invalid.\n", stderr);
        if (file) fclose(file);
        return EXIT_FAILURE;
    }
    /* 장치 콜백의 context가 이 mock을 가리키므로 robot을 쓰는 동안 mock의 수명이 유지되어야 한다. */
    RvcMockDevice mock;
    rvc_mock_init(&mock);
    rvc_mock_set_inputs(&mock, input);
    RvcDevice device = rvc_mock_device(&mock);
    Rvc *robot = rvc_create(&device);
    if (!robot) { if (file) fclose(file); return EXIT_FAILURE; }
    int exit_code = EXIT_SUCCESS;
    RvcStatus result = rvc_initialize(robot);
    if (result != RVC_OK) {
        fprintf(stderr, "Initialization failed: %d\n", (int)result);
        exit_code = EXIT_FAILURE;
    } else {
        RvcTelemetry telemetry;
        result = rvc_get_telemetry(robot, &telemetry);
        if (result != RVC_OK) {
            exit_code = EXIT_FAILURE;
        } else {
            puts("tick,F,L,R,D,state,elapsed,motor,cleaner,events");
            print_frame(0, &telemetry, &mock);
            uint64_t tick = 0;
            uint64_t deadline = period_ms ? monotonic_ms() + period_ms : 0;
            bool previous_front = input.obstacles.front_blocked;
            /* 3. 입력 한 행 -> 전방 변화 통지 -> Controller 한 번 실행 -> 결과 출력.
             * FSM의 elapsed_ticks는 rvc_tick 호출로만 증가하며, 실제 대기 시간은 판단에 넣지 않는다.
             */
            while (available == 1) {
                if (period_ms) wait_until(deadline);
                rvc_mock_clear_events(&mock);
                rvc_mock_set_inputs(&mock, input);
                /* 전방은 변화 통지 방식, 좌/우/먼지는 Tick 내부의 주기 읽기 방식으로 모사한다. */
                if (input.obstacles.front_blocked != previous_front) {
                    result = rvc_report_front(robot, input.obstacles.front_blocked);
                    previous_front = input.obstacles.front_blocked;
                }
                if (result == RVC_OK) result = rvc_tick(robot);
                if (result == RVC_OK) result = rvc_get_telemetry(robot, &telemetry);
                if (result != RVC_OK) {
                    fprintf(stderr, "Tick failed: %d\n", (int)result);
                    exit_code = EXIT_FAILURE;
                    break;
                }
                print_frame(++tick, &telemetry, &mock);
                if (period_ms) {
                    deadline += period_ms;
                    uint64_t now = monotonic_ms();
                    /* 처리가 지연됐으면 다음 대기 시각을 다시 잡는다. 누락 시간을 몰아서 Tick으로 만들지 않는다. */
                    if (deadline < now) deadline = now + period_ms;
                }
                if (file) available = read_frame(file, &input, &line);
                else if (++index < builtin_count) input = builtin[index];
                else available = 0;
                if (available < 0) {
                    fprintf(stderr, "Invalid scenario at line %u.\n", line);
                    exit_code = EXIT_FAILURE;
                }
            }
        }
    }
    /* 4. 이 공통 종료 경로로 들어온 정상 종료와 입력/장치 오류는 OFF와 STOP을 시도한다.
     * POSIX 시계 조회 자체의 치명적 실패는 monotonic_ms에서 즉시 exit하는 별도 경로다.
     * shutdown은 장치 종료, destroy는 메모리 해제다. destroy만 호출해도 정지되는 것은 아니다.
     */
    rvc_mock_clear_events(&mock);
    if (rvc_shutdown(robot) != RVC_OK) exit_code = EXIT_FAILURE;
    printf("# shutdown,");
    print_events(&mock);
    putchar('\n');
    rvc_destroy(robot);
    if (file) fclose(file);
    return exit_code;
}
