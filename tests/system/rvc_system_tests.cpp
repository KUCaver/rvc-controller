/*
 * RVC 시스템 테스트(블랙박스). 대상: 저장소 루트의 controller.c / controller.h.
 *
 * 시스템 경계(controller.h 머리말 "입력: 장애물 F/L/R, 먼지 D 및 논리 Tick. 출력: Motor/Cleaner Command")
 *   입력: 전방 보고(rvc_report_front), 좌·우·먼지 센서값, Tick(rvc_tick)
 *   출력: Motor Direction, Cleaner Command
 * 테스트 더블: RvcDevice 콜백 6개를 이 파일의 FakeDevice가 구현한다(하드웨어 접근 지점).
 *   controller.c의 Sensor/Motor/Cleaner Interface 코드까지 실제로 실행된다.
 *   controller.c는 RVC_NO_MAIN으로 빌드하므로 내장 모의 장치·main은 사용하지 않는다.
 * 판정: FakeDevice에 성공적으로 기록된 출력의 순서·개수만 비교한다. 내부 상태는 단언하지 않는다.
 * 시간: rvc_tick() 호출 1회 = 논리 Tick 1회. 실제 대기는 하지 않는다.
 *
 * 기대값의 출처: controller.c 머리말 "본 구현에서 채택한 FSM/시간 정책"(POL-n, 글머리표 순서)과
 * controller.h의 공개 함수 계약(H-*).
 *   POL-1 초기 상태 STOP_OFF: STOP, OFF 순서로 출력한 뒤 센서 입력을 확보
 *   POL-2 앞이 열린 전진 판단: 먼지 감지 또는 강화 유지 중이면 FORWARD+UP, 그 외 FORWARD+ON
 *   POL-3 앞이 막히면 우측 열림 RIGHT, 아니면 좌측 열림 LEFT, 양쪽 막힘 BACKWARD. 회피 중 청소 OFF
 *   POL-4 회전은 후속 5번째 Tick에 다시 판단. 진입 시 elapsed=0
 *   POL-5 후진 중 매 Tick 앞 → 오른쪽 → 왼쪽 순서로 빈 방향 확인, 있으면 그 Tick에 전진/회전으로 전환.
 *         모두 막히면 후진 유지, 후속 3번째 Tick에 후진 구간 재시작. 전환한 회전 시간은 0부터
 *   POL-6 먼지 감지 시 강화 유지 3으로 갱신: 마지막 감지 t=0, 미감지 t=1,2는 UP, t=3에 ON.
 *         재감지 우선. 회피 진입/초기화/종료/오류 시 취소. 회피 중 먼지는 기억하지 않고 전진 복귀 Tick의 값 사용
 *   POL-7 회피 진입은 필요 시 청소 OFF → 방향 명령, 전진 진입은 FORWARD → ON/UP.
 *         같은 전진 상태/출력은 재전송하지 않으며, 회피 재진입은 새 방향 명령을 보냄
 *   POL-9 센서/출력 오류 시 OFF와 STOP을 각각 시도하고 재초기화 전까지 Tick 거부
 *   H-INIT     rvc_initialize: STOP → OFF → 센서 초기 입력. 실패 후 미준비 유지
 *   H-SHUTDOWN rvc_shutdown: OFF와 STOP을 각각 시도한 뒤 미준비 상태로 전환
 *   H-TICK     rvc_tick: 미초기화 NOT_READY. 장치 오류 시 OFF/STOP 시도, 재초기화 요구
 *   H-FRONT    rvc_report_front: 전방 캐시만 갱신, 명령은 다음 Tick에서 결정.
 *              RvcDevice.read_front는 초기 읽기 전용
 * (POL-8 "Controller가 논리 Tick을 관리"는 시간 진행 방식 자체로, 모든 테스트가 따른다.)
 *
 * 구성: A 결정표, B 먼지, C 유지 시간, D 수명주기·오류, E 특이 케이스,
 *       F 입력 변화 전수·무작위 비교(SpecModel = 위 POL 문장만으로 만든 기대값 모델. 제품 코드를 호출하지 않음).
 */
#include <gtest/gtest.h>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "controller.h"

namespace {

/* 장치에 기록된 출력 순서. "M:<모터 명령>" 또는 "C:<청소 명령>". */
using Outputs = std::vector<std::string>;

/* 출력 목록을 "a,b" 문자열로 만든다(실패 메시지·실행 기록용). */
std::string Join(const Outputs &o)
{
    if (o.empty()) return "(none)";
    std::string text;
    for (const std::string &x : o) text += (text.empty() ? "" : ",") + x;
    return text;
}

/* 한 Tick에 시스템 경계로 들어오는 센서 입력. true = 장애물 감지 / 먼지 감지. */
struct Sensors {
    bool front;
    bool left;
    bool right;
    bool dust;
};
/* 표를 짧게 쓰기 위한 0/1 입력 생성기. 순서는 F, L, R, D. */
constexpr Sensors In(int front, int left, int right, int dust = 0)
{
    return {front != 0, left != 0, right != 0, dust != 0};
}
constexpr Sensors kClear = In(0, 0, 0, 0);
constexpr Sensors kAllBlocked = In(1, 1, 1, 0);

/* ---- 테스트 더블: RvcDevice 콜백 계층의 가짜 장치 ---- */
enum DeviceOp { kReadFront, kReadLeft, kReadRight, kReadDust, kWriteMotor, kWriteCleaner, kOpCount };

struct FakeDevice {
    Sensors inputs = kClear;
    Outputs log;                    /* 성공한 쓰기만 시간 순서대로 기록 */
    Outputs attempts;               /* 모든 쓰기 시도와 실패한 읽기를 순서대로 기록. 실패는 "(fail)" 표시 */
    int fail_next[kOpCount] = {};   /* 해당 작업의 다음 N회 호출을 IO_ERROR로 실패시킨다 */
};

bool ConsumeFault(FakeDevice *d, DeviceOp op)
{
    if (d->fail_next[op] == 0) return false;
    --d->fail_next[op];
    return true;
}
RvcStatus ReadInput(void *context, bool *out, DeviceOp op)
{
    static const char *const names[] = {"FRONT", "LEFT", "RIGHT", "DUST"};
    auto *d = static_cast<FakeDevice *>(context);
    if (!d || !out) return RVC_INVALID_ARGUMENT;
    if (ConsumeFault(d, op)) {
        d->attempts.push_back(std::string("R:") + names[op] + "(fail)");
        return RVC_IO_ERROR;
    }
    switch (op) {
    case kReadFront: *out = d->inputs.front; break;
    case kReadLeft: *out = d->inputs.left; break;
    case kReadRight: *out = d->inputs.right; break;
    default: *out = d->inputs.dust; break;
    }
    return RVC_OK;
}
RvcStatus ReadFront(void *c, bool *out) { return ReadInput(c, out, kReadFront); }
RvcStatus ReadLeft(void *c, bool *out) { return ReadInput(c, out, kReadLeft); }
RvcStatus ReadRight(void *c, bool *out) { return ReadInput(c, out, kReadRight); }
RvcStatus ReadDust(void *c, bool *out) { return ReadInput(c, out, kReadDust); }

std::string MotorName(RvcMotorCommand command)
{
    static const char *const names[] = {"STOP", "FORWARD", "LEFT", "RIGHT", "BACKWARD"};
    const int i = static_cast<int>(command);
    return i >= 0 && i < 5 ? names[i] : "INVALID";
}
std::string CleanerName(RvcCleanerCommand command)
{
    static const char *const names[] = {"OFF", "ON", "UP"};
    const int i = static_cast<int>(command);
    return i >= 0 && i < 3 ? names[i] : "INVALID";
}
RvcStatus WriteMotor(void *context, RvcMotorCommand command)
{
    auto *d = static_cast<FakeDevice *>(context);
    if (!d) return RVC_INVALID_ARGUMENT;
    const std::string entry = "M:" + MotorName(command);
    if (ConsumeFault(d, kWriteMotor)) {
        d->attempts.push_back(entry + "(fail)");
        return RVC_IO_ERROR;
    }
    d->attempts.push_back(entry);
    d->log.push_back(entry);
    return RVC_OK;
}
RvcStatus WriteCleaner(void *context, RvcCleanerCommand command)
{
    auto *d = static_cast<FakeDevice *>(context);
    if (!d) return RVC_INVALID_ARGUMENT;
    const std::string entry = "C:" + CleanerName(command);
    if (ConsumeFault(d, kWriteCleaner)) {
        d->attempts.push_back(entry + "(fail)");
        return RVC_IO_ERROR;
    }
    d->attempts.push_back(entry);
    d->log.push_back(entry);
    return RVC_OK;
}

/* 테스트 시작 시점. 모두 초기화와 센서 입력 이력만으로 도달한다. */
enum class Start {
    StopOff,         /* 초기화 직후 */
    ForwardOn,       /* 전진 + 일반 청소 */
    ForwardPower,    /* 전진 + 강화 청소(먼지 감지 Tick) */
    RightTurnExpiry, /* 우회전 진입 후 4 Tick 유지, 다음 Tick이 후속 5번째 Tick */
    BackwardTick1,   /* 후진 진입 직후, 다음 Tick이 후속 1번째 Tick */
    BackwardTick3    /* 후진 진입 후 모두 막힌 채 2 Tick 유지, 다음 Tick이 후속 3번째 Tick */
};

class RvcSystemTest : public ::testing::Test {
protected:
    FakeDevice dev;
    Rvc *robot = nullptr;
    bool reported_front = false;

    void SetUp() override
    {
        const RvcDevice device = {&dev, ReadFront, ReadLeft, ReadRight, ReadDust, WriteMotor, WriteCleaner};
        robot = rvc_create(&device);
        ASSERT_NE(robot, nullptr);
    }
    void TearDown() override { rvc_destroy(robot); }

    /* 마지막 조회 이후 장치에 기록된 출력을 꺼내고 기록을 비운다. */
    Outputs TakeOutputs()
    {
        Outputs out;
        out.swap(dev.log);
        return out;
    }
    /* 마지막 조회 이후의 모든 쓰기 시도(실패 포함)와 실패한 읽기를 꺼내고 비운다. */
    Outputs TakeAttempts()
    {
        Outputs out;
        out.swap(dev.attempts);
        return out;
    }
    /* 전원 투입: 초기 센서값을 두고 initialize. 초기화 중 장치 출력을 반환한다. */
    Outputs PowerOn(const Sensors &s = kClear)
    {
        dev.inputs = s;
        reported_front = s.front;
        EXPECT_EQ(rvc_initialize(robot), RVC_OK);
        return TakeOutputs();
    }
    /* 전방 센서가 바뀌었으면 보고한다(H-FRONT: 전방은 보고 방식 입력). */
    void ReportFrontIfChanged(const Sensors &s)
    {
        if (s.front != reported_front) {
            EXPECT_EQ(rvc_report_front(robot, s.front), RVC_OK);
            reported_front = s.front;
        }
    }
    /* 센서 입력 → 전방 보고 → Tick 1회. 그 Tick의 장치 출력을 반환한다. */
    Outputs Step(const Sensors &s)
    {
        dev.inputs = s;
        ReportFrontIfChanged(s);
        EXPECT_EQ(rvc_tick(robot), RVC_OK);
        return TakeOutputs();
    }
    /* ticks번 연속으로 출력이 없어야 한다. */
    void ExpectHold(int ticks, const Sensors &s)
    {
        for (int i = 1; i <= ticks; ++i)
            EXPECT_EQ(Step(s), Outputs{}) << "hold tick " << i;
    }
    /* 잠긴 상태: Tick은 NOT_READY이고 장치 출력이 없다. */
    void ExpectLocked()
    {
        EXPECT_EQ(rvc_tick(robot), RVC_NOT_READY);
        EXPECT_EQ(TakeOutputs(), Outputs{});
    }
    /* 지정 시점까지 입력 이력으로 진행한다. 도달 과정의 출력도 명세대로인지 확인한다. */
    void Reach(Start start)
    {
        ASSERT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));                     /* POL-1 */
        switch (start) {
        case Start::StopOff:
            break;
        case Start::ForwardOn:
            ASSERT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));             /* POL-2 */
            break;
        case Start::ForwardPower:
            ASSERT_EQ(Step(In(0, 0, 0, 1)), Outputs({"M:FORWARD", "C:UP"}));     /* POL-2 */
            break;
        case Start::RightTurnExpiry:
            ASSERT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
            ASSERT_EQ(Step(In(1, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));         /* POL-3 */
            ExpectHold(4, kClear);                                               /* POL-4 */
            break;
        case Start::BackwardTick1:
            ASSERT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
            ASSERT_EQ(Step(kAllBlocked), Outputs({"C:OFF", "M:BACKWARD"}));      /* POL-3 */
            break;
        case Start::BackwardTick3:
            ASSERT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
            ASSERT_EQ(Step(kAllBlocked), Outputs({"C:OFF", "M:BACKWARD"}));
            ExpectHold(2, kAllBlocked);                                          /* POL-5 */
            break;
        }
        ASSERT_FALSE(HasFailure()) << "precondition: could not reach start point";
        dev.attempts.clear();
    }
};

/* ===================================================================== */
/* A. 장애물(F/L/R 8가지) × 먼지 결정표: 시작 시점에서 Tick 1회의 출력     */
/* ===================================================================== */

struct DecisionCase {
    const char *name;
    const char *req;
    Start start;
    Sensors input;
    Outputs expected;
};

class DecisionTableTest : public RvcSystemTest,
                          public ::testing::WithParamInterface<DecisionCase> {};

TEST_P(DecisionTableTest, OneTickOutputsMatchSpecification)
{
    const DecisionCase &c = GetParam();
    SCOPED_TRACE(c.req);
    ASSERT_NO_FATAL_FAILURE(Reach(c.start));
    if (HasFailure()) return;
    EXPECT_EQ(Step(c.input), c.expected);
}

// clang-format off
const DecisionCase kDecisionTable[] = {
    /* 초기화 직후. F=0이면 L/R 무관 전진, F=1이면 오른쪽 → 왼쪽 → 후진.
     * 청소는 이미 OFF이므로 회피에서 OFF를 다시 보내지 않는다(POL-7 "필요 시"). */
    {"StopOff_F0L0R0_D0", "POL-2 POL-7",       Start::StopOff, In(0,0,0,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F0L0R1_D0", "POL-2 POL-7",       Start::StopOff, In(0,0,1,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F0L1R0_D0", "POL-2 POL-7",       Start::StopOff, In(0,1,0,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F0L1R1_D0", "POL-2 POL-7",       Start::StopOff, In(0,1,1,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F1L0R0_D0", "POL-3 POL-7",       Start::StopOff, In(1,0,0,0), {"M:RIGHT"}},
    {"StopOff_F1L0R1_D0", "POL-3 POL-7",       Start::StopOff, In(1,0,1,0), {"M:LEFT"}},
    {"StopOff_F1L1R0_D0", "POL-3 POL-7",       Start::StopOff, In(1,1,0,0), {"M:RIGHT"}},
    {"StopOff_F1L1R1_D0", "POL-3 POL-7",       Start::StopOff, In(1,1,1,0), {"M:BACKWARD"}},
    {"StopOff_F0L0R0_D1", "POL-2 POL-6 POL-7", Start::StopOff, In(0,0,0,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F0L0R1_D1", "POL-2 POL-6 POL-7", Start::StopOff, In(0,0,1,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F0L1R0_D1", "POL-2 POL-6 POL-7", Start::StopOff, In(0,1,0,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F0L1R1_D1", "POL-2 POL-6 POL-7", Start::StopOff, In(0,1,1,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F1L0R0_D1", "POL-3 POL-7",       Start::StopOff, In(1,0,0,1), {"M:RIGHT"}},
    {"StopOff_F1L0R1_D1", "POL-3 POL-7",       Start::StopOff, In(1,0,1,1), {"M:LEFT"}},
    {"StopOff_F1L1R0_D1", "POL-3 POL-7",       Start::StopOff, In(1,1,0,1), {"M:RIGHT"}},
    {"StopOff_F1L1R1_D1", "POL-3 POL-7",       Start::StopOff, In(1,1,1,1), {"M:BACKWARD"}},

    /* 전진 + 일반 청소. F=0이면 모터 재명령 없음, 먼지가 생기면 UP만.
     * F=1이면 청소 OFF가 회피 모터 명령보다 먼저(POL-7). */
    {"ForwardOn_F0L0R0_D0", "POL-2 POL-7",       Start::ForwardOn, In(0,0,0,0), {}},
    {"ForwardOn_F0L0R1_D0", "POL-2 POL-7",       Start::ForwardOn, In(0,0,1,0), {}},
    {"ForwardOn_F0L1R0_D0", "POL-2 POL-7",       Start::ForwardOn, In(0,1,0,0), {}},
    {"ForwardOn_F0L1R1_D0", "POL-2 POL-7",       Start::ForwardOn, In(0,1,1,0), {}},
    {"ForwardOn_F1L0R0_D0", "POL-3 POL-7",       Start::ForwardOn, In(1,0,0,0), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L0R1_D0", "POL-3 POL-7",       Start::ForwardOn, In(1,0,1,0), {"C:OFF", "M:LEFT"}},
    {"ForwardOn_F1L1R0_D0", "POL-3 POL-7",       Start::ForwardOn, In(1,1,0,0), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L1R1_D0", "POL-3 POL-7",       Start::ForwardOn, In(1,1,1,0), {"C:OFF", "M:BACKWARD"}},
    {"ForwardOn_F0L0R0_D1", "POL-2 POL-6 POL-7", Start::ForwardOn, In(0,0,0,1), {"C:UP"}},
    {"ForwardOn_F0L0R1_D1", "POL-2 POL-6 POL-7", Start::ForwardOn, In(0,0,1,1), {"C:UP"}},
    {"ForwardOn_F0L1R0_D1", "POL-2 POL-6 POL-7", Start::ForwardOn, In(0,1,0,1), {"C:UP"}},
    {"ForwardOn_F0L1R1_D1", "POL-2 POL-6 POL-7", Start::ForwardOn, In(0,1,1,1), {"C:UP"}},
    {"ForwardOn_F1L0R0_D1", "POL-3 POL-7",       Start::ForwardOn, In(1,0,0,1), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L0R1_D1", "POL-3 POL-7",       Start::ForwardOn, In(1,0,1,1), {"C:OFF", "M:LEFT"}},
    {"ForwardOn_F1L1R0_D1", "POL-3 POL-7",       Start::ForwardOn, In(1,1,0,1), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L1R1_D1", "POL-3 POL-7",       Start::ForwardOn, In(1,1,1,1), {"C:OFF", "M:BACKWARD"}},

    /* 회전 후속 5번째 Tick: 일반 판단. 청소는 이미 OFF. 같은 회피 재선택은 새 방향 명령(POL-7). */
    {"RightTurnExpiry_F0L0R0", "POL-4 POL-2 POL-7", Start::RightTurnExpiry, In(0,0,0), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F0L0R1", "POL-4 POL-2 POL-7", Start::RightTurnExpiry, In(0,0,1), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F0L1R0", "POL-4 POL-2 POL-7", Start::RightTurnExpiry, In(0,1,0), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F0L1R1", "POL-4 POL-2 POL-7", Start::RightTurnExpiry, In(0,1,1), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F1L0R0", "POL-4 POL-3 POL-7", Start::RightTurnExpiry, In(1,0,0), {"M:RIGHT"}},
    {"RightTurnExpiry_F1L0R1", "POL-4 POL-3 POL-7", Start::RightTurnExpiry, In(1,0,1), {"M:LEFT"}},
    {"RightTurnExpiry_F1L1R0", "POL-4 POL-3 POL-7", Start::RightTurnExpiry, In(1,1,0), {"M:RIGHT"}},
    {"RightTurnExpiry_F1L1R1", "POL-4 POL-3 POL-7", Start::RightTurnExpiry, In(1,1,1), {"M:BACKWARD"}},

    /* 후진 후속 1번째 Tick: 빈 방향이 있으면 시간을 기다리지 않고 즉시 전환(앞 → 오른쪽 → 왼쪽).
     * 모두 막히면 후진 유지(출력 없음). */
    {"BackwardTick1_F0L0R0", "POL-5 POL-2 POL-7", Start::BackwardTick1, In(0,0,0), {"M:FORWARD", "C:ON"}},
    {"BackwardTick1_F0L0R1", "POL-5 POL-2 POL-7", Start::BackwardTick1, In(0,0,1), {"M:FORWARD", "C:ON"}},
    {"BackwardTick1_F0L1R0", "POL-5 POL-2 POL-7", Start::BackwardTick1, In(0,1,0), {"M:FORWARD", "C:ON"}},
    {"BackwardTick1_F0L1R1", "POL-5 POL-2 POL-7", Start::BackwardTick1, In(0,1,1), {"M:FORWARD", "C:ON"}},
    {"BackwardTick1_F1L0R0", "POL-5 POL-7",       Start::BackwardTick1, In(1,0,0), {"M:RIGHT"}},
    {"BackwardTick1_F1L0R1", "POL-5 POL-7",       Start::BackwardTick1, In(1,0,1), {"M:LEFT"}},
    {"BackwardTick1_F1L1R0", "POL-5 POL-7",       Start::BackwardTick1, In(1,1,0), {"M:RIGHT"}},
    {"BackwardTick1_F1L1R1", "POL-5",             Start::BackwardTick1, In(1,1,1), {}},

    /* 후진 후속 3번째 Tick: 빈 방향이 있으면 전환, 모두 막히면 후진 구간 재시작(BACKWARD 재전송). */
    {"BackwardTick3_F0L0R0", "POL-5 POL-2 POL-7", Start::BackwardTick3, In(0,0,0), {"M:FORWARD", "C:ON"}},
    {"BackwardTick3_F0L0R1", "POL-5 POL-2 POL-7", Start::BackwardTick3, In(0,0,1), {"M:FORWARD", "C:ON"}},
    {"BackwardTick3_F0L1R0", "POL-5 POL-2 POL-7", Start::BackwardTick3, In(0,1,0), {"M:FORWARD", "C:ON"}},
    {"BackwardTick3_F0L1R1", "POL-5 POL-2 POL-7", Start::BackwardTick3, In(0,1,1), {"M:FORWARD", "C:ON"}},
    {"BackwardTick3_F1L0R0", "POL-5 POL-7",       Start::BackwardTick3, In(1,0,0), {"M:RIGHT"}},
    {"BackwardTick3_F1L0R1", "POL-5 POL-7",       Start::BackwardTick3, In(1,0,1), {"M:LEFT"}},
    {"BackwardTick3_F1L1R0", "POL-5 POL-7",       Start::BackwardTick3, In(1,1,0), {"M:RIGHT"}},
    {"BackwardTick3_F1L1R1", "POL-5 POL-7",       Start::BackwardTick3, In(1,1,1), {"M:BACKWARD"}},
};
// clang-format on

INSTANTIATE_TEST_SUITE_P(Spec, DecisionTableTest, ::testing::ValuesIn(kDecisionTable),
                         [](const ::testing::TestParamInfo<DecisionCase> &info) {
                             return std::string(info.param.name);
                         });

/* ===================================================================== */
/* B. 먼지 강화와 유지 시간                                               */
/* ===================================================================== */

/* REQ: POL-6, POL-7 — 마지막 감지 후 미감지 t=1,2는 UP 유지(출력 없음), t=3에 ON. 모터 재명령 없음. */
TEST_F(RvcSystemTest, DustBoostHoldsUntilThirdTickAfterLastDetection)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs({"C:UP"}));  /* t=0 감지 */
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});          /* t=1 */
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});          /* t=2 */
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs({"C:ON"}));  /* t=3 */
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});
}

/* REQ: POL-6 — 재감지는 만료보다 우선하며 유지 시간이 다시 3부터 시작된다. */
TEST_F(RvcSystemTest, DustRedetectionRestartsBoostHold)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs({"C:UP"}));
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs{});          /* 재감지: 새 t=0 */
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});          /* t=1 */
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});          /* t=2 */
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs({"C:ON"}));  /* t=3 */
}

/* REQ: POL-3, POL-7 — 강화 청소 중 장애물: 먼지가 있어도 OFF 후 회피. */
TEST_F(RvcSystemTest, ObstacleDuringPowerCleaningTurnsCleanerOffBeforeTurning)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardPower));
    EXPECT_EQ(Step(In(1, 0, 0, 1)), Outputs({"C:OFF", "M:RIGHT"}));
}

/* REQ: POL-6 — 회피 진입은 강화 유지를 취소한다. 회전 후 먼지가 없으면 ON으로 전진. */
TEST_F(RvcSystemTest, AvoidanceCancelsBoostHold)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardPower));
    ASSERT_EQ(Step(In(1, 0, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    ExpectHold(4, kClear);
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: POL-3, POL-4, POL-6 — 회전 중에는 먼지가 바뀌어도 청소 출력이 없다. */
TEST_F(RvcSystemTest, DustChangesDuringTurnProduceNoCleanerOutput)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(1, 0, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});
}

/* REQ: POL-6 — 회피 중 먼지는 기억하지 않는다. 전진 복귀 Tick에 먼지가 없으면 ON. */
TEST_F(RvcSystemTest, DustSeenOnlyDuringTurnIsNotRemembered)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(1, 0, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    ExpectHold(4, In(0, 0, 0, 1));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: POL-4, POL-6, POL-7 — 회피 종료 후 전진은 그 Tick의 먼지값으로 청소 모드를 정한다. */
TEST_F(RvcSystemTest, ForwardAfterTurnUsesDustOfThatTick)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::RightTurnExpiry));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs({"M:FORWARD", "C:UP"}));
}

/* REQ: POL-6, H-SHUTDOWN, H-INIT — 종료·재초기화는 강화 유지를 취소한다. */
TEST_F(RvcSystemTest, ReinitializationCancelsBoostHold)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardPower));
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* ===================================================================== */
/* C. 회전·후진 유지 시간                                                  */
/* ===================================================================== */

struct TurnCase {
    const char *name;
    Sensors entry;      /* 전진 중 이 입력으로 회전 진입 */
    Outputs entry_out;
    Outputs reselect_out; /* 만료 Tick에 같은 입력을 다시 주었을 때 */
};

class TurnTest : public RvcSystemTest, public ::testing::WithParamInterface<TurnCase> {
protected:
    void Enter()
    {
        ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
        ASSERT_EQ(Step(GetParam().entry), GetParam().entry_out);
    }
};

/* REQ: POL-4 — 진입 후 4 Tick은 출력 없이 유지, 후속 5번째 Tick에 다시 판단. */
TEST_P(TurnTest, HoldsFourTicksThenDecidesOnFifth)
{
    ASSERT_NO_FATAL_FAILURE(Enter());
    ExpectHold(4, kClear);
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: POL-4 — 유지 중 장애물·먼지 입력이 바뀌어도 동작을 바꾸지 않는다. */
TEST_P(TurnTest, InputChangesDuringHoldDoNotInterrupt)
{
    ASSERT_NO_FATAL_FAILURE(Enter());
    const Sensors noise[] = {In(1, 1, 1, 1), In(0, 0, 0, 1), In(1, 0, 0, 0), In(0, 1, 1, 0)};
    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(Step(noise[i]), Outputs{}) << "hold tick " << i + 1;
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: POL-4, POL-7 — 만료 시 같은 회전을 다시 고르면 방향 명령을 다시 보내고 시간이 0부터. */
TEST_P(TurnTest, ReselectingSameTurnRetriggersAndRestartsTimer)
{
    ASSERT_NO_FATAL_FAILURE(Enter());
    ExpectHold(4, kClear);
    EXPECT_EQ(Step(GetParam().entry), GetParam().reselect_out);
    ExpectHold(4, kClear);
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

INSTANTIATE_TEST_SUITE_P(Spec, TurnTest,
                         ::testing::Values(
                             TurnCase{"RightTurn", In(1, 0, 0), {"C:OFF", "M:RIGHT"}, {"M:RIGHT"}},
                             TurnCase{"LeftTurn", In(1, 0, 1), {"C:OFF", "M:LEFT"}, {"M:LEFT"}}),
                         [](const ::testing::TestParamInfo<TurnCase> &info) {
                             return std::string(info.param.name);
                         });

/* REQ: POL-5, POL-7 — 모두 막혀 있으면 후진을 유지하고 후속 3번째 Tick마다 후진 구간을 다시 시작한다. */
TEST_F(RvcSystemTest, BackwardWhileAllBlockedRestartsEveryThirdTick)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::BackwardTick1));
    for (int segment = 1; segment <= 3; ++segment) {
        SCOPED_TRACE(segment);
        ExpectHold(2, kAllBlocked);
        EXPECT_EQ(Step(kAllBlocked), Outputs({"M:BACKWARD"}));
    }
}

/* REQ: POL-5, POL-4 — 후진에서 전환한 회전의 시간은 0부터 센다(4 Tick 유지 후 5번째에 판단). */
TEST_F(RvcSystemTest, TurnEnteredFromBackwardCountsFromZero)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::BackwardTick1));
    EXPECT_EQ(Step(In(1, 1, 0)), Outputs({"M:RIGHT"}));
    ExpectHold(4, kClear);
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: POL-1~POL-7 — 전진·먼지 유지·우회전·후진 유지/재시작·후진 중 회전 전환·복귀를 Tick 단위로 확인. */
TEST_F(RvcSystemTest, EndToEndScenarioTickByTick)
{
    struct Row {
        const char *req;
        Sensors input;
        Outputs expected;
    };
    // clang-format off
    const Row rows[] = {
        {"POL-2",       In(0,0,0,0), {"M:FORWARD", "C:ON"}},     /* 1 */
        {"POL-6",       In(0,0,0,1), {"C:UP"}},                  /* 2  t=0 */
        {"POL-6",       In(0,0,0,0), {}},                        /* 3  t=1 */
        {"POL-6",       In(0,0,0,0), {}},                        /* 4  t=2 */
        {"POL-6",       In(0,0,0,0), {"C:ON"}},                  /* 5  t=3 */
        {"POL-3 POL-7", In(1,0,0,1), {"C:OFF", "M:RIGHT"}},      /* 6 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 7 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 8 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 9 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 10 */
        {"POL-4 POL-2", In(0,0,0,0), {"M:FORWARD", "C:ON"}},     /* 11 */
        {"POL-3 POL-7", In(1,1,1,0), {"C:OFF", "M:BACKWARD"}},   /* 12 */
        {"POL-5",       In(1,1,1,0), {}},                        /* 13 */
        {"POL-5",       In(1,1,1,0), {}},                        /* 14 */
        {"POL-5 POL-7", In(1,1,1,0), {"M:BACKWARD"}},            /* 15 재시작 */
        {"POL-5",       In(1,1,0,0), {"M:RIGHT"}},               /* 16 즉시 전환 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 17 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 18 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 19 */
        {"POL-4",       In(0,0,0,0), {}},                        /* 20 */
        {"POL-4 POL-6", In(0,0,0,1), {"M:FORWARD", "C:UP"}},     /* 21 */
    };
    // clang-format on
    ASSERT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"})); /* POL-1 */
    int tick = 0;
    for (const Row &r : rows) {
        ++tick;
        EXPECT_EQ(Step(r.input), r.expected) << "tick " << tick << " (" << r.req << ")";
    }
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"})); /* H-SHUTDOWN */
}

/* ===================================================================== */
/* D. 수명주기·오류                                                        */
/* ===================================================================== */

/* REQ: H-TICK, POL-9 — 초기화 전 Tick은 NOT_READY이고 장치 출력이 없다. */
TEST_F(RvcSystemTest, TickBeforeInitializationIsRejectedWithoutOutputs)
{
    ExpectLocked();
}

/* REQ: POL-1, H-INIT — 초기화는 STOP → OFF. */
TEST_F(RvcSystemTest, InitializationCommandsStopThenOff)
{
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
}

/* REQ: H-SHUTDOWN, H-INIT — 종료는 OFF → STOP, 이후 Tick 거부, 재초기화하면 다시 동작한다. */
TEST_F(RvcSystemTest, ShutdownCommandsOffThenStopAndRequiresReinitialization)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: H-FRONT — 전방 보고만으로는 출력이 없고, 다음 Tick에서 반영된다. */
TEST_F(RvcSystemTest, FrontReportAloneProducesNoOutputUntilNextTick)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = In(1, 0, 0);
    EXPECT_EQ(rvc_report_front(robot, true), RVC_OK);
    reported_front = true;
    EXPECT_EQ(TakeOutputs(), Outputs{});
    EXPECT_EQ(rvc_tick(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:RIGHT"}));
}

/* 초기화 중 장치 실패. attempts = 실패한 호출까지 포함한 장치 호출 순서(성공한 읽기는 기록하지 않음).
 * exact=false이면 attempts가 기록의 앞부분과 일치해야 하고, 그 뒤의 호출은 명세에 없어 판정하지 않는다. */
struct InitFailureCase {
    const char *name;
    DeviceOp op;
    Outputs attempts;
    bool exact;
};

class InitFailureTest : public RvcSystemTest, public ::testing::WithParamInterface<InitFailureCase> {};

/* REQ: H-INIT, POL-9 — 초기화 중 장치 작업이 실패해도
 *  - 쓰기 실패: 초기 명령 STOP → OFF를 순서대로 모두 시도한다(실패한 쪽 포함).
 *  - 읽기 실패: STOP → OFF 뒤 센서를 읽다 실패하면 OFF → STOP을 다시 시도한다.
 *  오류를 반환하고 미준비로 남으며, 다시 초기화하면 정상 동작한다. */
TEST_P(InitFailureTest, InitializationFailureAttemptsOffAndStopInOrderThenRecovers)
{
    const InitFailureCase &c = GetParam();
    dev.fail_next[c.op] = 1;
    EXPECT_EQ(rvc_initialize(robot), RVC_IO_ERROR);
    const Outputs attempts = TakeAttempts();
    std::printf("[ trace    ] init %s attempts: %s\n", c.name, Join(attempts).c_str());
    if (c.exact) {
        EXPECT_EQ(attempts, c.attempts);
    } else {
        ASSERT_GE(attempts.size(), c.attempts.size()) << Join(attempts);
        EXPECT_EQ(Outputs(attempts.begin(), attempts.begin() + static_cast<std::ptrdiff_t>(c.attempts.size())),
                  c.attempts);
    }
    TakeOutputs();
    ExpectLocked();
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(Spec, InitFailureTest, ::testing::Values(
    InitFailureCase{"MotorWrite",   kWriteMotor,   {"M:STOP(fail)", "C:OFF"}, false},
    InitFailureCase{"CleanerWrite", kWriteCleaner, {"M:STOP", "C:OFF(fail)"}, false},
    InitFailureCase{"FrontRead",    kReadFront,    {"M:STOP", "C:OFF", "R:FRONT(fail)", "C:OFF", "M:STOP"}, true},
    InitFailureCase{"LeftRead",     kReadLeft,     {"M:STOP", "C:OFF", "R:LEFT(fail)",  "C:OFF", "M:STOP"}, true},
    InitFailureCase{"RightRead",    kReadRight,    {"M:STOP", "C:OFF", "R:RIGHT(fail)", "C:OFF", "M:STOP"}, true},
    InitFailureCase{"DustRead",     kReadDust,     {"M:STOP", "C:OFF", "R:DUST(fail)",  "C:OFF", "M:STOP"}, true}),
    [](const ::testing::TestParamInfo<InitFailureCase> &info) { return std::string(info.param.name); });
// clang-format on

class TickReadFailureTest : public RvcSystemTest, public ::testing::WithParamInterface<InitFailureCase> {};

/* REQ: POL-9, H-TICK — Tick 중 센서 읽기 실패: 실패 직후 OFF → STOP 시도 후 잠금.
 * 전방은 Tick마다 읽지 않으므로(H-FRONT) 주기 센서 세 개만 대상이다. */
TEST_P(TickReadFailureTest, FailureTurnsOffAndStopsThenLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = kClear;
    dev.fail_next[GetParam().op] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), GetParam().attempts);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
    ExpectLocked();
}

INSTANTIATE_TEST_SUITE_P(Spec, TickReadFailureTest,
                         ::testing::Values(
                             InitFailureCase{"Left", kReadLeft, {"R:LEFT(fail)", "C:OFF", "M:STOP"}, true},
                             InitFailureCase{"Right", kReadRight, {"R:RIGHT(fail)", "C:OFF", "M:STOP"}, true},
                             InitFailureCase{"Dust", kReadDust, {"R:DUST(fail)", "C:OFF", "M:STOP"}, true}),
                         [](const ::testing::TestParamInfo<InitFailureCase> &info) {
                             return std::string(info.param.name);
                         });

/* REQ: POL-9, POL-7 — 회피 진입의 청소 OFF 쓰기가 실패하면 방향 명령은 시도조차 하지 않고,
 * 오류 처리로 OFF → STOP을 시도한 뒤 잠긴다. */
TEST_F(RvcSystemTest, CleanerWriteFailureOnAvoidanceNeverTurnsAndLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = In(1, 0, 0);
    ReportFrontIfChanged(In(1, 0, 0));
    dev.fail_next[kWriteCleaner] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"C:OFF(fail)", "C:OFF", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* REQ: POL-9 — 회피 방향 명령 쓰기가 실패하면 OFF와 STOP을 각각 시도한 뒤 잠긴다. */
TEST_F(RvcSystemTest, MotorWriteFailureOnAvoidanceTurnsOffAndStopsThenLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = In(1, 0, 0);
    ReportFrontIfChanged(In(1, 0, 0));
    dev.fail_next[kWriteMotor] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    /* 회피용 OFF(성공) → RIGHT(실패) → 오류 처리 OFF → STOP */
    EXPECT_EQ(TakeAttempts(), Outputs({"C:OFF", "M:RIGHT(fail)", "C:OFF", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* ===================================================================== */
/* E. 특이 케이스                                                         */
/* ===================================================================== */

/* --- E-1 전방 보고(값이 바뀔 때 / 안 바뀔 때) --- */

/* REQ: H-FRONT — 전방 센서 값이 바뀌어도 보고가 없으면 반영하지 않는다. 보고하면 다음 Tick에 반영. */
TEST_F(RvcSystemTest, FrontChangeWithoutReportIsNotUsed)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = In(1, 0, 0);                  /* 장치 값만 바뀜, 보고 없음 */
    EXPECT_EQ(rvc_tick(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs{});       /* 여전히 전진 */
    EXPECT_EQ(rvc_report_front(robot, true), RVC_OK);
    reported_front = true;
    EXPECT_EQ(rvc_tick(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:RIGHT"}));
}

/* REQ: H-FRONT — 같은 값을 여러 번 보고해도 출력이 생기지 않고 동작도 바뀌지 않는다. */
TEST_F(RvcSystemTest, RepeatedSameFrontReportChangesNothing)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    for (int i = 0; i < 3; ++i) EXPECT_EQ(rvc_report_front(robot, false), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs{});
    EXPECT_EQ(Step(kClear), Outputs{});
}

/* REQ: H-FRONT(해석) — Tick 사이에 여러 번 보고되면 캐시에는 마지막 값이 남는다.
 * 명세는 "캐시만 갱신"까지만 적고 있어 마지막 값 사용은 해석이다(보고서 확인 요청 항목). */
TEST_F(RvcSystemTest, LastFrontReportBeforeTickWins)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = kClear;
    EXPECT_EQ(rvc_report_front(robot, true), RVC_OK);
    EXPECT_EQ(rvc_report_front(robot, false), RVC_OK);   /* 막힘 → 열림 */
    EXPECT_EQ(rvc_tick(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs{});
    dev.inputs = In(1, 0, 0);
    EXPECT_EQ(rvc_report_front(robot, false), RVC_OK);
    EXPECT_EQ(rvc_report_front(robot, true), RVC_OK);    /* 열림 → 막힘 */
    reported_front = true;
    EXPECT_EQ(rvc_tick(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:RIGHT"}));
}

/* REQ: H-FRONT — 초기화 전 전방 보고는 NOT_READY이고 출력이 없다. */
TEST_F(RvcSystemTest, FrontReportBeforeInitializationIsRejected)
{
    EXPECT_EQ(rvc_report_front(robot, true), RVC_NOT_READY);
    EXPECT_EQ(TakeOutputs(), Outputs{});
}

/* REQ: H-FRONT, POL-3 — 초기화 때 이미 앞이 막혀 있으면(초기 읽기 값) 첫 Tick에 회피한다. */
TEST_F(RvcSystemTest, FrontBlockedAtInitializationIsUsedOnFirstTick)
{
    EXPECT_EQ(PowerOn(In(1, 0, 0)), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(In(1, 0, 0)), Outputs({"M:RIGHT"}));
}

/* REQ: POL-4 — 회전 유지 중 전방 보고가 바뀌어도 출력 없음, 만료 Tick에 마지막 값으로 판단. */
TEST_F(RvcSystemTest, FrontReportsDuringTurnHoldAreUsedOnlyAtExpiry)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(1, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    EXPECT_EQ(Step(In(0, 0, 0)), Outputs{});
    EXPECT_EQ(Step(In(1, 0, 0)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0)), Outputs{});
    EXPECT_EQ(Step(In(1, 1, 0)), Outputs{});
    EXPECT_EQ(Step(In(1, 1, 0)), Outputs({"M:RIGHT"}));  /* 만료: 앞 막힘, 오른쪽 열림 */
}

/* --- E-2 값이 오래 안 바뀔 때 --- */

/* REQ: POL-7 — 같은 입력이 계속되면 전진 중 출력은 한 번도 다시 나가지 않는다(100 Tick). */
TEST_F(RvcSystemTest, SteadyClearInputProducesNoOutputFor100Ticks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ExpectHold(100, kClear);
}

/* REQ: POL-6, POL-7 — 먼지가 계속 감지되면 UP을 유지하고 다시 보내지 않는다(100 Tick). */
TEST_F(RvcSystemTest, SteadyDustKeepsBoostWithoutResendFor100Ticks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardPower));
    ExpectHold(100, In(0, 0, 0, 1));
    EXPECT_EQ(Step(kClear), Outputs{});
    EXPECT_EQ(Step(kClear), Outputs{});
    EXPECT_EQ(Step(kClear), Outputs({"C:ON"}));
}

/* REQ: POL-5, POL-7 — 세 방향이 99 Tick 동안 막혀 있으면 3 Tick마다 BACKWARD만 다시 나간다(33회). */
TEST_F(RvcSystemTest, AllBlockedFor99TicksRestartsBackwardEveryThirdTick)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::BackwardTick1));
    int restarts = 0;
    for (int tick = 1; tick <= 99; ++tick) {
        const Outputs out = Step(kAllBlocked);
        if (tick % 3 == 0) {
            EXPECT_EQ(out, Outputs({"M:BACKWARD"})) << "tick " << tick;
            ++restarts;
        } else {
            EXPECT_EQ(out, Outputs{}) << "tick " << tick;
        }
    }
    EXPECT_EQ(restarts, 33);
}

/* --- E-3 경계가 겹칠 때 --- */

/* REQ: POL-5, POL-6 — 후진 중 앞이 열리고 먼지가 있으면 바로 FORWARD + UP. */
TEST_F(RvcSystemTest, BackwardExitToForwardWithDustBoostsImmediately)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::BackwardTick1));
    EXPECT_EQ(Step(In(0, 1, 1, 1)), Outputs({"M:FORWARD", "C:UP"}));
}

/* REQ: POL-3, POL-6 — 강화 해제 Tick(t=3)에 장애물이 오면 회피가 우선하고, 강화는 취소된다. */
TEST_F(RvcSystemTest, ObstacleOnBoostExpiryTickAvoidsAndCancelsBoost)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(0, 0, 0, 1)), Outputs({"C:UP"}));
    ExpectHold(2, kClear);
    EXPECT_EQ(Step(In(1, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    ExpectHold(4, kClear);
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: POL-6 — 강화 유지 중(t=1)에 먼지가 다시 사라졌다 나타났다 해도 UP을 다시 보내지 않는다. */
TEST_F(RvcSystemTest, DustFlickeringDuringBoostSendsNothing)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(0, 0, 0, 1)), Outputs({"C:UP"}));
    for (int i = 0; i < 10; ++i) EXPECT_EQ(Step(In(0, 0, 0, i % 2)), Outputs{}) << "tick " << i;
}

/* REQ: POL-4, POL-3 — 회전 만료에 반대 방향을 고르면 그 방향 명령만 나간다(청소는 이미 OFF). */
TEST_F(RvcSystemTest, TurnExpiryCanSwitchDirection)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::RightTurnExpiry));
    EXPECT_EQ(Step(In(1, 0, 1)), Outputs({"M:LEFT"}));
    ExpectHold(4, In(1, 0, 1));
    EXPECT_EQ(Step(In(1, 1, 1)), Outputs({"M:BACKWARD"}));
}

/* REQ: POL-3, POL-4, POL-7 — 5 Tick 주기로 1,0,1,0,1을 넣으면(만료 Tick마다 앞이 막혀 있음)
 * 유지 중 값이 바뀌어도 출력이 없고, 만료 Tick마다 RIGHT만 다시 나간다. 마지막에 열리면 전진 복귀. */
TEST_F(RvcSystemTest, FrontBlockedOnEveryTurnExpiryRetriggersTurn)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    for (int round = 0; round < 3; ++round) {
        SCOPED_TRACE(round);
        /* round 0: 회전 진입 / round 1, 2: 회전 만료 Tick(앞 막힘)에 재선택 */
        EXPECT_EQ(Step(In(1, 0, 0)), round == 0 ? Outputs({"C:OFF", "M:RIGHT"}) : Outputs({"M:RIGHT"}));
        EXPECT_EQ(Step(In(0, 0, 0)), Outputs{});  /* 유지 1 */
        EXPECT_EQ(Step(In(1, 0, 0)), Outputs{});  /* 유지 2 */
        EXPECT_EQ(Step(In(0, 0, 0)), Outputs{});  /* 유지 3 */
        EXPECT_EQ(Step(In(1, 0, 0)), Outputs{});  /* 유지 4 */
    }
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));  /* 만료 Tick, 앞 열림 */
}

/* REQ: POL-3, POL-4, POL-2, POL-7 — 앞이 매 Tick 1,0,1,0…으로 엄격히 번갈아 바뀌면
 * 회전 만료 Tick(6번째)은 앞이 열린 Tick이 되어 전진으로 복귀하고, 다음 Tick에 다시 회피한다(6 Tick 주기). */
TEST_F(RvcSystemTest, FrontAlternatingEveryTickCyclesTurnAndForward)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    for (int cycle = 0; cycle < 3; ++cycle) {
        SCOPED_TRACE(cycle);
        EXPECT_EQ(Step(In(1, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));  /* 회피 진입 */
        EXPECT_EQ(Step(In(0, 0, 0)), Outputs{});                      /* 유지 1 */
        EXPECT_EQ(Step(In(1, 0, 0)), Outputs{});                      /* 유지 2 */
        EXPECT_EQ(Step(In(0, 0, 0)), Outputs{});                      /* 유지 3 */
        EXPECT_EQ(Step(In(1, 0, 0)), Outputs{});                      /* 유지 4 */
        EXPECT_EQ(Step(In(0, 0, 0)), Outputs({"M:FORWARD", "C:ON"})); /* 만료, 앞 열림 */
    }
}

/* --- E-4 수명주기 특이 순서 --- */

/* REQ: H-INIT, POL-6 — 동작 중 종료 없이 다시 초기화하면 STOP, OFF 후 강화 유지가 취소된다. */
TEST_F(RvcSystemTest, ReinitializeWhileRunningResetsOutputsAndBoost)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardPower));
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: H-INIT, POL-4 — 회전 중 다시 초기화하면 회전 시간도 초기화되어 다음 Tick에 바로 판단한다. */
TEST_F(RvcSystemTest, ReinitializeDuringTurnResetsTimer)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(1, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: H-SHUTDOWN — 초기화 전에 종료해도 OFF, STOP을 시도하고 이후 Tick은 거부된다. */
TEST_F(RvcSystemTest, ShutdownBeforeInitializationStillCommandsOffThenStop)
{
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* REQ: H-SHUTDOWN — 종료를 두 번 호출해도 매번 OFF, STOP을 시도한다. */
TEST_F(RvcSystemTest, ShutdownTwiceCommandsOffThenStopEachTime)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP", "C:OFF", "M:STOP"}));
}

/* REQ: H-SHUTDOWN — 종료 중 청소 OFF가 실패해도 STOP은 시도하고 오류를 돌려준다. */
TEST_F(RvcSystemTest, ShutdownWithCleanerFailureStillStopsAndReportsError)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.fail_next[kWriteCleaner] = 1;
    EXPECT_EQ(rvc_shutdown(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"C:OFF(fail)", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"M:STOP"}));
    ExpectLocked();
}

/* REQ: H-SHUTDOWN — 종료 중 STOP이 실패해도 OFF는 나가고 오류를 돌려준다. */
TEST_F(RvcSystemTest, ShutdownWithMotorFailureStillTurnsOffAndReportsError)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.fail_next[kWriteMotor] = 1;
    EXPECT_EQ(rvc_shutdown(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"C:OFF", "M:STOP(fail)"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF"}));
    ExpectLocked();
}

/* REQ: H-SHUTDOWN, H-INIT — 해제(destroy)는 장치에 아무 명령도 보내지 않는다. */
TEST_F(RvcSystemTest, DestroyWithoutShutdownSendsNothing)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    rvc_destroy(robot);
    robot = nullptr;
    EXPECT_EQ(TakeOutputs(), Outputs{});
}

/* --- E-5 오류가 겹칠 때 --- */

/* REQ: POL-9 — 회피 OFF와 오류 처리 OFF가 모두 실패해도 STOP은 시도하고 잠긴다. */
TEST_F(RvcSystemTest, PersistentCleanerFailureStillStopsMotor)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = In(1, 0, 0);
    ReportFrontIfChanged(In(1, 0, 0));
    dev.fail_next[kWriteCleaner] = 2;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"C:OFF(fail)", "C:OFF(fail)", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"M:STOP"}));
    ExpectLocked();
}

/* REQ: POL-9 — 회전 유지 중 센서 읽기가 실패해도 OFF, STOP 후 잠긴다. */
TEST_F(RvcSystemTest, SensorFailureDuringTurnHoldStopsAndLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(1, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    ExpectHold(2, kClear);
    TakeAttempts();
    dev.fail_next[kReadRight] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"R:RIGHT(fail)", "C:OFF", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* REQ: POL-9 — 후진 중 센서 읽기가 실패해도 OFF, STOP 후 잠긴다. */
TEST_F(RvcSystemTest, SensorFailureDuringBackwardStopsAndLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::BackwardTick1));
    dev.fail_next[kReadDust] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"R:DUST(fail)", "C:OFF", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* REQ: POL-9 — 강화(UP) 쓰기가 실패하면 OFF, STOP 후 잠긴다. */
TEST_F(RvcSystemTest, BoostWriteFailureStopsAndLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = In(0, 0, 0, 1);
    dev.fail_next[kWriteCleaner] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"C:UP(fail)", "C:OFF", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* REQ: POL-9 — 정지 상태에서 FORWARD 쓰기가 실패하면 ON은 보내지 않고 OFF, STOP 후 잠긴다. */
TEST_F(RvcSystemTest, ForwardWriteFailureStopsAndLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::StopOff));
    dev.inputs = kClear;
    dev.fail_next[kWriteMotor] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"M:FORWARD(fail)", "C:OFF", "M:STOP"}));
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* REQ: POL-9, H-INIT — Tick 오류로 잠긴 뒤 다시 초기화하면 정상 동작을 재개한다. */
TEST_F(RvcSystemTest, RecoveryAfterTickErrorByReinitialization)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardPower));
    dev.fail_next[kReadLeft] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeAttempts(), Outputs({"R:LEFT(fail)", "C:OFF", "M:STOP"}));
    TakeOutputs();
    ExpectLocked();
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: H-INIT — 고장이 계속되면 초기화도 계속 실패하고, 고장이 사라지면 성공한다. */
TEST_F(RvcSystemTest, InitializationKeepsFailingWhileFaultPersists)
{
    dev.fail_next[kReadLeft] = 3;
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(rvc_initialize(robot), RVC_IO_ERROR) << "attempt " << i + 1;
        EXPECT_EQ(TakeAttempts(), Outputs({"M:STOP", "C:OFF", "R:LEFT(fail)", "C:OFF", "M:STOP"}))
            << "attempt " << i + 1;
        TakeOutputs();
        ExpectLocked();
    }
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
}

/* --- E-6 API 인자와 여러 대 --- */

/* REQ: controller.h 각 함수 주석 — NULL 인자와 빠진 콜백은 거부되고 장치 명령이 나가지 않는다. */
TEST(RvcApiEdgeCases, NullArgumentsAndMissingCallbacksAreRejected)
{
    FakeDevice dev;
    EXPECT_EQ(rvc_create(nullptr), nullptr);
    const RvcDevice full = {&dev, ReadFront, ReadLeft, ReadRight, ReadDust, WriteMotor, WriteCleaner};
    for (int missing = 0; missing < 6; ++missing) {
        RvcDevice d = full;
        switch (missing) {
        case 0: d.read_front = nullptr; break;
        case 1: d.read_left = nullptr; break;
        case 2: d.read_right = nullptr; break;
        case 3: d.read_dust = nullptr; break;
        case 4: d.write_motor = nullptr; break;
        default: d.write_cleaner = nullptr; break;
        }
        EXPECT_EQ(rvc_create(&d), nullptr) << "missing callback " << missing;
    }
    EXPECT_EQ(rvc_initialize(nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_tick(nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_shutdown(nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_report_front(nullptr, true), RVC_INVALID_ARGUMENT);
    RvcTelemetry t{};
    EXPECT_EQ(rvc_get_telemetry(nullptr, &t), RVC_INVALID_ARGUMENT);
    Rvc *robot = rvc_create(&full);
    ASSERT_NE(robot, nullptr);
    ASSERT_EQ(rvc_initialize(robot), RVC_OK);
    EXPECT_EQ(rvc_get_telemetry(robot, nullptr), RVC_INVALID_ARGUMENT);
    rvc_destroy(robot);
    rvc_destroy(nullptr); /* NULL 허용 */
    EXPECT_EQ(dev.log, Outputs({"M:STOP", "C:OFF"})); /* 초기화 외의 명령 없음 */
}

/* REQ: controller.h "인스턴스별 제어 상태" — 두 로봇의 입력·시간·출력이 섞이지 않는다. */
TEST(RvcApiEdgeCases, TwoRobotsDoNotShareInputsTimersOrOutputs)
{
    FakeDevice a, b;
    const RvcDevice da = {&a, ReadFront, ReadLeft, ReadRight, ReadDust, WriteMotor, WriteCleaner};
    const RvcDevice db = {&b, ReadFront, ReadLeft, ReadRight, ReadDust, WriteMotor, WriteCleaner};
    Rvc *ra = rvc_create(&da);
    Rvc *rb = rvc_create(&db);
    ASSERT_NE(ra, nullptr);
    ASSERT_NE(rb, nullptr);
    ASSERT_EQ(rvc_initialize(ra), RVC_OK);
    ASSERT_EQ(rvc_initialize(rb), RVC_OK);
    a.log.clear();
    b.log.clear();
    a.inputs = In(1, 0, 0);
    ASSERT_EQ(rvc_report_front(ra, true), RVC_OK);   /* A만 앞 막힘 */
    b.inputs = In(0, 0, 0, 1);                       /* B만 먼지 */
    for (int tick = 1; tick <= 5; ++tick) {
        ASSERT_EQ(rvc_tick(ra), RVC_OK);
        ASSERT_EQ(rvc_tick(rb), RVC_OK);
    }
    EXPECT_EQ(a.log, Outputs({"M:RIGHT"}));            /* 1 Tick 진입, 2~5 유지 */
    EXPECT_EQ(b.log, Outputs({"M:FORWARD", "C:UP"}));  /* 1 Tick 전진, 이후 유지 */
    rvc_destroy(ra);
    rvc_destroy(rb);
}

/* ===================================================================== */
/* F. 입력 변화 전수·무작위 비교                                           */
/* ===================================================================== */

/* SpecModel: POL-1~7 문장만으로 만든 기대값 모델. 제품 코드를 호출하지 않는다.
 * 앞 값은 시스템에 보고된 값(=입력)으로 본다. */
class SpecModel {
public:
    Outputs PowerOn()
    {
        mode_ = Mode::Stop;                 /* POL-1 */
        elapsed_ = 0;
        hold_ = 0;
        cleaner_ = "OFF";
        return {"M:STOP", "C:OFF"};
    }
    Outputs Tick(const Sensors &s)
    {
        if (mode_ == Mode::Right || mode_ == Mode::Left) {
            if (++elapsed_ < 5) return {};  /* POL-4: 후속 5번째 Tick에 판단 */
        } else if (mode_ == Mode::Back) {
            const bool open = !s.front || !s.left || !s.right;
            if (++elapsed_ < 3 && !open) return {};  /* POL-5 */
        }
        Mode next;
        if (s.front) {                      /* POL-3 */
            next = !s.right ? Mode::Right : !s.left ? Mode::Left : Mode::Back;
            hold_ = 0;                      /* POL-6: 회피 진입 시 취소 */
        } else {                            /* POL-2, POL-6 */
            hold_ = s.dust ? 3 : (hold_ > 0 ? hold_ - 1 : 0);
            next = hold_ > 0 ? Mode::FwdUp : Mode::FwdOn;
        }
        Outputs out;                        /* POL-7 */
        if (next == Mode::Right || next == Mode::Left || next == Mode::Back) {
            if (cleaner_ != "OFF") {
                out.push_back("C:OFF");
                cleaner_ = "OFF";
            }
            out.push_back(next == Mode::Right ? "M:RIGHT" : next == Mode::Left ? "M:LEFT" : "M:BACKWARD");
            elapsed_ = 0;
        } else {
            if (mode_ != Mode::FwdOn && mode_ != Mode::FwdUp) out.push_back("M:FORWARD");
            const std::string target = next == Mode::FwdUp ? "UP" : "ON";
            if (cleaner_ != target) {
                out.push_back("C:" + target);
                cleaner_ = target;
            }
        }
        mode_ = next;
        return out;
    }

private:
    enum class Mode { Stop, FwdOn, FwdUp, Right, Left, Back };
    Mode mode_ = Mode::Stop;
    int elapsed_ = 0;
    int hold_ = 0;
    std::string cleaner_ = "OFF";
};

/* 테스트마다 로봇을 새로 만들어 같은 입력을 시스템과 모델에 넣는 실행기. */
class SequenceRunner {
public:
    explicit SequenceRunner(bool report_every_tick) : report_every_tick_(report_every_tick)
    {
        const RvcDevice device = {&dev_, ReadFront, ReadLeft, ReadRight, ReadDust, WriteMotor, WriteCleaner};
        robot_ = rvc_create(&device);
    }
    ~SequenceRunner() { rvc_destroy(robot_); }
    SequenceRunner(const SequenceRunner &) = delete;
    SequenceRunner &operator=(const SequenceRunner &) = delete;

    /* 초기 입력 + Tick 입력열을 실행하고, 처음 어긋난 지점을 설명으로 돌려준다(일치하면 빈 문자열). */
    std::string Run(const Sensors &initial, const std::vector<Sensors> &ticks)
    {
        SpecModel model;
        dev_.inputs = initial;
        reported_ = initial.front;
        if (!robot_ || rvc_initialize(robot_) != RVC_OK) return "initialize failed";
        if (Take() != model.PowerOn()) return "initialize outputs differ";
        for (std::size_t i = 0; i < ticks.size(); ++i) {
            const Sensors &s = ticks[i];
            dev_.inputs = s;
            if (report_every_tick_ || s.front != reported_) {
                if (rvc_report_front(robot_, s.front) != RVC_OK) return "report failed";
                reported_ = s.front;
            }
            if (rvc_tick(robot_) != RVC_OK) return "tick " + std::to_string(i + 1) + " failed";
            const Outputs actual = Take();
            const Outputs expected = model.Tick(s);
            if (actual != expected)
                return "tick " + std::to_string(i + 1) + ": expected " + Join(expected) + ", actual " + Join(actual);
        }
        return {};
    }

    static std::string Code(const Sensors &s)
    {
        return std::string() + (s.front ? '1' : '0') + (s.left ? '1' : '0') + (s.right ? '1' : '0') +
               (s.dust ? '1' : '0');
    }
    static std::string Describe(const Sensors &initial, const std::vector<Sensors> &ticks)
    {
        std::string text = "init " + Code(initial) + " ->";
        for (const Sensors &s : ticks) text += " " + Code(s);
        return text;
    }

private:
    Outputs Take()
    {
        Outputs out;
        out.swap(dev_.log);
        return out;
    }
    FakeDevice dev_;
    Rvc *robot_ = nullptr;
    bool reported_ = false;
    bool report_every_tick_;
};

Sensors FromBits(int bits) { return In((bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, bits & 1); }

/* 시작 맥락(앞부분 입력열) 뒤에 16가지 입력을 3 Tick 동안 모든 조합으로 넣는다.
 * 같은 값 유지(이전과 동일)와 값 변화(F/L/R/D 각각 0↔1)가 모두 포함된다. */
struct ContextCase {
    const char *name;
    const char *req;
    std::vector<Sensors> prefix;
};

class TransitionCoverageTest : public ::testing::TestWithParam<ContextCase> {};

TEST_P(TransitionCoverageTest, EveryThreeTickContinuationMatchesSpecModel)
{
    const ContextCase &c = GetParam();
    SCOPED_TRACE(c.req);
    int failures = 0;
    int runs = 0;
    for (int a = 0; a < 16 && failures < 5; ++a)
        for (int b = 0; b < 16 && failures < 5; ++b)
            for (int d = 0; d < 16 && failures < 5; ++d) {
                std::vector<Sensors> ticks = c.prefix;
                ticks.push_back(FromBits(a));
                ticks.push_back(FromBits(b));
                ticks.push_back(FromBits(d));
                SequenceRunner runner(false);
                const std::string diff = runner.Run(kClear, ticks);
                ++runs;
                if (!diff.empty()) {
                    ++failures;
                    ADD_FAILURE() << SequenceRunner::Describe(kClear, ticks) << " | " << diff;
                }
            }
    if (failures == 0) EXPECT_EQ(runs, 4096);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(Spec, TransitionCoverageTest, ::testing::Values(
    ContextCase{"FromStopOff",         "POL-1~7", {}},
    ContextCase{"FromForwardOn",       "POL-2 POL-7", {In(0,0,0,0)}},
    ContextCase{"FromBoostT0",         "POL-6", {In(0,0,0,1)}},
    ContextCase{"FromBoostT2",         "POL-6", {In(0,0,0,1), In(0,0,0,0), In(0,0,0,0)}},
    ContextCase{"FromRightTurnTick3",  "POL-4", {In(0,0,0,0), In(1,0,0,0), In(0,0,0,0), In(0,0,0,0), In(0,0,0,0)}},
    ContextCase{"FromLeftTurnTick3",   "POL-4", {In(0,0,0,0), In(1,0,1,0), In(0,0,0,0), In(0,0,0,0), In(0,0,0,0)}},
    ContextCase{"FromBackwardEntry",   "POL-5", {In(0,0,0,0), In(1,1,1,0)}},
    ContextCase{"FromBackwardTick1",   "POL-5", {In(0,0,0,0), In(1,1,1,0), In(1,1,1,0)}}),
    [](const ::testing::TestParamInfo<ContextCase> &info) { return std::string(info.param.name); });
// clang-format on

/* 초기화 때의 입력 16가지 × 첫 3 Tick 16^3 = 65,536가지 전수. 초기 읽기 값(H-FRONT)과 첫 판단을 모두 덮는다. */
TEST(SpecModelComparison, EveryInitialInputAndFirstThreeTicks)
{
    int failures = 0;
    for (int init = 0; init < 16 && failures < 5; ++init)
        for (int a = 0; a < 16 && failures < 5; ++a)
            for (int b = 0; b < 16 && failures < 5; ++b)
                for (int d = 0; d < 16 && failures < 5; ++d) {
                    const std::vector<Sensors> ticks = {FromBits(a), FromBits(b), FromBits(d)};
                    SequenceRunner runner(false);
                    const std::string diff = runner.Run(FromBits(init), ticks);
                    if (!diff.empty()) {
                        ++failures;
                        ADD_FAILURE() << SequenceRunner::Describe(FromBits(init), ticks) << " | " << diff;
                    }
                }
}

/* 무작위 장시간 입력: 센서마다 매 Tick 일정 확률로 값이 바뀐다(안 바뀌는 구간과 바뀌는 구간이 섞임).
 * 시드 고정으로 매번 같은 입력이다. 전방 보고는 "바뀔 때만"과 "매 Tick" 두 방식 모두 검사한다. */
struct RandomCase {
    const char *name;
    double flip_probability;
    bool report_every_tick;
};

class RandomSequenceTest : public ::testing::TestWithParam<RandomCase> {};

TEST_P(RandomSequenceTest, LongRandomRunsMatchSpecModel)
{
    const RandomCase &c = GetParam();
    std::mt19937 rng(20261009u);
    std::bernoulli_distribution flip(c.flip_probability);
    int failures = 0;
    for (int run = 0; run < 500 && failures < 5; ++run) {
        Sensors s = FromBits(static_cast<int>(rng() % 16));
        const Sensors initial = s;
        std::vector<Sensors> ticks;
        for (int t = 0; t < 200; ++t) {
            if (flip(rng)) s.front = !s.front;
            if (flip(rng)) s.left = !s.left;
            if (flip(rng)) s.right = !s.right;
            if (flip(rng)) s.dust = !s.dust;
            ticks.push_back(s);
        }
        SequenceRunner runner(c.report_every_tick);
        const std::string diff = runner.Run(initial, ticks);
        if (!diff.empty()) {
            ++failures;
            ADD_FAILURE() << "run " << run << " | " << diff;
        }
    }
}

INSTANTIATE_TEST_SUITE_P(Spec, RandomSequenceTest,
                         ::testing::Values(RandomCase{"RareChanges_ReportOnChange", 0.05, false},
                                           RandomCase{"RareChanges_ReportEveryTick", 0.05, true},
                                           RandomCase{"FrequentChanges_ReportOnChange", 0.4, false},
                                           RandomCase{"FrequentChanges_ReportEveryTick", 0.4, true}),
                         [](const ::testing::TestParamInfo<RandomCase> &info) {
                             return std::string(info.param.name);
                         });

}  // namespace
