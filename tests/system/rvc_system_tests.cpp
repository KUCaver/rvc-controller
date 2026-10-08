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
 */
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include "controller.h"

namespace {

/* 장치에 기록된 출력 순서. "M:<모터 명령>" 또는 "C:<청소 명령>". */
using Outputs = std::vector<std::string>;

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
    auto *d = static_cast<FakeDevice *>(context);
    if (!d || !out) return RVC_INVALID_ARGUMENT;
    if (ConsumeFault(d, op)) return RVC_IO_ERROR;
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
    if (ConsumeFault(d, kWriteMotor)) return RVC_IO_ERROR;
    d->log.push_back("M:" + MotorName(command));
    return RVC_OK;
}
RvcStatus WriteCleaner(void *context, RvcCleanerCommand command)
{
    auto *d = static_cast<FakeDevice *>(context);
    if (!d) return RVC_INVALID_ARGUMENT;
    if (ConsumeFault(d, kWriteCleaner)) return RVC_IO_ERROR;
    d->log.push_back("C:" + CleanerName(command));
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

/* 초기화 중 장치 실패. */
struct InitFailureCase {
    const char *name;
    DeviceOp op;
};

class InitFailureTest : public RvcSystemTest, public ::testing::WithParamInterface<InitFailureCase> {};

/* REQ: H-INIT — 초기화 중 어떤 장치 작업이 실패해도 오류를 반환하고 미준비로 남으며, 재시도하면 정상 동작한다.
 * 실패 시 나머지 초기 명령을 시도하는지와 그때의 장치 출력은 명세에 없어 판정하지 않는다. */
TEST_P(InitFailureTest, InitializationFailsLocksAndRetryRecovers)
{
    dev.fail_next[GetParam().op] = 1;
    EXPECT_EQ(rvc_initialize(robot), RVC_IO_ERROR);
    TakeOutputs();
    ExpectLocked();
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

INSTANTIATE_TEST_SUITE_P(Spec, InitFailureTest,
                         ::testing::Values(InitFailureCase{"MotorWrite", kWriteMotor},
                                           InitFailureCase{"CleanerWrite", kWriteCleaner},
                                           InitFailureCase{"FrontRead", kReadFront},
                                           InitFailureCase{"LeftRead", kReadLeft},
                                           InitFailureCase{"RightRead", kReadRight},
                                           InitFailureCase{"DustRead", kReadDust}),
                         [](const ::testing::TestParamInfo<InitFailureCase> &info) {
                             return std::string(info.param.name);
                         });

class TickReadFailureTest : public RvcSystemTest, public ::testing::WithParamInterface<InitFailureCase> {};

/* REQ: POL-9, H-TICK — Tick 중 센서 읽기 실패: OFF → STOP 시도 후 잠금.
 * 전방은 Tick마다 읽지 않으므로(H-FRONT) 주기 센서 세 개만 대상이다. */
TEST_P(TickReadFailureTest, FailureTurnsOffAndStopsThenLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = kClear;
    dev.fail_next[GetParam().op] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
    ExpectLocked();
}

INSTANTIATE_TEST_SUITE_P(Spec, TickReadFailureTest,
                         ::testing::Values(InitFailureCase{"Left", kReadLeft},
                                           InitFailureCase{"Right", kReadRight},
                                           InitFailureCase{"Dust", kReadDust}),
                         [](const ::testing::TestParamInfo<InitFailureCase> &info) {
                             return std::string(info.param.name);
                         });

/* REQ: POL-9, POL-7 — 회피 진입의 청소 OFF 쓰기가 실패하면 방향 명령은 나가지 않고,
 * 오류 처리로 OFF → STOP을 시도한 뒤 잠긴다. (실패한 쓰기는 장치 기록에 남지 않는다.) */
TEST_F(RvcSystemTest, CleanerWriteFailureOnAvoidanceNeverTurnsAndLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    dev.inputs = In(1, 0, 0);
    ReportFrontIfChanged(In(1, 0, 0));
    dev.fail_next[kWriteCleaner] = 1;
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
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
    /* 회피용 OFF(성공) → RIGHT(실패, 기록 없음) → 오류 처리 OFF → STOP */
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "C:OFF", "M:STOP"}));
    ExpectLocked();
}

}  // namespace
