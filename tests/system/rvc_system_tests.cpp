/*
 * RVC 시스템 테스트(블랙박스). 기존 tests/rvc_tests.cpp의 모듈 직접 검사와 분리한다.
 *
 * 시스템 경계(06 §2 경계 표 "Context RVC")
 *   입력: 전방 보고(rvc_report_front), 좌·우·먼지 센서값, Tick(rvc_tick)
 *   출력: Motor Direction, Cleaner Command
 * 테스트 더블: RvcDevice 콜백 계층의 RvcMockDevice. 그 위의 Sensor/Motor/Cleaner Interface
 *   코드까지 실제로 실행된다. 내부 모듈(Determine*, Controller, Actions)은 호출하지 않는다.
 * 판정: mock 장치에 성공적으로 기록된 출력의 순서·개수만 비교한다. 내부 상태는 단언하지 않는다.
 * 시간: rvc_tick() 호출 1회 = 논리 Tick 1회. 실제 대기는 하지 않는다.
 *
 * 기대값의 출처: docs/06_implemented_design.md(06), docs/02_contracts_and_decisions.md(02).
 * 요구사항 ID
 *   INIT, S1~S5, F1~F4, T1~T2, B1~B4, ERR, END : 06 §5 전이표의 전이 ID
 *   06-CMD-FWD   06 §5 명령 묶음 `전진(ON/UP)`: 이전이 전진이 아니면 Forward, 청소가 목표와 다르면 ON/UP.
 *                둘 다 필요하면 Forward → ON/UP
 *   06-CMD-AVOID 06 §5 명령 묶음 `회피`: 청소가 이미 유효한 OFF가 아니면 OFF, 그 뒤 회피 Trigger.
 *                이미 OFF이면 OFF를 다시 쓰지 않는다
 *   06-CMD-HOLD  06 §5 명령 묶음 `유지`: 명령 없음
 *   06-DUST      06 §5 끝 문단: 먼지 타이머 없음. D=false 첫 전진 판단에서 ON 복귀. 회피 중에는 D와
 *                무관하게 OFF. 회피 종료 후 전진하면 그 Tick의 D 사용
 *   06-INIT-FAIL 06 §6: 초기 명령 중 하나가 실패해도 다른 초기 명령을 시도. 준비 전 Tick은 NOT_READY
 *   02-TIME-1    02 시간·출력 정책 ¶1: 진입 e=0, 회전 5번째/후진 3번째 후속 Tick에 완료 판단.
 *                만료 전 일반 입력 변화로 동작을 중단하지 않음
 *   02-TIME-2    02 시간·출력 정책 ¶3: 같은 회피를 다시 선택하면 Trigger 재호출, 경과 0 재설정
 *   02-OUT-1     02 시간·출력 정책 ¶3: 전진 유지 시 모터 반복 명령 없음, 먼지 모드가 바뀔 때만 ON/UP
 *   02-OUT-2     02 시간·출력 정책 ¶2: 회피 명령 전에 청소 OFF가 성립해야 함
 *   02-LIFE-4    02 수명주기 4: report_front는 캐시만 갱신, 즉시 전이·장치 출력 없음
 *   02-LIFE-5    02 수명주기 5: shutdown은 OFF 후 STOP, 이후 Tick에는 재초기화 필요
 *   02-ERR       02 수명주기와 오류: Tick 중 I/O 실패 시 OFF·STOP을 각각 시도한 뒤 잠금,
 *                재초기화 성공 전에는 정상 실행을 재개하지 않음
 */
#include <gtest/gtest.h>
#include <cstddef>
#include <string>
#include <vector>
#ifdef RVC_SINGLE_FILE
/* 통합본: 공개 API와 mock 선언을 제공한다. 함께 선언된 내부 모듈 함수는 이 파일에서 사용하지 않는다. */
#include "single_file_test_support.h"
#else
#include "rvc/rvc.h"
#include "mock_device.h"
#endif

namespace {

/* 장치에 기록된 출력 순서. "M:<모터 명령>" 또는 "C:<청소 명령>" 문자열로 표현한다. */
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

std::string MotorName(int command)
{
    static const char *const names[] = {"STOP", "FORWARD", "LEFT", "RIGHT", "BACKWARD"};
    return command >= 0 && command < 5 ? names[command] : "INVALID";
}
std::string CleanerName(int command)
{
    static const char *const names[] = {"OFF", "ON", "UP"};
    return command >= 0 && command < 3 ? names[command] : "INVALID";
}

/* 테스트 시작 시점. 모두 초기화와 센서 입력 이력만으로 도달한다. */
enum class Start {
    StopOff,         /* 초기화 직후 */
    ForwardOn,       /* 전진 + 일반 청소 */
    ForwardPower,    /* 전진 + 강화 청소 */
    RightTurnExpiry, /* 우회전 진입 후 4 Tick 유지, 다음 Tick이 5번째 후속 Tick */
    BackwardExpiry   /* 후진 진입 후 2 Tick 유지, 다음 Tick이 3번째 후속 Tick */
};

class RvcSystemTest : public ::testing::Test {
protected:
    RvcMockDevice mock{};
    Rvc *robot = nullptr;
    bool reported_front = false;

    void SetUp() override
    {
        rvc_mock_init(&mock);
        RvcDevice device = rvc_mock_device(&mock);
        robot = rvc_create(&device);
        ASSERT_NE(robot, nullptr);
    }
    void TearDown() override { rvc_destroy(robot); }

    /* 하드웨어 센서값만 바꾼다. Tick·전방 보고는 하지 않는다. */
    void Apply(const Sensors &s)
    {
        RvcSensorSnapshot snapshot{};
        snapshot.obstacles.front_blocked = s.front;
        snapshot.obstacles.left_blocked = s.left;
        snapshot.obstacles.right_blocked = s.right;
        snapshot.dust_detected = s.dust;
        rvc_mock_set_inputs(&mock, snapshot);
    }
    /* 마지막 조회 이후 장치에 기록된 출력을 꺼내고 로그를 비운다. */
    Outputs TakeOutputs()
    {
        Outputs out;
        for (std::size_t i = 0; i < mock.event_count; ++i) {
            const RvcMockEvent &e = mock.events[i];
            out.push_back(e.kind == RVC_MOCK_MOTOR ? "M:" + MotorName(e.command)
                                                   : "C:" + CleanerName(e.command));
        }
        rvc_mock_clear_events(&mock);
        return out;
    }
    /* 전원 투입: 초기 센서값을 두고 initialize. 초기화 중 장치 출력을 반환한다. */
    Outputs PowerOn(const Sensors &s = kClear)
    {
        Apply(s);
        reported_front = s.front;
        EXPECT_EQ(rvc_initialize(robot), RVC_OK);
        return TakeOutputs();
    }
    /* 전방 센서가 바뀌었으면 보고한다. Front는 보고 방식 입력이다(06 §4). */
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
        Apply(s);
        ReportFrontIfChanged(s);
        EXPECT_EQ(rvc_tick(robot), RVC_OK);
        return TakeOutputs();
    }
    /* ticks번 연속으로 출력이 없어야 한다(06-CMD-HOLD). */
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
        ASSERT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));                 /* INIT */
        switch (start) {
        case Start::StopOff:
            break;
        case Start::ForwardOn:
            ASSERT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));         /* S1 */
            break;
        case Start::ForwardPower:
            ASSERT_EQ(Step(In(0, 0, 0, 1)), Outputs({"M:FORWARD", "C:UP"})); /* S2 */
            break;
        case Start::RightTurnExpiry:
            ASSERT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));         /* S1 */
            ASSERT_EQ(Step(In(1, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));     /* F2 */
            ExpectHold(4, kClear);                                           /* T1 */
            break;
        case Start::BackwardExpiry:
            ASSERT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));         /* S1 */
            ASSERT_EQ(Step(In(1, 1, 1)), Outputs({"C:OFF", "M:BACKWARD"}));  /* F4 */
            ExpectHold(2, kClear);                                           /* B1 */
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
    /* 초기화 직후(STOP_OFF). F=0이면 L/R 무관 전진, F=1이면 오른쪽 → 왼쪽 → 후진 순.
     * 청소는 이미 OFF이므로 회피에서는 OFF를 다시 쓰지 않는다(06-CMD-AVOID). */
    /* name                     req                        start             F  L  R  D    expected */
    {"StopOff_F0L0R0_D0", "S1 06-CMD-FWD",              Start::StopOff, In(0,0,0,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F0L0R1_D0", "S1 06-CMD-FWD",              Start::StopOff, In(0,0,1,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F0L1R0_D0", "S1 06-CMD-FWD",              Start::StopOff, In(0,1,0,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F0L1R1_D0", "S1 06-CMD-FWD",              Start::StopOff, In(0,1,1,0), {"M:FORWARD", "C:ON"}},
    {"StopOff_F1L0R0_D0", "S3 06-CMD-AVOID",            Start::StopOff, In(1,0,0,0), {"M:RIGHT"}},
    {"StopOff_F1L0R1_D0", "S4 06-CMD-AVOID",            Start::StopOff, In(1,0,1,0), {"M:LEFT"}},
    {"StopOff_F1L1R0_D0", "S3 06-CMD-AVOID",            Start::StopOff, In(1,1,0,0), {"M:RIGHT"}},
    {"StopOff_F1L1R1_D0", "S5 06-CMD-AVOID",            Start::StopOff, In(1,1,1,0), {"M:BACKWARD"}},
    {"StopOff_F0L0R0_D1", "S2 06-CMD-FWD",              Start::StopOff, In(0,0,0,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F0L0R1_D1", "S2 06-CMD-FWD",              Start::StopOff, In(0,0,1,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F0L1R0_D1", "S2 06-CMD-FWD",              Start::StopOff, In(0,1,0,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F0L1R1_D1", "S2 06-CMD-FWD",              Start::StopOff, In(0,1,1,1), {"M:FORWARD", "C:UP"}},
    {"StopOff_F1L0R0_D1", "S3 06-CMD-AVOID 06-DUST",    Start::StopOff, In(1,0,0,1), {"M:RIGHT"}},
    {"StopOff_F1L0R1_D1", "S4 06-CMD-AVOID 06-DUST",    Start::StopOff, In(1,0,1,1), {"M:LEFT"}},
    {"StopOff_F1L1R0_D1", "S3 06-CMD-AVOID 06-DUST",    Start::StopOff, In(1,1,0,1), {"M:RIGHT"}},
    {"StopOff_F1L1R1_D1", "S5 06-CMD-AVOID 06-DUST",    Start::StopOff, In(1,1,1,1), {"M:BACKWARD"}},

    /* 전진 + 일반 청소(F_ON). F=0이면 모터 재명령 없음, 먼지 모드가 바뀔 때만 청소 명령.
     * F=1이면 청소 OFF가 회피 모터 명령보다 먼저(06-CMD-AVOID, 02-OUT-2). */
    {"ForwardOn_F0L0R0_D0", "F1 02-OUT-1",              Start::ForwardOn, In(0,0,0,0), {}},
    {"ForwardOn_F0L0R1_D0", "F1 02-OUT-1",              Start::ForwardOn, In(0,0,1,0), {}},
    {"ForwardOn_F0L1R0_D0", "F1 02-OUT-1",              Start::ForwardOn, In(0,1,0,0), {}},
    {"ForwardOn_F0L1R1_D0", "F1 02-OUT-1",              Start::ForwardOn, In(0,1,1,0), {}},
    {"ForwardOn_F1L0R0_D0", "F2 06-CMD-AVOID 02-OUT-2", Start::ForwardOn, In(1,0,0,0), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L0R1_D0", "F3 06-CMD-AVOID 02-OUT-2", Start::ForwardOn, In(1,0,1,0), {"C:OFF", "M:LEFT"}},
    {"ForwardOn_F1L1R0_D0", "F2 06-CMD-AVOID 02-OUT-2", Start::ForwardOn, In(1,1,0,0), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L1R1_D0", "F4 06-CMD-AVOID 02-OUT-2", Start::ForwardOn, In(1,1,1,0), {"C:OFF", "M:BACKWARD"}},
    {"ForwardOn_F0L0R0_D1", "F1 02-OUT-1 06-DUST",      Start::ForwardOn, In(0,0,0,1), {"C:UP"}},
    {"ForwardOn_F0L0R1_D1", "F1 02-OUT-1 06-DUST",      Start::ForwardOn, In(0,0,1,1), {"C:UP"}},
    {"ForwardOn_F0L1R0_D1", "F1 02-OUT-1 06-DUST",      Start::ForwardOn, In(0,1,0,1), {"C:UP"}},
    {"ForwardOn_F0L1R1_D1", "F1 02-OUT-1 06-DUST",      Start::ForwardOn, In(0,1,1,1), {"C:UP"}},
    {"ForwardOn_F1L0R0_D1", "F2 06-CMD-AVOID 06-DUST",  Start::ForwardOn, In(1,0,0,1), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L0R1_D1", "F3 06-CMD-AVOID 06-DUST",  Start::ForwardOn, In(1,0,1,1), {"C:OFF", "M:LEFT"}},
    {"ForwardOn_F1L1R0_D1", "F2 06-CMD-AVOID 06-DUST",  Start::ForwardOn, In(1,1,0,1), {"C:OFF", "M:RIGHT"}},
    {"ForwardOn_F1L1R1_D1", "F4 06-CMD-AVOID 06-DUST",  Start::ForwardOn, In(1,1,1,1), {"C:OFF", "M:BACKWARD"}},

    /* 회전 만료 Tick(5번째 후속 Tick): 일반 선택(06 §5). 청소는 이미 OFF. */
    {"RightTurnExpiry_F0L0R0", "T2 06-CMD-FWD",         Start::RightTurnExpiry, In(0,0,0), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F0L0R1", "T2 06-CMD-FWD",         Start::RightTurnExpiry, In(0,0,1), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F0L1R0", "T2 06-CMD-FWD",         Start::RightTurnExpiry, In(0,1,0), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F0L1R1", "T2 06-CMD-FWD",         Start::RightTurnExpiry, In(0,1,1), {"M:FORWARD", "C:ON"}},
    {"RightTurnExpiry_F1L0R0", "T2 02-TIME-2",          Start::RightTurnExpiry, In(1,0,0), {"M:RIGHT"}},
    {"RightTurnExpiry_F1L0R1", "T2 06-CMD-AVOID",       Start::RightTurnExpiry, In(1,0,1), {"M:LEFT"}},
    {"RightTurnExpiry_F1L1R0", "T2 02-TIME-2",          Start::RightTurnExpiry, In(1,1,0), {"M:RIGHT"}},
    {"RightTurnExpiry_F1L1R1", "T2 06-CMD-AVOID",       Start::RightTurnExpiry, In(1,1,1), {"M:BACKWARD"}},

    /* 후진 만료 Tick(3번째 후속 Tick): 후진 완료 선택. F와 무관, 직접 전진하지 않음(06 §5). */
    {"BackwardExpiry_F0L0R0", "B2",                     Start::BackwardExpiry, In(0,0,0), {"M:RIGHT"}},
    {"BackwardExpiry_F0L0R1", "B3",                     Start::BackwardExpiry, In(0,0,1), {"M:LEFT"}},
    {"BackwardExpiry_F0L1R0", "B2",                     Start::BackwardExpiry, In(0,1,0), {"M:RIGHT"}},
    {"BackwardExpiry_F0L1R1", "B4 02-TIME-2",           Start::BackwardExpiry, In(0,1,1), {"M:BACKWARD"}},
    {"BackwardExpiry_F1L0R0", "B2",                     Start::BackwardExpiry, In(1,0,0), {"M:RIGHT"}},
    {"BackwardExpiry_F1L0R1", "B3",                     Start::BackwardExpiry, In(1,0,1), {"M:LEFT"}},
    {"BackwardExpiry_F1L1R0", "B2",                     Start::BackwardExpiry, In(1,1,0), {"M:RIGHT"}},
    {"BackwardExpiry_F1L1R1", "B4 02-TIME-2",           Start::BackwardExpiry, In(1,1,1), {"M:BACKWARD"}},
};
// clang-format on

INSTANTIATE_TEST_SUITE_P(Spec, DecisionTableTest, ::testing::ValuesIn(kDecisionTable),
                         [](const ::testing::TestParamInfo<DecisionCase> &info) {
                             return std::string(info.param.name);
                         });

/* ===================================================================== */
/* B. 먼지 on/off의 여러 Tick 동작                                        */
/* ===================================================================== */

/* REQ: F1, 06-DUST, 02-OUT-1 — 먼지 감지 중 UP 유지, 사라진 첫 Tick에 ON 복귀, 모터 재명령 없음. */
TEST_F(RvcSystemTest, DustOnHoldOffReturnsToNormalCleaningWithoutMotorCommand)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs({"C:UP"}));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs({"C:ON"}));
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});
}

/* REQ: F2, 06-CMD-AVOID, 06-DUST, 02-OUT-2 — 강화 청소 중 장애물: 먼지가 있어도 OFF 후 회피. */
TEST_F(RvcSystemTest, ObstacleDuringPowerCleaningTurnsCleanerOffBeforeTurning)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardPower));
    EXPECT_EQ(Step(In(1, 0, 0, 1)), Outputs({"C:OFF", "M:RIGHT"}));
}

/* REQ: T1, 06-DUST, 06-CMD-HOLD — 회전 중에는 먼지가 바뀌어도 청소 출력이 없다. */
TEST_F(RvcSystemTest, DustChangesDuringTurnProduceNoCleanerOutput)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    ASSERT_EQ(Step(In(1, 0, 0, 0)), Outputs({"C:OFF", "M:RIGHT"}));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs{});
    EXPECT_EQ(Step(In(0, 0, 0, 0)), Outputs{});
}

/* REQ: T2, 06-DUST, 06-CMD-FWD — 회피 종료 후 전진은 그 Tick의 먼지값으로 청소 모드를 정한다. */
TEST_F(RvcSystemTest, ForwardAfterTurnUsesDustOfThatTick)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::RightTurnExpiry));
    EXPECT_EQ(Step(In(0, 0, 0, 1)), Outputs({"M:FORWARD", "C:UP"}));
}

/* ===================================================================== */
/* C. 회전·후진 유지 시간과 재선택                                         */
/* ===================================================================== */

struct ManeuverCase {
    const char *name;
    const char *req;
    Sensors entry;           /* 전진(F_ON) 중 이 입력으로 회피 진입 */
    Outputs entry_out;
    int duration;            /* 완료 판단까지의 후속 Tick 수 */
    Sensors reselect;        /* 만료 Tick에 같은 회피를 다시 선택하게 하는 입력 */
    Outputs reselect_out;
    Outputs expiry_clear_out; /* 만료 Tick에 사방이 열려 있을 때의 출력 */
};

class ManeuverTest : public RvcSystemTest, public ::testing::WithParamInterface<ManeuverCase> {
protected:
    void Enter()
    {
        ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
        ASSERT_EQ(Step(GetParam().entry), GetParam().entry_out);
    }
};

/* REQ: T1/B1, T2/B2, 02-TIME-1 — 진입 후 duration-1 Tick 유지, duration번째 Tick에 재판단. */
TEST_P(ManeuverTest, HoldsUntilDurationThenDecidesAgain)
{
    const ManeuverCase &c = GetParam();
    SCOPED_TRACE(c.req);
    ASSERT_NO_FATAL_FAILURE(Enter());
    ExpectHold(c.duration - 1, kClear);
    EXPECT_EQ(Step(kClear), c.expiry_clear_out);
}

/* REQ: 02-TIME-1, 06-CMD-HOLD, 06-DUST — 유지 중 장애물·먼지 입력이 바뀌어도 동작을 바꾸지 않는다. */
TEST_P(ManeuverTest, InputChangesDuringHoldDoNotInterrupt)
{
    const ManeuverCase &c = GetParam();
    SCOPED_TRACE(c.req);
    ASSERT_NO_FATAL_FAILURE(Enter());
    const Sensors noise[] = {In(1, 1, 1, 1), In(0, 0, 0, 1), In(1, 0, 0, 0), In(0, 1, 1, 0)};
    for (int i = 0; i < c.duration - 1; ++i)
        EXPECT_EQ(Step(noise[i % 4]), Outputs{}) << "hold tick " << i + 1;
    EXPECT_EQ(Step(kClear), c.expiry_clear_out);
}

/* REQ: T2/B4, 02-TIME-2 — 만료 시 같은 회피를 다시 고르면 모터 Trigger를 다시 보내고 경과가 0부터. */
TEST_P(ManeuverTest, ReselectingSameManeuverRetriggersAndRestartsTimer)
{
    const ManeuverCase &c = GetParam();
    SCOPED_TRACE(c.req);
    ASSERT_NO_FATAL_FAILURE(Enter());
    ExpectHold(c.duration - 1, kClear);
    EXPECT_EQ(Step(c.reselect), c.reselect_out);
    ExpectHold(c.duration - 1, kClear);
    EXPECT_EQ(Step(kClear), c.expiry_clear_out);
}

// clang-format off
const ManeuverCase kManeuvers[] = {
    {"RightTurn", "F2 T1 T2", In(1,0,0), {"C:OFF", "M:RIGHT"},    5, In(1,0,0), {"M:RIGHT"},    {"M:FORWARD", "C:ON"}},
    {"LeftTurn",  "F3 T1 T2", In(1,0,1), {"C:OFF", "M:LEFT"},     5, In(1,0,1), {"M:LEFT"},     {"M:FORWARD", "C:ON"}},
    {"Backward",  "F4 B1 B4 B2", In(1,1,1), {"C:OFF", "M:BACKWARD"}, 3, In(1,1,1), {"M:BACKWARD"}, {"M:RIGHT"}},
};
// clang-format on

INSTANTIATE_TEST_SUITE_P(Spec, ManeuverTest, ::testing::ValuesIn(kManeuvers),
                         [](const ::testing::TestParamInfo<ManeuverCase> &info) {
                             return std::string(info.param.name);
                         });

/* REQ: 06 §5 전이표 전체 흐름 — 전진·먼지·우회전·후진·좌회전·복귀를 한 번에 Tick 단위로 확인. */
TEST_F(RvcSystemTest, EndToEndScenarioTickByTick)
{
    struct Row {
        const char *req;
        Sensors input;
        Outputs expected;
    };
    // clang-format off
    const Row rows[] = {
        {"S1",            In(0,0,0,0), {"M:FORWARD", "C:ON"}},
        {"F1 06-DUST",    In(0,0,0,1), {"C:UP"}},
        {"F1 06-DUST",    In(0,0,0,0), {"C:ON"}},
        {"F2",            In(1,0,0,1), {"C:OFF", "M:RIGHT"}},
        {"T1",            In(0,0,0,0), {}},
        {"T1",            In(0,0,0,0), {}},
        {"T1",            In(0,0,0,0), {}},
        {"T1",            In(0,0,0,0), {}},
        {"T2",            In(0,0,0,0), {"M:FORWARD", "C:ON"}},
        {"F4",            In(1,1,1,0), {"C:OFF", "M:BACKWARD"}},
        {"B1",            In(0,0,1,0), {}},
        {"B1",            In(0,0,1,0), {}},
        {"B3",            In(0,0,1,0), {"M:LEFT"}},
        {"T1",            In(0,0,0,0), {}},
        {"T1",            In(0,0,0,0), {}},
        {"T1",            In(0,0,0,0), {}},
        {"T1",            In(0,0,0,0), {}},
        {"T2",            In(0,0,0,0), {"M:FORWARD", "C:ON"}},
    };
    // clang-format on
    ASSERT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"})); /* INIT */
    int tick = 0;
    for (const Row &r : rows) {
        ++tick;
        EXPECT_EQ(Step(r.input), r.expected) << "tick " << tick << " (" << r.req << ")";
    }
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"})); /* END */
}

/* ===================================================================== */
/* D. 수명주기·오류                                                        */
/* ===================================================================== */

/* REQ: 06-INIT-FAIL — 초기화 전 Tick은 NOT_READY이고 장치 출력이 없다. */
TEST_F(RvcSystemTest, TickBeforeInitializationIsRejectedWithoutOutputs)
{
    Apply(kClear);
    ExpectLocked();
}

/* REQ: INIT — 초기화는 Stop → Off. */
TEST_F(RvcSystemTest, InitializationCommandsStopThenOff)
{
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
}

/* REQ: END, 02-LIFE-5, INIT — 종료는 Off → Stop, 이후 Tick 거부, 재초기화하면 다시 동작한다. */
TEST_F(RvcSystemTest, ShutdownCommandsOffThenStopAndRequiresReinitialization)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    EXPECT_EQ(rvc_shutdown(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

/* REQ: 02-LIFE-4, F2 — 전방 보고만으로는 출력이 없고, 다음 Tick에서 반영된다. */
TEST_F(RvcSystemTest, FrontReportAloneProducesNoOutputUntilNextTick)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    Apply(In(1, 0, 0));
    EXPECT_EQ(rvc_report_front(robot, true), RVC_OK);
    reported_front = true;
    EXPECT_EQ(TakeOutputs(), Outputs{});
    EXPECT_EQ(rvc_tick(robot), RVC_OK);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:RIGHT"}));
}

/* 초기화 중 출력 쓰기 실패. */
struct InitWriteFailureCase {
    const char *name;
    RvcMockOperation operation;
    Outputs recorded; /* 실패하지 않은 나머지 초기 명령 */
};

class InitWriteFailureTest : public RvcSystemTest,
                             public ::testing::WithParamInterface<InitWriteFailureCase> {};

/* REQ: 06-INIT-FAIL, INIT — 초기 명령 하나가 실패해도 다른 하나를 시도하고, 준비되지 않으며,
 * 재시도하면 정상 초기화된다. */
TEST_P(InitWriteFailureTest, OtherInitialCommandIsStillAttemptedAndRetryRecovers)
{
    const InitWriteFailureCase &c = GetParam();
    Apply(kClear);
    rvc_mock_fail_next(&mock, c.operation, 1);
    EXPECT_EQ(rvc_initialize(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeOutputs(), c.recorded);
    ExpectLocked();
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

INSTANTIATE_TEST_SUITE_P(Spec, InitWriteFailureTest,
                         ::testing::Values(
                             InitWriteFailureCase{"MotorStopFails", RVC_MOCK_WRITE_MOTOR, {"C:OFF"}},
                             InitWriteFailureCase{"CleanerOffFails", RVC_MOCK_WRITE_CLEANER, {"M:STOP"}}),
                         [](const ::testing::TestParamInfo<InitWriteFailureCase> &info) {
                             return std::string(info.param.name);
                         });

/* 센서 읽기 실패. */
struct ReadFailureCase {
    const char *name;
    RvcMockOperation operation;
};

class InitReadFailureTest : public RvcSystemTest,
                            public ::testing::WithParamInterface<ReadFailureCase> {};

/* REQ: 06-INIT-FAIL — 초기 센서 읽기 실패 시 준비되지 않고, 재시도하면 정상 동작한다.
 * 이때의 장치 출력은 명세에 정의되어 있지 않아 단언하지 않는다(보고서의 미정의 항목). */
TEST_P(InitReadFailureTest, InitializationFailsAndRetryRecovers)
{
    Apply(kClear);
    rvc_mock_fail_next(&mock, GetParam().operation, 1);
    EXPECT_EQ(rvc_initialize(robot), RVC_IO_ERROR);
    TakeOutputs();
    ExpectLocked();
    EXPECT_EQ(PowerOn(), Outputs({"M:STOP", "C:OFF"}));
    EXPECT_EQ(Step(kClear), Outputs({"M:FORWARD", "C:ON"}));
}

INSTANTIATE_TEST_SUITE_P(Spec, InitReadFailureTest,
                         ::testing::Values(ReadFailureCase{"Front", RVC_MOCK_READ_FRONT},
                                           ReadFailureCase{"Left", RVC_MOCK_READ_LEFT},
                                           ReadFailureCase{"Right", RVC_MOCK_READ_RIGHT},
                                           ReadFailureCase{"Dust", RVC_MOCK_READ_DUST}),
                         [](const ::testing::TestParamInfo<ReadFailureCase> &info) {
                             return std::string(info.param.name);
                         });

class TickReadFailureTest : public RvcSystemTest,
                            public ::testing::WithParamInterface<ReadFailureCase> {};

/* REQ: ERR, 02-ERR — Tick 중 센서 읽기 실패: Off → Stop 시도 후 잠금. 재초기화 전에는 재개하지 않는다.
 * Front는 Tick마다 읽지 않으므로(06 §4) 주기 센서 세 개만 대상이다. */
TEST_P(TickReadFailureTest, FailureTurnsOffAndStopsThenLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    Apply(kClear);
    rvc_mock_fail_next(&mock, GetParam().operation, 1);
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
    ExpectLocked();
}

INSTANTIATE_TEST_SUITE_P(Spec, TickReadFailureTest,
                         ::testing::Values(ReadFailureCase{"Left", RVC_MOCK_READ_LEFT},
                                           ReadFailureCase{"Right", RVC_MOCK_READ_RIGHT},
                                           ReadFailureCase{"Dust", RVC_MOCK_READ_DUST}),
                         [](const ::testing::TestParamInfo<ReadFailureCase> &info) {
                             return std::string(info.param.name);
                         });

/* REQ: ERR, 02-ERR, 02-OUT-2 — 회피 진입의 청소 OFF 쓰기가 실패하면 회피 모터 명령은 나가지 않고,
 * 오류 처리로 Off → Stop을 시도한 뒤 잠긴다. (실패한 쓰기는 장치 기록에 남지 않는다.) */
TEST_F(RvcSystemTest, CleanerWriteFailureOnAvoidanceNeverTurnsAndLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    Apply(In(1, 0, 0));
    ReportFrontIfChanged(In(1, 0, 0));
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_CLEANER, 1);
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "M:STOP"}));
    ExpectLocked();
}

/* REQ: ERR, 02-ERR — 회피 모터 명령 쓰기가 실패하면 Off와 Stop을 각각 시도한 뒤 잠긴다. */
TEST_F(RvcSystemTest, MotorWriteFailureOnAvoidanceTurnsOffAndStopsThenLocks)
{
    ASSERT_NO_FATAL_FAILURE(Reach(Start::ForwardOn));
    Apply(In(1, 0, 0));
    ReportFrontIfChanged(In(1, 0, 0));
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_MOTOR, 1);
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    /* 회피용 OFF(성공) → RIGHT(실패, 기록 없음) → 오류 처리 OFF → STOP */
    EXPECT_EQ(TakeOutputs(), Outputs({"C:OFF", "C:OFF", "M:STOP"}));
    ExpectLocked();
}

}  // namespace
