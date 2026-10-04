/*
 * GoogleTest로 C 제품 코드를 검증하는 C++ 테스트 실행기.
 * 일반적인 읽기 순서: 준비(Initialize/Reach/SetInput) -> 실행(Tick/API 호출)
 * -> 확인(Telemetry/ExpectEvents). ASSERT는 해당 함수의 진행을 멈추고 EXPECT는 검사를 계속한다.
 * 센서 입력으로 상태에 도달하며 내부 상태를 강제로 덮어쓰지 않는다. 실제 시간 대기는 사용하지 않는다.
 * TEST_F는 공통 장치 준비를 쓰고, TEST_P는 같은 검사를 여러 입력에 반복한다.
 */
#include <gtest/gtest.h>
#include <array>
#include <initializer_list>
#include <string>
#include <vector>
#include "rvc/rvc.h"
#include "mock_device.h"
extern "C" {
#include "actions/actions.h"
#include "interfaces/interfaces.h"
#include "perception/perception.h"
}

namespace {
/* F/L/R/D 순서의 테스트 입력을 만든다. 장애물 true는 막힘, dust true는 먼지 감지다. */
RvcSensorSnapshot Input(bool front, bool left, bool right, bool dust = false)
{
    return {{front, left, right}, dust};
}
/* 0~15를 F(8), L(4), R(2), D(1) 비트로 해석하여 16가지 입력을 빠짐없이 만든다. */
RvcSensorSnapshot Combo(int bits)
{
    return Input((bits & 8) != 0, (bits & 4) != 0,
                 (bits & 2) != 0, (bits & 1) != 0);
}
/* 기대하는 모터 로그 한 건을 만든다. 실제 장치를 호출하지 않는다. */
RvcMockEvent Motor(RvcMotorCommand command) { return {RVC_MOCK_MOTOR, command}; }
/* 기대하는 청소 로그 한 건을 만든다. 실제 장치를 호출하지 않는다. */
RvcMockEvent Cleaner(RvcCleanerCommand command) { return {RVC_MOCK_CLEANER, command}; }
/* 명령의 개수·장치 종류·값·순서를 모두 비교한다. {}는 명령이 전혀 없어야 한다는 뜻이다. */
void ExpectEvents(const RvcMockDevice &mock, std::initializer_list<RvcMockEvent> expected)
{
    ASSERT_EQ(mock.event_count, expected.size());
    size_t index = 0;
    for (const auto &event : expected) {
        SCOPED_TRACE(index);
        EXPECT_EQ(mock.events[index].kind, event.kind);
        EXPECT_EQ(mock.events[index].command, event.command);
        ++index;
    }
}

/* 각 테스트에 독립적인 모의 장치/로봇을 제공한다. 다른 테스트의 센서·고장·타이머가 섞이지 않는다. */
class RobotTest : public ::testing::Test {
protected:
    RvcMockDevice mock{};
    Rvc *robot = nullptr;
    /* GoogleTest가 각 테스트 전에 실행한다. 장치 연결과 생성만 하고 초기화는 각 검사가 선택한다. */
    void SetUp() override
    {
        rvc_mock_init(&mock);
        RvcDevice device = rvc_mock_device(&mock);
        robot = rvc_create(&device);
        ASSERT_NE(robot, nullptr);
    }
    /* 각 테스트 뒤 메모리만 해제한다. shutdown의 명령 동작은 별도의 검사에서 확인한다. */
    void TearDown() override { rvc_destroy(robot); }
    /* 초기 입력을 설치하고 초기화한다. 이후 명령만 비교할 수 있게 초기 STOP/OFF 로그는 비운다. */
    void Initialize(RvcSensorSnapshot input = Input(false, false, false))
    {
        rvc_mock_set_inputs(&mock, input);
        ASSERT_EQ(rvc_initialize(robot), RVC_OK);
        rvc_mock_clear_events(&mock);
    }
    /* 주기 센서의 원시 입력과 전방 캐시를 함께 갱신한다. 이 함수 자체로 Tick은 진행하지 않는다. */
    void SetInput(RvcSensorSnapshot input)
    {
        rvc_mock_set_inputs(&mock, input);
        ASSERT_EQ(rvc_report_front(robot, input.obstacles.front_blocked), RVC_OK);
    }
    /* 정상적으로 준비된 로봇의 관찰값을 읽는다. API 성공 여부도 함께 검사한다. */
    RvcTelemetry Telemetry()
    {
        RvcTelemetry result{};
        EXPECT_EQ(rvc_get_telemetry(robot, &result), RVC_OK);
        return result;
    }
    /* 대기 없이 논리 Tick을 count번 호출한다. 실패가 예상되는 검사는 이 헬퍼 대신 API를 직접 호출한다. */
    void Tick(unsigned count = 1)
    {
        for (unsigned i = 0; i < count; ++i) ASSERT_EQ(rvc_tick(robot), RVC_OK);
    }
    /* 초기화와 실제 센서 입력으로 지정 상태에 진입한다. 진입 직후 e=0인 검사 시작점을 만든다. */
    void Reach(RvcState state)
    {
        Initialize();
        switch (state) {
        case RVC_STATE_STOP_OFF: break;
        case RVC_STATE_F_ON: SetInput(Input(false, false, false)); Tick(); break;
        case RVC_STATE_F_POWER1: SetInput(Input(false, false, false, true)); Tick(); break;
        case RVC_STATE_LEFT_OFF: SetInput(Input(true, false, true)); Tick(); break;
        case RVC_STATE_RIGHT_OFF: SetInput(Input(true, false, false)); Tick(); break;
        case RVC_STATE_BACK_OFF: SetInput(Input(true, true, true)); Tick(); break;
        default: FAIL() << "Unsupported state in input-driven fixture";
        }
        ASSERT_EQ(Telemetry().state, state);
        rvc_mock_clear_events(&mock);
    }
    /* 준비되지 않은 상태에서 Tick/통지가 거부되고, 실패한 조회가 호출자의 출력 인수를 보존하는지 검사한다. */
    void ExpectNotReady()
    {
        RvcTelemetry untouched{};
        untouched.elapsed_ticks = 12345;
        untouched.state = RVC_STATE_F_POWER1;
        EXPECT_EQ(rvc_get_telemetry(robot, &untouched), RVC_NOT_READY);
        EXPECT_EQ(untouched.elapsed_ticks, 12345u);
        EXPECT_EQ(untouched.state, RVC_STATE_F_POWER1);
        EXPECT_EQ(rvc_tick(robot), RVC_NOT_READY);
        EXPECT_EQ(rvc_report_front(robot, true), RVC_NOT_READY);
    }
};

/* 생성만으로 하드웨어에 접근하지 않으며, 초기화 전에는 제어/조회할 수 없어야 한다. */
TEST_F(RobotTest, CreateHasNoSensorReadsOrDeviceOutputs)
{
    EXPECT_EQ(mock.event_count, 0u);
    for (auto count : mock.read_counts) EXPECT_EQ(count, 0u);
    ExpectNotReady();
}
/* 초기화의 STOP -> OFF 순서, 전체 센서 1회 읽기, STOP_OFF/e=0 관찰값을 확인한다. */
TEST_F(RobotTest, InitializationCommandsStopThenOffAndSamplesEverySensor)
{
    rvc_mock_set_inputs(&mock, Input(true, false, true, true));
    ASSERT_EQ(rvc_initialize(robot), RVC_OK);
    ExpectEvents(mock, {Motor(RVC_MOTOR_STOP), Cleaner(RVC_CLEANER_OFF)});
    for (auto count : mock.read_counts) EXPECT_EQ(count, 1u);
    auto status = Telemetry();
    EXPECT_TRUE(status.initialized);
    EXPECT_TRUE(status.motor_valid);
    EXPECT_TRUE(status.cleaner_valid);
    EXPECT_EQ(status.state, RVC_STATE_STOP_OFF);
    EXPECT_EQ(status.elapsed_ticks, 0u);
    EXPECT_TRUE(status.sensors.obstacles.front_blocked);
    EXPECT_FALSE(status.sensors.obstacles.left_blocked);
    EXPECT_TRUE(status.sensors.obstacles.right_blocked);
    EXPECT_TRUE(status.sensors.dust_detected);
}
/* 초기 1회 읽기 이후 좌/우/먼지는 매 Tick 읽고 전방은 다시 읽지 않아야 한다. */
TEST_F(RobotTest, TickRefreshesPeriodicSensorsButDoesNotReadFrontAgain)
{
    Initialize();
    Tick(9);
    EXPECT_EQ(mock.read_counts[RVC_MOCK_READ_FRONT], 1u);
    EXPECT_EQ(mock.read_counts[RVC_MOCK_READ_LEFT], 10u);
    EXPECT_EQ(mock.read_counts[RVC_MOCK_READ_RIGHT], 10u);
    EXPECT_EQ(mock.read_counts[RVC_MOCK_READ_DUST], 10u);
}
/* 전방 통지는 캐시만 바꾼다. 다음 Tick에서야 상태가 바뀌고 OFF -> 우회전 명령이 발생한다. */
TEST_F(RobotTest, FrontReportDoesNotAdvanceStateOrEmitCommands)
{
    Reach(RVC_STATE_F_ON);
    ASSERT_EQ(rvc_report_front(robot, true), RVC_OK);
    EXPECT_EQ(Telemetry().state, RVC_STATE_F_ON);
    EXPECT_EQ(mock.event_count, 0u);
    Tick();
    EXPECT_EQ(Telemetry().state, RVC_STATE_RIGHT_OFF);
    ExpectEvents(mock, {Cleaner(RVC_CLEANER_OFF), Motor(RVC_MOTOR_RIGHT)});
}
/* 원시 전방 입력만 바꿔서는 캐시가 갱신되지 않는다. 변화 통지 방식의 계약을 검증한다. */
TEST_F(RobotTest, ChangingMockFrontWithoutReportDoesNotChangeCachedFront)
{
    Reach(RVC_STATE_F_ON);
    rvc_mock_set_inputs(&mock, Input(true, false, false));
    Tick();
    EXPECT_EQ(Telemetry().state, RVC_STATE_F_ON);
    EXPECT_FALSE(Telemetry().sensors.obstacles.front_blocked);
    EXPECT_EQ(mock.event_count, 0u);
}
/* 한 Tick 전에 전방 통지가 여러 번 오면 마지막 값으로 판단한다. */
TEST_F(RobotTest, LastFrontReportBeforeTickIsUsed)
{
    Reach(RVC_STATE_F_ON);
    ASSERT_EQ(rvc_report_front(robot, true), RVC_OK);
    ASSERT_EQ(rvc_report_front(robot, false), RVC_OK);
    Tick();
    EXPECT_EQ(Telemetry().state, RVC_STATE_F_ON);
    EXPECT_EQ(mock.event_count, 0u);
}

/* 설계 전이표에서 별도로 작성한 기대값. 제품의 판단 함수를 다시 호출해 정답으로 삼지 않는다.
 * 인덱스는 FLRD 이진 조합이다. normal은 전진 가능 여부를 먼저 보고,
 * after_back은 전방이 열려도 우/좌 회전 또는 후진 재시작을 선택한다.
 */
const std::array<RvcState, 16> normal_decisions = {
    RVC_STATE_F_ON, RVC_STATE_F_POWER1, RVC_STATE_F_ON, RVC_STATE_F_POWER1,
    RVC_STATE_F_ON, RVC_STATE_F_POWER1, RVC_STATE_F_ON, RVC_STATE_F_POWER1,
    RVC_STATE_RIGHT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_LEFT_OFF, RVC_STATE_LEFT_OFF,
    RVC_STATE_RIGHT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_BACK_OFF, RVC_STATE_BACK_OFF
};
const std::array<RvcState, 16> after_back_decisions = {
    RVC_STATE_RIGHT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_LEFT_OFF, RVC_STATE_LEFT_OFF,
    RVC_STATE_RIGHT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_BACK_OFF, RVC_STATE_BACK_OFF,
    RVC_STATE_RIGHT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_LEFT_OFF, RVC_STATE_LEFT_OFF,
    RVC_STATE_RIGHT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_BACK_OFF, RVC_STATE_BACK_OFF
};
/* 검사 시작 상태와 이번에 공급할 FLRD 조합을 한 개의 매개변수로 묶는다. */
struct DecisionCase { RvcState start; int bits; };
/* 제어 가능한 6개 상태 × 16개 입력 = 96개의 독립 테스트 인스턴스를 만든다. */
std::vector<DecisionCase> Decisions()
{
    std::vector<DecisionCase> cases;
    for (auto state : {RVC_STATE_STOP_OFF, RVC_STATE_F_ON, RVC_STATE_F_POWER1,
                       RVC_STATE_LEFT_OFF, RVC_STATE_RIGHT_OFF, RVC_STATE_BACK_OFF}) {
        for (int bits = 0; bits < 16; ++bits) cases.push_back({state, bits});
    }
    return cases;
}
/* 출력 순서 검사를 위해 두 전진 상태를 한 범주로 분류한다. 상태 전이 결정에는 사용하지 않는다. */
bool ForwardState(RvcState state)
{ return state == RVC_STATE_F_ON || state == RVC_STATE_F_POWER1; }
/* 설계상 각 상태가 유지해야 할 모터 출력을 반환한다. */
RvcMotorCommand ExpectedMotor(RvcState state)
{
    switch (state) {
    case RVC_STATE_F_ON: case RVC_STATE_F_POWER1: return RVC_MOTOR_FORWARD;
    case RVC_STATE_LEFT_OFF: return RVC_MOTOR_LEFT;
    case RVC_STATE_RIGHT_OFF: return RVC_MOTOR_RIGHT;
    case RVC_STATE_BACK_OFF: return RVC_MOTOR_BACKWARD;
    default: return RVC_MOTOR_STOP;
    }
}
/* 일반 전진은 ON, 강화 전진은 UP, 나머지 상태는 OFF여야 한다. */
RvcCleanerCommand ExpectedCleaner(RvcState state)
{
    if (state == RVC_STATE_F_ON) return RVC_CLEANER_ON;
    if (state == RVC_STATE_F_POWER1) return RVC_CLEANER_UP;
    return RVC_CLEANER_OFF;
}
/* RobotTest의 준비 기능과 DecisionCase 매개변수를 함께 사용하는 테스트 묶음. */
class DecisionTest : public RobotTest, public ::testing::WithParamInterface<DecisionCase> {};
/* 회피 중이라면 종료 직전까지 먼저 진행하고, 마지막 1 Tick에서 16개 조합을 공급한다.
 * 다음 상태뿐 아니라 e=0, 저장된 센서값, 최종 출력, 중복 명령 유무와 명령 순서까지 확인한다.
 */
TEST_P(DecisionTest, SensorCombinationAtDecisionBoundaryMatchesModelAndOutputOrder)
{
    /* 준비(Arrange): 지정 상태 및 판단 경계 직전까지 실제 입력으로 진행한다. */
    const auto test = GetParam();
    Reach(test.start);
    if (test.start == RVC_STATE_LEFT_OFF || test.start == RVC_STATE_RIGHT_OFF) Tick(4);
    if (test.start == RVC_STATE_BACK_OFF) Tick(2);
    rvc_mock_clear_events(&mock);
    /* 실행(Act): 이번 조합을 넣고 판단 Tick을 한 번만 진행한다. */
    const auto input = Combo(test.bits);
    SetInput(input);
    Tick();
    /* 확인(Assert): 독립 전이표와 실제 결과를 비교한다. */
    const auto expected = (test.start == RVC_STATE_BACK_OFF ? after_back_decisions : normal_decisions)[test.bits];
    const auto actual = Telemetry();
    EXPECT_EQ(actual.state, expected);
    EXPECT_EQ(actual.elapsed_ticks, 0u);
    EXPECT_EQ(actual.motor, ExpectedMotor(expected));
    EXPECT_EQ(actual.cleaner, ExpectedCleaner(expected));
    EXPECT_EQ(actual.sensors.obstacles.front_blocked, input.obstacles.front_blocked);
    EXPECT_EQ(actual.sensors.obstacles.left_blocked, input.obstacles.left_blocked);
    EXPECT_EQ(actual.sensors.obstacles.right_blocked, input.obstacles.right_blocked);
    EXPECT_EQ(actual.sensors.dust_detected, input.dust_detected);
    if (!ForwardState(expected) && ForwardState(test.start)) {
        ExpectEvents(mock, {Cleaner(RVC_CLEANER_OFF), Motor(ExpectedMotor(expected))});
    } else if (!ForwardState(expected)) {
        ExpectEvents(mock, {Motor(ExpectedMotor(expected))});
    } else if (!ForwardState(test.start)) {
        ExpectEvents(mock, {Motor(RVC_MOTOR_FORWARD), Cleaner(ExpectedCleaner(expected))});
    } else if (expected != test.start) {
        ExpectEvents(mock, {Cleaner(ExpectedCleaner(expected))});
    } else {
        ExpectEvents(mock, {});
    }
}
/* 실패 결과 이름의 State번호/FLRD를 보면 어떤 시작 상태와 센서 조합인지 재현할 수 있다. */
INSTANTIATE_TEST_SUITE_P(AllStatesAndInputs, DecisionTest, ::testing::ValuesIn(Decisions()),
    [](const ::testing::TestParamInfo<DecisionCase> &info) {
        return "State" + std::to_string(info.param.start) + "_FLRD" +
               std::to_string((info.param.bits >> 3) & 1) +
               std::to_string((info.param.bits >> 2) & 1) +
               std::to_string((info.param.bits >> 1) & 1) +
               std::to_string(info.param.bits & 1);
    });

/* 회전 5 Tick, 후진 3 Tick: 진입 Tick은 e=0이고 이후 호출 횟수로 기간을 센다. */
struct TimingCase { RvcState state; unsigned duration; };
class TimingTest : public RobotTest, public ::testing::WithParamInterface<TimingCase> {};
/* 만료 전에는 센서를 갱신해도 회피 상태를 유지하고 출력 재전송을 하지 않아야 한다. */
TEST_P(TimingTest, EntryStartsAtZeroAndHoldingDoesNotWriteOutputs)
{
    const auto test = GetParam();
    Reach(test.state);
    ASSERT_EQ(Telemetry().elapsed_ticks, 0u);
    SetInput(Input(false, false, false, true));
    for (unsigned elapsed = 1; elapsed < test.duration; ++elapsed) {
        Tick();
        EXPECT_EQ(Telemetry().state, test.state);
        EXPECT_EQ(Telemetry().elapsed_ticks, elapsed);
        EXPECT_EQ(mock.event_count, 0u);
        EXPECT_TRUE(Telemetry().sensors.dust_detected);
    }
    Tick();
    EXPECT_EQ(Telemetry().state, test.state == RVC_STATE_BACK_OFF ? RVC_STATE_RIGHT_OFF : RVC_STATE_F_POWER1);
    EXPECT_EQ(Telemetry().elapsed_ticks, 0u);
}
/* 만료 후 같은 회피가 필요하면 모터 Trigger를 재전송하고 e를 0부터 새로 센다. */
TEST_P(TimingTest, SameManeuverAtExpiryRetriggersAndResetsCounter)
{
    const auto test = GetParam();
    Reach(test.state);
    Tick(test.duration);
    EXPECT_EQ(Telemetry().state, test.state);
    EXPECT_EQ(Telemetry().elapsed_ticks, 0u);
    ExpectEvents(mock, {Motor(ExpectedMotor(test.state))});
    rvc_mock_clear_events(&mock);
    Tick();
    EXPECT_EQ(Telemetry().elapsed_ticks, 1u);
    EXPECT_EQ(mock.event_count, 0u);
}
/* 좌회전·우회전·후진 각각에 위 두 경계 검사를 적용한다(총 6개). */
INSTANTIATE_TEST_SUITE_P(TurnAndBackBoundaries, TimingTest,
    ::testing::Values(TimingCase{RVC_STATE_LEFT_OFF, 5},
                      TimingCase{RVC_STATE_RIGHT_OFF, 5},
                      TimingCase{RVC_STATE_BACK_OFF, 3}));

/* 현재 PPT 정책: 먼지 미감지 시 다음 Tick에 ON으로 복귀하며 별도 강화 유지시간은 두지 않는다.
 * ON/UP 전환은 청소 명령만 보내고, 같은 모드가 유지되는 동안 중복 명령을 보내지 않는다.
 */
TEST_F(RobotTest, DustLossImmediatelyRestoresNormalCleaningWithoutMotorWrite)
{
    Reach(RVC_STATE_F_POWER1);
    SetInput(Input(false, false, false, false));
    Tick();
    EXPECT_EQ(Telemetry().state, RVC_STATE_F_ON);
    ExpectEvents(mock, {Cleaner(RVC_CLEANER_ON)});
    rvc_mock_clear_events(&mock);
    SetInput(Input(false, false, false, true));
    Tick();
    EXPECT_EQ(Telemetry().state, RVC_STATE_F_POWER1);
    ExpectEvents(mock, {Cleaner(RVC_CLEANER_UP)});
    rvc_mock_clear_events(&mock);
    Tick(20);
    ExpectEvents(mock, {});
}
/* 후진 3 Tick 뒤에는 전방이 열렸어도 먼저 회전하고, 회전 5 Tick 뒤 전진/청소를 재개한다. */
TEST_F(RobotTest, BackwardMustTurnBeforeForwardEvenWhenFrontBecomesClear)
{
    Reach(RVC_STATE_BACK_OFF);
    SetInput(Input(false, false, false, true));
    Tick(3);
    EXPECT_EQ(Telemetry().state, RVC_STATE_RIGHT_OFF);
    ExpectEvents(mock, {Motor(RVC_MOTOR_RIGHT)});
    rvc_mock_clear_events(&mock);
    Tick(5);
    EXPECT_EQ(Telemetry().state, RVC_STATE_F_POWER1);
    ExpectEvents(mock, {Motor(RVC_MOTOR_FORWARD), Cleaner(RVC_CLEANER_UP)});
}
/* 종료는 OFF -> STOP 후 NOT_READY로 만든다. 재초기화하면 STOP -> OFF로 운전 준비를 다시 한다. */
TEST_F(RobotTest, ShutdownWritesOffThenStopAndRequiresReinitialization)
{
    Reach(RVC_STATE_F_POWER1);
    ASSERT_EQ(rvc_shutdown(robot), RVC_OK);
    ExpectEvents(mock, {Cleaner(RVC_CLEANER_OFF), Motor(RVC_MOTOR_STOP)});
    ExpectNotReady();
    rvc_mock_clear_events(&mock);
    ASSERT_EQ(rvc_initialize(robot), RVC_OK);
    EXPECT_EQ(Telemetry().state, RVC_STATE_STOP_OFF);
    ExpectEvents(mock, {Motor(RVC_MOTOR_STOP), Cleaner(RVC_CLEANER_OFF)});
}
/* 회전 도중 재초기화해도 타이머/상태가 초기값으로 돌아가고 전방 원시 센서를 다시 읽어야 한다. */
TEST_F(RobotTest, ReinitializeResetsActiveTimerAndReadsFrontAgain)
{
    Reach(RVC_STATE_RIGHT_OFF);
    Tick(2);
    rvc_mock_set_inputs(&mock, Input(false, true, true));
    rvc_mock_clear_events(&mock);
    ASSERT_EQ(rvc_initialize(robot), RVC_OK);
    EXPECT_EQ(Telemetry().state, RVC_STATE_STOP_OFF);
    EXPECT_EQ(Telemetry().elapsed_ticks, 0u);
    EXPECT_EQ(mock.read_counts[RVC_MOCK_READ_FRONT], 2u);
    EXPECT_FALSE(Telemetry().sensors.obstacles.front_blocked);
    ExpectEvents(mock, {Motor(RVC_MOTOR_STOP), Cleaner(RVC_CLEANER_OFF)});
}
/* 메모리 해제와 장치 정지는 별개 API다. destroy가 숨은 장치 명령을 만들지 않는지 확인한다. */
TEST_F(RobotTest, DestroyDoesNotImplicitlyWriteDevices)
{
    Reach(RVC_STATE_F_ON);
    rvc_destroy(robot);
    robot = nullptr;
    EXPECT_EQ(mock.event_count, 0u);
}
/* 로봇 두 대를 다르게 운전하여 센서·타이머·로그를 전역 변수로 공유하지 않는지 확인한다. */
TEST_F(RobotTest, SeparateInstancesHaveIndependentSensorsTimersAndLogs)
{
    RvcMockDevice other_mock{};
    rvc_mock_init(&other_mock);
    RvcDevice other_device = rvc_mock_device(&other_mock);
    Rvc *other = rvc_create(&other_device);
    ASSERT_NE(other, nullptr);
    Reach(RVC_STATE_RIGHT_OFF);
    rvc_mock_set_inputs(&other_mock, Input(false, true, true, true));
    ASSERT_EQ(rvc_initialize(other), RVC_OK);
    rvc_mock_clear_events(&other_mock);
    ASSERT_EQ(rvc_tick(other), RVC_OK);
    Tick(2);
    RvcTelemetry other_status{};
    ASSERT_EQ(rvc_get_telemetry(other, &other_status), RVC_OK);
    EXPECT_EQ(other_status.state, RVC_STATE_F_POWER1);
    EXPECT_EQ(other_status.elapsed_ticks, 0u);
    EXPECT_EQ(Telemetry().state, RVC_STATE_RIGHT_OFF);
    EXPECT_EQ(Telemetry().elapsed_ticks, 2u);
    EXPECT_EQ(mock.event_count, 0u);
    ExpectEvents(other_mock, {Motor(RVC_MOTOR_FORWARD), Cleaner(RVC_CLEANER_UP)});
    rvc_destroy(other);
}

/* 실패시킬 장치 콜백을 매개변수로 받아 초기화 오류 처리를 반복 검증한다. */
class InitFailureTest : public RobotTest, public ::testing::WithParamInterface<RvcMockOperation> {};
/* 초기화의 어느 콜백이 한 번 실패해도 오류/NOT_READY를 보고하고, 재시도로 회복 가능해야 한다. */
TEST_P(InitFailureTest, FailureIsReportedAndRetryCanRecover)
{
    rvc_mock_fail_next(&mock, GetParam(), 1);
    EXPECT_EQ(rvc_initialize(robot), RVC_IO_ERROR);
    EXPECT_GE(mock.write_counts[RVC_MOCK_MOTOR], 1u);
    EXPECT_GE(mock.write_counts[RVC_MOCK_CLEANER], 1u);
    ExpectNotReady();
    ASSERT_EQ(rvc_initialize(robot), RVC_OK);
    EXPECT_EQ(Telemetry().state, RVC_STATE_STOP_OFF);
}
/* 네 센서 읽기와 두 장치 쓰기를 각각 한 번씩 실패시키는 6가지 검사. */
INSTANTIATE_TEST_SUITE_P(EveryDeviceOperation, InitFailureTest,
    ::testing::Values(RVC_MOCK_READ_FRONT, RVC_MOCK_READ_LEFT, RVC_MOCK_READ_RIGHT,
                      RVC_MOCK_READ_DUST, RVC_MOCK_WRITE_MOTOR, RVC_MOCK_WRITE_CLEANER));

/* Tick에서 실제로 읽는 주기 센서의 실패를 하나씩 주입한다. */
class TickReadFailureTest : public RobotTest, public ::testing::WithParamInterface<RvcMockOperation> {};
/* 센서 오류 시 OFF/STOP을 시도하고 제어를 중단한다. 자동 재개 대신 재초기화가 필요하다. */
TEST_P(TickReadFailureTest, FailureAttemptsOffAndStopThenLatchesNotReady)
{
    Reach(RVC_STATE_F_POWER1);
    rvc_mock_fail_next(&mock, GetParam(), 1);
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    ExpectEvents(mock, {Cleaner(RVC_CLEANER_OFF), Motor(RVC_MOTOR_STOP)});
    ExpectNotReady();
    ASSERT_EQ(rvc_initialize(robot), RVC_OK);
    EXPECT_EQ(Telemetry().state, RVC_STATE_STOP_OFF);
}
/* 전방은 Tick 콜백 읽기가 아니라 통지 방식이므로 여기에는 좌/우/먼지만 포함한다. */
INSTANTIATE_TEST_SUITE_P(PeriodicSensors, TickReadFailureTest,
    ::testing::Values(RVC_MOCK_READ_LEFT, RVC_MOCK_READ_RIGHT, RVC_MOCK_READ_DUST));

/* 회피 직전 OFF가 실패하면 회전 명령을 보내지 않고, 오류 처리에서 OFF/STOP을 다시 시도한다. */
TEST_F(RobotTest, CleanerFailurePreventsTurnThenRecoveryAttemptsBothOutputs)
{
    Reach(RVC_STATE_F_ON);
    SetInput(Input(true, false, false));
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_CLEANER, 1);
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    ExpectEvents(mock, {Cleaner(RVC_CLEANER_OFF), Motor(RVC_MOTOR_STOP)});
    ExpectNotReady();
}
/* OFF 성공 뒤 회전 실패 시, 오류 처리의 OFF/STOP까지 기록되어야 하며 회전 성공으로 보고하면 안 된다. */
TEST_F(RobotTest, MotorFailureAfterOffTriggersRecoveryAndDoesNotClaimTurnSuccess)
{
    Reach(RVC_STATE_F_ON);
    SetInput(Input(true, false, false));
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_MOTOR, 1);
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    ExpectEvents(mock, {Cleaner(RVC_CLEANER_OFF), Cleaner(RVC_CLEANER_OFF), Motor(RVC_MOTOR_STOP)});
    ExpectNotReady();
}
/* OFF가 회피 준비와 오류 처리에서 연속 실패해도 STOP은 시도한다. 실패한 출력의 자동 복원은 가정하지 않는다. */
TEST_F(RobotTest, PersistentCleanerFailureStillAttemptsMotorStop)
{
    Reach(RVC_STATE_F_ON);
    SetInput(Input(true, false, false));
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_CLEANER, 2);
    const auto cleaner_attempts = mock.write_counts[RVC_MOCK_CLEANER];
    EXPECT_EQ(rvc_tick(robot), RVC_IO_ERROR);
    EXPECT_EQ(mock.write_counts[RVC_MOCK_CLEANER], cleaner_attempts + 2);
    ExpectEvents(mock, {Motor(RVC_MOTOR_STOP)});
    EXPECT_EQ(mock.cleaner, RVC_CLEANER_ON); // OFF가 두 번 실패했으므로 마지막 성공 출력 ON이 남는다.
    ExpectNotReady();
}
/* 종료 중 OFF 실패가 STOP 시도를 가로막지 않아야 하며, 종료 API는 실패를 호출자에게 알려야 한다. */
TEST_F(RobotTest, ShutdownCleanerFailureStillAttemptsStopAndReturnsFailure)
{
    Reach(RVC_STATE_F_ON);
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_CLEANER, 1);
    EXPECT_EQ(rvc_shutdown(robot), RVC_IO_ERROR);
    ExpectEvents(mock, {Motor(RVC_MOTOR_STOP)});
    ExpectNotReady();
}
/* Forward Disable은 전진 활성 표시 해제다. 실제 모터 정지는 별도의 Stop Motor 호출로 확인한다. */
TEST_F(RobotTest, ForwardDisableDoesNotMeanStopMotor)
{
    Initialize();
    ASSERT_EQ(rvc_move_forward(robot, RVC_FORWARD_ENABLE), RVC_OK);
    rvc_mock_clear_events(&mock);
    ASSERT_EQ(rvc_move_forward(robot, RVC_FORWARD_DISABLE), RVC_OK);
    EXPECT_EQ(mock.event_count, 0u);
    EXPECT_EQ(mock.motor, RVC_MOTOR_FORWARD);
    ASSERT_EQ(rvc_stop_motor(robot), RVC_OK);
    ExpectEvents(mock, {Motor(RVC_MOTOR_STOP)});
}
/* 범위 밖 열거값은 INVALID_ARGUMENT이며 장치 명령을 생성하지 않아야 한다. */
TEST_F(RobotTest, InvalidActionEnumsAreRejectedWithoutWrites)
{
    Initialize();
    EXPECT_EQ(rvc_move_forward(robot, static_cast<RvcForwardControl>(99)), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_set_cleaning(robot, static_cast<RvcCleaningMode>(99)), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_motor_apply(robot, static_cast<RvcMotorCommand>(99)), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_cleaner_apply(robot, static_cast<RvcCleanerCommand>(99)), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(mock.event_count, 0u);
}
/* 복수 센서 중 일부만 읽혔어도 실패라면 호출자의 out을 덮어쓰지 않는 입력/출력 계약을 검사한다. */
TEST_F(RobotTest, DetectorFailureDoesNotPublishPartialOutput)
{
    Initialize(Input(false, false, false, false));
    RvcObstacles obstacles{true, true, true};
    rvc_mock_fail_next(&mock, RVC_MOCK_READ_RIGHT, 1);
    EXPECT_EQ(rvc_determine_obstacles(robot, &obstacles), RVC_IO_ERROR);
    EXPECT_TRUE(obstacles.front_blocked);
    EXPECT_TRUE(obstacles.left_blocked);
    EXPECT_TRUE(obstacles.right_blocked);
    bool dust = true;
    rvc_mock_fail_next(&mock, RVC_MOCK_READ_DUST, 1);
    EXPECT_EQ(rvc_determine_dust(robot, &dust), RVC_IO_ERROR);
    EXPECT_TRUE(dust);
    EXPECT_EQ(mock.event_count, 0u);
}
/* 출력 인터페이스를 직접 실패시키면 valid=false가 되고 마지막 성공 값은 남는다.
 * Controller 경유 호출과 구별되는 하위 모듈 검사이므로 여기서 자동 shutdown을 기대하지 않는다.
 */
TEST_F(RobotTest, OutputInterfaceFailureInvalidatesTelemetryWithoutInventingOutput)
{
    Initialize();
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_MOTOR, 1);
    EXPECT_EQ(rvc_motor_apply(robot, RVC_MOTOR_FORWARD), RVC_IO_ERROR);
    auto status = Telemetry();
    EXPECT_FALSE(status.motor_valid);
    EXPECT_EQ(status.motor, RVC_MOTOR_STOP);
    EXPECT_EQ(mock.motor, RVC_MOTOR_STOP);
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_CLEANER, 1);
    EXPECT_EQ(rvc_cleaner_apply(robot, RVC_CLEANER_UP), RVC_IO_ERROR);
    status = Telemetry();
    EXPECT_FALSE(status.cleaner_valid);
    EXPECT_EQ(status.cleaner, RVC_CLEANER_OFF);
    EXPECT_EQ(mock.cleaner, RVC_CLEANER_OFF);
    EXPECT_EQ(mock.event_count, 0u);
}

/* 공개 API의 NULL 방어와 6개 필수 장치 콜백의 누락 거부를 검사한다. */
TEST(PublicArguments, NullPointersAndMissingCallbacksAreRejected)
{
    EXPECT_EQ(rvc_create(nullptr), nullptr);
    EXPECT_EQ(rvc_initialize(nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_shutdown(nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_tick(nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(rvc_report_front(nullptr, true), RVC_INVALID_ARGUMENT);
    RvcTelemetry telemetry{};
    EXPECT_EQ(rvc_get_telemetry(nullptr, &telemetry), RVC_INVALID_ARGUMENT);
    rvc_destroy(nullptr);
    RvcMockDevice mock{};
    rvc_mock_init(&mock);
    for (int missing = 0; missing < 6; ++missing) {
        RvcDevice device = rvc_mock_device(&mock);
        switch (missing) {
        case 0: device.read_front = nullptr; break;
        case 1: device.read_left = nullptr; break;
        case 2: device.read_right = nullptr; break;
        case 3: device.read_dust = nullptr; break;
        case 4: device.write_motor = nullptr; break;
        case 5: device.write_cleaner = nullptr; break;
        }
        EXPECT_EQ(rvc_create(&device), nullptr);
    }
    RvcDevice device = rvc_mock_device(&mock);
    Rvc *robot = rvc_create(&device);
    ASSERT_NE(robot, nullptr);
    EXPECT_EQ(rvc_get_telemetry(robot, nullptr), RVC_INVALID_ARGUMENT);
    rvc_destroy(robot);
}
/* 테스트 도구 자체의 계약: 읽기 실패는 out을 보존하고, 쓰기 실패는 마지막 성공 출력을 보존한다. */
TEST(MockContract, FailuresPreserveReadOutputAndSuccessfulDeviceState)
{
    RvcMockDevice mock{};
    rvc_mock_init(&mock);
    RvcDevice device = rvc_mock_device(&mock);
    bool value = true;
    rvc_mock_fail_next(&mock, RVC_MOCK_READ_FRONT, 1);
    EXPECT_EQ(device.read_front(device.context, &value), RVC_IO_ERROR);
    EXPECT_TRUE(value);
    ASSERT_EQ(device.write_motor(device.context, RVC_MOTOR_FORWARD), RVC_OK);
    rvc_mock_fail_next(&mock, RVC_MOCK_WRITE_MOTOR, 1);
    EXPECT_EQ(device.write_motor(device.context, RVC_MOTOR_RIGHT), RVC_IO_ERROR);
    EXPECT_EQ(mock.motor, RVC_MOTOR_FORWARD);
    ExpectEvents(mock, {Motor(RVC_MOTOR_FORWARD)});
}
/* NULL/잘못된 명령/가득 찬 로그를 실패로 처리하고 성공 기록이나 valid를 꾸며내지 않는지 확인한다. */
TEST(MockContract, FullLogAndInvalidValuesAreErrorsWithoutSuccessfulWrites)
{
    RvcMockDevice mock{};
    rvc_mock_init(&mock);
    RvcDevice device = rvc_mock_device(&mock);
    EXPECT_EQ(device.read_left(nullptr, nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(device.read_left(device.context, nullptr), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(device.write_motor(device.context, static_cast<RvcMotorCommand>(99)), RVC_INVALID_ARGUMENT);
    EXPECT_EQ(device.write_cleaner(device.context, static_cast<RvcCleanerCommand>(99)), RVC_INVALID_ARGUMENT);
    mock.event_count = RVC_MOCK_MAX_EVENTS;
    EXPECT_EQ(device.write_motor(device.context, RVC_MOTOR_FORWARD), RVC_IO_ERROR);
    EXPECT_EQ(device.write_cleaner(device.context, RVC_CLEANER_ON), RVC_IO_ERROR);
    EXPECT_EQ(mock.event_count, static_cast<size_t>(RVC_MOCK_MAX_EVENTS));
    EXPECT_FALSE(mock.motor_valid);
    EXPECT_FALSE(mock.cleaner_valid);
}
} // namespace
