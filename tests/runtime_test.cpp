#include <eir_loco_runtime/runtime.hpp>
#include <gtest/gtest.h>
#include <atomic>
#include <cstdlib>
#include <condition_variable>
#include <limits>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace eir::loco;
using namespace std::chrono_literals;
namespace {
Config hardwareConfig() {
    Config c;
    c.backend = BackendType::Hardware;
    c.joint_names = {"a", "encoder_joint"}; c.motor_names = {"b", "a"};
    c.motor_slots = {3, 1}; c.encoder_names = {"encoder_joint"};
    c.feedback_timeout = 1000ms; c.command_timeout = 1000ms; c.startup_timeout = 50ms;
    return c;
}
RobotState feedback(std::size_t count = 32) {
    RobotState state{};
    state.timestamp=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    state.motor_states.q.resize(count); state.motor_states.dq.resize(count); state.motor_states.tau.resize(count);
    state.motor_states.status_code.assign(count, 1); state.motor_states.error_code.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        state.motor_states.q[i] = i; state.motor_states.dq[i] = i + .1; state.motor_states.tau[i] = i + .2;
    }
    state.encoder_states.q = {7, 8}; state.encoder_states.dq = {.7, .8};
    return state;
}
class FakeBackend : public Backend {
public:
    RobotState initial{feedback()};
    bool provide_initial{true}, reject_writes{false}, throw_start{false}, reject_release{false};
    int releases{0};
    bool releaseSuspension() override { ++releases; return !reject_release; }
    std::atomic<int> stops{0}, estops{0};
    std::mutex mutex;
    StateCallback callback;
    std::vector<RobotCmd> commands;
    std::vector<EECmd> hands;
    bool reject_hands{false};
    void start(const Config&, StateCallback cb) override {
        { std::lock_guard lock(mutex); callback = std::move(cb); }
        if (throw_start) throw std::runtime_error("start failed");
        if (provide_initial) emit(initial);
    }
    void emit(const RobotState& state) {
        StateCallback cb;
        { std::lock_guard lock(mutex); cb = callback; }
        if (cb) cb(state);
    }
    void stop() noexcept override { ++stops; }
    bool write(const RobotCmd& command) override {
        std::lock_guard lock(mutex); commands.push_back(command); return !reject_writes;
    }
    bool estop() override { ++estops; return true; }
    bool writeHand(const EECmd& command) override {
        std::lock_guard lock(mutex); hands.push_back(command); return !reject_hands;
    }
    RobotCmd last() { std::lock_guard lock(mutex); return commands.back(); }
    std::size_t count() { std::lock_guard lock(mutex); return commands.size(); }
};
TEST(Runtime, HandCommandsShareDampingEstopAndCommandDeadline) {
    auto config = hardwareConfig(); config.hand_control = true;
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    Runtime runtime(config, std::move(backend));
    ASSERT_TRUE(runtime.start());
    EXPECT_EQ(fake->hands.back().mode, std::vector<int16_t>({0,0}));
    ASSERT_TRUE(runtime.setDamping(false));
    Command cmd(2); cmd.hand_enabled = true; cmd.hand_position[0] = 50;
    ASSERT_TRUE(runtime.write(cmd));
    EXPECT_EQ(fake->hands.back().q[0], 50);
    EXPECT_EQ(fake->hands.back().q[5], 0);
    EXPECT_EQ(fake->hands.back().mode, std::vector<int16_t>({0,0})); // Reference LEGACY protocol.
    ASSERT_TRUE(runtime.setDamping(true));
    EXPECT_EQ(fake->hands.back().mode, std::vector<int16_t>({0,0}));
    EXPECT_EQ(fake->hands.back().Kp[0], 0);
    ASSERT_TRUE(runtime.setDamping(false));
    fake->reject_hands = true;
    EXPECT_FALSE(runtime.write(cmd));
    EXPECT_EQ(runtime.status().mode, Mode::Estopped);
    EXPECT_EQ(fake->estops, 1);
    const auto count = fake->hands.size();
    EXPECT_FALSE(runtime.write(cmd)); EXPECT_EQ(fake->hands.size(), count);
}
#if EIR_WITH_HARDWARE && !EIR_HARDWARE_EQR
TEST(Runtime, HardwareRejectsWrongHandTypeBeforeSdkInitialization) {
    const auto file = std::filesystem::temp_directory_path() /
        ("eir-hand-type-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    struct Cleanup { std::filesystem::path file; ~Cleanup() { std::filesystem::remove(file); } } cleanup{file};
    for (const auto* type : {"EIRGRIPPER", "NO_TYPE", ""}) {
        std::ofstream(file) << "humanoid_hardware_node:\n  RobotVersion: aiq11-pro\n  Versions:\n    aiq11-pro:\n      EndEffectorMapping:\n        type: '" << type << "'\n";
        auto config = hardwareConfig(); config.hand_control = true; config.hardware_config = file;
        Runtime runtime(config); EXPECT_FALSE(runtime.start());
        EXPECT_NE(runtime.status().message.find("explicit AIQ11 PRO LEGACY"), std::string::npos);
    }
    std::ofstream(file) << "humanoid_hardware_node:\n  RobotVersion: aiq11-pro\n  Versions:\n    aiq11-pro:\n      EndEffectorMapping:\n        type: LEGACY\n        Left: {type: EIRGRIPPER}\n";
    auto config = hardwareConfig(); config.hand_control = true; config.hardware_config = file;
    Runtime runtime(config); EXPECT_FALSE(runtime.start());
    EXPECT_NE(runtime.status().message.find("Conflicting"), std::string::npos);
    // No valid hardware type is initialized by this test; no robot connection.
}
#endif
TEST(Runtime, RejectsInvalidOrUnconfiguredHandCommands) {
    for (int kind = 0; kind < 6; ++kind) {
        auto config = hardwareConfig(); config.hand_control = kind != 0;
        auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
        Runtime runtime(config, std::move(backend)); ASSERT_TRUE(runtime.start());
        ASSERT_TRUE(runtime.setDamping(false));
        Command cmd(2); cmd.hand_enabled = true;
        if (kind == 1) cmd.hand_position[0] = 101;
        if (kind == 2) cmd.hand_position[5] = 1;
        if (kind == 3) cmd.hand_position[0] = std::numeric_limits<double>::quiet_NaN();
        if (kind == 4) cmd.hand_kp = -1;
        if (kind == 5) cmd.hand_kd = std::numeric_limits<double>::infinity();
        const auto count = fake->hands.size();
        EXPECT_FALSE(runtime.write(cmd)); EXPECT_EQ(fake->hands.size(), count);
        EXPECT_EQ(runtime.status().mode, Mode::Estopped);
    }
}
bool waitForStop(Runtime& runtime) {
    for (int i = 0; i < 200; ++i) {
        if (runtime.status().mode == Mode::Estopped) return true;
        std::this_thread::sleep_for(2ms);
    }
    return false;
}

class BlockedBackend : public FakeBackend {
public:
    std::mutex gate_mutex;
    std::condition_variable gate;
    bool block{false}, block_hand{false}, entered{false}, released{false};
    std::atomic<bool> inhibited{false};
    bool write(const RobotCmd& command) override {
        {
            std::unique_lock lock(gate_mutex);
            if (block && !block_hand) {
                entered = true; gate.notify_all();
                gate.wait(lock, [&] { return released; });
            }
        }
        // Simulate a late success return; inhibition prevents applying it.
        if (!inhibited) FakeBackend::write(command);
        return true;
    }
    bool writeHand(const EECmd& command) override {
        {
            std::unique_lock lock(gate_mutex);
            if (block && block_hand) {
                entered = true; gate.notify_all();
                gate.wait(lock, [&] { return released; });
            }
        }
        if (!inhibited) FakeBackend::writeHand(command);
        return true;
    }
    bool estop() override { inhibited = true; return FakeBackend::estop(); }
    bool waitForEntry() {
        std::unique_lock lock(gate_mutex);
        return gate.wait_for(lock, 1s, [&] { return entered; });
    }
    void release() { std::lock_guard lock(gate_mutex); released = true; gate.notify_all(); }
};
TEST(Runtime, WatchdogInhibitsBlockedHandWriteAndRejectsLateSuccess) {
    auto config = hardwareConfig(); config.hand_control = true; config.command_timeout = 30ms;
    auto backend = std::make_unique<BlockedBackend>(); auto* fake = backend.get();
    Runtime runtime(config, std::move(backend)); ASSERT_TRUE(runtime.start());
    ASSERT_TRUE(runtime.setDamping(false));
    fake->block = fake->block_hand = true;
    const auto count = fake->hands.size();
    Command cmd(2); cmd.hand_enabled = true; cmd.hand_position[0] = 50;
    bool result = true;
    std::thread writer([&] { result = runtime.write(cmd); });
    EXPECT_TRUE(fake->waitForEntry()); EXPECT_TRUE(waitForStop(runtime));
    for (int i = 0; i < 200 && !fake->inhibited; ++i) std::this_thread::sleep_for(2ms);
    EXPECT_TRUE(fake->inhibited);
    fake->release(); writer.join();
    EXPECT_FALSE(result); EXPECT_EQ(fake->hands.size(), count);
    EXPECT_EQ(runtime.status().mode, Mode::Estopped); EXPECT_EQ(fake->estops, 1);
}

TEST(Runtime, WatchdogInhibitsBlockedWritesAndRejectsLateSuccess) {
    for (bool damping : {false, true}) {
        auto config = hardwareConfig(); config.command_timeout = 30ms;
        auto backend = std::make_unique<BlockedBackend>(); auto* fake = backend.get();
        Runtime runtime(config, std::move(backend));
        ASSERT_TRUE(runtime.start());
        if (!damping) ASSERT_TRUE(runtime.setDamping(false));
        fake->block = true;
        const auto count = fake->count();
        bool result = true;
        std::thread writer([&] { result = runtime.write(Command(2)); });
        EXPECT_TRUE(fake->waitForEntry());
        EXPECT_TRUE(waitForStop(runtime));
        for (int i = 0; i < 200 && !fake->inhibited; ++i) std::this_thread::sleep_for(2ms);
        EXPECT_TRUE(fake->inhibited); // Must happen before releasing the blocked write.
        EXPECT_EQ(fake->estops, 1);
        fake->release(); writer.join();
        EXPECT_FALSE(result);
        EXPECT_EQ(fake->count(), count);
        EXPECT_EQ(runtime.status().mode, Mode::Estopped);
        EXPECT_FALSE(runtime.setDamping(false));
        EXPECT_FALSE(runtime.write(Command(2)));
        runtime.stop();
        EXPECT_EQ(fake->estops, 1);
    }
}

TEST(Runtime, WatchdogAlsoProtectsInitialDampingWrite) {
    auto config = hardwareConfig(); config.command_timeout = 30ms;
    auto backend = std::make_unique<BlockedBackend>(); auto* fake = backend.get();
    fake->block = true;
    Runtime runtime(config, std::move(backend));
    bool started = true;
    std::thread starter([&] { started = runtime.start(); });
    EXPECT_TRUE(fake->waitForEntry());
    EXPECT_TRUE(waitForStop(runtime));
    for (int i = 0; i < 200 && !fake->inhibited; ++i) std::this_thread::sleep_for(2ms);
    EXPECT_TRUE(fake->inhibited);
    fake->release(); starter.join();
    EXPECT_FALSE(started);
    EXPECT_EQ(fake->estops, 1);
    EXPECT_EQ(fake->count(), 0U);
}

TEST(Runtime, ExplicitEstopDoesNotWaitForBlockedWrite) {
    auto backend = std::make_unique<BlockedBackend>(); auto* fake = backend.get();
    Runtime runtime(hardwareConfig(), std::move(backend));
    ASSERT_TRUE(runtime.start()); ASSERT_TRUE(runtime.setDamping(false));
    fake->block = true;
    bool written = true;
    std::thread writer([&] { written = runtime.write(Command(2)); });
    EXPECT_TRUE(fake->waitForEntry());
    std::thread stopper([&] { runtime.estop(); });
    for (int i = 0; i < 200 && !fake->inhibited; ++i) std::this_thread::sleep_for(2ms);
    EXPECT_TRUE(fake->inhibited);
    fake->release(); writer.join(); stopper.join();
    EXPECT_FALSE(written);
    EXPECT_EQ(runtime.status().mode, Mode::Estopped);
}

TEST(Runtime, MapsHardwareFeedbackEncodersAndCommands) {
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    Runtime runtime(hardwareConfig(), std::move(backend));
    ASSERT_TRUE(runtime.start());
    EXPECT_EQ(runtime.status().mode, Mode::Damping);
    EXPECT_DOUBLE_EQ(fake->last().motor_cmds.Kp[3], 0);
    EXPECT_DOUBLE_EQ(fake->last().motor_cmds.Kd[3], 5);
    State state;
    ASSERT_TRUE(runtime.read(state));
    EXPECT_DOUBLE_EQ(state.motors[0].position, 3);
    EXPECT_DOUBLE_EQ(state.motors[1].position, 1);
    EXPECT_DOUBLE_EQ(state.joints[0].position, 1);
    EXPECT_DOUBLE_EQ(state.joints[1].position, 7);
    EXPECT_DOUBLE_EQ(state.joints[1].velocity, .7);
    ASSERT_TRUE(runtime.setDamping(false));
    Command command(2); command.position = {10, 20}; command.kp = {2, 3}; command.kd = {1, 1};
    ASSERT_TRUE(runtime.write(command));
    const auto sent = fake->last();
    EXPECT_DOUBLE_EQ(sent.motor_cmds.q[3], 10);
    EXPECT_DOUBLE_EQ(sent.motor_cmds.q[1], 20);
    EXPECT_EQ(sent.motor_cmds.names[3], "b");
    EXPECT_EQ(sent.motor_cmds.control_mode[0], 0);
    EXPECT_DOUBLE_EQ(sent.motor_cmds.Kd[0], 0);
    EXPECT_EQ(sent.encoder_cmds.zero_cmd, (std::vector<std::uint8_t>{0, 0}));
    EXPECT_GT(sent.timestamp, 0);
    runtime.stop();
    const auto count = fake->count();
    EXPECT_FALSE(runtime.write(command)); EXPECT_EQ(fake->count(), count);
    EXPECT_FALSE(runtime.read(state));
}
TEST(Runtime, PerMotorDampingUsesMappedSlotsOnStartupWriteAndReentry) {
    auto config = hardwareConfig();
    config.motor_damping_kd = {.5, 5}; // Control order b,a maps to SDK slots 3,1.
    auto backend = std::make_unique<FakeBackend>(); auto *fake = backend.get();
    Runtime runtime(config, std::move(backend));
    auto check = [&] {
        const auto m = fake->last().motor_cmds;
        EXPECT_DOUBLE_EQ(m.Kd[3], .5);
        EXPECT_DOUBLE_EQ(m.Kd[1], 5);
        EXPECT_DOUBLE_EQ(m.Kd[0], 0); // Unmapped slots stay disabled.
        for (const auto slot : {3, 1}) {
            EXPECT_DOUBLE_EQ(m.Kp[slot], 0);
            EXPECT_DOUBLE_EQ(m.dq[slot], 0);
            EXPECT_DOUBLE_EQ(m.tau[slot], 0);
        }
    };
    ASSERT_TRUE(runtime.start());
    check();
    Command command(2); command.kd = {2, 3}; command.kp = {10, 20};
    ASSERT_TRUE(runtime.write(command)); // Damping ignores ordinary command gains.
    check();
    ASSERT_TRUE(runtime.setDamping(false));
    ASSERT_TRUE(runtime.write(command));
    EXPECT_DOUBLE_EQ(fake->last().motor_cmds.Kd[3], 2);
    EXPECT_DOUBLE_EQ(fake->last().motor_cmds.Kd[1], 3);
    ASSERT_TRUE(runtime.setDamping(true));
    check();
}
TEST(Runtime, RejectsInvalidPerMotorDampingBeforeBackendStartup) {
    const std::vector<std::vector<double>> invalid{{.5}, {.5, 5, 5}, {.5, 0}, {.5, -1},
        {.5, std::numeric_limits<double>::infinity()}, {.5, std::numeric_limits<double>::quiet_NaN()}};
    for (const auto &values : invalid) {
        auto config = hardwareConfig(); config.motor_damping_kd = values;
        EXPECT_THROW(Runtime(config, std::make_unique<FakeBackend>()), std::invalid_argument);
    }
}
TEST(Runtime, PassiveObservationZerosFirstCommandAndCannotEnableControl) {
    auto config = hardwareConfig(); config.passive_observation = true;
    auto backend = std::make_unique<FakeBackend>(); auto *fake = backend.get();
    Runtime runtime(config, std::move(backend));
    ASSERT_TRUE(runtime.start());
    auto check = [&] {
        const auto m = fake->last().motor_cmds;
        for (std::size_t i = 0; i < 32; ++i) {
            EXPECT_DOUBLE_EQ(m.Kp[i], 0);
            EXPECT_DOUBLE_EQ(m.Kd[i], 0);
            EXPECT_DOUBLE_EQ(m.dq[i], 0);
            EXPECT_DOUBLE_EQ(m.tau[i], 0);
        }
    };
    check();
    EXPECT_FALSE(runtime.setDamping(false));
    EXPECT_EQ(runtime.status().mode, Mode::Damping);
    Command command(2); command.kp = {100, 100}; command.kd = {5, 5}; command.torque = {10, 10};
    ASSERT_TRUE(runtime.write(command));
    check();
    EXPECT_TRUE(fake->hands.empty());
    runtime.estop();
    EXPECT_EQ(fake->estops, 1);
    EXPECT_FALSE(runtime.write(command));
    EXPECT_FALSE(runtime.setDamping(false));
}
TEST(Runtime, PassiveObservationRejectsHandControl) {
    auto c = hardwareConfig(); c.passive_observation = true; c.hand_control = true;
    EXPECT_THROW(Runtime(c, std::make_unique<FakeBackend>()), std::invalid_argument);
}

TEST(Runtime, SplitsSimulationJointAndMotorFeedback) {
    Config config; config.joint_names = {"joint"}; config.motor_names = {"motor"};
    auto backend = std::make_unique<FakeBackend>(); backend->initial = feedback(2);
    Runtime runtime(config, std::move(backend));
    ASSERT_TRUE(runtime.start());
    State state; ASSERT_TRUE(runtime.read(state));
    EXPECT_DOUBLE_EQ(state.joints[0].position, 0);
    EXPECT_DOUBLE_EQ(state.motors[0].position, 1);
}

TEST(Runtime, RejectsInvalidMappingsBeforeBackendStarts) {
    auto config = hardwareConfig(); config.motor_slots = {1, 1};
    EXPECT_THROW(Runtime runtime(config), std::invalid_argument);
    config = hardwareConfig(); config.motor_slots = {1, 32};
    EXPECT_THROW(Runtime runtime(config), std::invalid_argument);
    config = hardwareConfig(); config.joint_names = {"unknown"};
    EXPECT_THROW(Runtime runtime(config), std::invalid_argument);
    config = hardwareConfig(); config.control_mode = 2;
    EXPECT_THROW(Runtime runtime(config), std::invalid_argument);
}

TEST(Runtime, RejectsUnsafeFingerHoldConfiguration) {
    Config base; base.joint_names = {"joint"}; base.motor_names = {"motor"};
    base.finger_joint_names.resize(12); base.finger_joint_names[0] = "finger";
    base.finger_hold_position.assign(12, 0); base.finger_hold_kp = 20; base.finger_hold_kd = 1;
    base.finger_torque_limits = {1.39, 1.39};
    EXPECT_NO_THROW(Runtime runtime(base));
    auto c = base; c.finger_hold_position[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.finger_hold_position[0] = 101;
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.finger_hold_kd = -1;
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.finger_torque_limits = {1.39};
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.finger_torque_limits[0] = std::numeric_limits<double>::infinity();
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.finger_joint_names[0] = "motor";
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.finger_joint_names[1] = "finger";
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.finger_hold_position.clear();
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
    c = base; c.backend = BackendType::Hardware;
    EXPECT_THROW(Runtime runtime(c), std::invalid_argument);
}

TEST(Runtime, InvalidCommandsLatchEstopAndNeverReachBackend) {
    for (bool wrong_size : {false, true}) {
        auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
        Runtime runtime(hardwareConfig(), std::move(backend));
        ASSERT_TRUE(runtime.start()); ASSERT_TRUE(runtime.setDamping(false));
        Command bad(wrong_size ? 1 : 2);
        if (!wrong_size) bad.torque[0] = std::numeric_limits<double>::quiet_NaN();
        const auto count = fake->count();
        EXPECT_FALSE(runtime.write(bad)); EXPECT_EQ(fake->count(), count);
        EXPECT_EQ(runtime.status().mode, Mode::Estopped); EXPECT_EQ(fake->estops, 1);
        EXPECT_FALSE(runtime.setDamping(false)); EXPECT_FALSE(runtime.write(Command(2)));
    }
}

TEST(Runtime, WatchdogStopsOnFeedbackTimeoutWithoutReadOrWrite) {
    auto config = hardwareConfig(); config.feedback_timeout = 30ms;
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    Runtime runtime(config, std::move(backend)); ASSERT_TRUE(runtime.start());
    ASSERT_TRUE(waitForStop(runtime));
    EXPECT_EQ(runtime.status().message, "Feedback timeout");
    State state; EXPECT_FALSE(runtime.read(state));
    runtime.stop(); EXPECT_EQ(fake->estops, 1);
}

TEST(Runtime, WatchdogStopsOnCommandTimeoutAfterExplicitEnable) {
    auto config = hardwareConfig(); config.command_timeout = 30ms;
    Runtime runtime(config, std::make_unique<FakeBackend>());
    ASSERT_TRUE(runtime.start()); ASSERT_TRUE(runtime.setDamping(false));
    ASSERT_TRUE(waitForStop(runtime)); EXPECT_EQ(runtime.status().message, "Command timeout");
}

TEST(Runtime, MalformedAndFaultyFeedbackLatchEstop) {
    for (int kind = 0; kind < 3; ++kind) {
        auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
        Runtime runtime(hardwareConfig(), std::move(backend)); ASSERT_TRUE(runtime.start());
        auto state = feedback();
        if (kind == 0) state.motor_states.q.clear();
        if (kind == 1) state.motor_states.error_code[3] = 12;
        if (kind == 2) state.motor_states.q[1] = std::numeric_limits<double>::infinity();
        fake->emit(state);
        EXPECT_EQ(runtime.status().mode, Mode::Estopped);
        fake->emit(feedback()); // Good feedback never silently clears the latch.
        EXPECT_FALSE(runtime.setDamping(false)); EXPECT_FALSE(runtime.write(Command(2)));
    }
}

TEST(Runtime, FailedInitializationCleansUpAndCanRetry) {
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    fake->throw_start = true;
    Runtime runtime(hardwareConfig(), std::move(backend));
    EXPECT_FALSE(runtime.start()); EXPECT_GE(fake->stops, 1); EXPECT_EQ(fake->estops, 1);
    fake->throw_start = false;
    EXPECT_TRUE(runtime.start()); runtime.stop(); EXPECT_TRUE(runtime.start()); runtime.stop();
}

TEST(Runtime, MissingInitialFeedbackFailsStartup) {
    auto backend = std::make_unique<FakeBackend>(); backend->provide_initial = false;
    Runtime runtime(hardwareConfig(), std::move(backend));
    EXPECT_FALSE(runtime.start()); EXPECT_EQ(runtime.status().mode, Mode::Estopped);
}

TEST(Runtime, PublishFailureTriggersEstop) {
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    Runtime runtime(hardwareConfig(), std::move(backend)); ASSERT_TRUE(runtime.start());
    ASSERT_TRUE(runtime.setDamping(false)); fake->reject_writes = true;
    EXPECT_FALSE(runtime.write(Command(2))); EXPECT_EQ(fake->estops, 1);
}

TEST(Runtime, FeedbackSnapshotIsOwnedAndCallbacksAfterStopAreIgnored) {
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    Runtime runtime(hardwareConfig(), std::move(backend)); ASSERT_TRUE(runtime.start());
    auto raw = feedback(); fake->emit(raw); raw.motor_states.q[3] = 99;
    State state; ASSERT_TRUE(runtime.read(state)); EXPECT_DOUBLE_EQ(state.motors[0].position, 3);
    runtime.stop(); fake->emit(raw); EXPECT_FALSE(runtime.read(state));
    EXPECT_EQ(runtime.status().mode, Mode::Stopped);
}

TEST(Runtime, RepeatedEnableDoesNotRefreshCommandWatchdog) {
    auto config = hardwareConfig(); config.command_timeout = 30ms;
    Runtime runtime(config, std::make_unique<FakeBackend>());
    ASSERT_TRUE(runtime.start()); ASSERT_TRUE(runtime.setDamping(false));
    const auto deadline = std::chrono::steady_clock::now() + 300ms;
    while (runtime.status().mode != Mode::Estopped && std::chrono::steady_clock::now() < deadline) {
        runtime.setDamping(false);
        std::this_thread::sleep_for(2ms);
    }
    EXPECT_EQ(runtime.status().mode, Mode::Estopped);
    EXPECT_EQ(runtime.status().message, "Command timeout");
}

TEST(Runtime, SnapshotsAndWritesCanRunAlongsideSdkCallbacks) {
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    Runtime runtime(hardwareConfig(), std::move(backend));
    ASSERT_TRUE(runtime.start()); ASSERT_TRUE(runtime.setDamping(false));
    std::atomic<bool> done{false}, consistent{true};
    std::thread producer([&] {
        for (int i = 0; i < 500; ++i) {
            auto raw = feedback();
            raw.motor_states.q[3] = i; raw.motor_states.dq[3] = i;
            fake->emit(raw);
        }
        done = true;
    });
    State state;
    while (!done) {
        if (runtime.read(state) && state.sequence > 1 && state.motors[0].position != state.motors[0].velocity)
            consistent = false;
        runtime.write(Command(2));
    }
    producer.join();
    EXPECT_TRUE(consistent); EXPECT_EQ(runtime.status().mode, Mode::Running);
}
} // namespace

TEST(RuntimeSuspension, LifecycleIdempotenceFailureAndHardwareRejection) {
    for (bool reject : {false, true}) {
        Config config; config.joint_names = {"joint"}; config.motor_names = {"motor"};
        auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
        fake->initial = feedback(2); fake->reject_release = reject;
        Runtime runtime(config, std::move(backend));
        EXPECT_FALSE(runtime.releaseSuspension());
        ASSERT_TRUE(runtime.start());
        EXPECT_TRUE(runtime.status().suspended);
        EXPECT_EQ(runtime.releaseSuspension(), !reject);
        EXPECT_EQ(fake->releases, 1);
        if (reject) {
            EXPECT_EQ(runtime.status().mode, Mode::Estopped);
            EXPECT_TRUE(runtime.status().suspended);
        } else {
            EXPECT_FALSE(runtime.status().suspended);
            EXPECT_TRUE(runtime.releaseSuspension());
            EXPECT_EQ(fake->releases, 1);
        }
        runtime.estop();
        EXPECT_FALSE(runtime.releaseSuspension());
        runtime.stop();
        EXPECT_FALSE(runtime.releaseSuspension());
        ASSERT_TRUE(runtime.start());
        EXPECT_TRUE(runtime.status().suspended);
    }
    auto backend = std::make_unique<FakeBackend>(); auto* fake = backend.get();
    Runtime hardware(hardwareConfig(), std::move(backend));
    ASSERT_TRUE(hardware.start());
    EXPECT_FALSE(hardware.status().suspended);
    EXPECT_FALSE(hardware.releaseSuspension());
    EXPECT_EQ(fake->releases, 0);
    EXPECT_EQ(hardware.status().mode, Mode::Damping);
}

namespace {
Config eqrConfig(bool wheel=false) {
    Config c; c.backend=BackendType::Hardware;c.hardware_family=HardwareFamily::Eqr;
    c.hardware_robot=wheel?"eqr11w":"eqr11";c.hardware_motor_slots=16;c.encoder_count=0;
    const char* legs[]{"fl","fr","hl","hr"};
    for(int i=0;i<16;++i){if(!wheel && i%4==3)continue;
        c.motor_slots.push_back(i);c.joint_names.push_back(std::string(legs[i/4])+(i%4==3?"_wheel":"_leg_"+std::to_string(i%4+1)));}
    return c;
}
TEST(RuntimeEqr,MappingAndInactiveSlots) {
    for(bool wheel:{false,true}){
        auto c=eqrConfig(wheel);auto backend=std::make_unique<FakeBackend>();auto* fake=backend.get();fake->initial=feedback(16);
        Runtime runtime(c,std::move(backend));ASSERT_TRUE(runtime.start());ASSERT_TRUE(runtime.setDamping(false));
        Command cmd(c.joint_names.size());for(std::size_t i=0;i<cmd.position.size();++i){cmd.position[i]=i*.01;cmd.kp[i]=20;cmd.kd[i]=2;}
        ASSERT_TRUE(runtime.write(cmd));const auto wire=fake->last();EXPECT_EQ(wire.motor_cmds.q.size(),16u);EXPECT_TRUE(wire.encoder_cmds.zero_cmd.empty());
        for(std::size_t i=0;i<c.motor_slots.size();++i)EXPECT_EQ(wire.motor_cmds.q[c.motor_slots[i]],cmd.position[i]);
        if(!wheel)for(int i=3;i<16;i+=4){EXPECT_EQ(wire.motor_cmds.control_mode[i],0);EXPECT_EQ(wire.motor_cmds.Kd[i],0);EXPECT_EQ(wire.motor_cmds.Kp[i],0);EXPECT_EQ(wire.motor_cmds.q[i],0);}
        auto bad=feedback(16);bad.motor_states.error_code[0]=1;fake->emit(bad);EXPECT_EQ(runtime.status().mode,Mode::Estopped);EXPECT_FALSE(runtime.write(cmd));
    }
}
TEST(RuntimeEqr,RejectsVariantSlotsAndUnsupportedInterfaces) {
    for(int kind=0;kind<7;++kind){auto c=eqrConfig();
        switch(kind){case 0:c.hardware_motor_slots=32;break;case 1:c.encoder_count=2;break;case 2:c.hardware_robot="eqr11w";break;case 3:c.motor_slots[2]=3;break;case 4:c.hand_control=true;break;case 5:std::swap(c.motor_slots[0],c.motor_slots[1]);break;case 6:c.motor_slots.clear();break;}
        EXPECT_THROW(Runtime(c,std::make_unique<FakeBackend>()),std::invalid_argument);
    }
}
#if EIR_WITH_HARDWARE
TEST(RuntimeEqr,RealBackendRejectsWrongFamilyBeforeOpeningTransport) {
#if EIR_HARDWARE_EQR
    Runtime runtime(hardwareConfig());
#else
    Runtime runtime(eqrConfig());
#endif
    EXPECT_FALSE(runtime.start());EXPECT_NE(runtime.status().message.find("built for"),std::string::npos);
}
#if EIR_HARDWARE_EQR
TEST(RuntimeEqr,RealBackendRejectsInvalidProfileBeforeOpeningTransport) {
    auto c=eqrConfig();c.hardware_config="/missing-eqr-profile.yaml";Runtime runtime(c);
    EXPECT_FALSE(runtime.start());
    const auto expected=std::getenv("EIR_TEST_EQR_ABI_MISMATCH") ? "ABI" : "profile";
    EXPECT_NE(runtime.status().message.find(expected),std::string::npos);
}
#endif
#endif
}

TEST(RuntimeEqr,SourceAgeIsNotRefreshedByCallback) {
    auto c=eqrConfig();auto backend=std::make_unique<FakeBackend>();auto* fake=backend.get();fake->initial=feedback(16);
    Runtime runtime(c,std::move(backend));ASSERT_TRUE(runtime.start());
    fake->emit(fake->initial);EXPECT_EQ(runtime.status().mode,Mode::Estopped);
    auto stale=std::make_unique<FakeBackend>();stale->initial=feedback(16);stale->initial.timestamp-=200000000;
    Runtime expired(c,std::move(stale));EXPECT_FALSE(expired.start());
}

TEST(RuntimeEqr,RejectsWrongSdkFeedbackDimensions) {
    Runtime runtime(eqrConfig(),std::make_unique<FakeBackend>()); // AIQ 32-slot feedback.
    EXPECT_FALSE(runtime.start());EXPECT_NE(runtime.status().message.find("exactly 16"),std::string::npos);
}

TEST(Runtime, FlightRecorderExemptsHeadOfflineWhileBodyFaultStillLocksAndSaves) {
    auto c = hardwareConfig();
    c.robot_identity = "pro";
    c.flight_recorder.enabled = true;
    c.flight_recorder.pre_seconds = 1;
    c.flight_recorder.post_seconds = 0;
    c.flight_recorder.min_free_bytes = 0;
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("eir-runtime-record-" + std::to_string(flight::Recorder::monotonicNow()));
    struct Cleanup {
        std::filesystem::path p;
        ~Cleanup() { std::filesystem::remove_all(p); }
    } cleanup{directory};
    c.flight_recorder.directory = directory;
    c.flight_recorder.data |= flight::Head;
    c.flight_recorder.trigger_exemptions = {
        {flight::Event::HardwareOffline, flight::Component::HeadServo, {}}};
    auto backend = std::make_unique<FakeBackend>();
    auto *fake = backend.get();
    fake->initial.head_state.servo_online = {0, 1};
    Runtime r(c, std::move(backend));
    ASSERT_TRUE(r.start());
    for (int i = 0; i < 100 && !r.flightStatistics().exempted; ++i)
        std::this_thread::sleep_for(2ms);
    EXPECT_EQ(r.flightStatistics().exempted, 1u);
    EXPECT_EQ(r.flightStatistics().completed, 0u);
    EXPECT_EQ(r.status().mode, Mode::Damping);
    auto raw = feedback();
    raw.head_state.servo_online = {0, 1};
    raw.motor_states.error_code[3] = 7;
    fake->emit(raw);
    EXPECT_EQ(r.status().mode, Mode::Estopped);
    r.estop();
    for (int i = 0; i < 100 && !r.flightStatistics().completed; ++i)
        std::this_thread::sleep_for(2ms);
    EXPECT_EQ(r.flightStatistics().completed, 1u);
    r.stop();
    EXPECT_EQ(r.flightStatistics().completed, 1u);
}
TEST(Runtime, FlightRecorderOutputFailureDoesNotPreventStartAndRestart) {
    auto c = hardwareConfig();
    c.robot_identity = "pro";
    c.flight_recorder.enabled = true;
    c.flight_recorder.directory = "/dev/null/flight";
    Runtime r(c, std::make_unique<FakeBackend>());
    ASSERT_TRUE(r.start());
    for (int i = 0; i < 100 && !r.flightStatistics().write_failures; ++i)
        std::this_thread::sleep_for(2ms);
    EXPECT_GT(r.flightStatistics().write_failures, 0u);
    EXPECT_EQ(r.status().mode, Mode::Damping);
    r.stop();
    ASSERT_TRUE(r.start());
    r.stop();
}

TEST(Runtime, ProtectiveStopInhibitsAndRecordsApplicationFaultInsteadOfOperatorEstop) {
    for(bool protective:{true,false}) {
        auto c=hardwareConfig();c.robot_identity="pro";
        c.flight_recorder.enabled=true;c.flight_recorder.post_seconds=0;
        const auto directory=std::filesystem::temp_directory_path()/
            ("eir-protective-stop-"+std::to_string(flight::Recorder::monotonicNow())+(protective?"p":"e"));
        c.flight_recorder.directory=directory.string();
        auto b=std::make_unique<FakeBackend>();auto *backend=b.get();Runtime r(c,std::move(b));
        ASSERT_TRUE(r.start());
        if(protective)r.protectiveStop();else r.estop();
        EXPECT_EQ(r.status().mode,Mode::Estopped);EXPECT_EQ(backend->estops,1);
        EXPECT_FALSE(r.write(Command(2)));
        for(int i=0;i<100 && !r.flightStatistics().completed;++i)std::this_thread::sleep_for(2ms);
        r.stop();EXPECT_EQ(r.flightStatistics().completed,1u);
        bool found=false;
        for(const auto &run:std::filesystem::directory_iterator(directory)) if(run.is_directory())
            for(const auto &file:std::filesystem::directory_iterator(run.path()))
                if(file.path().filename().string().rfind("incident_",0)==0 && file.path().extension()==".json") {
                    std::ifstream stream(file.path());std::string text((std::istreambuf_iterator<char>(stream)),{});
                    EXPECT_NE(text.find(protective?"application_fault":"estop"),std::string::npos);found=true;
                }
        EXPECT_TRUE(found);std::filesystem::remove_all(directory);
    }
}

TEST(Runtime, EventOnlyOperationsSurviveDisabledApplicationDiagnostics) {
    auto c = hardwareConfig(); c.robot_identity = "pro";
    c.flight_recorder.enabled = true; c.flight_recorder.data &= ~flight::Application;
    c.flight_recorder.min_free_bytes = 0;
    const auto directory = std::filesystem::temp_directory_path() /
        ("eir-fr01-disabled-app-" + std::to_string(flight::Recorder::monotonicNow()));
    c.flight_recorder.directory = directory.string();
    Runtime runtime(c, std::make_unique<FakeBackend>());
    for (unsigned i=0;i<500 && !runtime.flightStatistics().healthy;++i) std::this_thread::sleep_for(2ms);
    ASSERT_TRUE(runtime.flightStatistics().healthy);
    const auto before = runtime.flightStatistics().accepted;
    flight::Packet request; request.flags = flight::Packet::EventOnly;
    request.correlation = 991;
    runtime.recordApplication(request);
    runtime.recordApplication(flight::Packet{}); // Ordinary diagnostics remain disabled.
    runtime.stop();
    EXPECT_EQ(runtime.flightStatistics().accepted, before+1);
    EXPECT_EQ(runtime.flightStatistics().dropped, 0u);
    bool found=false;
    for (const auto &e:std::filesystem::recursive_directory_iterator(directory))
        if(e.path().filename()=="events.bin") {
            found=true;EXPECT_EQ(std::filesystem::file_size(e.path()),124u);
        }
    EXPECT_TRUE(found);
    std::filesystem::remove_all(directory);
}
