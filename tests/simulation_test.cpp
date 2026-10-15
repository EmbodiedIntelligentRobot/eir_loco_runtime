#include <eir_loco_runtime/runtime.hpp>
#include <gtest/gtest.h>
#include <thread>
#include <limits>
using namespace eir::loco;
using namespace std::chrono_literals;

TEST(Simulation, RejectsWrongModelIdentityBeforeStartingPhysics) {
    Config config;
    config.model_path = TEST_MODEL;
    config.model_name = "different_robot";
    config.joint_names = {"joint"}; config.motor_names = {"motor"};
    Runtime runtime(config);
    EXPECT_FALSE(runtime.start());
    EXPECT_NE(runtime.status().message.find("model identity mismatch"), std::string::npos);
}

TEST(Simulation, HeadlessSdkStartControlFeedbackAndRestart) {
    Config config;
    config.model_path = TEST_MODEL;
    config.joint_names = {"joint"}; config.motor_names = {"motor"}; config.suspend = false;
    config.feedback_timeout = 500ms; config.command_timeout = 500ms;
    Runtime runtime(config);
    for (int run = 0; run < 2; ++run) {
        ASSERT_TRUE(runtime.start()) << runtime.status().message;
        State state; ASSERT_TRUE(runtime.read(state));
        const auto sequence = state.sequence;
        ASSERT_TRUE(runtime.setDamping(false));
        Command command(1); command.position[0] = .2; command.kp[0] = 2; command.kd[0] = .1;
        const auto end = std::chrono::steady_clock::now() + 300ms;
        while (std::chrono::steady_clock::now() < end) {
            ASSERT_TRUE(runtime.write(command)) << runtime.status().message;
            std::this_thread::sleep_for(2ms);
        }
        ASSERT_TRUE(runtime.read(state));
        EXPECT_GT(state.sequence, sequence);
        EXPECT_GT(state.joints[0].position, .01);
        EXPECT_NEAR(state.joints[0].position, state.motors[0].position, 1e-5);
        runtime.estop(); EXPECT_EQ(runtime.status().mode, Mode::Estopped);
        EXPECT_FALSE(runtime.write(command)); runtime.stop();
    }
}

TEST(Simulation, RejectsMissingActuatorBeforeStartingWorkers) {
    Config config; config.model_path = TEST_MODEL;
    config.joint_names = {"joint"}; config.motor_names = {"missing_motor"};
    Runtime runtime(config);
    EXPECT_FALSE(runtime.start());
    EXPECT_NE(runtime.status().message.find("Unknown MuJoCo actuator"), std::string::npos);
}

TEST(Simulation, RejectsMissingFingerBeforeStartingWorkers) {
    Config config; config.model_path = TEST_MODEL;
    config.joint_names = {"joint"}; config.motor_names = {"motor"};
    config.finger_joint_names.resize(12); config.finger_joint_names[0] = "missing_finger";
    Runtime runtime(config);
    EXPECT_FALSE(runtime.start());
    EXPECT_NE(runtime.status().message.find("Unknown MuJoCo finger joint"), std::string::npos);
}

TEST(Simulation, StartupDampingIgnoresMotionCommandsAndNeedsNoCommandHeartbeat) {
    Config config; config.model_path = TEST_MODEL;
    config.joint_names = {"joint"}; config.motor_names = {"motor"}; config.suspend = false;
    config.feedback_timeout = 500ms; config.command_timeout = 50ms;
    Runtime runtime(config);
    ASSERT_TRUE(runtime.start()) << runtime.status().message;
    Command command(1); command.position[0] = .5; command.kp[0] = 10;
    ASSERT_TRUE(runtime.write(command));
    // Several command deadlines pass while the damping watchdog maintains feedback.
    std::this_thread::sleep_for(200ms);
    State state; ASSERT_TRUE(runtime.read(state));
    EXPECT_EQ(runtime.status().mode, Mode::Damping);
    EXPECT_NEAR(state.joints[0].position, 0, 1e-6);
    EXPECT_NEAR(state.joints[0].velocity, 0, 1e-6);
    EXPECT_GT(state.sequence, 1U);
}

TEST(Simulation, CommandTimeoutLatchesEstopAndExplicitRestartRecovers) {
    Config config; config.model_path = TEST_MODEL;
    config.joint_names = {"joint"}; config.motor_names = {"motor"}; config.suspend = false;
    config.feedback_timeout = 1000ms; config.command_timeout = 100ms;
    Runtime runtime(config);
    ASSERT_TRUE(runtime.start()) << runtime.status().message;
    ASSERT_TRUE(runtime.setDamping(false));
    Command command(1); command.position[0] = .2; command.kp[0] = 2; command.kd[0] = .1;
    ASSERT_TRUE(runtime.write(command));
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (runtime.status().mode != Mode::Estopped && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(5ms);
    ASSERT_EQ(runtime.status().mode, Mode::Estopped);
    EXPECT_EQ(runtime.status().message, "Command timeout");
    EXPECT_FALSE(runtime.write(command));
    EXPECT_FALSE(runtime.setDamping(false));
    runtime.stop();
    ASSERT_TRUE(runtime.start()) << runtime.status().message;
    EXPECT_EQ(runtime.status().mode, Mode::Damping);
    State state; ASSERT_TRUE(runtime.read(state));
    EXPECT_NEAR(state.joints[0].position, 0, 1e-6);
}

TEST(Simulation, SuspensionReleaseAllowsGravityAndRestartRestoresSuspension) {
    Config config; config.model_path = SUSPENDED_MODEL;
    config.joint_names = {"joint"}; config.motor_names = {"motor"};
    config.feedback_timeout = 500ms;
    Runtime runtime(config);
    for (int run = 0; run < 2; ++run) {
        ASSERT_TRUE(runtime.start()) << runtime.status().message;
        std::this_thread::sleep_for(300ms);
        State held, released;
        ASSERT_TRUE(runtime.read(held));
        EXPECT_TRUE(runtime.status().suspended);
        EXPECT_LT(std::abs(held.joints[0].velocity), .05);
        ASSERT_TRUE(runtime.releaseSuspension());
        std::this_thread::sleep_for(300ms);
        ASSERT_TRUE(runtime.read(released));
        EXPECT_FALSE(runtime.status().suspended);
        EXPECT_LT(released.joints[0].position, held.joints[0].position - .1);
        EXPECT_LT(released.joints[0].velocity, -.5);
        runtime.stop();
    }
}

TEST(Simulation, DepthAcquisitionTimeAdvancesAndStopsWithBackend) {
    Config c; c.model_path = TEST_MODEL; c.joint_names = {"joint"}; c.motor_names = {"motor"};
    c.suspend = false; c.depth.enabled = true; c.depth.site = "depth_camera";
    Runtime runtime(c);
    ASSERT_TRUE(runtime.start()) << runtime.status().message;
    DepthStack first, next;
    for (int i=0; i<50 && !runtime.readDepth(first); ++i) std::this_thread::sleep_for(2ms);
    ASSERT_TRUE(first.valid); ASSERT_GT(first.stamp, 0U);
    std::this_thread::sleep_for(60ms);
    ASSERT_TRUE(runtime.readDepth(next));
    EXPECT_GT(next.sequence, first.sequence); EXPECT_GT(next.stamp, first.stamp);
    runtime.estop();
    std::this_thread::sleep_for(20ms);
    ASSERT_TRUE(runtime.readDepth(first));
    std::this_thread::sleep_for(30ms);
    ASSERT_TRUE(runtime.readDepth(next));
    EXPECT_EQ(next.sequence, first.sequence); EXPECT_EQ(next.stamp, first.stamp);
    runtime.stop(); EXPECT_FALSE(runtime.readDepth(next)); EXPECT_FALSE(next.valid);
}
TEST(Simulation, DepthSiteTypoFailsBeforeStarting) {
    Config c; c.model_path = TEST_MODEL; c.joint_names = {"joint"}; c.motor_names = {"motor"};
    c.depth.enabled = true; c.depth.site = "typo";
    Runtime runtime(c); EXPECT_FALSE(runtime.start());
    EXPECT_NE(runtime.status().message.find("Unknown depth camera site"), std::string::npos);
}

TEST(Simulation, RejectsInvalidSensorRangeBeforeStarting) {
    Config c; c.model_path = TEST_MODEL; c.joint_names = {"joint"}; c.motor_names = {"motor"};
    c.depth.enabled = true; c.depth.site = "depth_camera";
    for (double invalid : {-1., 2.5, std::numeric_limits<double>::quiet_NaN()}) {
        c.depth.sensor_min_distance = invalid;
        EXPECT_THROW(Runtime{c}, std::invalid_argument);
    }
    c.depth.sensor_min_distance = .25;
    c.depth.sensor_max_distance = std::numeric_limits<double>::infinity();
    EXPECT_THROW(Runtime{c}, std::invalid_argument);
}
