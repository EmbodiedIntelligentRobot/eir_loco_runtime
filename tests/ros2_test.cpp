#include <eir_loco_runtime_ros2/state_publisher.hpp>
#include <gtest/gtest.h>
#include <thread>
using namespace eir::loco;
using namespace std::chrono_literals;

class RosPublisherTest : public ::testing::Test {
protected:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(RosPublisherTest, PublishesStateImuAndAlarmsWithoutControlInputs) {
    auto node = std::make_shared<rclcpp::Node>("runtime_publish_test");
    auto observer = std::make_shared<rclcpp::Node>("runtime_observer_test");
    Config config; config.joint_names = {"joint"}; config.motor_names = {"motor"};
    ros2::StatePublisher publisher(node, config);
    EXPECT_TRUE(node->get_subscriptions_info_by_topic("joint_states").empty());
    EXPECT_TRUE(node->get_subscriptions_info_by_topic("imu").empty());
    EXPECT_TRUE(node->get_subscriptions_info_by_topic("diagnostics").empty());
    sensor_msgs::msg::JointState::SharedPtr joints;
    sensor_msgs::msg::Imu::SharedPtr imu;
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic;
    auto sub1 = observer->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::SharedPtr m) { joints = m; });
    auto sub2 = observer->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::Imu::SharedPtr m) { imu = m; });
    auto sub3 = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "diagnostics", 10, [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { diagnostic = m; });
    State state; state.sequence = 7; state.joints = {{.2, .3, .4}}; state.motors.resize(1);
    state.motors[0].temperature = 42; state.sdk.alarm_states.alarm_code = {23};
    state.sdk.imu_state.resize(1); state.sdk.imu_state[0] = {};
    state.sdk.imu_state[0].quat[0] = 1; state.sdk.imu_state[0].gyro[2] = .5;
    Status status; status.mode = Mode::Running; status.feedback_received = true;
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while ((!joints || !imu || !diagnostic) && std::chrono::steady_clock::now() < deadline) {
        publisher.publish(state, status); rclcpp::spin_some(observer);
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(joints); ASSERT_TRUE(imu); ASSERT_TRUE(diagnostic);
    EXPECT_EQ(joints->name[0], "joint"); EXPECT_DOUBLE_EQ(joints->position[0], .2);
    EXPECT_DOUBLE_EQ(imu->orientation.w, 1); EXPECT_DOUBLE_EQ(imu->angular_velocity.z, .5);
    ASSERT_EQ(diagnostic->status.size(), 3U);
    EXPECT_EQ(diagnostic->status[2].name, "sdk_alarm/0");
    EXPECT_EQ(diagnostic->status[2].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(RosPublisherTest, AlarmOnlyModeReportsUnavailableFeedback) {
    auto node = std::make_shared<rclcpp::Node>("alarm_publish_test");
    auto observer = std::make_shared<rclcpp::Node>("alarm_observer_test");
    Config config; config.joint_names = {"joint"};
    ros2::PublisherOptions options; options.joint_states = false; options.imu = false;
    options.diagnostic_topic = "runtime_alerts";
    ros2::StatePublisher publisher(node, config, options);
    EXPECT_EQ(node->count_publishers("joint_states"), 0U);
    EXPECT_EQ(node->count_publishers("imu"), 0U);
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr received;
    auto sub = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "runtime_alerts", 10, [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { received = m; });
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
        publisher.publish(State{}, Status{}); rclcpp::spin_some(observer);
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(received); ASSERT_EQ(received->status.size(), 1U);
    EXPECT_EQ(received->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::STALE);
}

TEST_F(RosPublisherTest, StaleFeedbackPublishesDiagnosticsWithoutJointOrImuData) {
    auto node = std::make_shared<rclcpp::Node>("stale_publish_test");
    auto observer = std::make_shared<rclcpp::Node>("stale_observer_test");
    Config config; config.joint_names = {"joint"}; config.motor_names = {"motor"};
    ros2::StatePublisher publisher(node, config);
    int joints = 0, imus = 0;
    diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic;
    auto sub1 = observer->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::SharedPtr) { ++joints; });
    auto sub2 = observer->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::Imu::SharedPtr) { ++imus; });
    auto sub3 = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "diagnostics", 10, [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { diagnostic = m; });
    State state; state.joints.resize(1); state.motors.resize(1); state.sdk.imu_state.resize(1);
    state.sdk.imu_state[0] = {}; state.sdk.imu_state[0].quat[0] = 1;
    Status status; status.mode = Mode::Running; status.feedback_received = true;
    status.feedback_age = config.feedback_timeout + 1ms;
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!diagnostic && std::chrono::steady_clock::now() < deadline) {
        publisher.publish(state, status); rclcpp::spin_some(observer);
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(diagnostic); ASSERT_EQ(diagnostic->status.size(), 1U);
    EXPECT_EQ(diagnostic->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::STALE);
    EXPECT_EQ(joints, 0); EXPECT_EQ(imus, 0);
    // A fresh frame through the same publishers must reach both subscriptions.
    status.feedback_age = 0ms;
    const auto fresh_deadline = std::chrono::steady_clock::now() + 3s;
    while ((!joints || !imus) && std::chrono::steady_clock::now() < fresh_deadline) {
        publisher.publish(state, status); rclcpp::spin_some(observer);
        std::this_thread::sleep_for(10ms);
    }
    EXPECT_GT(joints, 0); EXPECT_GT(imus, 0);
}
