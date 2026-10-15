#pragma once
#include <eir_loco_runtime/runtime.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace eir::loco::ros2 {
struct PublisherOptions {
    bool joint_states{true};
    bool imu{true};
    bool diagnostics{true}; // Set the other two false for alarm-only publishing.
    std::string joint_topic{"joint_states"};
    std::string imu_topic{"imu"};
    std::string diagnostic_topic{"diagnostics"};
    std::string imu_frame{"imu_link"};
};

// Read-only adapter. The application calls publish from a monitoring timer,
// never from the SDK feedback callback or the control loop.
class StatePublisher {
public:
    StatePublisher(rclcpp::Node::SharedPtr node, Config config, PublisherOptions options = {});
    void publish(const State& state, const Status& status);
private:
    rclcpp::Node::SharedPtr node_;
    Config config_;
    PublisherOptions options_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};
} // namespace eir::loco::ros2
