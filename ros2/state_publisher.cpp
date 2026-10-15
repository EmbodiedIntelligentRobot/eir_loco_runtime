#include <eir_loco_runtime_ros2/state_publisher.hpp>
#include <cmath>
#include <stdexcept>

namespace eir::loco::ros2 {
StatePublisher::StatePublisher(rclcpp::Node::SharedPtr node, Config config, PublisherOptions options)
    : node_(std::move(node)), config_(std::move(config)), options_(std::move(options)) {
    if (!node_) throw std::invalid_argument("ROS 2 publisher requires a node");
    if (config_.motor_names.empty()) config_.motor_names = config_.joint_names;
    if (options_.joint_states)
        joints_ = node_->create_publisher<sensor_msgs::msg::JointState>(options_.joint_topic, rclcpp::SensorDataQoS());
    if (options_.imu)
        imu_ = node_->create_publisher<sensor_msgs::msg::Imu>(options_.imu_topic, rclcpp::SensorDataQoS());
    if (options_.diagnostics)
        diagnostics_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(options_.diagnostic_topic, 10);
}

void StatePublisher::publish(const State& state, const Status& status) {
    const auto stamp = node_->now(); // SDK steady-clock timestamps are not ROS timestamps.
    const bool dimensions_ok = state.joints.size() == config_.joint_names.size() &&
                               state.motors.size() == config_.motor_names.size();
    const bool fresh = status.feedback_received && status.feedback_age <= config_.feedback_timeout && dimensions_ok;
    if (joints_ && fresh) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp; msg.name = config_.joint_names;
        for (const auto& joint : state.joints) {
            msg.position.push_back(joint.position);
            msg.velocity.push_back(joint.velocity);
            msg.effort.push_back(joint.torque);
        }
        joints_->publish(msg);
    }
    if (imu_ && fresh && !state.sdk.imu_state.empty()) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp; msg.header.frame_id = options_.imu_frame;
        const auto& source = state.sdk.imu_state.front();
        const double norm = std::sqrt(source.quat[0]*source.quat[0] + source.quat[1]*source.quat[1] +
                                      source.quat[2]*source.quat[2] + source.quat[3]*source.quat[3]);
        if (std::isfinite(norm) && norm > 1e-12) {
            msg.orientation.w = source.quat[0] / norm; msg.orientation.x = source.quat[1] / norm;
            msg.orientation.y = source.quat[2] / norm; msg.orientation.z = source.quat[3] / norm;
        } else msg.orientation_covariance[0] = -1;
        if (std::isfinite(source.gyro[0]) && std::isfinite(source.gyro[1]) && std::isfinite(source.gyro[2])) {
            msg.angular_velocity.x = source.gyro[0]; msg.angular_velocity.y = source.gyro[1];
            msg.angular_velocity.z = source.gyro[2];
        } else msg.angular_velocity_covariance[0] = -1;
        if (std::isfinite(source.acc[0]) && std::isfinite(source.acc[1]) && std::isfinite(source.acc[2])) {
            msg.linear_acceleration.x = source.acc[0]; msg.linear_acceleration.y = source.acc[1];
            msg.linear_acceleration.z = source.acc[2];
        } else msg.linear_acceleration_covariance[0] = -1;
        imu_->publish(msg);
    }
    if (!diagnostics_) return;
    using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = stamp;
    auto value = [](Diagnostic& d, const std::string& key, auto v) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key; kv.value = std::to_string(v); d.values.push_back(std::move(kv));
    };
    Diagnostic runtime;
    runtime.name = "eir_loco_runtime";
    runtime.hardware_id = config_.backend == BackendType::Hardware ? "hardware" : "simulation";
    runtime.level = status.mode == Mode::Estopped ? Diagnostic::ERROR : !fresh ? Diagnostic::STALE :
                    status.mode == Mode::Running ? Diagnostic::OK : Diagnostic::WARN;
    runtime.message = !fresh && status.mode != Mode::Estopped ? "Feedback unavailable or stale" : status.message;
    value(runtime, "feedback_age_ms", status.feedback_age.count());
    value(runtime, "source_timestamp_ns", state.sdk.timestamp);
    value(runtime, "sequence", state.sequence);
    msg.status.push_back(runtime);
    if (fresh) {
        for (std::size_t i = 0; i < state.motors.size(); ++i) {
            const auto& motor = state.motors[i];
            Diagnostic d;
            d.name = "motor/" + config_.motor_names[i]; d.hardware_id = runtime.hardware_id;
            const bool fault = motor.error_code || (config_.backend == BackendType::Hardware &&
                                                    (motor.status_code == 0 || motor.status_code > 3));
            d.level = fault ? Diagnostic::ERROR : Diagnostic::OK;
            d.message = fault ? "Motor fault" : "Motor feedback";
            value(d, "status_code", motor.status_code); value(d, "error_code", motor.error_code);
            value(d, "loss_count", motor.loss_count); value(d, "current_a", motor.current);
            value(d, "temperature_c", motor.temperature); value(d, "mos_temperature_c", motor.mos_temperature);
            msg.status.push_back(std::move(d));
        }
        for (std::size_t i = 0; i < state.sdk.alarm_states.alarm_code.size(); ++i) {
            const auto code = state.sdk.alarm_states.alarm_code[i];
            if (!code) continue;
            Diagnostic d;
            d.name = "sdk_alarm/" + std::to_string(i); d.hardware_id = runtime.hardware_id;
            d.level = Diagnostic::ERROR; d.message = "SDK alarm"; value(d, "code", code);
            msg.status.push_back(std::move(d));
        }
    }
    diagnostics_->publish(msg);
}
} // namespace eir::loco::ros2
