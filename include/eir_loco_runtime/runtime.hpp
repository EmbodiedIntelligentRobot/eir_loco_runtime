#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <eir_loco_runtime/depth.hpp>
#include <eir_loco_runtime/flight_recorder.hpp>
#include <eir_lowlevel_sdk/EirLowLevelApi.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace eir::loco {

enum class HardwareFamily { Aiq, Eqr };
enum class BackendType { Simulation, Hardware };
enum class Mode { Stopped, Running, Damping, Estopped };

struct FingerCoupling {
    std::string curve, active_joint, passive_joint;
};

struct Config {
    DepthConfig depth;
    flight::Config flight_recorder;
    std::string robot_identity; // Required when recording: pro/sport/eqr11/eqr11w.
    std::vector<bool> continuous_joints;
    std::vector<std::string> recorder_controllers;
    std::string recorder_config_digest;
    BackendType backend{BackendType::Simulation};
    std::string model_path;        // MuJoCo MJCF; no changes to its physics options.
    std::string model_name;        // Optional expected MJCF identity, checked before starting threads.
    HardwareFamily hardware_family{HardwareFamily::Aiq}; // Default preserves AIQ32/2.
    std::string hardware_robot;    // EQR requires eqr11 or eqr11w.
    std::string hardware_config;   // SDK YAML; RobotVersion must already be selected.
    std::vector<std::string> joint_names;
    std::vector<std::string> motor_names; // Empty means joint_names.
    std::size_t hardware_motor_slots{32};
    std::vector<std::size_t> motor_slots; // Hardware: control index -> SDK slot.
    std::vector<std::string> encoder_names; // Hardware: SDK encoder order.
    std::size_t encoder_count{2};
    int control_mode{1};
    double damping_kd{5.0};
    // Optional per-motor damping in motor_names order; empty uses damping_kd for all.
    std::vector<double> motor_damping_kd;
    bool passive_observation{false}; // Explicit test mode: zero body gains/torque; cannot enable control.
    std::chrono::milliseconds startup_timeout{2000};
    std::chrono::milliseconds feedback_timeout{100};
    std::chrono::milliseconds command_timeout{100};
    std::chrono::milliseconds watchdog_period{5};
    bool headless{true};           // Simulation owns a paced physics thread.
    bool suspend{true};
    double sensor_delay_ms{0.0};
    std::vector<std::string> finger_joint_names; // Empty or 12 SDK slots.
    std::string finger_coupling_csv;
    std::vector<FingerCoupling> finger_couplings;
    // Optional simulation-only static hand posture, 12 SDK slots, 0=open/100=closed.
    // Empty retains the simulator's legacy hand behavior. No hardware hand commands.
    std::vector<double> finger_hold_position;
    double finger_hold_kp{0}, finger_hold_kd{0};
    std::vector<double> finger_torque_limits; // Two joint-side limits, Nm, one per hand.
    bool hand_control{false}; // Optional LEGACY three-finger commands, 12 SDK slots.
};

struct JointState {
    double position{0}, velocity{0}, torque{0};
};
struct MotorState : JointState {
    double current{0}, temperature{0}, mos_temperature{0};
    std::uint32_t status_code{0}, error_code{0};
    std::uint16_t loss_count{0};
};
struct State {
    std::uint64_t sequence{0};
    std::vector<JointState> joints; // Config joint order; encoder overrides on hardware.
    std::vector<MotorState> motors; // Config motor order, independent of SDK slots.
    RobotState sdk{};              // Full snapshot: IMU, hands, BMS, force, remote, alarms.
};
struct Command {
    std::vector<double> position, velocity, torque, kp, kd;
    explicit Command(std::size_t count = 0);
    bool hand_enabled{false};
    std::array<double, 12> hand_position{}; // Percent closure: 0=open, 100=closed; slots 5/11 zero.
    double hand_kp{0}, hand_kd{0}; // Simulation gains; hardware LEGACY uses onboard position control.
};
struct Status {
    Mode mode{Mode::Stopped};
    bool feedback_received{false};
    bool suspended{false}; // SDK suspension request; not a physical contact sensor.
    std::chrono::milliseconds feedback_age{0};
    std::string message;
};

// Small injection seam for tests or additional SDK backends. stop() must join
// workers and finish all callbacks. Never call Runtime methods from a callback.
class Backend {
public:
    using StateCallback = std::function<void(const RobotState&)>;
    virtual ~Backend() = default;
    virtual void start(const Config&, StateCallback) = 0;
    virtual void stop() noexcept = 0;
    virtual bool write(const RobotCmd&) = 0;
    // Same concurrent estop inhibition contract as write(). Called only when configured.
    virtual bool writeHand(const EECmd&) { return false; }
    // Concurrent with write/releaseSuspension: must promptly latch inhibition,
    // without waiting for IO locks, joining threads or destroying SDK resources.
    // Pending/new commands must not resume motion after this call. stop() owns
    // teardown. A backend unable to meet this contract must not be used.
    virtual bool estop() = 0;
    virtual bool releaseSuspension() { return false; }
    virtual bool readDepth(DepthStack&) { return false; }
    virtual bool readFinalTorque(std::array<double, 64> &, uint32_t &, uint64_t &) { return false; }
};

class Runtime {
public:
    explicit Runtime(Config config, std::unique_ptr<Backend> backend = {});
    ~Runtime();
    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;

    // Lifecycle calls must be serialized by the application. read/status may
    // run on a monitoring thread; ordinary SDK calls are serialized; estop is independent.
    bool start();                 // Waits for the first valid feedback snapshot.
    void stop() noexcept;         // Estop, stop callbacks/backend, drain and join recorder.
    bool read(State& output) const; // False if absent/stale; output remains unchanged.
    bool readDepth(DepthStack& output); // Non-RT, serialized with SDK lifecycle; caller checks age.
    bool write(const Command& command);
    bool setDamping(bool enabled); // Fault recovery requires stop/start.
    bool releaseSuspension();     // Simulation only, non-RT; no automatic reattachment.
    void estop(bool record = true); // false is reserved for normal application teardown.
    void protectiveStop(); // Same independent inhibition; event is application_fault, not operator estop.
    bool saveFlightRecord() noexcept; // Queued only; not a durability acknowledgement.
    flight::Statistics flightStatistics() const noexcept;
    void recordApplication(flight::Packet) noexcept; // Stop producers before stop(); no restart races.
    Status status() const;
    const Config& config() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace eir::loco
