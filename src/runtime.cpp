#include <eir_loco_runtime/runtime.hpp>
#include "backends.hpp"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <mutex>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_set>

namespace eir::loco {
namespace {
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::invalid_argument(message);
}
void uniqueNames(const std::vector<std::string>& names) {
    std::unordered_set<std::string> seen;
    for (const auto& name : names)
        require(!name.empty() && seen.insert(name).second, "Names must be nonempty and unique: " + name);
}
std::size_t indexOf(const std::vector<std::string>& names, const std::string& name) {
    return static_cast<std::size_t>(std::find(names.begin(), names.end(), name) - names.begin());
}
Config validate(Config c) {
    if (c.depth.enabled) {
        const auto& d = c.depth;
        require(c.backend == BackendType::Simulation && !d.site.empty(),
                "Depth capture requires simulation and an explicit camera site");
        require(d.frequency_hz > 0 && d.frequency_hz <= 100 &&
                std::isfinite(d.min_distance) && std::isfinite(d.max_distance) &&
                d.min_distance >= 0 && d.max_distance > d.min_distance &&
                std::isfinite(d.sensor_min_distance) && std::isfinite(d.sensor_max_distance) &&
                d.sensor_min_distance >= 0 && d.sensor_max_distance > d.sensor_min_distance &&
                std::isfinite(d.horizontal_fov_deg) && std::isfinite(d.vertical_fov_deg) &&
                d.horizontal_fov_deg > 0 && d.horizontal_fov_deg < 180 &&
                d.vertical_fov_deg > 0 && d.vertical_fov_deg < 180, "Invalid depth camera configuration");
    }
    if (c.motor_names.empty()) c.motor_names = c.joint_names;
    require(!c.joint_names.empty() && !c.motor_names.empty(), "Joint and motor names are required");
    uniqueNames(c.joint_names); uniqueNames(c.motor_names); uniqueNames(c.encoder_names);
    require(c.joint_names.size() <= 64 && c.motor_names.size() <= 64, "At most 64 joints/motors are supported");
    require(c.encoder_names.size() <= c.encoder_count, "Too many encoder names");
    require(c.control_mode == 1, "Runtime accepts normal control mode 1 only; calibration belongs in SDK tools");
    require(std::isfinite(c.damping_kd) && c.damping_kd > 0, "damping_kd must be finite and positive");
    if (c.motor_damping_kd.empty()) c.motor_damping_kd.assign(c.motor_names.size(), c.damping_kd);
    require(c.motor_damping_kd.size() == c.motor_names.size(), "Motor damping size mismatch");
    for (double kd : c.motor_damping_kd)
        require(std::isfinite(kd) && (c.passive_observation ? kd >= 0 : kd > 0),
                "Motor damping must be finite and positive (zero allowed only in passive observation)");
    if (c.passive_observation) {
        require(!c.hand_control && c.finger_hold_position.empty(), "Passive observation cannot command hands");
        std::fill(c.motor_damping_kd.begin(), c.motor_damping_kd.end(), 0.0);
    }
    require(c.startup_timeout.count() > 0 && c.feedback_timeout.count() > 0 &&
            c.command_timeout.count() > 0 && c.watchdog_period.count() > 0 &&
            c.watchdog_period <= c.feedback_timeout && c.watchdog_period <= c.command_timeout,
            "Timeouts must be positive and at least watchdog_period");
    require(std::isfinite(c.sensor_delay_ms) && c.sensor_delay_ms >= 0, "Invalid sensor delay");
    require(c.finger_joint_names.empty() || c.finger_joint_names.size() == 12, "Finger mapping needs 12 slots");
    std::vector<std::string> fingers;
    for (const auto& name : c.finger_joint_names)
        if (!name.empty()) fingers.push_back(name);
    uniqueNames(fingers);
    require(c.finger_coupling_csv.empty() == c.finger_couplings.empty(), "Coupling table and mappings must be supplied together");
    if (c.hand_control && c.backend == BackendType::Simulation)
        require(c.finger_hold_position.size() == 12 && !c.finger_couplings.empty() &&
                    c.finger_joint_names.size() == 12 && c.finger_joint_names[5].empty() &&
                    c.finger_joint_names[11].empty() && fingers.size() == 10,
                "Dynamic three-finger control requires 5 active slots per hand and calibrated impedance hold");
    if (!c.finger_hold_position.empty()) {
        require(c.backend == BackendType::Simulation, "Static finger hold is simulation-only");
        require(c.finger_joint_names.size() == 12 && c.finger_hold_position.size() == 12,
                "Finger hold needs 12 mapped SDK slots and positions");
        require(!fingers.empty(), "Finger hold needs at least one active joint");
        for (const auto& name : fingers)
            require(indexOf(c.motor_names, name) == c.motor_names.size(),
                    "Finger actuator cannot also receive body commands: " + name);
        require(std::isfinite(c.finger_hold_kp) && c.finger_hold_kp >= 0 &&
                std::isfinite(c.finger_hold_kd) && c.finger_hold_kd >= 0, "Invalid finger hold gains");
        require(c.finger_torque_limits.size() == 2, "Finger torque limits need two values");
        for (double q : c.finger_hold_position)
            require(std::isfinite(q) && q >= 0 && q <= 100, "Finger hold position outside [0,100]");
        for (double limit : c.finger_torque_limits)
            require(std::isfinite(limit) && limit >= .01 && limit <= 12, "Invalid finger torque limit");
    } else {
        require(c.finger_torque_limits.empty() && c.finger_hold_kp == 0 && c.finger_hold_kd == 0,
                "Finger gains/limits require explicit hold positions");
    }
    if (c.backend == BackendType::Hardware) {
        if (c.hardware_family == HardwareFamily::Aiq) {
            require(c.hardware_motor_slots == 32, "AIQ hardware SDK requires exactly 32 motor slots");
            require(c.encoder_count == 2, "AIQ hardware SDK requires exactly 2 encoder slots");
        } else {
            require(c.hardware_robot == "eqr11" || c.hardware_robot == "eqr11w", "EQR requires an explicit robot variant");
            require(c.hardware_motor_slots == 16 && c.encoder_count == 0 && c.encoder_names.empty(),
                    "EQR hardware requires 16 motor slots and no encoders");
            require(!c.hand_control && c.finger_joint_names.empty() && c.finger_couplings.empty(),
                    "EQR has no hand interface");
            require(c.motor_names.size() == (c.hardware_robot == "eqr11" ? 12u : 16u) &&
                    c.joint_names == c.motor_names && c.motor_slots.size() == c.motor_names.size(),
                    "EQR requires all active joints and explicit motor slots");
            const char* legs[]{"fl", "fr", "hl", "hr"};
            for (std::size_t i=0; i<c.motor_slots.size(); ++i) {
                const auto slot=c.motor_slots[i];
                require(slot < 16 && (c.hardware_robot == "eqr11w" || slot%4 != 3), "Inactive EQR motor slot");
                const auto name=std::string(legs[slot/4])+(slot%4 == 3 ? "_wheel" : "_leg_"+std::to_string(slot%4+1));
                require(c.motor_names[i] == name, "EQR joint/SDK slot mismatch: "+c.motor_names[i]);
            }
        }
        if (c.motor_slots.empty()) {
            c.motor_slots.resize(c.motor_names.size());
            std::iota(c.motor_slots.begin(), c.motor_slots.end(), 0);
        }
        require(c.motor_slots.size() == c.motor_names.size(), "Motor mapping size mismatch");
        std::unordered_set<std::size_t> slots;
        for (auto slot : c.motor_slots)
            require(slot < c.hardware_motor_slots && slots.insert(slot).second, "Invalid or duplicate motor slot");
        for (const auto& joint : c.joint_names)
            require(indexOf(c.motor_names, joint) < c.motor_names.size() ||
                    indexOf(c.encoder_names, joint) < c.encoder_names.size(),
                    "Hardware joint has neither motor nor encoder feedback: " + joint);
    } else {
        require(c.motor_slots.empty(), "Simulation uses motor_names order; hardware slot mapping must be empty");
    }
    return c;
}

template<class T> T optionalAt(const std::vector<T>& values, std::size_t i) {
    return i < values.size() ? values[i] : T{};
}
bool finite(const std::vector<double>& values, std::size_t count, bool nonnegative = false) {
    return values.size() == count && std::all_of(values.begin(), values.end(), [=](double x) {
        return std::isfinite(x) && (!nonnegative || x >= 0);
    });
}
State translate(const Config& c, const RobotState& raw) {
    State out;
    out.sdk = raw;
    const auto& m = raw.motor_states;
    const auto required = c.backend == BackendType::Hardware ? c.hardware_motor_slots :
                          c.joint_names.size() + c.motor_names.size();
    require(m.q.size() >= required && m.dq.size() >= required && m.tau.size() >= required,
            "Feedback position/velocity/torque dimensions are invalid");
    if (c.backend == BackendType::Hardware && c.hardware_family == HardwareFamily::Eqr)
        require(m.q.size() == 16 && m.dq.size() == 16 && m.tau.size() == 16 &&
                m.status_code.size() == 16 && m.error_code.size() == 16,
                "EQR feedback requires exactly 16 motor slots");
    if (c.backend == BackendType::Hardware)
        require(m.status_code.size() >= required && m.error_code.size() >= required,
                "Hardware feedback is missing motor status/error codes");
    for (std::size_t i = 0; i < c.motor_names.size(); ++i) {
        const auto slot = c.backend == BackendType::Hardware ? c.motor_slots[i] : c.joint_names.size() + i;
        MotorState motor;
        motor.position = m.q[slot]; motor.velocity = m.dq[slot]; motor.torque = m.tau[slot];
        motor.current = optionalAt(m.current, slot);
        motor.temperature = optionalAt(m.motor_T, slot);
        motor.mos_temperature = optionalAt(m.mos_T, slot);
        motor.status_code = optionalAt(m.status_code, slot);
        motor.error_code = optionalAt(m.error_code, slot);
        motor.loss_count = optionalAt(m.loss_num, slot);
        require(std::isfinite(motor.position) && std::isfinite(motor.velocity) && std::isfinite(motor.torque),
                "Nonfinite motor feedback");
        out.motors.push_back(motor);
    }
    for (std::size_t i = 0; i < c.joint_names.size(); ++i) {
        JointState joint;
        if (c.backend == BackendType::Simulation) {
            joint = {m.q[i], m.dq[i], m.tau[i]};
        } else {
            const auto motor = indexOf(c.motor_names, c.joint_names[i]);
            if (motor < out.motors.size()) joint = out.motors[motor];
            const auto encoder = indexOf(c.encoder_names, c.joint_names[i]);
            if (encoder < c.encoder_names.size()) {
                require(encoder < raw.encoder_states.q.size() && encoder < raw.encoder_states.dq.size(),
                        "Missing configured encoder feedback");
                joint.position = raw.encoder_states.q[encoder];
                joint.velocity = raw.encoder_states.dq[encoder];
            }
        }
        require(std::isfinite(joint.position) && std::isfinite(joint.velocity) && std::isfinite(joint.torque),
                "Nonfinite joint feedback");
        out.joints.push_back(joint);
    }
    return out;
}
} // namespace

Command::Command(std::size_t n) : position(n), velocity(n), torque(n), kp(n), kd(n) {}

namespace {
flight::Layout recorderLayout(const Config &c) {
    flight::Layout l;
    l.family = c.hardware_family == HardwareFamily::Aiq ? "aiq" : "eqr";
    l.robot = c.robot_identity.empty() ? c.hardware_robot : c.robot_identity;
    if (l.robot.empty() && !c.flight_recorder.enabled)
        l.robot = "unspecified";
    l.backend = c.backend == BackendType::Simulation ? "simulation" : "hardware";
    l.joint_names = c.joint_names;
    l.motor_names = c.motor_names;
    l.encoder_names = c.encoder_names;
    for (size_t i = l.encoder_names.size(); i < c.encoder_count; ++i)
        l.encoder_names.push_back("sdk_encoder_" + std::to_string(i));
    l.backend_slots =
        c.backend == BackendType::Hardware ? c.hardware_motor_slots : c.motor_names.size();
    l.raw_motor_count = c.backend == BackendType::Hardware
                            ? c.hardware_motor_slots
                            : c.joint_names.size() + c.motor_names.size();
    for (size_t i = 0; i < c.motor_names.size(); ++i)
        l.motor_slots.push_back(c.backend == BackendType::Hardware ? c.motor_slots[i] : i);
    l.hands = c.hand_control || !c.finger_joint_names.empty();
    l.continuous_joints = c.continuous_joints;
    l.controllers = c.recorder_controllers;
    l.config_digest = c.recorder_config_digest;
    l.software = "eir_loco_runtime/flight_schema_1/" EIR_RUNTIME_REVISION;
    return l;
}
void vectorField(flight::Packet &p, uint16_t tag, const std::vector<double> &values, size_t limit) {
    if (values.size() > limit)
        p.flags |= 1;
    p.doubles(tag, values.data(), std::min(values.size(), limit));
}
template <class T>
void codeField(flight::Packet &p, uint16_t tag, const std::vector<T> &values, size_t limit) {
    std::array<uint32_t, 128> v{};
    const auto n = std::min({values.size(), limit, v.size()});
    for (size_t i = 0; i < n; ++i)
        v[i] = static_cast<uint32_t>(values[i]);
    if (n < values.size())
        p.flags |= 1;
    p.integers(tag, v.data(), n);
}
flight::Packet feedbackPacket(const Config &c, const RobotState &raw, const State *state, Mode mode,
                              flight::Event event) {
    flight::Packet p;
    p.source_ns = raw.timestamp;
    p.event = event;
    if (mode == Mode::Estopped)
        p.flags |= 2;
    const auto mask = c.flight_recorder.data;
    uint32_t status[] = {static_cast<uint32_t>(mode)};
    p.integers(90, status, 1);
    if (mask & flight::Motors) {
        const auto &m = raw.motor_states;
        const auto n = c.backend == BackendType::Hardware
                           ? c.hardware_motor_slots
                           : c.motor_names.size() + c.joint_names.size();
        uint32_t dimensions[] = {uint32_t(m.q.size()),           uint32_t(m.dq.size()),
                                 uint32_t(m.tau.size()),         uint32_t(m.current.size()),
                                 uint32_t(m.motor_T.size()),     uint32_t(m.mos_T.size()),
                                 uint32_t(m.status_code.size()), uint32_t(m.error_code.size()),
                                 uint32_t(m.loss_num.size())};
        p.integers(99, dimensions, 9);
        vectorField(p, 100, m.q, n);
        vectorField(p, 101, m.dq, n);
        vectorField(p, 102, m.tau, n);
        vectorField(p, 103, m.current, n);
        vectorField(p, 104, m.motor_T, n);
        vectorField(p, 105, m.mos_T, n);
        codeField(p, 110, m.status_code, n);
        codeField(p, 111, m.error_code, n);
        codeField(p, 112, m.loss_num, n);
        if (state) {
            std::array<double, 64> q{}, dq{}, tau{};
            for (size_t i = 0; i < state->joints.size(); ++i) {
                q[i] = state->joints[i].position;
                dq[i] = state->joints[i].velocity;
                tau[i] = state->joints[i].torque;
            }
            p.doubles(120, q.data(), state->joints.size());
            p.doubles(121, dq.data(), state->joints.size());
            p.doubles(122, tau.data(), state->joints.size());
            p.correlation = state->sequence;
        }
    }
    if ((mask & flight::Imu) && !raw.imu_state.empty()) {
        p.doubles(200, raw.imu_state[0].quat, 4);
        p.doubles(201, raw.imu_state[0].gyro, 3);
        p.doubles(202, raw.imu_state[0].acc, 3);
    }
    if (mask & flight::Encoders) {
        vectorField(p, 210, raw.encoder_states.q, c.encoder_count);
        vectorField(p, 211, raw.encoder_states.dq, c.encoder_count);
        codeField(p, 212, raw.encoder_states.status, c.encoder_count);
    }
    if ((mask & flight::Bms) && !raw.bms_state.empty()) {
        const auto &b = raw.bms_state[0];
        double v[] = {b.voltage, b.current, b.SOC, b.temperature};
        p.doubles(220, v, 4);
        uint32_t code[] = {b.bms_status};
        p.integers(221, code, 1);
    }
    if (mask & flight::Hands) {
        vectorField(p, 230, raw.hand_states.q, 12);
        vectorField(p, 231, raw.hand_states.dq, 12);
        vectorField(p, 232, raw.hand_states.tau, 12);
        codeField(p, 233, raw.hand_states.code, 12);
    }
    if (mask & flight::Forces) {
        vectorField(p, 240, raw.force_states.fx, 4);
        vectorField(p, 241, raw.force_states.fy, 4);
        vectorField(p, 242, raw.force_states.fz, 4);
        vectorField(p, 243, raw.force_states.mx, 4);
        vectorField(p, 244, raw.force_states.my, 4);
        vectorField(p, 245, raw.force_states.mz, 4);
        codeField(p, 246, raw.force_states.code, 4);
    }
    if (mask & flight::Remote)
        codeField(p, 250, raw.remote_state.sbus_ch, 16);
    if (mask & flight::Head) {
        vectorField(p, 270, raw.head_state.q, 2);
        codeField(p, 271, raw.head_state.servo_online, 2);
    }
    if (mask & flight::Alarms)
        codeField(p, 260, raw.alarm_states.alarm_code, 64);
    return p;
}
} // namespace
struct Runtime::Impl {
    struct Shared {
        std::mutex mutex;
        std::condition_variable changed;
        State state;
        Mode mode{Mode::Stopped};
        std::string message;
        bool received{false}, accepting{false}, stop_worker{false}, suspended{false}, publishing{false};
        std::array<uint32_t, 64> previous_errors{}, previous_status{};
        std::array<uint8_t, 2> previous_head{255, 255};
        Clock::time_point feedback_at{}, command_at{}, publish_at{};
    };
    Config config;
    std::unique_ptr<Backend> backend;
    std::shared_ptr<Shared> shared{std::make_shared<Shared>()};
    std::mutex io_mutex, estop_mutex;
    std::thread watchdog;
    std::atomic<bool> active{false};
    bool estop_sent{false}; // Protected by estop_mutex.
    std::shared_ptr<flight::Recorder> recorder;
    bool recorder_finished{false};
    RobotCmd wire{};
    EECmd hand{};

    Impl(Config c, std::unique_ptr<Backend> b) : config(validate(std::move(c))), backend(std::move(b)) {
        recorder =
            std::make_shared<flight::Recorder>(config.flight_recorder, recorderLayout(config));
        const auto count = config.backend == BackendType::Hardware ? config.hardware_motor_slots : config.motor_names.size();
        auto& m = wire.motor_cmds;
        m.q.resize(count); m.dq.resize(count); m.tau.resize(count);
        m.Kp.resize(count); m.Kd.resize(count); m.Ki.resize(count);
        m.control_mode.resize(count); m.names.resize(count);
        wire.encoder_cmds.zero_cmd.resize(config.encoder_count);
        hand.q.assign(12, 0); hand.dq.assign(12, 0); hand.tau.assign(12, 0);
        hand.Kp.assign(12, 0); hand.Kd.assign(12, 0); hand.Ki.assign(12, 0);
        hand.mode.assign(2, 0);
        if (!config.finger_hold_position.empty()) hand.q = config.finger_hold_position;
        for (std::size_t i = 0; i < config.motor_names.size(); ++i)
            m.names[slot(i)] = config.motor_names[i];
    }
    std::size_t slot(std::size_t i) const {
        return config.backend == BackendType::Hardware ? config.motor_slots[i] : i;
    }
    void event(flight::Event e, flight::Source source = flight::Source::Lifecycle) {
        if (config.flight_recorder.enabled) {
            flight::Packet p;
            p.event = e;
            recorder->submit(source, p);
        }
    }
    void fail(const std::string &message, flight::Event reason = flight::Event::BackendFailure) {
        std::lock_guard lock(shared->mutex);
        if (shared->mode != Mode::Estopped || reason == flight::Event::Stopped)
            event(reason);
        if (shared->mode != Mode::Estopped) shared->message = message;
        shared->mode = Mode::Estopped;
    }
    void checkTimeouts() { // Shared mutex held.
        const auto now = Clock::now();
        if (shared->mode == Mode::Stopped || shared->mode == Mode::Estopped) return;
        if (!shared->received || now - shared->feedback_at > config.feedback_timeout) {
            shared->mode = Mode::Estopped;
            shared->message = "Feedback timeout";
            event(flight::Event::FeedbackTimeout, flight::Source::Watchdog);
        } else if (shared->publishing && now - shared->publish_at > config.command_timeout) {
            shared->mode = Mode::Estopped;
            shared->message = "Backend command timeout";
            event(flight::Event::CommandTimeout, flight::Source::Watchdog);
        } else if (shared->mode == Mode::Running && now - shared->command_at > config.command_timeout) {
            shared->mode = Mode::Estopped;
            shared->message = "Command timeout";
            event(flight::Event::CommandTimeout, flight::Source::Watchdog);
        }
    }
    void sendEstop() { // Independent of IO; backend must latch without waiting for write().
        std::lock_guard safety(estop_mutex);
        if (!estop_sent && backend) {
            estop_sent = true;
            try {
                if (!backend->estop()) {
                    std::lock_guard lock(shared->mutex);
                    shared->message += "; backend rejected Estop";
                }
            } catch (...) {
                std::lock_guard lock(shared->mutex);
                shared->message += "; backend Estop threw an exception";
            }
        }
    }
    void prepare(const Command* command) { // io_mutex held.
        if (config.passive_observation) command = nullptr;
        auto& m = wire.motor_cmds;
        // Unmapped hardware slots remain disabled (mode=0, all gains zero).
        for (std::size_t i = 0; i < config.motor_names.size(); ++i) {
            const auto j = slot(i);
            m.q[j] = command ? command->position[i] : 0;
            m.dq[j] = command ? command->velocity[i] : 0;
            m.tau[j] = command ? command->torque[i] : 0;
            m.Kp[j] = command ? command->kp[i] : 0;
            m.Kd[j] = command ? command->kd[i] : config.motor_damping_kd[i];
            m.control_mode[j] = config.control_mode;
        }
        wire.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    }
    bool publish(const Command* command, const Command* received = nullptr) { // io_mutex held.
        prepare(command);
        {
            std::lock_guard lock(shared->mutex);
            checkTimeouts();
            if (shared->mode == Mode::Estopped) return false;
            shared->publishing = true;
            shared->publish_at = Clock::now();
        }
        const auto submit_begin = flight::Recorder::monotonicNow();
        bool accepted = false;
        try {
            accepted = backend->write(wire);
            if (!accepted) fail("Backend rejected motor command");
            if (accepted && config.hand_control) {
                bool allowed;
                { std::lock_guard lock(shared->mutex); checkTimeouts();
                  allowed = shared->mode != Mode::Estopped; }
                if (allowed) {
                    const bool enabled = command && command->hand_enabled;
                    if (enabled) std::copy(command->hand_position.begin(), command->hand_position.end(), hand.q.begin());
                    const bool simulation = config.backend == BackendType::Simulation;
                    std::fill(hand.Kp.begin(), hand.Kp.end(), enabled ? command->hand_kp :
                              (simulation ? config.finger_hold_kp : 0));
                    std::fill(hand.Kd.begin(), hand.Kd.end(), enabled ? command->hand_kd :
                              (simulation ? config.finger_hold_kd : 0));
                    // LEGACY mode=0 matches the reference bridge. Do not reuse
                    // body motor enable/calibration codes for the hand protocol.
                    accepted = backend->writeHand(hand);
                    if (!accepted) fail("Backend rejected hand command");
                } else accepted = false;
            }
        } catch (const std::exception& e) { fail(e.what()); }
        catch (...) { fail("Backend command exception"); }
        {
            std::lock_guard lock(shared->mutex);
            // Check before clearing publishing or refreshing the deadline. A late
            // success cannot erase a watchdog fault or renew an expired command.
            checkTimeouts();
            shared->publishing = false;
            accepted = accepted && shared->mode != Mode::Estopped;
            if (accepted) shared->command_at = shared->publish_at;
        }
        if (config.flight_recorder.enabled) {
            flight::Packet record;
            record.source_ns = wire.timestamp;
            record.correlation = static_cast<uint64_t>(wire.timestamp);
            double times[] = {double(submit_begin), double(flight::Recorder::monotonicNow())};
            record.doubles(10, times, 2);
            uint32_t result[] = {accepted ? 1u : 0u};
            record.integers(11, result, 1);
            if (config.flight_recorder.data & flight::Commands) {
                const auto input=received?received:command;
                if (input) {
                    vectorField(record, 20, input->position, 64);
                    vectorField(record, 21, input->velocity, 64);
                    vectorField(record, 22, input->torque, 64);
                    vectorField(record, 23, input->kp, 64);
                    vectorField(record, 24, input->kd, 64);
                }
                vectorField(record, 30, wire.motor_cmds.q, 64);
                vectorField(record, 31, wire.motor_cmds.dq, 64);
                vectorField(record, 32, wire.motor_cmds.tau, 64);
                vectorField(record, 33, wire.motor_cmds.Kp, 64);
                vectorField(record, 34, wire.motor_cmds.Kd, 64);
                codeField(record, 35, wire.motor_cmds.control_mode, 64);
            }
            if ((config.flight_recorder.data & flight::Hands) && config.hand_control) {
                vectorField(record, 40, hand.q, 12);
                vectorField(record, 41, hand.Kp, 12);
                vectorField(record, 42, hand.Kd, 12);
            }
            if (config.flight_recorder.data & flight::ActuatorTorque) {
                std::array<double, 64> torque{};
                uint32_t count = 0;
                uint64_t seq = 0;
                if (backend->readFinalTorque(torque, count, seq)) {
                    record.doubles(50, torque.data(), std::min<uint32_t>(count, 64));
                    uint32_t code[] = {uint32_t(seq), uint32_t(seq >> 32)};
                    record.integers(51, code, 2);
                }
            }
            recorder->submit(flight::Source::Command, record);
        }
        if (!accepted) sendEstop();
        return accepted;
    }
    void watch() {
        for (;;) {
            Mode mode;
            {
                std::unique_lock lock(shared->mutex);
                if (shared->changed.wait_for(lock, config.watchdog_period, [&] { return shared->stop_worker; })) return;
                checkTimeouts();
                mode = shared->mode;
            }
            if (mode == Mode::Estopped) sendEstop();
            // No ordinary IO on this thread. Damping is sent on entry and kept
            // by the backend; write() in damping mode also sends only damping.
        }
    }

};

Runtime::Runtime(Config config, std::unique_ptr<Backend> backend)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(backend))) {}
Runtime::~Runtime() { stop(); }
const Config& Runtime::config() const noexcept { return impl_->config; }

bool Runtime::start() {
    auto& p = *impl_;
    std::lock_guard io(p.io_mutex);
    if (p.active) return false;
    if (p.recorder_finished) {
        p.recorder =
            std::make_shared<flight::Recorder>(p.config.flight_recorder, recorderLayout(p.config));
        p.recorder_finished = false;
    }
    p.event(flight::Event::Started);
    std::fill(p.hand.q.begin(), p.hand.q.end(), 0);
    if (!p.config.finger_hold_position.empty()) p.hand.q = p.config.finger_hold_position;
    {
        std::lock_guard lock(p.shared->mutex);
        p.shared->previous_errors.fill(0);
        p.shared->previous_status.fill(std::numeric_limits<uint32_t>::max());
        p.shared->previous_head.fill(255);
        p.shared->state = State{}; p.shared->received = false;
        p.shared->mode = Mode::Damping; p.shared->message = "Started in damping; explicitly enable control when ready";
        p.shared->accepting = true; p.shared->stop_worker = false; p.shared->publishing = false;
        p.shared->suspended = p.config.backend == BackendType::Simulation && p.config.suspend;
    }
    { std::lock_guard safety(p.estop_mutex); p.estop_sent = false; }
    try {
        if (!p.backend) p.backend = makeBackend(p.config.backend);
        const auto shared = p.shared;
        const auto config = p.config;
        const auto recorder = p.recorder;
        p.backend->start(config, [shared, config, recorder](const RobotState &raw) {
            try {
                State next = translate(config, raw);
                std::lock_guard lock(shared->mutex);
                if (!shared->accepting)
                    return;
                auto received_at = Clock::now();
                if (config.backend == BackendType::Hardware && config.hardware_family == HardwareFamily::Eqr) {
                    const auto source = Clock::time_point(std::chrono::nanoseconds(raw.timestamp));
                    require(raw.timestamp > 0 && source <= received_at && received_at-source <= config.feedback_timeout,
                            "EQR feedback source timestamp is invalid or stale");
                    require(!shared->received || raw.timestamp > shared->state.sdk.timestamp,
                            "EQR feedback source timestamp did not advance");
                    received_at = source;
                }
                next.sequence = shared->state.sequence + 1;
                shared->state = std::move(next);
                shared->received = true;
                shared->feedback_at = received_at;
                flight::Packet record;

                for (size_t i = 0; i < shared->state.motors.size(); ++i) {
                    const auto &motor = shared->state.motors[i];
                    const bool offline = config.backend == BackendType::Hardware &&
                                         (motor.status_code == 0 || motor.status_code > 3);
                    if (motor.error_code || offline) {
                        if (shared->mode != Mode::Estopped)
                            shared->message = "Motor fault or offline status";
                        shared->mode = Mode::Estopped;
                        if (config.flight_recorder.enabled &&
                            (motor.error_code != shared->previous_errors[i] ||
                             motor.status_code != shared->previous_status[i])) {
                            auto event = feedbackPacket(config,raw,&shared->state,shared->mode,
                                offline?flight::Event::HardwareOffline:flight::Event::MotorFault);
                            event.component = flight::Component::Motor;
                            event.component_index = i;
                            event.flags |= 2;
                            recorder->submit(flight::Source::Feedback, event);
                        }
                    }
                    shared->previous_errors[i] = motor.error_code;
                    shared->previous_status[i] = motor.status_code;
                }
                if (config.flight_recorder.enabled) {
                    record=feedbackPacket(config,raw,&shared->state,shared->mode,flight::Event::None);
                    if (config.flight_recorder.data & flight::Head)
                        for (size_t i = 0;
                             i < std::min<size_t>(2, raw.head_state.servo_online.size()); ++i) {
                            const auto online = raw.head_state.servo_online[i];
                            if (online == 0 && shared->previous_head[i] != online) {
                                auto event = record;
                                event.event = flight::Event::HardwareOffline;
                                event.component = flight::Component::HeadServo;
                                event.component_index = i;
                                recorder->submit(flight::Source::Feedback, event);
                            }
                            shared->previous_head[i] = online;
                        }
                    recorder->submit(flight::Source::Feedback, record);
                }
            } catch (const std::exception& e) {
                if (config.flight_recorder.enabled)
                    recorder->submit(flight::Source::Feedback,
                                     feedbackPacket(config, raw, nullptr, Mode::Estopped,
                                                    flight::Event::InvalidFeedback));
                std::lock_guard lock(shared->mutex);
                if (!shared->accepting)
                    return;
                if (shared->mode != Mode::Estopped)
                    shared->message = e.what();
                shared->mode = Mode::Estopped;
            }
            shared->changed.notify_all();
        });
        {
            std::unique_lock lock(p.shared->mutex);
            if (!p.shared->changed.wait_for(lock, p.config.startup_timeout, [&] {
                    return p.shared->received || p.shared->mode == Mode::Estopped; }))
                throw std::runtime_error("No feedback before startup timeout");
            if (p.shared->mode == Mode::Estopped) throw std::runtime_error(p.shared->message);
        }
        p.active = true;
        p.watchdog = std::thread([&p] { p.watch(); });
        if (!p.publish(nullptr)) throw std::runtime_error("Initial damping command failed");
        return true;
    } catch (const std::exception& e) { p.fail(e.what()); }
    catch (...) { p.fail("Backend startup exception"); }
    { std::lock_guard lock(p.shared->mutex); p.shared->accepting = false; p.shared->stop_worker = true; }
    p.shared->changed.notify_all();
    if (p.watchdog.joinable()) p.watchdog.join();
    p.sendEstop();
    { std::lock_guard safety(p.estop_mutex); if (p.backend) p.backend->stop(); }
    p.active = false;
    return false;
}

void Runtime::stop() noexcept {
    auto& p = *impl_;
    {
        std::lock_guard lock(p.shared->mutex);
        p.shared->stop_worker = true; p.shared->accepting = false;
    }
    p.shared->changed.notify_all();
    if (p.active) {
        p.fail("Runtime stopping", flight::Event::Stopped);
        p.sendEstop();
    }
    if (p.watchdog.joinable()) p.watchdog.join();
    std::lock_guard io(p.io_mutex);
    if (p.active) p.sendEstop();
    { std::lock_guard safety(p.estop_mutex); if (p.backend) p.backend->stop(); }
    p.active = false;
    if (p.recorder) {
        p.recorder->finish();
        p.recorder_finished = true;
    }
    std::lock_guard lock(p.shared->mutex);
    p.shared->mode = Mode::Stopped; p.shared->received = false; p.shared->suspended = false;
}

bool Runtime::read(State& output) const {
    const auto& p = *impl_;
    std::lock_guard lock(p.shared->mutex);
    if (!p.shared->received || Clock::now() - p.shared->feedback_at > p.config.feedback_timeout) return false;
    output = p.shared->state;
    return true;
}
Status Runtime::status() const {
    const auto& p = *impl_;
    std::lock_guard lock(p.shared->mutex);
    Status status;
    status.mode = p.shared->mode; status.feedback_received = p.shared->received;
    status.message = p.shared->message;
    status.suspended = p.shared->suspended;
    if (status.feedback_received)
        status.feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - p.shared->feedback_at);
    return status;
}

bool Runtime::readDepth(DepthStack& output) {
    auto& p = *impl_;
    std::lock_guard io(p.io_mutex);
    output = {};
    return p.active && p.config.depth.enabled && p.backend->readDepth(output);
}

bool Runtime::write(const Command& command) {
    auto& p = *impl_;
    std::lock_guard io(p.io_mutex);
    if (!p.active) return false;
    const auto n = p.config.motor_names.size();
    if (!finite(command.position, n) || !finite(command.velocity, n) || !finite(command.torque, n) ||
        !finite(command.kp, n, true) || !finite(command.kd, n, true)) {
        if (p.config.flight_recorder.enabled) {
            flight::Packet record;
            record.event = flight::Event::InvalidCommand;
            if (p.config.flight_recorder.data & flight::Commands) {
                vectorField(record, 20, command.position, 64);
                vectorField(record, 21, command.velocity, 64);
                vectorField(record, 22, command.torque, 64);
                vectorField(record, 23, command.kp, 64);
                vectorField(record, 24, command.kd, 64);
            }
            uint32_t dimensions[] = {uint32_t(command.position.size()),
                                     uint32_t(command.velocity.size()),
                                     uint32_t(command.torque.size()), uint32_t(command.kp.size()),
                                     uint32_t(command.kd.size())};
            record.integers(12, dimensions, 5);
            p.recorder->submit(flight::Source::Command, record);
        }
        p.fail("Invalid command dimensions, nonfinite values or negative gains",
               flight::Event::InvalidCommand);
        p.sendEstop();
        return false;
    }
    if (command.hand_enabled && (!p.config.hand_control ||
        !std::all_of(command.hand_position.begin(), command.hand_position.end(), [](double q) {
            return std::isfinite(q) && q >= 0 && q <= 100;
        }) || command.hand_position[5] != 0 || command.hand_position[11] != 0 ||
        !std::isfinite(command.hand_kp) || command.hand_kp < 0 || command.hand_kp > 100 ||
        !std::isfinite(command.hand_kd) || command.hand_kd < 0 || command.hand_kd > 10)) {
        p.fail("Invalid hand command or hand control disabled", flight::Event::InvalidCommand);
        p.sendEstop();
        return false;
    }
    Mode mode;
    { std::lock_guard lock(p.shared->mutex); p.checkTimeouts(); mode = p.shared->mode; }
    if (mode == Mode::Estopped) { p.sendEstop(); return false; }
    return p.publish(mode == Mode::Damping ? nullptr : &command, &command);
}

bool Runtime::setDamping(bool enabled) {
    auto& p = *impl_;
    std::lock_guard io(p.io_mutex);
    if (!p.active) return false;
    if (p.config.passive_observation && !enabled) return false;
    {
        std::lock_guard lock(p.shared->mutex);
        p.checkTimeouts();
        if (p.shared->mode != Mode::Estopped &&
            p.shared->mode != (enabled ? Mode::Damping : Mode::Running)) {
            p.shared->mode = enabled ? Mode::Damping : Mode::Running;
            p.shared->message = enabled ? "Damping requested" : "Control enabled";
            p.shared->command_at = Clock::now();
        }
    }
    if (status().mode == Mode::Estopped) { p.sendEstop(); return false; }
    return !enabled || p.publish(nullptr);
}
bool Runtime::releaseSuspension() {
    auto& p = *impl_;
    std::lock_guard io(p.io_mutex);
    if (!p.active || p.config.backend != BackendType::Simulation) return false;
    {
        std::lock_guard lock(p.shared->mutex);
        p.checkTimeouts();
    }
    if (status().mode == Mode::Estopped) { p.sendEstop(); return false; }
    if (!status().suspended) return true;
    try {
        if (p.backend->releaseSuspension()) {
            std::lock_guard lock(p.shared->mutex);
            p.shared->suspended = false;
            return true;
        }
        p.fail("Backend rejected suspension release");
    } catch (const std::exception& e) { p.fail(e.what()); }
    catch (...) { p.fail("Suspension release exception"); }
    p.sendEstop();
    return false;
}
void Runtime::estop(bool record) {
    auto& p = *impl_;
    if (!p.active) return;
    p.fail("Emergency stop requested", record ? flight::Event::Estop : flight::Event::Stopped);
    p.sendEstop();
}
void Runtime::protectiveStop() {
    auto& p = *impl_;
    if (!p.active) return;
    p.fail("Application protection stop", flight::Event::ApplicationFault);
    p.sendEstop();
}
bool Runtime::saveFlightRecord() noexcept {
    return impl_->recorder && impl_->recorder->requestSave();
}
flight::Statistics Runtime::flightStatistics() const noexcept {
    return impl_->recorder ? impl_->recorder->statistics() : flight::Statistics{};
}
void Runtime::recordApplication(flight::Packet p) noexcept {
    if (impl_->recorder && impl_->config.flight_recorder.enabled &&
        ((impl_->config.flight_recorder.data & flight::Application) ||
         p.event != flight::Event::None || (p.flags & flight::Packet::EventOnly)))
        impl_->recorder->submit(flight::Source::Application, p);
}
} // namespace eir::loco
