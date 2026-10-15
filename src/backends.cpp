#include "backends.hpp"
#include <atomic>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <thread>
#if EIR_WITH_HARDWARE
#if EIR_HARDWARE_EQR
#include <eir_base_sdk_eqr/EqrLowLevelApi.h>
#else
#include <eir_base_sdk/AiqLowLevelApi.h>
#endif
#include <yaml-cpp/yaml.h>
#endif
#if EIR_WITH_MUJOCO
#include <eir_mujoco/mj_bridge.h>
#endif

namespace eir::loco {
namespace {
#if EIR_WITH_HARDWARE
class HardwareBackend final : public Backend {
    #if EIR_HARDWARE_EQR
    using Api = EIR_Robotics::EqrLowLevelApi;
#else
    using Api = EIR_Robotics::AiqLowLevelApi;
#endif
    std::unique_ptr<Api> api_;
public:
    ~HardwareBackend() override { stop(); }
    void start(const Config& config, StateCallback callback) override {
        #if EIR_HARDWARE_EQR
        if (config.hardware_family != HardwareFamily::Eqr)
            throw std::runtime_error("This runtime is built for EQR hardware");
        if (std::string(Api::InterfaceVersion()) != EIR_EQR_ABI_TAG)
            throw std::runtime_error("Loaded EQR SDK ABI does not match its headers");
        if (!Api::ValidateConfiguration(config.hardware_config, config.hardware_robot))
            throw std::runtime_error("EQR SDK profile/robot mismatch or invalid configuration");
#else
        if (config.hardware_family != HardwareFamily::Aiq)
            throw std::runtime_error("This runtime is built for AIQ hardware");
#endif
        if (!std::filesystem::is_regular_file(config.hardware_config))
            throw std::runtime_error("Hardware SDK config file does not exist");
        if (config.hand_control) {
            // Check before Init can open hardware. Percent closure is NOT the
            // EIRGRIPPER motor's [0,1] rad interface.
            const auto n = YAML::LoadFile(config.hardware_config)["humanoid_hardware_node"];
            const auto version = n["RobotVersion"].as<std::string>();
            const auto ee = n["Versions"][version]["EndEffectorMapping"];
            if (version != "aiq11-pro" || !ee || ee["type"].as<std::string>("") != "LEGACY")
                throw std::runtime_error("Three-finger control requires explicit AIQ11 PRO LEGACY SDK mapping");
            for (const auto* side : {"Left", "Right"})
                if (ee[side] && ee[side]["type"] && ee[side]["type"].as<std::string>() != "LEGACY")
                    throw std::runtime_error("Conflicting three-finger SDK side type");
        }
        api_ = std::make_unique<Api>();
        api_->SubscribeRobotState([callback](const RobotStatePtr state) {
            if (state) callback(*state);
        });
        if (!api_->Init(config.hardware_config))
            throw std::runtime_error("Hardware SDK initialization failed");
    }
    void stop() noexcept override {
        if (api_) { api_->Shutdown(); api_.reset(); }
    }
    bool write(const RobotCmd& command) override { return api_ && api_->PublishRobotCmd(command); }
    bool writeHand(const EECmd& command) override { return api_ && api_->PublishEndEffectorCmd(command); }
    bool estop() override { return api_ && api_->Estop(); }
};
#endif

#if EIR_WITH_MUJOCO
class SimulationBackend final : public Backend {
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    std::unique_ptr<MJ_BRIDGE::MJ_Bridge> api_;
    std::atomic<bool> running_{false};
    std::thread physics_;
public:
    ~SimulationBackend() override { stop(); }
    void start(const Config& config, StateCallback callback) override {
        if (!std::filesystem::is_regular_file(config.model_path))
            throw std::runtime_error("MuJoCo model file does not exist");
        if (mj_version() != mjVERSION_HEADER)
            throw std::runtime_error("MuJoCo headers and loaded library differ");
        // Load and validate names before launching a GUI or physics thread.
        char error[2048]{};
        model_.reset(mj_loadXML(config.model_path.c_str(), nullptr, error, sizeof(error)));
        if (!model_) throw std::runtime_error(std::string("MJCF: ") + error);
        // The first string in MuJoCo's name buffer is the model name.
        if (!config.model_name.empty() && config.model_name != model_->names)
            throw std::runtime_error("MuJoCo model identity mismatch: expected " + config.model_name);
        for (const auto& name : config.joint_names)
            if (mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str()) < 0)
                throw std::runtime_error("Unknown MuJoCo joint: " + name);
        for (const auto& name : config.motor_names)
            if (mj_name2id(model_.get(), mjOBJ_ACTUATOR, name.c_str()) < 0)
                throw std::runtime_error("Unknown MuJoCo actuator: " + name);
        for (const auto& name : config.finger_joint_names)
            if (!name.empty() && mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str()) < 0)
                throw std::runtime_error("Unknown MuJoCo finger joint: " + name);
        api_ = std::make_unique<MJ_BRIDGE::MJ_Bridge>(config.model_path);
        api_->SubscribeRobotState([callback](const RobotStatePtr state) {
            if (state) callback(*state);
        });
        api_->setSuspendEnable(config.suspend);
        if (config.depth.enabled) {
            // Reject a typo instead of silently binding the SDK's legacy alias.
            if (mj_name2id(model_.get(), mjOBJ_SITE, config.depth.site.c_str()) < 0)
                throw std::runtime_error("Unknown depth camera site: " + config.depth.site);
            MJ_BRIDGE::DepthCameraConfig depth;
            depth.camera_site_name = config.depth.site;
            depth.preprocessing_version = config.depth.preprocessing_version;
            depth.raw_width = config.depth.raw_width;
            depth.raw_height = config.depth.raw_height;
            depth.crop_up = config.depth.crop_up;
            depth.crop_down = config.depth.crop_down;
            depth.crop_left = config.depth.crop_left;
            depth.crop_right = config.depth.crop_right;
            depth.capture_frequency_hz = config.depth.frequency_hz;
            depth.min_distance = config.depth.min_distance;
            depth.max_distance = config.depth.max_distance;
            depth.sensor_min_distance = config.depth.sensor_min_distance;
            depth.sensor_max_distance = config.depth.sensor_max_distance;
            depth.horizontal_fov_deg = config.depth.horizontal_fov_deg;
            depth.vertical_fov_deg = config.depth.vertical_fov_deg;
            MJ_BRIDGE::DepthCamera probe(depth);
            std::string depth_error;
            if (!probe.bind(model_.get(), &depth_error)) throw std::runtime_error(depth_error);
            api_->setDepthCameraConfig(depth);
            api_->setDepthPublishEnabled(true);
        }
        if (config.headless) {
            data_.reset(mj_makeData(model_.get()));
            if (!data_) throw std::runtime_error("mj_makeData failed");
            mj_forward(model_.get(), data_.get());
            api_->setup(model_.get(), data_.get());
        } else {
            model_.reset(); // The SDK GUI owns its own model and data.
            if (!api_->Init()) throw std::runtime_error("MuJoCo GUI initialization failed");
        }
        if (!config.finger_joint_names.empty() && !api_->setFingerJointSequence(config.finger_joint_names))
            throw std::runtime_error("Invalid finger joint mapping");
        if (!config.finger_couplings.empty()) {
            std::vector<MJ_BRIDGE::FingerCouplingJoint> couplings;
            for (const auto& entry : config.finger_couplings)
                couplings.push_back({entry.curve, entry.active_joint, entry.passive_joint});
            if (!api_->setFingerCouplingTable(config.finger_coupling_csv, couplings))
                throw std::runtime_error("Invalid finger coupling table or mapping");
        }
        if (!api_->setJointSequence(config.joint_names))
            throw std::runtime_error("Failed to configure MuJoCo joint order");
        api_->setSensorDelay(config.sensor_delay_ms);
        if (!config.finger_hold_position.empty()) {
            if (!api_->setFingerImpedanceControl(true) ||
                !api_->setFingerTorqueLimits(config.finger_torque_limits))
                throw std::runtime_error("Finger hold requires actuators and valid torque limits");
            EECmd hand{};
            hand.q = config.finger_hold_position;
            hand.dq.assign(12, 0.0); hand.tau.assign(12, 0.0);
            hand.Kp.assign(12, config.finger_hold_kp); hand.Kd.assign(12, config.finger_hold_kd);
            if (!api_->PublishEndEffectorCmd(hand))
                throw std::runtime_error("Failed to initialize simulation hand posture");
        }
        // This enables GUI physics: finish all mappings and hand targets first.
        if (!api_->setMotorSequence(config.motor_names))
            throw std::runtime_error("Failed to configure MuJoCo motor order");
        if (config.headless) {
            const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(model_->opt.timestep));
            if (period.count() <= 0) throw std::runtime_error("Invalid simulation timestep");
            running_ = true;
            physics_ = std::thread([this, period] {
                auto next = std::chrono::steady_clock::now();
                try {
                    while (running_) {
                        api_->bridgeStep();
                        next += period;
                        // Avoid a catch-up burst after a long scheduling pause.
                        if (next < std::chrono::steady_clock::now()) next = std::chrono::steady_clock::now();
                        std::this_thread::sleep_until(next);
                    }
                } catch (...) { running_ = false; } // Runtime feedback watchdog reports the failure.
            });
        }
    }
    bool readFinalTorque(std::array<double, 64> &values, uint32_t &count,
                         uint64_t &sequence) override {
        if (!api_)
            return false;
        const auto snapshot = api_->getFinalTorqueSnapshot();
        values = snapshot.torque;
        count = snapshot.count;
        sequence = snapshot.sequence;
        return snapshot.valid;
    }
    void stop() noexcept override {
        running_ = false;
        if (physics_.joinable()) physics_.join();
        if (api_) { api_->stop_sim(); api_.reset(); }
        data_.reset(); model_.reset();
    }
    bool write(const RobotCmd& command) override { return api_ && api_->PublishRobotCmd(command); }
    bool writeHand(const EECmd& command) override { return api_ && api_->PublishEndEffectorCmd(command); }
    bool releaseSuspension() override {
        if (!api_) return false;
        api_->setSuspendEnable(false);
        return true;
    }
    bool readDepth(DepthStack& output) override {
        MJ_BRIDGE::DepthStack source;
        if (!api_ || !api_->getDepthStack(source)) return false;
        output.data = source.data;
        output.sequence = source.source_sequence;
        output.stamp = source.captured_monotonic_ns;
        output.valid = source.valid;
        return output.valid;
    }
    bool estop() override {
        // Inhibit physics immediately; join/SDK teardown belong to stop().
        running_ = false;
        if (!api_) return false;
        api_->requestEstop();
        return true;
    }
};
#endif
} // namespace

std::unique_ptr<Backend> makeBackend(BackendType type) {
    if (type == BackendType::Hardware) {
#if EIR_WITH_HARDWARE
        return std::make_unique<HardwareBackend>();
#else
        throw std::runtime_error("Hardware backend was disabled at build time");
#endif
    }
#if EIR_WITH_MUJOCO
    return std::make_unique<SimulationBackend>();
#else
    throw std::runtime_error("MuJoCo backend was disabled at build time");
#endif
}
} // namespace eir::loco
