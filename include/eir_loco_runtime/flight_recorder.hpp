#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace eir::loco::flight {
// File codes are append-only. They do not encode a particular controller or robot.
enum class Source : uint16_t { Command, Feedback, Watchdog, Application, Lifecycle, Manual, Count };
enum class Event : uint16_t {
    None,
    Started,
    Stopped,
    ModeChanged,
    InvalidCommand,
    InvalidFeedback,
    FeedbackTimeout,
    CommandTimeout,
    BackendFailure,
    MotorFault,
    Estop,
    ApplicationFault,
    ManualSave,
    ClockJump,
    Warning,
    HardwareOffline,
    Count
};
enum class Component : uint16_t { None, Motor, HeadServo };
struct Exemption {
    Event event;
    Component component;
    std::vector<std::string> names;
};
enum Data : uint32_t {
    Commands = 1,
    Motors = 2,
    Imu = 4,
    Encoders = 8,
    Bms = 16,
    Hands = 32,
    Forces = 64,
    Remote = 128,
    Alarms = 256,
    Application = 512,
    ActuatorTorque = 1024,
    Head = 2048
};
constexpr uint64_t bit(Event e) { return uint64_t{1} << static_cast<unsigned>(e); }
constexpr uint64_t defaultTriggers = bit(Event::InvalidCommand) | bit(Event::InvalidFeedback) |
                                     bit(Event::FeedbackTimeout) | bit(Event::CommandTimeout) |
                                     bit(Event::BackendFailure) | bit(Event::MotorFault) |
                                     bit(Event::Estop) | bit(Event::ApplicationFault) |
                                     bit(Event::ManualSave) | bit(Event::HardwareOffline);
struct Config {
    bool enabled{false};
    std::string directory{"log/flight_recorder"};
    unsigned detailed_hz{500}, application_hz{50}, continuous_hz{50};
    unsigned pre_seconds{10}, post_seconds{5}, cooldown_seconds{10}, sync_ms{100};
    unsigned queue_frames{256}, max_pending{2};
    uint64_t max_memory_bytes{256ULL << 20}, max_storage_bytes{2ULL << 30};
    uint64_t min_free_bytes{1ULL << 30}, continuous_file_bytes{64ULL << 20};
    uint32_t data{Commands | Motors | Imu | Encoders | Bms | Remote | Alarms | Application |
                  ActuatorTorque};
    uint64_t triggers{defaultTriggers};
    std::vector<Exemption> trigger_exemptions;
};
struct Layout {
    std::string family, robot, backend, run_id, software, config_digest;
    std::vector<std::string> joint_names, motor_names, encoder_names;
    std::vector<uint32_t> motor_slots;
    std::vector<bool> continuous_joints;
    std::vector<std::string> controllers; // Stable app code + identity/model digest description.
    uint32_t raw_motor_count{0}, backend_slots{0};
    bool hands{false};
};
// Payload is explicit little-endian TLV: tag:u16, type:u16 (1=f64,2=u32), count:u32, values.
// A packet never allocates. Overflow is explicit; dimensions are recorded separately.
struct Packet {
    // Persist an operation/request in events.bin without consuming diagnostic history.
    // Triggering events still retain their snapshot and incident payload.
    static constexpr uint32_t EventOnly = 8;
    int64_t system_ns{0}, monotonic_ns{0}, source_ns{0};
    uint64_t sequence{0}, correlation{0};
    Event event{Event::None};
    Component component{Component::None};
    int32_t component_index{-1};
    uint32_t flags{0},
        used{0}; // 1=truncated, 2=fault, 4=clock jump, 8=EventOnly; app flags start at bit 8.
    std::array<uint8_t, 12288> payload{};
    void doubles(uint16_t tag, const double *values, std::size_t count) noexcept;
    void integers(uint16_t tag, const uint32_t *values, std::size_t count) noexcept;
};
struct Statistics {
    uint64_t accepted{}, dropped{}, completed{}, incomplete{}, write_failures{}, quota_rejections{};
    uint64_t memory_bytes{}, max_submit_ns{};
    bool enabled{}, healthy{};
    uint64_t exempted{};
};
class Recorder {
  public:
    Recorder(Config, Layout);
    ~Recorder();
    Recorder(const Recorder &) = delete;
    Recorder &operator=(const Recorder &) = delete;
    // Nonblocking, bounded copy. Concurrent calls are permitted; overlapping producers may drop.
    bool submit(Source, Packet) noexcept;
    bool requestSave() noexcept; // Accepted means queued, not durable. One outstanding request.
    Statistics statistics() const noexcept;
    void finish() noexcept; // Non-RT, after producers stop; finishes existing incidents only.
    static int64_t monotonicNow() noexcept;
    static int64_t systemNow() noexcept;
    static const char *eventName(Event) noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace eir::loco::flight
