#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <eir_loco_runtime/flight_recorder.hpp>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace eir::loco::flight {
#ifdef EIR_FLIGHT_TEST_HOOKS
void flightTestHook(unsigned stage); // Test build only; absent from the runtime library.
#endif
static_assert(std::atomic<uint64_t>::is_always_lock_free &&
                  std::atomic<size_t>::is_always_lock_free,
              "Recorder requires lock-free counters");
namespace {
namespace fs = std::filesystem;
constexpr size_t sources = static_cast<size_t>(Source::Count), envelope = 60;
uint32_t hash(const uint8_t *p, size_t n) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i)
        h = (h ^ p[i]) * 16777619u;
    return h;
}
void put(uint8_t *p, uint64_t n, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i)
        p[i] = uint8_t(n >> (8 * i));
}
uint64_t get(const uint8_t *p, size_t bytes) {
    uint64_t n = 0;
    for (size_t i = 0; i < bytes; ++i)
        n |= uint64_t(p[i]) << (8 * i);
    return n;
}
size_t encode(const Packet &p, Source source, uint8_t *b) {
    const size_t size = 52 + p.used;
    put(b, size, 4);
    put(b + 8, static_cast<uint16_t>(source), 2);
    put(b + 10, static_cast<uint16_t>(p.event), 2);
    put(b + 12, p.flags, 4);
    put(b + 16, p.system_ns, 8);
    put(b + 24, p.monotonic_ns, 8);
    put(b + 32, p.source_ns, 8);
    put(b + 40, p.sequence, 8);
    put(b + 48, p.correlation, 8);
    put(b + 56, p.used, 4);
    std::memcpy(b + 60, p.payload.data(), p.used);
    put(b + 4, hash(b + 8, size), 4);
    return size + 8;
}
bool writeAll(int fd, const void *data, size_t bytes) {
    auto p = static_cast<const uint8_t *>(data);
    while (bytes) {
        auto n = ::write(fd, p, bytes);
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return false;
        p += n;
        bytes -= n;
    }
    return true;
}
std::string quote(const std::string &s) {
    std::string r = "\"";
    const char *digits = "0123456789abcdef";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') {
            r += '\\';
            r += c;
        } else if (c < 32) {
            r += "\\u00";
            r += digits[c >> 4];
            r += digits[c & 15];
        } else
            r += c;
    }
    return r + '"';
}
void strings(std::ostream &out, const std::vector<std::string> &v) {
    out << '[';
    bool first = true;
    for (const auto &x : v) {
        if (!first)
            out << ',';
        out << quote(x);
        first = false;
    }
    out << ']';
}
bool artifactName(const std::string &name) {
    if (name == ".eir_flight" || name == "metadata.json" || name == "statistics.json" ||
        name == "continuous.bin" || name == "continuous.bin.1" || name == "events.bin" ||
        name == "events.bin.1")
        return true;
    size_t begin = name.rfind("incident_", 0) == 0 ? 9 : (name.rfind("snapshot_", 0) == 0 ? 9 : 0);
    if (!begin)
        return false;
    const auto middle = name.find('_', begin), end = name.find('.', begin);
    if (middle == std::string::npos || end == std::string::npos || middle == begin ||
        middle + 1 >= end)
        return false;
    const auto extension = name.substr(end);
    if (extension != ".bin" && extension != ".json" && extension != ".bin.tmp")
        return false;
    for (size_t i = begin; i < end; ++i)
        if (i != middle && (name[i] < '0' || name[i] > '9'))
            return false;
    return true;
}
// Consumer exclusively owns these buffers. No reader touches an actively written producer slot.
struct Ring {
    size_t stride{}, capacity{}, count{}, next{};
    std::vector<uint8_t> bytes;
    Ring(size_t s, size_t c) : stride(s), capacity(c), bytes(s * c) {}
    void push(const uint8_t *p, size_t n) {
        auto dst = bytes.data() + next * stride;
        put(dst, n, 4);
        std::memcpy(dst + 4, p, n);
        next = (next + 1) % capacity;
        count = std::min(count + 1, capacity);
    }
    template <class F> void each(F f) const {
        for (size_t i = 0; i < count; ++i) {
            const auto p = bytes.data() + ((next + capacity - count + i) % capacity) * stride;
            f(p + 4, size_t(get(p, 4)));
        }
    }
    int64_t oldestMono() const {
        return count ? int64_t(get(bytes.data() + ((next + capacity - count) % capacity) * stride +
                                       4 + 24,
                                   8))
                     : 0;
    }
    void clear() { count = next = 0; }
};
struct Queue {
    std::unique_ptr<Packet[]> slots;
    size_t capacity{};
    std::atomic<size_t> head{0}, tail{0};
    std::atomic_flag producer = ATOMIC_FLAG_INIT;
    std::atomic<uint64_t> &sequence;
    uint64_t last_bucket{uint64_t(-1)};
    explicit Queue(size_t c, std::atomic<uint64_t> &seq)
        : slots(new Packet[c]), capacity(c), sequence(seq) {}
    bool push(Packet p, unsigned hz) noexcept {
        if (producer.test_and_set(std::memory_order_acquire))
            return false;
        const auto h = head.load(std::memory_order_relaxed),
                   t = tail.load(std::memory_order_acquire);
        const auto bucket = uint64_t(p.monotonic_ns) / (1000000000ULL / hz);
        const bool sampled =
            (p.event == Event::None || p.event == Event::Warning) && !(p.flags & Packet::EventOnly);
        if (sampled && bucket == last_bucket) {
            producer.clear(std::memory_order_release);
            return true;
        }
        p.sequence = sequence.fetch_add(1, std::memory_order_relaxed);
        if (h - t == capacity) {
            producer.clear(std::memory_order_release);
            return false;
        }
        if (sampled)
            last_bucket = bucket;
        slots[h % capacity] = p;
        head.store(h + 1, std::memory_order_release);
        producer.clear(std::memory_order_release);
        return true;
    }
    bool pop(Packet &p) noexcept {
        const auto t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire))
            return false;
        p = slots[t % capacity];
        tail.store(t + 1, std::memory_order_release);
        return true;
    }
};
} // namespace
void Packet::doubles(uint16_t tag, const double *v, size_t n) noexcept {
    if (used > payload.size() - 8 || n > (payload.size() - used - 8) / 8) {
        flags |= 1;
        return;
    }
    auto p = payload.data() + used;
    put(p, tag, 2);
    put(p + 2, 1, 2);
    put(p + 4, n, 4);
    for (size_t i = 0; i < n; ++i) {
        uint64_t bits;
        std::memcpy(&bits, v + i, 8);
        put(p + 8 + 8 * i, bits, 8);
    }
    used += 8 + 8 * n;
}
void Packet::integers(uint16_t tag, const uint32_t *v, size_t n) noexcept {
    if (used > payload.size() - 8 || n > (payload.size() - used - 8) / 4) {
        flags |= 1;
        return;
    }
    auto p = payload.data() + used;
    put(p, tag, 2);
    put(p + 2, 2, 2);
    put(p + 4, n, 4);
    for (size_t i = 0; i < n; ++i)
        put(p + 8 + 4 * i, v[i], 4);
    used += 8 + 4 * n;
}
int64_t Recorder::systemNow() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}
int64_t Recorder::monotonicNow() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
const char *Recorder::eventName(Event e) noexcept {
    static const char *names[] = {"none",
                                  "started",
                                  "stopped",
                                  "mode_changed",
                                  "invalid_command",
                                  "invalid_feedback",
                                  "feedback_timeout",
                                  "command_timeout",
                                  "backend_failure",
                                  "motor_fault",
                                  "estop",
                                  "application_fault",
                                  "manual_save",
                                  "clock_jump",
                                  "warning",
                                  "hardware_offline"};
    return static_cast<size_t>(e) < static_cast<size_t>(Event::Count)
               ? names[static_cast<size_t>(e)]
               : "unknown";
}
struct Recorder::Impl {
    Config config;
    Layout layout;
    std::array<std::unique_ptr<Queue>, sources> queues, event_queues;
    std::array<std::atomic<uint64_t>, sources> sequences{};
    std::array<std::unique_ptr<Ring>, sources> history;
    struct Incident {
        bool active{};
        uint64_t id{}, drops{};
        int64_t trigger{}, until{}, system{};
        Event root{};
        std::array<std::unique_ptr<Ring>, sources> records;
    };
    std::vector<Incident> incidents;
    std::array<size_t, sources> strides{};
    std::array<unsigned, sources> rates{};
    std::array<int64_t, sources> last_system{}, last_mono{};
    std::array<uint64_t, sources> last_sequence{};
    std::array<uint64_t, sources> history_bucket{};
    Packet latest_feedback{}, latest_application{};
    std::atomic<bool> stop{false}, ready{false}, healthy{false}, manual{false};
    std::atomic<uint64_t> accepted{0}, dropped{0}, completed{0}, incomplete{0}, failures{0},
        quota{0}, max_submit{0}, exempted{0};
    uint64_t memory{}, next_incident{}, continuous_size{};
    int lock_fd{-1}, continuous_fd{-1}, event_fd{-1};
    bool continuous_failed{false}, event_failed{false};
    uint64_t event_bytes{};
    int64_t event_sync{};
    int64_t begin_system{Recorder::systemNow()}, begin_mono{Recorder::monotonicNow()},
        next_continuous{}, last_sync{};
    fs::path run;
    std::thread worker;
    Impl(Config c, Layout l) : config(std::move(c)), layout(std::move(l)) {
        auto require = [](bool ok, const char *reason) {
            if (!ok)
                throw std::invalid_argument(reason);
        };
        require(config.detailed_hz > 0 && config.detailed_hz <= 2000 && config.application_hz > 0 &&
                    config.application_hz <= 2000 && config.continuous_hz > 0 &&
                    config.continuous_hz <= config.detailed_hz,
                "Invalid flight recorder rates");
        require(config.pre_seconds > 0 && config.pre_seconds <= 120 && config.post_seconds <= 120 &&
                    config.cooldown_seconds <= 120 && config.sync_ms > 0 &&
                    config.sync_ms <= 10000 && config.queue_frames >= 4 &&
                    config.queue_frames <= 4096 && config.max_pending > 0 &&
                    config.max_pending <= 4,
                "Invalid flight recorder windows or queue limits");
        require(config.max_memory_bytes <= (1ULL << 30) && config.max_storage_bytes >= 4096 &&
                    config.continuous_file_bytes >= 4096 &&
                    config.continuous_file_bytes <= config.max_storage_bytes / 2,
                "Invalid flight recorder resource limits");
        require((config.data & ~uint32_t(4095)) == 0 &&
                    (config.triggers >> static_cast<unsigned>(Event::Count)) == 0 &&
                    !(config.triggers & (bit(Event::None) | bit(Event::Started) |
                                         bit(Event::Stopped) | bit(Event::ModeChanged))),
                "Invalid flight recorder whitelist");
        require(!layout.family.empty() && !layout.robot.empty() && !layout.backend.empty(),
                "Recorder needs explicit robot identity");
        require(layout.motor_names.size() <= 64 && layout.joint_names.size() <= 64 &&
                    layout.raw_motor_count <= 128 && layout.backend_slots <= 64 &&
                    layout.encoder_names.size() <= 64 &&
                    layout.motor_slots.size() == layout.motor_names.size() &&
                    (layout.continuous_joints.empty() ||
                     layout.continuous_joints.size() == layout.joint_names.size()),
                "Recorder layout size mismatch");
        for (const auto *v : {&layout.joint_names, &layout.motor_names, &layout.encoder_names}) {
            auto names = *v;
            std::sort(names.begin(), names.end());
            require(std::adjacent_find(names.begin(), names.end()) == names.end() &&
                        std::none_of(names.begin(), names.end(),
                                     [](const auto &n) { return n.empty(); }),
                    "Invalid recorder axis names");
        }
        auto slots = layout.motor_slots;
        std::sort(slots.begin(), slots.end());
        require(std::adjacent_find(slots.begin(), slots.end()) == slots.end() &&
                    std::all_of(slots.begin(), slots.end(),
                                [&](auto s) { return s < layout.backend_slots; }),
                "Invalid recorder motor slots");
        require(!config.enabled || !(config.data & Hands) || layout.hands,
                "Hand whitelist conflicts with robot capabilities");
        for (const auto &rule : config.trigger_exemptions) {
            require(rule.event == Event::HardwareOffline || rule.event == Event::MotorFault,
                    "Exemptions must specify hardware_offline or motor_fault");
            require(rule.component == Component::Motor || rule.component == Component::HeadServo,
                    "Invalid exemption component");
            for (const auto &name : rule.names)
                require(rule.component == Component::Motor
                            ? std::find(layout.motor_names.begin(), layout.motor_names.end(),
                                        name) != layout.motor_names.end()
                            : (name == "head_servo_0" || name == "head_servo_1"),
                        "Unknown hardware name in trigger exemption");
        }
        if (!config.enabled)
            return;
        history_bucket.fill(uint64_t(-1));
        strides.fill(1024);
        strides[0] = 1152 + 40 * (layout.motor_names.size() + layout.backend_slots) +
                     4 * layout.backend_slots;
        strides[1] =
            1792 + // Fixed feedback fields, TLV headers and envelope (including hands/forces).
            ((config.data & Motors) ? 60 * layout.raw_motor_count + 24 * layout.joint_names.size()
                                    : 0) +
            24 * layout.encoder_names.size();
        strides[3] = 4096;
        rates = {config.detailed_hz, config.detailed_hz, 200, config.application_hz, 200, 10};
        for (size_t i = 0; i < sources; ++i) {
            require(strides[i] >= envelope + 4 &&
                        strides[i] <= sizeof(Packet::payload) + envelope + 4,
                    "Recorder stride exceeds format limit");
            memory += sizeof(Packet) * (config.queue_frames + 64);
            memory += strides[i] * (size_t(rates[i]) * config.pre_seconds + 65);
            memory += config.max_pending * strides[i] *
                      (size_t(rates[i]) * (config.pre_seconds + config.post_seconds) +
                       std::max<size_t>(config.queue_frames, 65));
        }
        memory +=
            1024 * 1024; // Metadata, control objects and bounded serialization scratch reserve.
        if (memory > config.max_memory_bytes)
            throw std::invalid_argument("Flight recorder cache requires " + std::to_string(memory) +
                                        " bytes, budget is " +
                                        std::to_string(config.max_memory_bytes) +
                                        "; reduce rate/window or increase budget");
        for (size_t i = 0; i < sources; ++i) {
            queues[i] = std::make_unique<Queue>(config.queue_frames, sequences[i]);
            event_queues[i] = std::make_unique<Queue>(64, sequences[i]);
            history[i] =
                std::make_unique<Ring>(strides[i], size_t(rates[i]) * config.pre_seconds + 65);
        }
        incidents.resize(config.max_pending);
        for (auto &incident : incidents)
            for (size_t i = 0; i < sources; ++i)
                incident.records[i] = std::make_unique<Ring>(
                    strides[i], size_t(rates[i]) * (config.pre_seconds + config.post_seconds) +
                                    std::max<size_t>(config.queue_frames, 65));
        ready = true;
        worker = std::thread([this] { loop(); });
    }
    ~Impl() { finish(); }
    void finish() noexcept {
        ready = false;
        stop = true;
#ifdef EIR_FLIGHT_TEST_HOOKS
        flightTestHook(2);
#endif
        if (worker.joinable())
            worker.join();
    }
    bool header(int fd) {
        uint8_t b[64]{};
        std::memcpy(b, "EIRFLT01", 8);
        put(b + 8, 1, 4);
        put(b + 12, 64, 4);
        put(b + 16, begin_system, 8);
        put(b + 24, begin_mono, 8);
        return writeAll(fd, b, 64);
    }
    void failed() {
        ++failures;
        healthy = false;
    }
    void metadata() {
        std::ofstream f(run / "metadata.json");
        f << "{\"schema\":1,\"family\":" << quote(layout.family)
          << ",\"robot\":" << quote(layout.robot) << ",\"backend\":" << quote(layout.backend)
          << ",\"run_id\":" << quote(layout.run_id) << ",\"software\":" << quote(layout.software)
          << ",\"config_digest\":" << quote(layout.config_digest)
          << ",\"data_mask\":" << config.data << ",\"trigger_mask\":" << config.triggers
          << ",\"detailed_hz\":" << config.detailed_hz
          << ",\"application_hz\":" << config.application_hz
          << ",\"continuous_hz\":" << config.continuous_hz
          << ",\"pre_seconds\":" << config.pre_seconds
          << ",\"post_seconds\":" << config.post_seconds << ",\"memory_bytes\":" << memory
          << ",\"joint_names\":";
        strings(f, layout.joint_names);
        f << ",\"motor_names\":";
        strings(f, layout.motor_names);
        f << ",\"encoder_names\":";
        strings(f, layout.encoder_names);
        f << ",\"controllers\":";
        strings(f, layout.controllers);
        f << ",\"raw_motor_count\":" << layout.raw_motor_count
          << ",\"backend_slots\":" << layout.backend_slots;
        f << ",\"trigger_exemptions\":[";
        bool first = true;
        for (const auto &rule : config.trigger_exemptions) {
            if (!first)
                f << ',';
            first = false;
            f << "{\"event\":" << quote(Recorder::eventName(rule.event))
              << ",\"component\":" << unsigned(rule.component) << ",\"names\":";
            strings(f, rule.names);
            f << '}';
        }
        f << "],\"motor_slots\":[";
        for (size_t i = 0; i < layout.motor_slots.size(); ++i) {
            if (i)
                f << ',';
            f << layout.motor_slots[i];
        }
        f << "],\"continuous_joints\":[";
        for (size_t i = 0; i < layout.continuous_joints.size(); ++i) {
            if (i)
                f << ',';
            f << (layout.continuous_joints[i] ? "true" : "false");
        }
        f << "],\"quaternion_order\":\"wxyz\",\"axis_units\":\"rad,rad/s,Nm\",\"hand_unit\":\"SDK "
             "native feedback; command percent\"}";
        f.close();
        if (!f)
            throw std::runtime_error("Recorder metadata write failed");
        int fd = ::open((run / "metadata.json").c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        const bool durable = fd >= 0 && ::fsync(fd) == 0;
        if (fd >= 0)
            ::close(fd);
        if (!durable)
            throw std::runtime_error("Recorder metadata sync failed");
    }
    bool owned(const fs::path &path) {
        if (fs::is_symlink(path) || !fs::is_directory(path) ||
            path.filename().string().rfind("eirflight_", 0) != 0)
            return false;
        if (fs::is_symlink(path / ".eir_flight"))
            return false;
        std::ifstream marker(path / ".eir_flight");
        std::string line;
        std::getline(marker, line);
        return line == "EIR_FLIGHT_1";
    }
    uint64_t bytes(const fs::path &p) {
        uint64_t n = 0;
        for (const auto &e : fs::directory_iterator(p))
            if (e.is_regular_file() && !e.is_symlink())
                n += e.file_size();
        return n;
    }
    bool room(uint64_t additional) {
        std::vector<fs::path> old;
        uint64_t total = 0;
        for (const auto &e : fs::directory_iterator(config.directory))
            if (owned(e.path())) {
                total += bytes(e.path());
                if (e.path() != run)
                    old.push_back(e.path());
            }
        std::sort(old.begin(), old.end());
        auto enough = [&] {
            auto available = fs::space(config.directory).available;
            return additional <= config.max_storage_bytes &&
                   total <= config.max_storage_bytes - additional &&
                   available >= config.min_free_bytes &&
                   available - config.min_free_bytes >= additional;
        };
        for (const auto &path : old) {
            if (enough())
                break;
            bool safe = true;
            for (const auto &e : fs::directory_iterator(path)) {
                auto name = e.path().filename().string();
                safe &= e.is_regular_file() && !e.is_symlink() && artifactName(name);
            }
            if (safe) {
                const auto n = bytes(path);
                fs::remove_all(path);
                total -= n;
            }
        }
        // In a long-lived run, retire only completed incidents with their manifest.
        if (!enough()) {
            std::vector<fs::path> completed_files;
            for (const auto &e : fs::directory_iterator(run))
                if (!e.is_symlink() && e.is_regular_file() &&
                    e.path().filename().string().rfind("incident_", 0) == 0 &&
                    e.path().extension() == ".json" && artifactName(e.path().filename().string()))
                    completed_files.push_back(e.path());
            std::sort(completed_files.begin(), completed_files.end());
            for (const auto &manifest : completed_files) {
                if (enough())
                    break;
                auto data = manifest;
                data.replace_extension(".bin");
                uint64_t n = fs::file_size(manifest);
                if (fs::is_regular_file(data) && !fs::is_symlink(data)) {
                    n += fs::file_size(data);
                    fs::remove(data);
                }
                auto snapshot = manifest.parent_path() /
                                ("snapshot_" + manifest.stem().string().substr(9) + ".bin");
                if (fs::is_regular_file(snapshot) && !fs::is_symlink(snapshot)) {
                    n += fs::file_size(snapshot);
                    fs::remove(snapshot);
                }
                fs::remove(manifest);
                total -= n;
            }
        }
        if (!enough()) {
            ++quota;
            healthy = false;
            return false;
        }
        return true;
    }
    void syncDirectory() {
        int fd = ::open(run.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
        if (fd < 0 || ::fsync(fd))
            failed();
        if (fd >= 0)
            ::close(fd);
    }
    bool save(Incident &incident, bool final) {
        const auto stem =
            "incident_" + std::to_string(incident.system) + "_" + std::to_string(incident.id);
        uint64_t estimate = 64;
        for (const auto &ring : incident.records)
            ring->each([&](const uint8_t *, size_t n) { estimate += n; });
        if (!room(estimate + 4096))
            return false;
        int fd = ::open((run / (stem + ".bin.tmp")).c_str(),
                        O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600);
        bool ok = fd >= 0 && header(fd);
        for (const auto &ring : incident.records)
            ring->each([&](const uint8_t *p, size_t n) {
                if (ok)
                    ok = writeAll(fd, p, n);
            });
        if (fd >= 0) {
            if (::fsync(fd))
                ok = false;
            ::close(fd);
        }
        if (!ok) {
            failed();
            return false;
        }
        fs::rename(run / (stem + ".bin.tmp"), run / (stem + ".bin"));
        syncDirectory();
        if (final) {
            const bool partial = stop.load() && Recorder::monotonicNow() < incident.until;
            const bool missing =
                dropped.load() != 0; // Conservative: prior history may contain losses.
            std::ofstream manifest(run / (stem + ".json"));
            manifest << "{\"root\":" << quote(Recorder::eventName(incident.root))
                     << ",\"trigger_monotonic_ns\":" << incident.trigger << ",\"window_start_ns\":"
                     << incident.trigger - int64_t(config.pre_seconds) * 1000000000LL
                     << ",\"window_end_ns\":" << incident.until
                     << ",\"incomplete\":" << (partial || missing ? "true" : "false")
                     << ",\"ended_on_shutdown\":" << (partial ? "true" : "false")
                     << ",\"drops_at_trigger\":" << incident.drops
                     << ",\"drops_at_finish\":" << dropped.load()
                     << ",\"history_may_be_short_at_startup\":true}";
            manifest.close();
            if (!manifest) {
                failed();
                return false;
            }
            int mf = ::open((run / (stem + ".json")).c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
            if (mf < 0 || ::fsync(mf)) {
                if (mf >= 0)
                    ::close(mf);
                failed();
                return false;
            }
            ::close(mf);
            syncDirectory();
            if (partial || missing)
                ++incomplete;
            else
                ++completed;
        }
        return true;
    }
    bool exempt(const Packet &packet) {
        for (const auto &rule : config.trigger_exemptions)
            if (rule.event == packet.event && rule.component == packet.component) {
                if (rule.names.empty())
                    return true;
                const auto name = packet.component == Component::Motor &&
                                          packet.component_index >= 0 &&
                                          size_t(packet.component_index) < layout.motor_names.size()
                                      ? layout.motor_names[packet.component_index]
                                      : "head_servo_" + std::to_string(packet.component_index);
                if (std::find(rule.names.begin(), rule.names.end(), name) != rule.names.end())
                    return true;
            }
        return false;
    }
    void trigger(const Packet &packet, Source source) {
        if (exempt(packet)) {
            ++exempted;
            return;
        }
        for (auto &i : incidents)
            if (i.id && packet.monotonic_ns >= i.trigger &&
                packet.monotonic_ns <=
                    i.trigger + int64_t(std::max(config.post_seconds, config.cooldown_seconds)) *
                                    1000000000LL)
                return;
        auto it = std::find_if(incidents.begin(), incidents.end(),
                               [](const auto &i) { return !i.active; });
        if (it == incidents.end()) {
            ++dropped;
            return;
        }
        auto &i = *it;
        i.active = true;
        i.id = ++next_incident;
        i.drops = dropped.load();
        i.root = packet.event;
        i.trigger = packet.monotonic_ns;
        i.system = packet.system_ns;
        i.until = i.trigger + int64_t(config.post_seconds) * 1000000000LL;
        for (size_t s = 0; s < sources; ++s) {
            i.records[s]->clear();
            history[s]->each([&](const uint8_t *b, size_t n) {
                const auto stamp = int64_t(get(b + 24, 8));
                if (stamp >= i.trigger - int64_t(config.pre_seconds) * 1000000000LL &&
                    stamp <= i.until)
                    i.records[s]->push(b, n);
            });
        }
        // Make the small trigger durable before serializing the much larger history.
        const auto snapshot =
            run / ("snapshot_" + std::to_string(i.system) + "_" + std::to_string(i.id) + ".bin");
        std::array<uint8_t, 12348> b{};
        const auto n = encode(packet, source, b.data());
        if (room(n + 64)) {
            int fd = ::open(snapshot.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                            0600);
            bool ok = fd >= 0 && header(fd) && writeAll(fd, b.data(), n);
            if (fd >= 0) {
                if (::fsync(fd))
                    ok = false;
                ::close(fd);
            }
            if (!ok)
                failed();
            else
                syncDirectory();
        }
        save(i, false); // Save evidence now, before waiting for the post window.
    }
    void writeEvent(const uint8_t *data, size_t n) {
        if (event_failed)
            return;
        if (event_bytes + n > config.continuous_file_bytes) {
            if (event_fd >= 0) {
                if (::fdatasync(event_fd))
                    failed();
                ::close(event_fd);
                event_fd = -1;
            }
            fs::remove(run / "events.bin.1");
            fs::rename(run / "events.bin", run / "events.bin.1");
            event_bytes = 0;
        }
        if (!room(n + 64))
            return;
        if (event_fd < 0) {
            event_fd = ::open((run / "events.bin").c_str(),
                              O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600);
            if (event_fd < 0 || !header(event_fd)) {
                event_failed = true;
                failed();
                return;
            }
            event_bytes = 64;
            syncDirectory();
        }
        if (!writeAll(event_fd, data, n)) {
            event_failed = true;
            failed();
        } else
            event_bytes += n;
    }
    void consume(Source source, Packet packet) {
        const auto s = static_cast<size_t>(source);
        std::array<uint8_t, 12348> data{};
        const auto n = encode(packet, source, data.data());
        if (n + 4 > strides[s]) {
            ++dropped;
            return;
        }
        if (packet.sequence > last_sequence[s] &&
            ((last_system[s] && packet.system_ns < last_system[s]) ||
             (last_mono[s] && packet.monotonic_ns < last_mono[s]))) {
            packet.flags |= 4;
            if (packet.event == Event::None)
                packet.event = Event::ClockJump;
            encode(packet, source, data.data());
        }
        if (packet.sequence >= last_sequence[s]) {
            last_system[s] = packet.system_ns;
            last_mono[s] = packet.monotonic_ns;
            last_sequence[s] = packet.sequence;
        }
        const bool triggered = (config.triggers & bit(packet.event)) != 0;
        const bool event_only = (packet.flags & Packet::EventOnly) && !triggered;
        if (packet.event != Event::None || (packet.flags & Packet::EventOnly))
            writeEvent(data.data(), n);
        // Queued events are not rate-limited, but history has a sampled budget.
        // Preserve every event in its journal; only sampled diagnostics and explicit
        // trigger evidence occupy history/incident slots. Triggers never lose a slot
        // merely because a diagnostic in the same bucket was already recorded.
        const auto bucket = uint64_t(packet.monotonic_ns) / (1000000000ULL / rates[s]);
        if (triggered || (!event_only && bucket != history_bucket[s])) {
            if (!triggered)
                history_bucket[s] = bucket;
            if (history[s]->count == history[s]->capacity &&
                history[s]->oldestMono() >
                    packet.monotonic_ns - int64_t(config.pre_seconds) * 1000000000LL)
                ++dropped;
            history[s]->push(data.data(), n);
            for (auto &i : incidents)
                if (i.active && packet.monotonic_ns <= i.until &&
                    packet.monotonic_ns >= i.trigger - int64_t(config.pre_seconds) * 1000000000LL) {
                    if (i.records[s]->count == i.records[s]->capacity)
                        ++dropped;
                    else
                        i.records[s]->push(data.data(), n);
                }
        }
        if (source == Source::Feedback && !event_only)
            latest_feedback = packet;
        if (source == Source::Application && !event_only)
            latest_application = packet;
        if (triggered)
            trigger(packet, source);
    }

    void continuous() {
        if (continuous_failed)
            return;
        const auto now = Recorder::monotonicNow();
        if (now < next_continuous)
            return;
        next_continuous = now + 1000000000LL / config.continuous_hz;
        Packet p;
        p.system_ns = Recorder::systemNow();
        p.monotonic_ns = now;
        p.correlation = latest_feedback.correlation;
        p.flags = latest_feedback.flags;
        double ages[] = {
            latest_feedback.monotonic_ns ? double(now - latest_feedback.monotonic_ns) / 1e6 : -1,
            latest_application.monotonic_ns ? double(now - latest_application.monotonic_ns) / 1e6
                                            : -1};
        p.doubles(900, ages, 2);
        for (const auto *input : {&latest_feedback, &latest_application})
            for (size_t at = 0; at + 8 <= input->used;) {
                const auto tag = get(input->payload.data() + at, 2),
                           type = get(input->payload.data() + at + 2, 2),
                           count = get(input->payload.data() + at + 4, 4);
                const size_t bytes = 8 + count * (type == 1 ? 8 : 4);
                if (at + bytes > input->used)
                    break;
                if (tag == 200 || tag == 201 || tag == 220 || tag == 500 || tag == 501) {
                    if (p.used + bytes <= p.payload.size()) {
                        std::memcpy(p.payload.data() + p.used, input->payload.data() + at, bytes);
                        p.used += bytes;
                    }
                }
                at += bytes;
            }
        std::array<uint8_t, 12348> b{};
        const auto n = encode(p, Source::Manual, b.data());
        if (continuous_size + n > config.continuous_file_bytes) {
            if (continuous_fd >= 0) {
                if (::fdatasync(continuous_fd))
                    failed();
                ::close(continuous_fd);
                continuous_fd = -1;
            }
            fs::remove(run / "continuous.bin.1");
            fs::rename(run / "continuous.bin", run / "continuous.bin.1");
            continuous_size = 0;
        }
        if (!room(n + 64))
            return;
        if (continuous_fd < 0) {
            continuous_fd = ::open((run / "continuous.bin").c_str(),
                                   O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600);
            if (continuous_fd < 0 || !header(continuous_fd)) {
                continuous_failed = true;
                failed();
                return;
            }
            continuous_size = 64;
            syncDirectory();
        }
        if (!writeAll(continuous_fd, b.data(), n)) {
            continuous_failed = true;
            failed();
            ::close(continuous_fd);
            continuous_fd = -1;
            return;
        }
        continuous_size += n;
        if (now - last_sync >= int64_t(config.sync_ms) * 1000000LL) {
            if (::fdatasync(continuous_fd))
                failed();
            last_sync = now;
        }
    }
    void loop() noexcept {
        try {
            if (fs::is_symlink(config.directory))
                throw std::runtime_error("Recorder output root cannot be a symlink");
            fs::create_directories(config.directory);
            lock_fd = ::open((fs::path(config.directory) / ".eir_flight.lock").c_str(),
                             O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
            if (lock_fd < 0 || ::flock(lock_fd, LOCK_EX | LOCK_NB))
                throw std::runtime_error("Recorder directory already in use");
            static std::atomic<uint64_t> serial{0};
            run = fs::path(config.directory) /
                  ("eirflight_" + std::to_string(begin_system) + "_" + std::to_string(::getpid()) +
                   "_" + std::to_string(serial++));
            fs::create_directory(run);
            fs::permissions(run, fs::perms::owner_all);
            {
                std::ofstream marker(run / ".eir_flight");
                marker << "EIR_FLIGHT_1\n";
                if (!marker)
                    throw std::runtime_error("Recorder marker write failed");
            }
            if (!room(4096))
                throw std::runtime_error("Recorder storage unavailable");
            metadata();
            healthy = true;
            do {
                // finish() is called after producers stop. Observe it BEFORE draining:
                // a stop arriving during this pass must cause one final complete pass.
                const bool finishing = stop.load();
                Packet p;
                for (size_t s = 0; s < sources; ++s)
                    for (unsigned n = 0; n < config.queue_frames && queues[s]->pop(p); ++n)
                        consume(static_cast<Source>(s), p);
                for (size_t s = 0; s < sources; ++s)
                    for (unsigned n = 0; n < 64 && event_queues[s]->pop(p); ++n)
                        consume(static_cast<Source>(s), p);
                if (manual.exchange(false)) {
                    Packet request;
                    request.system_ns = Recorder::systemNow();
                    request.monotonic_ns = Recorder::monotonicNow();
                    request.event = Event::ManualSave;
                    request.sequence = sequences[static_cast<size_t>(Source::Manual)].fetch_add(1);
                    consume(Source::Manual, request);
                }
#ifdef EIR_FLIGHT_TEST_HOOKS
                flightTestHook(1);
#endif
                const auto now = Recorder::monotonicNow();
                if (event_fd >= 0 && now - event_sync >= int64_t(config.sync_ms) * 1000000LL) {
                    if (::fdatasync(event_fd))
                        failed();
                    event_sync = now;
                }
                for (auto &i : incidents)
                    if (i.active && (finishing || now >= i.until)) {
                        if (!save(i, true))
                            ++incomplete;
                        i.active = false;
                    }
                if (finishing)
                    break;
                if (!stop)
                    continuous();
                if (!stop)
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } while (true);
            std::ofstream stats(run / "statistics.json");
            stats << "{\"accepted\":" << accepted.load() << ",\"dropped\":" << dropped.load()
                  << ",\"completed\":" << completed.load()
                  << ",\"incomplete\":" << incomplete.load()
                  << ",\"write_failures\":" << failures.load()
                  << ",\"quota_rejections\":" << quota.load() << ",\"exempted\":" << exempted.load()
                  << ",\"max_submit_ns\":" << max_submit.load() << "}";
        } catch (...) {
            failed();
        }
        ready = false;
        if (event_fd >= 0) {
            if (::fdatasync(event_fd))
                failed();
            ::close(event_fd);
            event_fd = -1;
        }
        if (continuous_fd >= 0) {
            if (::fdatasync(continuous_fd))
                failed();
            ::close(continuous_fd);
            continuous_fd = -1;
        }
        if (lock_fd >= 0) {
            ::flock(lock_fd, LOCK_UN);
            ::close(lock_fd);
            lock_fd = -1;
        }
    }
};
Recorder::Recorder(Config c, Layout l)
    : impl_(std::make_unique<Impl>(std::move(c), std::move(l))) {}
Recorder::~Recorder() = default;
void Recorder::finish() noexcept { impl_->finish(); }
bool Recorder::submit(Source source, Packet p) noexcept {
    auto &i = *impl_;
    const auto s = static_cast<size_t>(source);
    if (!i.ready || s >= sources)
        return false;
    if (p.used > p.payload.size() ||
        static_cast<unsigned>(p.event) >= static_cast<unsigned>(Event::Count)) {
        ++i.dropped;
        return false;
    }
    const auto begin = monotonicNow();
    if (!p.monotonic_ns)
        p.monotonic_ns = begin;
    if (!p.system_ns)
        p.system_ns = systemNow();
    if (p.component != Component::None) {
        uint32_t component[] = {static_cast<uint32_t>(p.component),
                                static_cast<uint32_t>(p.component_index)};
        p.integers(80, component, 2);
    }
    const bool priority = p.event != Event::None && p.event != Event::Warning &&
                          p.event != Event::ModeChanged && p.event != Event::Started &&
                          p.event != Event::Stopped;
    const bool ok = (priority ? i.event_queues[s] : i.queues[s])->push(p, i.rates[s]);
    if (ok)
        ++i.accepted;
    else
        ++i.dropped;
    const auto elapsed = uint64_t(monotonicNow() - begin);
    auto old = i.max_submit.load();
    if (elapsed > old)
        i.max_submit.compare_exchange_strong(old, elapsed);
    return ok;
}
bool Recorder::requestSave() noexcept {
    auto &i = *impl_;
    bool expected = false;
    return i.ready && i.healthy && (i.config.triggers & bit(Event::ManualSave)) &&
           i.manual.compare_exchange_strong(expected, true);
}
Statistics Recorder::statistics() const noexcept {
    auto &i = *impl_;
    return {i.accepted.load(), i.dropped.load(), i.completed.load(), i.incomplete.load(),
            i.failures.load(), i.quota.load(),   i.memory,           i.max_submit.load(),
            i.config.enabled,  i.healthy.load(), i.exempted.load()};
}
} // namespace eir::loco::flight
