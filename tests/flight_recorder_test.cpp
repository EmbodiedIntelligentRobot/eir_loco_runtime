#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <eir_loco_runtime/flight_recorder.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <mutex>
#include <thread>
#include <unistd.h>
using namespace eir::loco::flight;
#ifdef EIR_FLIGHT_TEST_HOOKS
namespace {
struct ShutdownBarrier {
    std::mutex mutex;
    std::condition_variable cv;
    bool armed{}, drained{}, stopped{}, released{};
} shutdown_barrier;
struct ArmShutdownBarrier {
    ArmShutdownBarrier() {
        std::lock_guard lock(shutdown_barrier.mutex);
        shutdown_barrier.armed = true;
        shutdown_barrier.drained = shutdown_barrier.stopped = shutdown_barrier.released = false;
    }
    void release() {
        std::lock_guard lock(shutdown_barrier.mutex);
        shutdown_barrier.released = true;
        shutdown_barrier.cv.notify_all();
    }
    ~ArmShutdownBarrier() {
        release();
        std::lock_guard lock(shutdown_barrier.mutex);
        shutdown_barrier.armed = false;
    }
};
} // namespace
namespace eir::loco::flight {
void flightTestHook(unsigned stage) {
    std::unique_lock lock(shutdown_barrier.mutex);
    if (!shutdown_barrier.armed)
        return;
    if (stage == 2) {
        shutdown_barrier.stopped = true;
        shutdown_barrier.cv.notify_all();
    } else if (stage == 1 && !shutdown_barrier.drained) {
        shutdown_barrier.drained = true;
        shutdown_barrier.cv.notify_all();
        shutdown_barrier.cv.wait(lock, [] { return shutdown_barrier.released; });
    }
}
} // namespace eir::loco::flight
#endif
namespace {
namespace fs = std::filesystem;
using namespace std::chrono_literals;
struct Temp {
    fs::path path = fs::temp_directory_path() / ("eir-flight-test-" + std::to_string(getpid()) +
                                                 "-" + std::to_string(Recorder::monotonicNow()));
    ~Temp() { fs::remove_all(path); }
};
Layout profile(const std::string &robot, size_t n, bool hardware = false) {
    Layout l;
    l.family = robot.rfind("eqr", 0) == 0 ? "eqr" : "aiq";
    l.robot = robot;
    l.backend = hardware ? "hardware" : "simulation";
    l.backend_slots = hardware ? (l.family == "eqr" ? 16 : 32) : n;
    l.raw_motor_count = hardware ? l.backend_slots : 2 * n;
    for (size_t i = 0; i < n; ++i) {
        l.joint_names.push_back("joint_" + std::to_string(i));
        l.motor_names.push_back("motor_" + std::to_string(i));
        l.motor_slots.push_back(hardware && robot == "eqr11" ? (i / 3) * 4 + i % 3 : i);
        l.continuous_joints.push_back(robot == "eqr11w" && i % 4 == 3);
    }
    l.hands = robot == "pro";
    if (l.family == "aiq")
        l.encoder_names = {"pitch", "roll"};
    return l;
}
Config settings(const fs::path &directory) {
    Config c;
    c.enabled = true;
    c.directory = directory;
    c.pre_seconds = 1;
    c.post_seconds = 0;
    c.detailed_hz = 100;
    c.continuous_hz = 20;
    c.min_free_bytes = 0;
    return c;
}
bool wait(const std::function<bool()> &predicate) {
    for (int i = 0; i < 500; ++i) {
        if (predicate())
            return true;
        std::this_thread::sleep_for(2ms);
    }
    return false;
}
uint64_t little(const uint8_t *p, size_t n) {
    uint64_t x = 0;
    for (size_t i = 0; i < n; ++i)
        x |= uint64_t(p[i]) << (i * 8);
    return x;
}
std::vector<std::vector<uint8_t>> readRecords(const fs::path &file) {
    std::ifstream in(file, std::ios::binary);
    std::array<char, 64> header{};
    in.read(header.data(), 64);
    EXPECT_EQ(std::string(header.data(), 8), "EIRFLT01");
    std::vector<std::vector<uint8_t>> result;
    std::array<uint8_t, 8> prefix{};
    while (in.read(reinterpret_cast<char *>(prefix.data()), 8)) {
        const auto n = little(prefix.data(), 4);
        if (n > 12340 || n < 52) {
            ADD_FAILURE();
            break;
        }
        std::vector<uint8_t> body(n);
        in.read(reinterpret_cast<char *>(body.data()), n);
        EXPECT_TRUE(in);
        uint32_t h = 2166136261;
        for (auto v : body)
            h = (h ^ v) * 16777619;
        EXPECT_EQ(h, little(prefix.data() + 4, 4));
        result.push_back(std::move(body));
    }
    return result;
}
std::vector<fs::path> incidents(const fs::path &dir) {
    std::vector<fs::path> files;
    if (!fs::exists(dir))
        return files;
    for (const auto &e : fs::recursive_directory_iterator(dir))
        if (e.path().filename().string().rfind("incident_", 0) == 0 &&
            e.path().extension() == ".bin")
            files.push_back(e.path());
    return files;
}
TEST(FlightRecorder, AllExistingRobotLayoutsFitStandardBudget) {
    for (auto [robot, n] : {std::pair{"pro", 32u}, {"sport", 23u}, {"eqr11", 12u}, {"eqr11w", 16u}})
        for (bool hardware : {false, true}) {
            Temp tmp;
            Config c;
            c.enabled = true;
            c.directory = tmp.path;
            c.min_free_bytes = 0;
            Recorder r(c, profile(robot, n, hardware));
            EXPECT_LT(r.statistics().memory_bytes, c.max_memory_bytes);
            ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
            r.finish();
            EXPECT_EQ(r.statistics().write_failures, 0u);
        }
}
TEST(FlightRecorder, FullPro32AxisPacketsFitReservedStrides) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.data = Commands | Motors | Imu | Encoders | Bms | Hands | Forces | Remote | Alarms |
             Application | ActuatorTorque | Head;
    Recorder r(c, profile("pro", 32));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    std::array<double, 64> values{};
    std::array<uint32_t, 64> codes{};
    Packet command, feedback;
    command.doubles(10, values.data(), 2);
    command.integers(11, codes.data(), 1);
    for (auto tag : {20, 21, 22, 23, 24, 30, 31, 32, 33, 34})
        command.doubles(tag, values.data(), 32);
    command.integers(35, codes.data(), 32);
    for (auto tag : {40, 41, 42})
        command.doubles(tag, values.data(), 12);
    command.doubles(50, values.data(), 64);
    command.integers(51, codes.data(), 2);
    feedback.integers(90, codes.data(), 1);
    feedback.integers(99, codes.data(), 9);
    for (auto tag : {100, 101, 102, 103, 104, 105})
        feedback.doubles(tag, values.data(), 64);
    for (auto tag : {110, 111, 112})
        feedback.integers(tag, codes.data(), 64);
    for (auto tag : {120, 121, 122})
        feedback.doubles(tag, values.data(), 32);
    feedback.doubles(200, values.data(), 4);
    for (auto tag : {201, 202})
        feedback.doubles(tag, values.data(), 3);
    for (auto tag : {210, 211, 270})
        feedback.doubles(tag, values.data(), 2);
    for (auto tag : {212, 271})
        feedback.integers(tag, codes.data(), 2);
    feedback.doubles(220, values.data(), 4);
    feedback.integers(221, codes.data(), 1);
    for (auto tag : {230, 231, 232})
        feedback.doubles(tag, values.data(), 12);
    feedback.integers(233, codes.data(), 12);
    for (auto tag : {240, 241, 242, 243, 244, 245})
        feedback.doubles(tag, values.data(), 4);
    feedback.integers(246, codes.data(), 4);
    feedback.integers(250, codes.data(), 16);
    feedback.integers(260, codes.data(), 64);
    ASSERT_TRUE(r.submit(Source::Command, command));
    ASSERT_TRUE(r.submit(Source::Feedback, feedback));
    ASSERT_TRUE(r.requestSave());
    r.finish();
    EXPECT_EQ(r.statistics().dropped, 0u);
    EXPECT_EQ(r.statistics().write_failures, 0u);
    ASSERT_EQ(incidents(tmp.path).size(), 1u);
    EXPECT_EQ(readRecords(incidents(tmp.path).front()).size(), 3u);
}
TEST(FlightRecorder, SustainedWarningsAreSampledAndModeChangesRemainEvents) {
    Temp tmp;
    Recorder r(settings(tmp.path), profile("sport", 23));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet p;
    p.monotonic_ns = Recorder::monotonicNow();
    p.event = Event::Warning;
    for (int i = 0; i < 1000; ++i)
        ASSERT_TRUE(r.submit(Source::Application, p));
    p.event = Event::ModeChanged;
    ASSERT_TRUE(r.submit(Source::Application, p));
    ASSERT_TRUE(r.requestSave());
    r.finish();
    ASSERT_EQ(incidents(tmp.path).size(), 1u);
    EXPECT_EQ(readRecords(incidents(tmp.path).front()).size(), 2u);
    for (const auto &e : fs::recursive_directory_iterator(tmp.path))
        if (e.path().filename() == "events.bin")
            EXPECT_EQ(readRecords(e.path()).size(), 3u); // warning, mode change, manual save
    EXPECT_EQ(r.statistics().dropped, 0u);
}
TEST(FlightRecorder, ExtraEventsDoNotOverwriteSampledTenSecondHistory) {
    for (bool event_only : {false, true}) {
        Temp tmp;
        Config c;
        c.enabled = true;
        c.directory = tmp.path;
        c.min_free_bytes = 0;
        c.post_seconds = 0;
        Recorder r(c, profile("sport", 23));
        ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
        const auto start = Recorder::monotonicNow() - 30000000000LL;
        std::array<double, 250> values{};
        for (unsigned i = 0; i < 1200; ++i) {
            Packet sample;
            sample.monotonic_ns = start + i * 20000000LL;
            sample.correlation = i + 1;
            sample.doubles(999, values.data(), values.size());
            ASSERT_TRUE(r.submit(Source::Application, sample));
            if (i >= 400 && i < 700) {
                Packet event;
                event.monotonic_ns = sample.monotonic_ns + 1000000;
                event.event = Event::ModeChanged;
                if (event_only)
                    event.flags |= Packet::EventOnly;
                ASSERT_TRUE(r.submit(Source::Application, event));
            }
            if (i % 10 == 9)
                std::this_thread::sleep_for(3ms);
        }
        Packet fault;
        fault.monotonic_ns = start + 1200 * 20000000LL;
        fault.event = Event::ApplicationFault;
        // EventOnly never suppresses explicitly triggering evidence.
        fault.flags = Packet::EventOnly;
        ASSERT_TRUE(r.submit(Source::Application, fault));
        r.finish();
        EXPECT_EQ(r.statistics().dropped, 0u);
        EXPECT_EQ(r.statistics().write_failures, 0u);
        EXPECT_EQ(r.statistics().completed, 1u);
        ASSERT_EQ(incidents(tmp.path).size(), 1u);
        const auto records = readRecords(incidents(tmp.path).front());
        if (event_only) {
            EXPECT_EQ(std::count_if(records.begin(), records.end(),
                                    [](const auto &b) { return little(b.data() + 40, 8) != 0; }),
                      500); // All 50 Hz diagnostics in the last ten seconds survive.
        }
        for (const auto &e : fs::recursive_directory_iterator(tmp.path))
            if (e.path().filename() == "events.bin")
                EXPECT_EQ(readRecords(e.path()).size(), 301u);
    }
}
TEST(FlightRecorder, RequestEventsPreserveEveryIdentityAndNeverReplaceSummary) {
    Temp tmp;
    Recorder r(settings(tmp.path), profile("pro", 32));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet diagnostic;
    diagnostic.monotonic_ns = Recorder::monotonicNow();
    uint32_t codes[] = {28, 0, 0};
    diagnostic.integers(500, codes, 3);
    ASSERT_TRUE(r.submit(Source::Application, diagnostic));
    for (uint32_t i = 1; i <= 100; ++i) {
        Packet request;
        request.flags = Packet::EventOnly;
        request.monotonic_ns = diagnostic.monotonic_ns;
        request.correlation = i;
        request.event = i % 2 ? Event::None : Event::Warning;
        ASSERT_TRUE(r.submit(Source::Application, request));
    }
    ASSERT_TRUE(wait([&] {
        for (const auto &e : fs::recursive_directory_iterator(tmp.path))
            if (e.path().filename() == "continuous.bin" && fs::file_size(e.path()) >= 252)
                return true;
        return false;
    }));
    r.finish();
    EXPECT_EQ(r.statistics().dropped, 0u);
    for (const auto &e : fs::recursive_directory_iterator(tmp.path)) {
        if (e.path().filename() == "events.bin") {
            const auto records = readRecords(e.path());
            ASSERT_EQ(records.size(), 100u);
            for (unsigned i = 0; i < records.size(); ++i)
                EXPECT_EQ(little(records[i].data() + 40, 8), i + 1u);
        }
        if (e.path().filename() == "continuous.bin") {
            const auto records = readRecords(e.path());
            ASSERT_FALSE(records.empty());
            EXPECT_TRUE(std::any_of(records.begin(), records.end(), [](const auto &b) {
                return b.size() >= 96 && little(b.data() + 76, 2) == 500 &&
                       little(b.data() + 84, 4) == 28;
            }));
        }
    }
}
TEST(FlightRecorder, CompletedIncidentRetainsSampledPreAndPostWindowWithRequestTraffic) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.post_seconds = 1;
    Recorder r(c, profile("sport", 23));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    const auto start = Recorder::monotonicNow();
    const auto wall = std::chrono::steady_clock::now();
    for (unsigned i = 0; i < 120; ++i) {
        std::this_thread::sleep_until(wall + std::chrono::milliseconds(i * 20));
        Packet sample;
        sample.monotonic_ns = start + int64_t(i) * 20000000;
        sample.correlation = i + 1;
        sample.event = Event::None;
        ASSERT_TRUE(r.submit(Source::Application, sample));
        Packet request;
        request.monotonic_ns = sample.monotonic_ns;
        request.flags = Packet::EventOnly;
        ASSERT_TRUE(r.submit(Source::Application, request));
        if (i == 55)
            ASSERT_TRUE(r.requestSave());
    }
    ASSERT_TRUE(wait([&] { return r.statistics().completed == 1; }));
    r.finish();
    EXPECT_EQ(r.statistics().dropped, 0u);
    EXPECT_EQ(r.statistics().incomplete, 0u);
    const auto records = readRecords(incidents(tmp.path).front());
    std::vector<uint64_t> frames;
    for (const auto &b : records)
        if (little(b.data(), 2) == unsigned(Source::Application))
            frames.push_back(little(b.data() + 40, 8));
    ASSERT_GE(frames.size(), 98u);
    EXPECT_LE(frames.size(), 102u);
    for (size_t i = 1; i < frames.size(); ++i)
        EXPECT_EQ(frames[i], frames[i - 1] + 1);
    EXPECT_LT(frames.front(), 56u);
    EXPECT_GT(frames.back(), 56u);
}
TEST(FlightRecorder, RepeatTriggerWindowStartsAtFirstTrigger) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.post_seconds = 5;
    c.cooldown_seconds = 10;
    Recorder r(c, profile("sport", 23));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet p;
    p.event = Event::BackendFailure;
    // Synthetic past times exercise the boundary without waiting ten real seconds.
    const auto start = Recorder::monotonicNow() - 20000000000LL;
    p.monotonic_ns = start;
    ASSERT_TRUE(r.submit(Source::Watchdog, p));
    ASSERT_TRUE(wait([&] { return r.statistics().completed == 1; }));
    p.monotonic_ns = start + 10000000000LL;
    ASSERT_TRUE(r.submit(Source::Watchdog, p));
    p.monotonic_ns += 1;
    ASSERT_TRUE(r.submit(Source::Watchdog, p));
    ASSERT_TRUE(wait([&] { return r.statistics().completed == 2; }));
    r.finish();
    EXPECT_EQ(incidents(tmp.path).size(), 2u);
}
TEST(FlightRecorder, RejectsInvalidIdentityCapacityAndExemptionNames) {
    Temp tmp;
    auto c = settings(tmp.path);
    auto l = profile("pro", 31);
    c.max_memory_bytes = 1024;
    EXPECT_THROW(Recorder(c, l), std::invalid_argument);
    c = settings(tmp.path);
    l.motor_slots[0] = 999;
    EXPECT_THROW(Recorder(c, l), std::invalid_argument);
    l = profile("sport", 23);
    c.data |= Hands;
    EXPECT_THROW(Recorder(c, l), std::invalid_argument);
    c = settings(tmp.path);
    c.trigger_exemptions = {{Event::HardwareOffline, Component::Motor, {"nonexistent"}}};
    EXPECT_THROW(Recorder(c, l), std::invalid_argument);
    l = profile("pro", 31);
    l.robot.clear();
    EXPECT_THROW(Recorder(settings(tmp.path), l), std::invalid_argument);
}
TEST(FlightRecorder, HeadOfflineExemptionDoesNotSuppressBodyMotorOffline) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.trigger_exemptions = {{Event::HardwareOffline, Component::HeadServo, {}}};
    Recorder r(c, profile("pro", 31));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet p;
    p.event = Event::HardwareOffline;
    p.component = Component::HeadServo;
    p.component_index = 0;
    ASSERT_TRUE(r.submit(Source::Feedback, p));
    ASSERT_TRUE(wait([&] { return r.statistics().exempted == 1; }));
    EXPECT_TRUE(incidents(tmp.path).empty());
    p.component = Component::Motor;
    p.component_index = 2;
    ASSERT_TRUE(r.submit(Source::Feedback, p));
    ASSERT_TRUE(wait([&] { return r.statistics().completed == 1; }));
    r.finish();
    ASSERT_EQ(incidents(tmp.path).size(), 1u);
    const auto records = readRecords(incidents(tmp.path)[0]);
    EXPECT_TRUE(std::any_of(records.begin(), records.end(), [](const auto &b) {
        return little(b.data() + 2, 2) == unsigned(Event::HardwareOffline);
    }));
}
TEST(FlightRecorder, NamedMotorExemptionOnlyMatchesThatMotor) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.trigger_exemptions = {{Event::MotorFault, Component::Motor, {"motor_0"}}};
    Recorder r(c, profile("sport", 23));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet p;
    p.event = Event::MotorFault;
    p.component = Component::Motor;
    p.component_index = 0;
    r.submit(Source::Feedback, p);
    ASSERT_TRUE(wait([&] { return r.statistics().exempted == 1; }));
    p.component_index = 1;
    r.submit(Source::Feedback, p);
    ASSERT_TRUE(wait([&] { return r.statistics().completed == 1; }));
}
TEST(FlightRecorder, ManualCaptureRetainsOriginalTimestampAndNonfiniteEvidence) {
    Temp tmp;
    auto c = settings(tmp.path);
    Recorder r(c, profile("eqr11w", 16));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet p;
    p.monotonic_ns = Recorder::monotonicNow() - 100000000;
    p.system_ns = 123456;
    p.source_ns = 999;
    p.correlation = 42;
    double values[] = {-3.25, std::numeric_limits<double>::quiet_NaN()};
    p.doubles(20, values, 2);
    ASSERT_TRUE(r.submit(Source::Command, p));
    std::this_thread::sleep_for(10ms);
    ASSERT_TRUE(r.requestSave());
    ASSERT_TRUE(wait([&] { return r.statistics().completed == 1; }));
    r.finish();
    bool found = false;
    for (const auto &b : readRecords(incidents(tmp.path)[0]))
        if (little(b.data(), 2) == unsigned(Source::Command)) {
            found = true;
            EXPECT_EQ(little(b.data() + 8, 8), 123456u);
            EXPECT_EQ(little(b.data() + 24, 8), 999u);
            EXPECT_EQ(little(b.data() + 40, 8), 42u);
        }
    EXPECT_TRUE(found);
}
TEST(FlightRecorder, NormalShutdownDoesNotCreateIncidentAndConcurrentProducersAreBounded) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.queue_frames = 4;
    Recorder r(c, profile("pro", 31));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    std::array<std::thread, 4> workers;
    for (auto &worker : workers)
        worker = std::thread([&] {
            for (int i = 0; i < 1000; ++i) {
                Packet p;
                p.event = Event::ModeChanged;
                r.submit(Source::Application, p);
            }
        });
    for (auto &worker : workers)
        worker.join();
    r.finish();
    EXPECT_TRUE(incidents(tmp.path).empty());
    EXPECT_GT(r.statistics().dropped, 0u);
}
#ifdef EIR_FLIGHT_TEST_HOOKS
TEST(FlightRecorder, ShutdownDrainsDataTriggersAndManualRequestArrivingAfterScan) {
    for (bool hardware_trigger : {false, true}) {
        Temp tmp;
        auto c = settings(tmp.path);
        c.post_seconds = 2;
        Recorder r(c, profile("pro", 32));
        ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
        ArmShutdownBarrier barrier;
        {
            std::unique_lock lock(shutdown_barrier.mutex);
            ASSERT_TRUE(
                shutdown_barrier.cv.wait_for(lock, 2s, [] { return shutdown_barrier.drained; }));
        }
        // Worker has scanned every queue and the manual flag. Enqueue the tail now.
        Packet tail;
        tail.event = Event::ModeChanged;
        tail.correlation = 4242;
        EXPECT_TRUE(r.submit(Source::Application, tail));
        if (hardware_trigger) {
            Packet fault;
            fault.event = Event::MotorFault;
            fault.correlation = 4343;
            EXPECT_TRUE(r.submit(Source::Feedback, fault));
        }
        EXPECT_TRUE(r.requestSave());
        std::thread shutdown([&] { r.finish(); });
        bool stopped;
        {
            std::unique_lock lock(shutdown_barrier.mutex);
            stopped =
                shutdown_barrier.cv.wait_for(lock, 2s, [] { return shutdown_barrier.stopped; });
        }
        barrier.release();
        shutdown.join();
        ASSERT_TRUE(stopped);
        ASSERT_EQ(incidents(tmp.path).size(), 1u);
        const auto records = readRecords(incidents(tmp.path).front());
        EXPECT_EQ(records.size(), hardware_trigger ? 3u : 2u);
        EXPECT_TRUE(std::any_of(records.begin(), records.end(),
                                [](const auto &b) { return little(b.data() + 40, 8) == 4242; }));
        EXPECT_TRUE(std::any_of(records.begin(), records.end(), [](const auto &b) {
            return little(b.data() + 2, 2) == unsigned(Event::ManualSave);
        }));
        EXPECT_EQ(r.statistics().incomplete, 1u);
        EXPECT_EQ(r.statistics().dropped, 0u);
    }
}
#endif
TEST(FlightRecorder, TriggerSnapshotExistsBeforePostWindowAndShutdownMarksIncomplete) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.post_seconds = 2;
    Recorder r(c, profile("pro", 31));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet p;
    p.event = Event::Estop;
    r.submit(Source::Lifecycle, p);
    ASSERT_TRUE(wait([&] { return !incidents(tmp.path).empty(); }));
    EXPECT_EQ(r.statistics().completed, 0u);
    r.finish();
    EXPECT_EQ(r.statistics().incomplete, 1u);
    EXPECT_FALSE(readRecords(incidents(tmp.path)[0]).empty());
}
TEST(FlightRecorder, OutputFailureAndStorageQuotaDoNotThrowFromSubmit) {
    Temp tmp;
    fs::create_directories(tmp.path);
    std::ofstream(tmp.path / "file") << "x";
    auto c = settings(tmp.path / "file" / "child");
    Recorder bad(c, profile("pro", 31));
    ASSERT_TRUE(wait([&] { return bad.statistics().write_failures > 0; }));
    EXPECT_FALSE(bad.statistics().healthy);
    bad.finish();
    c = settings(tmp.path / "quota");
    c.max_storage_bytes = 8192;
    c.continuous_file_bytes = 4096;
    c.min_free_bytes = std::numeric_limits<uint64_t>::max();
    Recorder full(c, profile("pro", 31));
    ASSERT_TRUE(wait([&] { return full.statistics().quota_rejections > 0; }));
    Packet p;
    p.event = Event::Estop;
    EXPECT_NO_THROW(full.submit(Source::Lifecycle, p));
    full.finish();
}
TEST(FlightRecorder, DisabledRecorderDoesNotAllocateOrTouchFilesystem) {
    Temp tmp;
    Config c;
    c.directory = tmp.path;
    Recorder r(c, profile("eqr11", 12));
    EXPECT_EQ(r.statistics().memory_bytes, 0u);
    EXPECT_FALSE(r.requestSave());
    EXPECT_FALSE(fs::exists(tmp.path));
}
} // namespace

TEST(FlightRecorder, EventsPersistEvenWhenNoIncidentIsTriggered) {
    Temp tmp;
    auto c = settings(tmp.path);
    Recorder r(c, profile("pro", 31));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    Packet p;
    p.event = Event::ModeChanged;
    ASSERT_TRUE(r.submit(Source::Application, p));
    r.finish();
    EXPECT_TRUE(incidents(tmp.path).empty());
    bool found = false;
    for (const auto &e : fs::recursive_directory_iterator(tmp.path))
        if (e.path().filename() == "events.bin") {
            found = true;
            const auto records = readRecords(e.path());
            ASSERT_EQ(records.size(), 1u);
            EXPECT_EQ(little(records[0].data() + 2, 2), unsigned(Event::ModeChanged));
        }
    EXPECT_TRUE(found);
}

TEST(FlightRecorder, RetentionOnlyDeletesOwnedRecordsAndDirectoryLockIsExclusive) {
    Temp tmp;
    fs::create_directories(tmp.path / "eirflight_000");
    std::ofstream(tmp.path / "eirflight_000" / ".eir_flight") << "EIR_FLIGHT_1\n";
    std::ofstream(tmp.path / "eirflight_000" / "continuous.bin") << std::string(6000, 'x');
    fs::create_directories(tmp.path / "unrelated");
    std::ofstream(tmp.path / "unrelated" / "keep.txt") << "preserve";
    auto c = settings(tmp.path);
    c.max_storage_bytes = 8192;
    c.continuous_file_bytes = 4096;
    Recorder first(c, profile("eqr11", 12));
    ASSERT_TRUE(wait([&] { return first.statistics().healthy; }));
    EXPECT_FALSE(fs::exists(tmp.path / "eirflight_000"));
    EXPECT_TRUE(fs::exists(tmp.path / "unrelated" / "keep.txt"));
    Recorder second(c, profile("eqr11", 12));
    ASSERT_TRUE(wait([&] { return second.statistics().write_failures > 0; }));
    EXPECT_TRUE(first.statistics().healthy);
    second.finish();
    first.finish();
}

TEST(FlightRecorder, SevereTriggerHasSeparateQueueFromOrdinaryPressure) {
    Temp tmp;
    auto c = settings(tmp.path);
    c.queue_frames = 4;
    Recorder r(c, profile("pro", 31));
    ASSERT_TRUE(wait([&] { return r.statistics().healthy; }));
    std::atomic<bool> flood{true};
    std::thread producer([&] {
        while (flood) {
            Packet p;
            p.event = Event::ModeChanged;
            r.submit(Source::Application, p);
        }
    });
    Packet fault;
    fault.event = Event::Estop;
    ASSERT_TRUE(r.submit(Source::Application, fault));
    std::this_thread::sleep_for(20ms);
    flood = false;
    producer.join();
    ASSERT_TRUE(wait([&] { return !incidents(tmp.path).empty(); }));
    r.finish();
    EXPECT_GT(r.statistics().dropped, 0u);
}
