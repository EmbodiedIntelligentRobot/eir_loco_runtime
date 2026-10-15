#include <algorithm>
#include <eir_loco_runtime/flight_recorder.hpp>
#include <filesystem>
#include <iostream>
#include <thread>
using namespace eir::loco::flight;
int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: benchmark output-directory\n";
        return 2;
    }
    Layout l;
    l.family = "aiq";
    l.robot = "pro";
    l.backend = "simulation";
    l.backend_slots = 32;
    l.raw_motor_count = 64;
    for (unsigned i = 0; i < 32; ++i) {
        l.joint_names.push_back("joint_" + std::to_string(i));
        l.motor_names.push_back("motor_" + std::to_string(i));
        l.motor_slots.push_back(i);
    }
    l.encoder_names = {"pitch", "roll"};
    Config c;
    c.enabled = true;
    c.directory = argv[1];
    c.min_free_bytes = 0;
    Recorder r(c, l);
    std::vector<int64_t> times;
    times.reserve(1500);
    std::array<double, 64> data{};
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < 1500; ++i) {
        const auto begin = Recorder::monotonicNow();
        Packet command, feedback;
        for (unsigned tag = 20; tag <= 24; ++tag)
            command.doubles(tag, data.data(), 32);
        for (unsigned tag = 30; tag <= 34; ++tag)
            command.doubles(tag, data.data(), 32);
        for (unsigned tag = 100; tag <= 105; ++tag)
            feedback.doubles(tag, data.data(), 64);
        r.submit(Source::Command, command);
        r.submit(Source::Feedback, feedback);
        times.push_back(Recorder::monotonicNow() - begin);
        next += std::chrono::milliseconds(2);
        std::this_thread::sleep_until(next);
    }
    r.requestSave();
    r.finish();
    std::sort(times.begin(), times.end());
    const auto stats = r.statistics();
    std::cout << "sample_count=" << times.size()
              << " combined_two_source_p99_ns=" << times[times.size() * 99 / 100]
              << " max_ns=" << times.back() << " drops=" << stats.dropped
              << " write_failures=" << stats.write_failures
              << " allocated_cache_bytes=" << stats.memory_bytes << "\n";
    return stats.write_failures || times[times.size() * 99 / 100] >= 100000 ? 1 : 0;
}
