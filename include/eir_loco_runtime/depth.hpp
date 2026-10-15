#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace eir::loco {
// parkour_depth_v2: image-plane distance / max_distance, [frame,y,x], oldest first.
// CPU-only snapshots; no ROS or MuJoCo headers in the public perception contract.
struct DepthStack {
    static constexpr std::size_t frames = 8, height = 18, width = 32;
    std::array<float, frames * height * width> data{};
    std::uint32_t sequence{};
    std::uint64_t stamp{}; // steady_clock acquisition time, not read time
    bool valid{};
};
struct DepthConfig {
    bool enabled{false};
    std::string site, preprocessing_version{"depth_v1"};
    int raw_width{64}, raw_height{40}, crop_up{22}, crop_down{0}, crop_left{16}, crop_right{16};
    int frequency_hz{50};
    double min_distance{0.28}, max_distance{2.5};
    double sensor_min_distance{0.25}, sensor_max_distance{2.5}; // parkour axial-Z validity
    double horizontal_fov_deg{90}, vertical_fov_deg{65};
};
} // namespace eir::loco
