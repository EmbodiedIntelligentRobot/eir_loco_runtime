#include <eir_loco_runtime/runtime.hpp>
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: eir_runtime_example model.xml joint_name motor_name\n";
        return 2;
    }
    try {
        eir::loco::Config config;
        config.model_path = argv[1];
        config.joint_names = {argv[2]}; config.motor_names = {argv[3]};
        config.suspend = false;
        eir::loco::Runtime runtime(config);
        if (!runtime.start()) { std::cerr << runtime.status().message << '\n'; return 1; }
        // Remain in damping: this example never commands a motion or starts hardware.
        for (int i = 0; i < 100; ++i) {
            eir::loco::State state;
            if (!runtime.read(state) || runtime.status().mode == eir::loco::Mode::Estopped) {
                std::cerr << runtime.status().message << '\n'; return 1;
            }
            if (i % 20 == 0)
                std::cout << "sequence=" << state.sequence << " position=" << state.joints[0].position << '\n';
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        runtime.stop();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
