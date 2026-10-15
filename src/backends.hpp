#pragma once
#include <eir_loco_runtime/runtime.hpp>
namespace eir::loco {
std::unique_ptr<Backend> makeBackend(BackendType type);
}
