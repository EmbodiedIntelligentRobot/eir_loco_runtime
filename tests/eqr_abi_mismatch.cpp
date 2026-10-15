#include <eir_base_sdk_eqr/EqrLowLevelApi.h>
// Loader fault injection only; no SDK construction or device access.
const char* EIR_Robotics::EqrLowLevelApi::InterfaceVersion() noexcept { return "incompatible-test-abi"; }
