#pragma once
#include <atomic>
#include <memory>
#include <string_view>

#include "utils/state_registry.hpp"

struct MavlinkContext {
    std::atomic<bool> use_fcu_enu = true;

    explicit MavlinkContext(StateRegistry& reg) {}
};