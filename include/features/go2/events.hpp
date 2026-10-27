#pragma once
#include <cstdint>

#include "core/event_result.hpp"
#include "dk/engine.hpp"

// @JSON_ENABLE
struct Go2StateData {
    uint8_t mode;
    uint8_t battery;
};

// 定位输出开关: 同时控制 /loc_base 发布与 odom->base_link TF 广播
// @JSON_ENABLE
struct ToggleLocOutputEvent : dk::AsyncEvent<EventResult> {
    bool enable = false;
};
