#pragma once

#include <array>
#include <chrono>
#include <exception>
#include <optional>
#include <stdexcept>
#include <tuple>

#include "./events.hpp"
#include "core/global_config.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/adapters/web/protocal.hpp"
#include "nlohmann/json.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"

// 和控制相关的事件监听器
class PlandEventListener
    : public dk::BaseEventListener<RobotContext, PlandEventListener> {
   public:
    using AllowedEvents = std::tuple<StartPlandDetectEvent>;

    void on_event(const StartPlandDetectEvent& event, RobotContext& ctx) {
        ctx.land_detector->set_target_id(0);
        ctx.land_detector->start(30);
        event.resolve({"success", "OK"});
    }
};
