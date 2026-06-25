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
    using AllowedEvents = std::tuple<StartPlandDetectEvent, SetPlandTarget>;

    void on_event(const StartPlandDetectEvent& event, RobotContext& ctx) {
        ctx.land_detector->start(30);
        event.resolve({"success", "OK"});
    }

    void on_event(const SetPlandTarget& event, RobotContext& ctx) {
        auto pos = ctx.pos_enu.load();
        spdlog::info("[Pland] Set Target: [{:.2f}, {:.2f}, {:.2f}]", pos.x(),
                     pos.y(), pos.z());
        ctx.pland_target.store(
            Eigen::Vector3d{pos.x(), pos.y(), ctx.yaw_enu.load()});
        event.resolve({"success", "OK"});
    }
};
