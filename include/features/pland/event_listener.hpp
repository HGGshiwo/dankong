#pragma once
#include "states/state_utils.hpp"
#ifdef USE_ROS
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
    using AllowedEvents = std::tuple<StartPlandDetectEvent, SetPlandTarget,
                                     StartOffsetEstimate, StopOffsetEstimate>;

    void on_event(const StartPlandDetectEvent& event, RobotContext& ctx) {
        ctx.land_detector->start(30);
        event.resolve({"success", "OK"});
    }

    void on_event(const StartOffsetEstimate& event, RobotContext& ctx) {
        ctx.land_detector->start(30, event.x, event.y, event.z);
        event.resolve({"success", "OK"});
    }

    void on_event(const StopOffsetEstimate& event, RobotContext& ctx) {
        auto out = ctx.land_detector->stop(event.save);
        event.resolve({"success", out});
    }

    void on_event(const SetPlandTarget& event, RobotContext& ctx) {
        Eigen::Vector3d pos;
        if (!event.position.has_value()) {
            pos = ctx.pos_enu.load();
        } else {
            if (!ctx.odom_ok) {
                event.resolve({"success", "Odom NOT OK"});
                return;
            }
            auto lla = event.position.value();
            pos = state_utils::gps_to_enu(ctx.lon_lat_alt.load(),
                                          ctx.pos_enu.load(), lla);
        }
        spdlog::info("[Pland] Set Target: [{:.2f}, {:.2f}, {:.2f}]", pos.x(),
                     pos.y(), pos.z());

        ctx.pland_target.store(PlandTarget{
            ctx.engine->get_time_provider()->now(), pos, event.velocity});

        event.resolve({"success", "OK"});
    }
};
#endif