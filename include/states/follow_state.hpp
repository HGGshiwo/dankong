#pragma once
#include <chrono>

#include "core/global_config.hpp"
#include "features/tracker/tracker.hpp"
#include "state_common.hpp"
#include "states/hover_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"
#include "states/waypoint_state.hpp"

template <typename ParentState>
class FollowState : public dk::BaseState<RobotContext, FollowState<ParentState>,
                                         ParentState> {
   public:
    double last_time_;

    using AllowedEvents = std::tuple<DetectEvent>;

    FollowState(const DetectEvent& event, RobotContext& ctx) {
        last_time_ = ctx.engine->get_time_provider()->now();
        on_event(event, ctx);
    }

    StateAction on_tick(double dt, RobotContext& ctx);

    StateAction on_event(const DetectEvent& event, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "跟随模式"; }
};

template <typename ParentState>
StateAction FollowState<ParentState>::on_event(const DetectEvent& event,
                                               RobotContext& ctx) {
    last_time_ = ctx.engine->get_time_provider()->now();
    Eigen::Vector3d cmd_vel =
        event.score >= 0 ? event.cmd_vel : Eigen::Vector3d::Zero();

    if (event.score > 0) {
        auto gps = state_utils::enu_to_gps(
            ctx.lon_lat_alt.load(), ctx.pos_enu.load(), event.target_pos);
        DetectTargetEvent target_event;
        target_event.pos = gps;
        ctx.engine->dispatch_internal(target_event);
    }

    ctx.tracker->send_vel_cmd(cmd_vel, std::nullopt, std::nullopt,
                              CmdFrame::BODY);
    return StateAction::handled();
}

template <typename ParentState>
StateAction FollowState<ParentState>::on_tick(double dt, RobotContext& ctx) {
    double now = ctx.engine->get_time_provider()->now();
    if (state_utils::get_time_span(last_time_, now) >
        GlobalConfig.GetConfig().follow_timeout) {
        spdlog::error("[FollowState] follow timeout!");
        if constexpr (std::is_same_v<ParentState, HoverState>)
            return this->template step<HoverState>();
        if constexpr (std::is_same_v<ParentState, WaypointState>)
            return this->template step<WaypointState::LiftingState>();
        return this->template step<HoverState>();
    }
    // 必须handle住，否则走的是Hover的
    return StateAction::handled();
}