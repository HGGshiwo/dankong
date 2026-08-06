#pragma once
#include <chrono>
#include <memory>
#include <variant>

#include "core/global_config.hpp"
#include "features/control/events.hpp"
#include "features/mavlink/events.hpp"
#include "ground_state.hpp"
#include "hover_state.hpp"
#include "robot_context.hpp"
#include "state_common.hpp"
#include "state_utils.hpp"
#include "waypoint_state.hpp"

class TakeoffState : public dk::BaseState<RobotContext, TakeoffState, void> {
   public:
    using AllowedEvents = std::tuple<>;
    using TriggerEvent = std::variant<SetWaypointEvent, TakeoffEvent>;
    double start_time_;
    TriggerEvent event_;
    double alt_;

    // 和waypoint相关
    bool step_waypoint_ = false;  // 是否进入waypoint

    std::shared_ptr<state_utils::StallChecker<1>> checker_;

   public:
    TakeoffState(TakeoffEvent e);
    TakeoffState(SetWaypointEvent e);

    StateAction on_enter(RobotContext& ctx) override;
    StateAction on_tick(double dt, RobotContext& ctx) override;

    void report_takeoff(RobotContext& ctx) {
        ctx.ws_manager->publish_reliable(
            nlohmann::json{{"type", "event"}, {"event", "takeoff"}});
    }

    static constexpr std::string_view static_name() { return "起飞状态"; }
};