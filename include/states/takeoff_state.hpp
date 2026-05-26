#pragma once
#include <chrono>
#include <memory>
#include <variant>

#include "core/global_config.hpp"
#include "features/control/events.hpp"
#include "ground_state.hpp"
#include "hover_state.hpp"
#include "state_common.hpp"
#include "state_utils.hpp"
#include "waypoint_state.hpp"

class TakeoffState : public dk::BaseState<RobotContext, TakeoffState, void> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;
    using TriggerEvent = std::variant<SetWaypointEvent, TakeoffEvent>;
    std::chrono::steady_clock::time_point start_time_;
    TriggerEvent event_;
    double alt_;

    // 和waypoint相关
    bool step_waypoint_ = false;  // 是否进入waypoint

   public:
    class PrearmCheckState;
    class ArmState;
    class TakingoffState;

    TakeoffState(TakeoffEvent e);

    TakeoffState(SetWaypointEvent e);

    void report_takeoff(RobotContext& ctx) {
        ctx.ws_manager->publish_reliable(
            nlohmann::json{{"type", "event"}, {"event", "takeoff"}});
    }

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "起飞状态"; }
};

class TakeoffState::PrearmCheckState
    : public dk::BaseState<RobotContext, PrearmCheckState, TakeoffState> {
   public:
    using AllowedEvents = std::tuple<SysStatusEvent, StatusTextEvent>;

    void on_enter(RobotContext& ctx) override;

    StateAction on_event(const SysStatusEvent& event, RobotContext& ctx);

    StateAction on_event(const StatusTextEvent& event, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "起飞检查"; }
};

class TakeoffState::ArmState
    : public dk::BaseState<RobotContext, ArmState, TakeoffState> {
   public:
    using AllowedEvents = std::tuple<ArmEvent, StatusTextEvent>;

    void on_enter(RobotContext& ctx) override;

    StateAction on_event(const ArmEvent& event, RobotContext& ctx);

    StateAction on_event(const StatusTextEvent& event, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "解锁状态"; }
};

class TakeoffState::TakingoffState
    : public dk::BaseState<RobotContext, TakingoffState, TakeoffState> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;

    std::shared_ptr<state_utils::StallChecker<1>> checker_;

    std::chrono::steady_clock::time_point start_time_;

    void on_enter(RobotContext& ctx) override;

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "执行起飞"; }
};