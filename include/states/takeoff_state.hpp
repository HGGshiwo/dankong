#pragma once
#include <chrono>
#include <memory>
#include <variant>

#include "ground_state.hpp"
#include "hover_state.hpp"
#include "state_common.hpp"
#include "state_utils.hpp"
#include "waypoint_state.hpp"

inline double PREARM_TIMEOUT = 3.0;

class TakeoffState : public dk::BaseState<RobotContext, TakeoffState, void> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;
    using TriggerEvent = std::variant<SetWaypointEvent, TakeoffEvent>;
    std::chrono::steady_clock::time_point start_time_;
    TriggerEvent event_;
    bool step_waypoint_ = false;  // 是否进入waypoint
    double alt_;
    state_utils::FinishAction action_;

   public:
    class PrearmCheckState;
    class ArmState;
    class TakingoffState;

    TakeoffState(TakeoffEvent e);

    TakeoffState(SetWaypointEvent e, state_utils::FinishAction action);

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "起飞状态"; }
};

class TakeoffState::PrearmCheckState : public dk::BaseState<RobotContext, PrearmCheckState, TakeoffState> {
   public:
    using AllowedEvents = std::tuple<SysStatusEvent, StatusTextEvent>;

    void on_enter(RobotContext& ctx) override;

    StateAction on_event(const SysStatusEvent& event, RobotContext& ctx);

    StateAction on_event(const StatusTextEvent& event, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "起飞检查"; }
};

class TakeoffState::ArmState : public dk::BaseState<RobotContext, ArmState, TakeoffState> {
   public:
    using AllowedEvents = std::tuple<ArmEvent>;

    void on_enter(RobotContext& ctx) override;

    StateAction on_event(const ArmEvent& event, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "解锁状态"; }
};

class TakeoffState::TakingoffState : public dk::BaseState<RobotContext, TakingoffState, TakeoffState> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;

    bool takeoff_res_;

    std::shared_ptr<state_utils::StallChecker<1>> checker_;

    std::chrono::steady_clock::time_point start_time_;

    void on_enter(RobotContext& ctx) override;

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "执行起飞"; }
};