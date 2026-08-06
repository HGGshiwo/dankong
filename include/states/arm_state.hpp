#pragma once
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "state_common.hpp"
#include "states/hover_state.hpp"

class ArmState : public dk::BaseState<RobotContext, ArmState, void> {
   private:
    dk::StateAction<RobotContext> next_action_;
    double start_time_;

   public:
    using AllowedEvents = std::tuple<ArmEvent, StatusTextEvent>;

    ArmState(dk::StateAction<RobotContext> next_action =
                 dk::StateAction<RobotContext>::step<HoverState>())
        : next_action_(next_action) {}

    dk::StateAction<RobotContext> on_enter(RobotContext& ctx) override;
    dk::StateAction<RobotContext> on_tick(double dt,
                                          RobotContext& ctx) override;
    dk::StateAction<RobotContext> on_event(const ArmEvent& event,
                                           RobotContext& ctx);
    dk::StateAction<RobotContext> on_event(const StatusTextEvent& event,
                                           RobotContext& ctx);

    static constexpr std::string_view static_name() { return "解锁状态"; }
};
