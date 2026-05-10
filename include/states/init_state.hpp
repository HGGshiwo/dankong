#pragma once
#include "state_common.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"

class InitState : public dk::BaseState<RobotContext, InitState, void> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;
    using StateAction = dk::StateAction<RobotContext>;
    StateAction on_event(dk::TickEvent e, RobotContext& ctx);
    static constexpr std::string_view static_name() { return "初始状态"; }
};