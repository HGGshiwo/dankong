#pragma once
#include "state_common.hpp"

class GroundState : public dk::BaseState<RobotContext, GroundState, void> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;
    using StateAction = dk::StateAction<RobotContext>;
    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);
    static constexpr std::string_view static_name() { return "地面状态"; }
};