#pragma once
#include "state_common.hpp"

class LandState : public dk::BaseState<RobotContext, LandState, void> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;
    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);
    static constexpr std::string_view static_name() { return "地面状态"; }
};