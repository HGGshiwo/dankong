#pragma once
#include "state_common.hpp"

class HoverState : public dk::PureState<RobotEvent, RobotContext, HoverState> {
   public:
    StatePtr on_event(const dk::TickEvent& e, RobotContext& ctx);
};