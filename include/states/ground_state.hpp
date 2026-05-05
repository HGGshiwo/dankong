#pragma once
#include "state_common.hpp"

class GroundState : public dk::PureState<RobotEvent, RobotContext, GroundState> {
   public:
    StatePtr on_event(const dk::TickEvent& e, RobotContext& ctx);
    StatePtr on_event(const TakeoffEvent& e, RobotContext& ctx);
};