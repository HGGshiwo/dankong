#pragma once
#include "state_common.hpp"

class LandState : public dk::PureState<RobotEvent, RobotContext, LandState> {
   public:
    StatePtr on_event(const dk::TickEvent& e, RobotContext& ctx);
};