#pragma once
#include "state_common.hpp"

class WaypointState : public dk::PureState<RobotEvent, RobotContext, WaypointState> {
   public:
    StatePtr on_event(const dk::TickEvent& e, RobotContext& ctx);
    StatePtr on_event(const dk::EnterEvent& e, RobotContext& ctx);
};
