#pragma once
#include "state_common.hpp"

class TakeoffState : public dk::PureState<RobotEvent, RobotContext, TakeoffState> {
   public:
    StatePtr on_event(const dk::TickEvent& e, RobotContext& ctx);
};

class TakeoffState2 : public dk::PureState<RobotEvent, RobotContext, TakeoffState2> {
   public:
    StatePtr on_event(const dk::TickEvent& e, RobotContext& ctx);
};