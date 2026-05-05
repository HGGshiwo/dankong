#pragma once
#include "state_common.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"

class InitState : public dk::PureState<RobotEvent, RobotContext, InitState> {
   public:
    const std::string name() const override { return "初始状态"; }
    StatePtr on_event(dk::TickEvent e, RobotContext& ctx);
};