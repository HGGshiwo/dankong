#include "states/ground_state.hpp"

#include "robot_context.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"

StateAction GroundState::on_tick(double dt, RobotContext& ctx) {
    return ctx.robot->check_hover(ctx) ? step<HoverState>()
                                       : StateAction::unhandled();
}