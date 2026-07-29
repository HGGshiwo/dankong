#include "states/init_state.hpp"

StateAction InitState::on_tick(double dt, RobotContext& ctx) {
    if (!ctx.odom_ok) return StateAction::unhandled();
    return ctx.robot->check_hover(ctx) ? step<HoverState>()
                                       : step<GroundState>();
}