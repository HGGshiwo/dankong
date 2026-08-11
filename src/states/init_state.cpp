#include "states/init_state.hpp"

StateAction InitState::on_tick(double dt, RobotContext& ctx) {
    if (!ctx.odom_ok) return StateAction::unhandled();
    if (ctx.robot->check_hover(ctx)) {
        LOG_STATE_STEP("HoverState");
        return step<HoverState>();
    } else {
        LOG_STATE_STEP("GroundState");
        return step<GroundState>();
    }
}