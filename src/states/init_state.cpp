#include "states/init_state.hpp"

StateAction InitState::on_event(dk::TickEvent e, RobotContext& ctx) {
    if (!ctx.odom_ok) return StateAction::unhandled();
    return ctx.robot->check_hover(ctx.arm, ctx.pos_enu.get().z()) ? step<HoverState>() : step<GroundState>();
}