#include "states/init_state.hpp"

StatePtr InitState::on_event(dk::TickEvent e, RobotContext& ctx) {
    if (!ctx.odom_ok) return nullptr;
    return ctx.robot->check_hover(ctx.arm, ctx.pos.get().z()) ? HoverState::instance() : GroundState::instance();
}