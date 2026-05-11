#include "states/ground_state.hpp"

#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"

StateAction GroundState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    return ctx.robot->check_hover(ctx.arm.get(), ctx.pos_enu.get().z()) ? step<HoverState>() : StateAction::unhandled();
}