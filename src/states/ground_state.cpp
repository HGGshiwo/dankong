#include "states/ground_state.hpp"

#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"

StatePtr GroundState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    return ctx.robot->check_hover(ctx.arm, ctx.pos.get().z()) ? HoverState::instance() : nullptr;
}

StatePtr GroundState::on_event(const TakeoffEvent& e, RobotContext& ctx) {
    state_utils::takeoff_vehicle(ctx, e.alt)
        .then([e, &ctx]() mutable -> void {
            ctx.engine->step(TakeoffState::instance());
            e.resolve({"success", "OK"});
        })
        .catch_error([e](std::exception_ptr err) mutable -> void { e.reject(err); });
    return nullptr;
}
