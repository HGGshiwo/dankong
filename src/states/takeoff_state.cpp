#include "states/takeoff_state.hpp"

#include "states/hover_state.hpp"
#include "states/lifting_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"

bool base_on_event(RobotContext& ctx) {
    if (!ctx.robot->is_alt_enable()) {
        // 只检查悬停
        if (!ctx.robot->check_hover(ctx.arm, ctx.pos.get().z())) {
            return false;
        }
    }
    bool arrive = state_utils::check_alt(ctx, ctx.takeoff_pos.get().z());
    if (!arrive) {
        return false;
    }
    ctx.engine->dispatch_internal(TakeoffDoneEvent());
    return true;
}

StatePtr TakeoffState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    if (base_on_event(ctx)) {
        return HoverState::instance();
    }
    return nullptr;
}

StatePtr TakeoffState2::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    if (base_on_event(ctx)) {
        return LiftingState::instance();
    }
    return nullptr;
}