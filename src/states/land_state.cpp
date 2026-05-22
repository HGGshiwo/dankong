#include "states/land_state.hpp"

#include "context_config.hpp"
#include "states/ground_state.hpp"

void LandState::on_enter(RobotContext& ctx) {
    if (land_target_id_ < 0) {
        ctx.robot->land();
        return;
    }
    // 触发精准降落
}

StateAction LandState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    if (ctx.robot->is_landed(ctx)) {
        return step<GroundState>();
    }
    return StateAction::unhandled();
}