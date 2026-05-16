#include "states/land_state.hpp"

#include "robot_context.hpp"
#include "states/ground_state.hpp"

void LandState::on_enter(RobotContext& ctx) {
    if (land_target_id_ < 0) {
        ctx.robot->land(ctx);
        return;
    }
    // 触发精准降落
}

StateAction LandState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    double throttle = ctx.throttle;
    double rangefinder_alt = ctx.rangefinder_alt;
    bool disarmed = ctx.arm.get() == false;
    bool ground_check = throttle >= 0 && throttle < 0.01 &&
                        rangefinder_alt >= 0 && rangefinder_alt < 0.5;
    if (disarmed || ground_check) {
        return step<GroundState>();
    }
    return StateAction::unhandled();
}