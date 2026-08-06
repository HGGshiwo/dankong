#include "states/arm_state.hpp"

#include "core/global_config.hpp"
#include "spdlog/spdlog.h"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"

dk::StateAction<RobotContext> ArmState::on_enter(RobotContext& ctx) {
    start_time_ = ctx.engine->get_time_provider()->now();
    if (ctx.arm.load()) {
        return next_action_;
    }
    ctx.robot->arm();
    return dk::StateAction<RobotContext>::unhandled();
}

dk::StateAction<RobotContext> ArmState::on_event(const ArmEvent& event,
                                                 RobotContext& ctx) {
    if (event.armed) {
        return next_action_;
    }
    return dk::StateAction<RobotContext>::unhandled();
}

dk::StateAction<RobotContext> ArmState::on_event(const StatusTextEvent& event,
                                                 RobotContext& ctx) {
    if (ctx.robot->is_prearm_msg(event.text)) {
        spdlog::warn("[ArmState] Prearm check failed: {}", event.text);
        if (ctx.robot->check_hover(ctx)) {
            return step<HoverState>();
        } else {
            return step<GroundState>();
        }
    }
    return dk::StateAction<RobotContext>::unhandled();
}

dk::StateAction<RobotContext> ArmState::on_tick(double dt, RobotContext& ctx) {
    double now = ctx.engine->get_time_provider()->now();
    if (state_utils::get_time_span(start_time_, now) >
        GlobalConfig.GetConfig().prearm_timeout) {
        spdlog::error("[ArmState] Arming timeout!");
        if (ctx.robot->check_hover(ctx)) {
            return step<HoverState>();
        } else {
            return step<GroundState>();
        }
    }
    return dk::StateAction<RobotContext>::unhandled();
}
