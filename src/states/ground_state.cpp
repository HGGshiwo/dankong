#include "states/ground_state.hpp"

#include "robot_context.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"

StateAction GroundState::on_enter(RobotContext& ctx) {
    // 降落结束 (或重启回到地面): 任务返航点已消费/作废, 清理
    ctx.mission_home_enu.store(std::nullopt);
    return StateAction::unhandled();
}

StateAction GroundState::on_tick(double dt, RobotContext& ctx) {
    if (ctx.robot->check_hover(ctx)) {
        LOG_STATE_STEP("HoverState");
        return step<HoverState>();
    } else {
        return StateAction::unhandled();
    }
}