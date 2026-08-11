#include "states/hover_state.hpp"

#include <optional>

#include "features/tracker/tracker.hpp"

StateAction HoverState::on_enter(RobotContext& ctx) {
    // 必须用send_vel_cmd覆盖位置控制
    ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt,
                              std::nullopt, CmdFrame::BODY);
    return StateAction::unhandled();
}
