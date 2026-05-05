#include "states/hover_state.hpp"

StatePtr HoverState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d::Zero(), std::nullopt, std::nullopt, 0, CmdFrame::BODY);
    return nullptr;
}
