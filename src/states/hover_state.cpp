#include "states/hover_state.hpp"

#include "features/tracker/tracker.hpp"

StateAction HoverState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    if (!ctx.enable_joystick.load()) {
        ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt, 0,
                                  CmdFrame::BODY);
    }

    return StateAction::unhandled();
}
