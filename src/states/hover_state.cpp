#include "states/hover_state.hpp"

#include "features/tracker/tracker.hpp"

StateAction HoverState::on_enter(RobotContext& ctx) {
    ctx.tracker->send_zero_velocity();
    return StateAction::unhandled();
}
