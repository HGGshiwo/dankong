#include "states/land_state.hpp"

#include "states/ground_state.hpp"

StatePtr LandState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    double throttle = ctx.throttle;
    double rangefinder_alt = ctx.rangefinder_alt;
    if (throttle > 0 && throttle < 0.01 && rangefinder_alt > 0 && rangefinder_alt < 0.5) {
        return GroundState::instance();
    }
    return nullptr;
}