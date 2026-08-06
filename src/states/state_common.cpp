#include "states/state_common.hpp"

#include "states/follow_state.hpp"
#include "states/posvel_state.hpp"
#include "states/takeoff_state.hpp"
#include "states/waypoint_state.hpp"

StateFlags get_state_flags(RobotContext& ctx) {
    StateFlags flags;
    auto states = ctx.engine->get_active_states_view();
    for (auto* s : states) {
        if (!s) continue;
        if (dynamic_cast<TakeoffState*>(s)) {
            flags.is_takeoff = true;
        } else if (auto* ws = dynamic_cast<WaypointState*>(s)) {
            flags.is_waypoint = true;
            flags.local = ws->event_.local;
        } else if (dynamic_cast<dk::TmplBase<FollowState>*>(s)) {
            flags.is_follow = true;
        } else if (dynamic_cast<dk::TmplBase<PosVelState>*>(s)) {
            flags.is_posvel = true;
        } else if (dynamic_cast<HoverState*>(s)) {
            flags.is_hover = true;
        } else if (dynamic_cast<LandState*>(s)) {
            flags.is_land = true;
        }
    }
    return flags;
}