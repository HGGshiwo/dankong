#include "states/waypoint_state.hpp"

#include "states/hover_state.hpp"
#include "states/lifting_state.hpp"
#include "states/state_utils.hpp"
#include "takeoff_state.hpp"
#include "waypoint_state.hpp"

void run_wp_envet(std::string e) {}
void waypoint_finish() {}

bool check_arrive(RobotContext& ctx) {
    double TOLERANCE = 2.0;
    if (!ctx.odom_ok) return false;

    double dist = ctx.robot->get_distance(ctx.pos.get(), ctx.wp_goal.get());
    ctx.wp_dist = TOLERANCE;
    return dist < TOLERANCE;
}

StatePtr WaypointState::on_event(const dk::EnterEvent& e, RobotContext& ctx) {
    Eigen::Vector3d wp;
    ctx.waypoint.write([&wp, &ctx](std::vector<Eigen::Vector3d> vec) -> void { wp = vec.at(ctx.wp_idx); });
    auto enu = state_utils::gps_to_enu(ctx, wp.x(), wp.y(), wp.z());

    ctx.wp_goal.set(enu);
    if (!ctx.planner_enable) {
        ctx.robot->send_cmd(enu, std::nullopt, std::nullopt, std::nullopt, std::nullopt, CmdFrame::ENU);
    }
    ctx.event_list.write([&ctx](auto event_list) -> void {
        auto events = event_list.at(ctx.wp_idx);
        for (const auto& event : events) {
            run_wp_envet(event);
        }
    });
    return nullptr;
}

StatePtr WaypointState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    auto arrive = check_arrive(ctx);
    if (!ctx.planner_enable && arrive) {
        waypoint_finish();
    }
}
