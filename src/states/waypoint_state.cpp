#include "states/waypoint_state.hpp"

#include <chrono>
#include <optional>

#include "mavlink/imavlink.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"

WaypointState::WaypointState(SetWaypointEvent e, state_utils::FinishAction action)
    : wp_list_(e.waypoint), wp_idx_(0), action_(action) {
    wp_list_.erase(wp_list_.begin());
}

void WaypointState::WaypointState::on_enter(RobotContext& ctx) {
    if (action_ == state_utils::FinishAction::RETURN) {
        // 添加一个返航点
        wp_list_.push_back(ctx.takeoff_lon_lat_alt.get());
    }
}

Eigen::Vector3d WaypointState::get_cur_wp() {
    return wp_list_.at(wp_idx_);
}

WaypointState::LiftingState::LiftingState() : start_time_(std::chrono::steady_clock::now()) {};

void WaypointState::LiftingState::on_enter(RobotContext& ctx) {
    auto cur_wp = parent()->get_cur_wp();
    auto diff = state_utils::get_relevant_enu(ctx.lon_lat_alt.get(), cur_wp);
    target_alt_ = cur_wp.z();

    target_yaw_ = state_utils::get_heading(diff.x(), diff.y());

    last_alt_ = ctx.pos_enu.get().z();
    last_yaw_ = ctx.yaw_ned;
}

double MAX_Z_VEL = 1.0;
double YAW_STALL_THRESH = 0.1;
double ALT_STALL_THRESH = 0.1;
double YAW_TOLERANCE = 0.1;
double ALT_TOLERANCE = 2.0;
double STALL_TIMEOUT = 3.0;

StateAction WaypointState::LiftingState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    // 达到目标的高度和航向退出
    double yaw_diff = state_utils::get_yaw_diff(target_yaw_, ctx.yaw_enu);
    double alt_diff = fabs(target_alt_ - ctx.lon_lat_alt.get().z());

    if (yaw_diff < YAW_TOLERANCE && (!ctx.robot->is_alt_enable() || alt_diff < ALT_TOLERANCE)) {
        ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d::Zero(), std::nullopt, std::nullopt, std::nullopt,
                            CmdFrame::ENU);
        return step<WaypointState::ExcuteState>();
    }

    double span = state_utils::get_time_span(start_time_);
    if (span > STALL_TIMEOUT) {
        double cur_yaw = ctx.yaw_enu;
        double cur_alt = ctx.lon_lat_alt.get().z();
        // 卡死超时，强制进入 WP
        if (fabs(last_yaw_ - cur_yaw) < YAW_STALL_THRESH || fabs(last_alt_ - cur_alt)) {
            ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d::Zero(), std::nullopt, std::nullopt, std::nullopt,
                                CmdFrame::ENU);
            return step<WaypointState::ExcuteState>();
        }
        // 变化超过阈值，则让它继续变化
        start_time_ = std::chrono::steady_clock::now();
        last_yaw_ = cur_yaw;
        last_alt_ = cur_alt;
    }

    double vz = std::clamp(target_alt_ - ctx.lon_lat_alt.get().z(), -MAX_Z_VEL, MAX_Z_VEL);
    ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d{0.0, 0.0, vz}, std::nullopt, target_yaw_, std::nullopt,
                        CmdFrame::ENU);
    return StateAction::unhandled();
}

void WaypointState::LiftingState::on_exit(RobotContext& ctx) {
    ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d{0.0, 0.0, 0.0}, std::nullopt, std::nullopt, std::nullopt,
                        CmdFrame::ENU);
}

//============= excute_state =============

double TOLERANCE = 2.0;
bool WaypointState::ExcuteState::check_arrive(RobotContext& ctx) {
    if (!ctx.odom_ok) return false;
    double dist = ctx.robot->get_distance(ctx.pos_enu.get(), wp_goal_);
    return dist < TOLERANCE;
}

// 执行当前wp_idx指向的航点
void WaypointState::ExcuteState::on_enter(RobotContext& ctx) {
    auto wp = parent()->get_cur_wp();
    wp_goal_ = state_utils::gps_to_enu(ctx, wp);

    if (!ctx.planner_enable) {
        ctx.robot->send_cmd(wp_goal_, std::nullopt, std::nullopt, std::nullopt, std::nullopt, CmdFrame::ENU);
    }
    // if (event_list.has_value()) {
    //     auto events = wp_data.event_list.value().at(wp_data.wp_idx);
    //     for (const auto& event : events) {
    //         run_wp_envet(event);
    //     }
    // }
}

StateAction WaypointState::ExcuteState::on_event(const dk::TickEvent& event, RobotContext& ctx) {
    // 如果到达了目标点
    auto arrive = check_arrive(ctx);
    if (arrive) {
        parent()->wp_idx_ += 1;
        if (parent()->wp_idx_ >= parent()->wp_list_.size()) {
            if (parent()->action_ != state_utils::HOVER)
                return step<LandState>();
            else
                return step<HoverState>();
        }
        return step<WaypointState::LiftingState>();
    }
    return StateAction::unhandled();
}

void WaypointState::run_wp_envet(std::string e) {}