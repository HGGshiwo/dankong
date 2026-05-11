#include "states/waypoint_state.hpp"

#include <chrono>
#include <optional>

#include "mavlink/imavlink.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"

using josn = nlohmann::json;

WaypointState::WaypointState(SetWaypointEvent e, state_utils::FinishAction action)
    : wp_list_(e.waypoint), wp_idx_(0), action_(action) {}

void WaypointState::WaypointState::on_enter(RobotContext& ctx) {
    if (action_ == state_utils::FinishAction::RETURN) {
        // 添加一个返航点
        wp_list_.push_back(ctx.takeoff_lon_lat_alt.get());
    }
    if (wp_list_.size() == 0) {
        // 何意味？直接悬停吧
        ctx.engine->step<HoverState>();
        return;
    } else if (wp_list_.size() == 1) {
        // 偷懒没有传第一个点
        wp_list_.insert(wp_list_.begin(), ctx.takeoff_lon_lat_alt.get());
    }
    // 计算真实的航点
    std::vector<json> wp_data;
    wp_data.reserve(wp_list_.size());

    for (uint i = 0; i < wp_list_.size(); ++i) {
        std::string command = "wp";
        if (i == 0) {
            command = "takeoff";
        } else if (i == wp_list_.size() - 1 && action_ != state_utils::FinishAction::HOVER) {
            command = "land";
        }
        auto cur_wp = wp_list_[i];
        wp_data.push_back(
            json{{"num", i}, {"lon", cur_wp.x()}, {"lat", cur_wp.y()}, {"alt", cur_wp.z()}, {"command", command}});
    }
    ctx.mission_data.set(wp_data);
    wp_list_.erase(wp_list_.begin());
    ctx.wp_idx.set(0);
}

void WaypointState::WaypointState::on_exit(RobotContext& ctx) {
    // 清空数据
    ctx.dist_to_target.set(0.0);
}

Eigen::Vector3d WaypointState::get_cur_wp() {
    return wp_list_.at(wp_idx_);
}

double MAX_Z_VEL = 1.0;
double YAW_STALL_THRESH = 0.1;
double ALT_STALL_THRESH = 0.1;
double YAW_TOLERANCE = 0.1;
double ALT_TOLERANCE = 2.0;
double STALL_TIMEOUT = 3.0;

WaypointState::LiftingState::LiftingState()
    : start_time_(std::chrono::steady_clock::now()),
      checker_(std::make_shared<state_utils::StallChecker<2>>(std::array<double, 2>{YAW_STALL_THRESH, ALT_STALL_THRESH},
                                                              STALL_TIMEOUT)) {};

void WaypointState::LiftingState::on_enter(RobotContext& ctx) {
    auto cur_wp = parent()->get_cur_wp();
    auto diff = state_utils::get_relevant_enu(ctx.lon_lat_alt.get(), cur_wp);
    target_alt_ = cur_wp.z();

    target_yaw_ = state_utils::get_heading(diff.x(), diff.y());

    last_alt_ = ctx.pos_enu.get().z();
    last_yaw_ = ctx.yaw_ned.get();
}

StateAction WaypointState::LiftingState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    // 达到目标的高度和航向退出
    double yaw_diff = state_utils::get_yaw_diff(target_yaw_, ctx.yaw_enu);
    double alt_diff = fabs(target_alt_ - ctx.lon_lat_alt.get().z());

    if (yaw_diff < YAW_TOLERANCE && (!ctx.robot->is_alt_enable() || alt_diff < ALT_TOLERANCE)) {
        ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d::Zero(), std::nullopt, std::nullopt, std::nullopt,
                            CmdFrame::ENU);
        return step<WaypointState::ExcuteState>();
    }

    if (checker_->is_stall({ctx.yaw_enu, ctx.lon_lat_alt.get().z()})) {
        return step<WaypointState::ExcuteState>();
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
    ctx.dist_to_target.set(dist);
    return dist < TOLERANCE;
}

// 执行当前wp_idx指向的航点
void WaypointState::ExcuteState::on_enter(RobotContext& ctx) {
    auto wp = parent()->get_cur_wp();
    wp_goal_ = state_utils::gps_to_enu(ctx, wp);

    if (!ctx.planner_enable.get()) {
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
        ctx.wp_idx.set(parent()->wp_idx_);

        // 发布数据
        json j = json{
            {"total", parent()->wp_list_.size()}, {"cur", ctx.wp_idx.get()}, {"type", "event"}, {"event", "progress"}};
        ctx.ws_manager->publish_state("progress", j);

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