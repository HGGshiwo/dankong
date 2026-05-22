#pragma once

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>  // 包含 std::clamp, std::min, std::max
#include <cmath>
#include <optional>
#include <string_view>

#include "context_config.hpp"
#include "dk/state.hpp"
#include "features/control/events.hpp"
#include "features/tracker/tracker.hpp"
#include "spdlog/spdlog.h"
#include "states/hover_state.hpp"
#include "states/posvel_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"
#include "states/waypoint_state.hpp"

template <typename ParentState>
class PosVelState : public dk::BaseState<RobotContext, PosVelState<ParentState>,
                                         ParentState> {
   public:
    static constexpr std::string_view static_name() { return "指点控制"; }

    explicit PosVelState(SetPosVelEvent e);

    // [修改点 1]：增加父状态的 on_enter，作为控制开始时的初始化
    void on_enter(RobotContext& ctx) override {
        start_time_ = std::chrono::steady_clock::now();
        last_tick_time_ = start_time_;
        current_vel_xy_ = Eigen::Vector2d::Zero();
        current_z_speed_ = 0.0;
        is_first_tick_ = true;
    }

    void on_exit(RobotContext& ctx) override {
        ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt,
                                  std::nullopt, CmdFrame::ENU);
    }

    using AllowedEvents = std::tuple<dk::TickEvent, SetPosVelEvent>;

    SetPosVelEvent event_;

    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    // [修改点 2]：将控制相关的持续性状态变量移到父状态中
    std::chrono::time_point<std::chrono::steady_clock> last_tick_time_{};
    Eigen::Vector2d current_vel_xy_ = Eigen::Vector2d::Zero();
    double current_z_speed_ = 0.0;  // 新增：用于 Z 轴的平滑加减速
    bool is_first_tick_ = true;

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    StateAction on_event(const SetPosVelEvent& e, RobotContext& ctx);

    StateAction get_next_state();

    class MoveState;
    class LaterTurnState;

    void set_target(const SetPosVelEvent& e);
};

template <typename ParentState>
class PosVelState<ParentState>::MoveState
    : public dk::BaseState<RobotContext, MoveState, PosVelState<ParentState>> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;

    // 父状态已接管初始化，这里可以保持为空，或者仅做特定子状态的日志等
    void on_enter(RobotContext& ctx);

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    StateAction on_event(const SetPosVelEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "指点移动"; }

   private:
    void base_move(RobotContext& ctx);
};

template <typename ParentState>
class PosVelState<ParentState>::LaterTurnState
    : public dk::BaseState<RobotContext, LaterTurnState,
                           PosVelState<ParentState>> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;
    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "指点转向"; }

    void on_enter(RobotContext& ctx);
};

const double POSVEL_TIMEOUT = 3.0;
const double POS_TOLERANCE = 1.0;
const double YAW_TOLERANCE = 0.08;  // Approx 4.5 degrees

template <typename ParentState>
PosVelState<ParentState>::PosVelState(SetPosVelEvent e)
    : event_(e), start_time_(std::chrono::steady_clock::now()) {
    event_.yaw = state_utils::yaw_ned_to_enu(event_.yaw);
}

// 工具函数，设置posvel目标
template <typename ParentState>
void PosVelState<ParentState>::set_target(const SetPosVelEvent& e) {
    event_ = e;
    start_time_ = std::chrono::steady_clock::now();
    event_.yaw = state_utils::yaw_ned_to_enu(event_.yaw);

    // 由于目标更新，重置首次 tick 标志，使得下一次重新计算 dt
    // 此时不会归零当前速度 (current_xy_speed_ /
    // current_z_speed_)，保证切换目标时的平滑性
    is_first_tick_ = true;
}

template <typename ParentState>
StateAction PosVelState<ParentState>::on_event(const SetPosVelEvent& e,
                                               RobotContext& ctx) {
    set_target(e);
    if (!ctx.engine->is_active_state<PosVelState<ParentState>::MoveState>()) {
        return this->template step<PosVelState<ParentState>::MoveState>(
            std::tuple<>(), std::tuple<>());
    }
    return StateAction::handled();
}

template <typename ParentState>
StateAction PosVelState<ParentState>::get_next_state() {
    if constexpr (std::is_same_v<ParentState, HoverState>)
        return this->template step<HoverState>();
    if constexpr (std::is_same_v<ParentState, WaypointState>)
        return this->template step<WaypointState::LiftingState>();

    spdlog::warn("[PosvelState] Unexpected parent, switch to Hover");
    return this->template step<HoverState>();
}

template <typename ParentState>
StateAction PosVelState<ParentState>::on_event(const dk::TickEvent& e,
                                               RobotContext& ctx) {
    if (state_utils::get_time_span(start_time_) > POSVEL_TIMEOUT) {
        spdlog::error("[PosvelState] set_posvel timeout!");
        return get_next_state();
    }
    return StateAction::handled();
}

const double FIX_YAW_DIST = 1.0;

// 工具函数，调用ctx.tracker
template <typename ParentState>
void PosVelState<ParentState>::MoveState::base_move(RobotContext& ctx) {
    auto p = this->parent();
    auto current_pos = ctx.pos_enu.get();
    auto target_pos = state_utils::gps_to_enu(ctx.lon_lat_alt.get(),
                                              current_pos, p->event_.pos);
    // [可选] 如果固定角度，就传；否则传 nullopt，Tracker 会自动计算朝向
    std::optional<double> target_yaw = std::nullopt;
    if (p->event_.fix_yaw) {
        target_yaw = p->event_.yaw;
    }
    // 只在进入状态时下发一次指令 (Tracker 内部会接管 50Hz 的连续控制)
    ctx.tracker->send_pos_cmd(
        target_pos,    // 目标位置
        target_yaw,    // 目标偏航角
        std::nullopt,  // 没有前馈速度
        std::nullopt,  // 没有前馈角速度
        p->event_.vel,  // XY 巡航速度限制 (你原代码里的 target_speed_mag)
        std::nullopt,  // Z 巡航速度限制
        CmdFrame::ENU);
}

template <typename ParentState>
void PosVelState<ParentState>::MoveState::on_enter(RobotContext& ctx) {
    base_move(ctx);
}

template <typename ParentState>
StateAction PosVelState<ParentState>::MoveState::on_event(
    const SetPosVelEvent& e, RobotContext& ctx) {
    this->parent()->set_target(e);
    base_move(ctx);
    return StateAction::handled();
}

template <typename ParentState>
StateAction PosVelState<ParentState>::MoveState::on_event(
    const dk::TickEvent& e, RobotContext& ctx) {
    auto p = this->parent();
    auto current_pos = ctx.pos_enu.get();
    double current_yaw = ctx.yaw_enu;
    auto target_pos = state_utils::gps_to_enu(ctx.lon_lat_alt.get(),
                                              current_pos, p->event_.pos);

    // 计算误差向量
    Eigen::Vector3d pos_err = target_pos - current_pos;
    Eigen::Vector2d xy_err = pos_err.head<2>();
    double dist_3d = pos_err.norm();
    double dist_xy = xy_err.norm();

    bool arrive = ctx.robot->is_alt_enable() ? (dist_3d < POS_TOLERANCE)
                                             : (dist_xy < POS_TOLERANCE);
    // 1. 检查是否到达
    if (arrive) {
        if (p->event_.fix_yaw) {
            return p->get_next_state();
        } else {
            return this->template step<LaterTurnState>();
        }
    }

    return StateAction::unhandled();  // 让父状态处理超时
}

template <typename ParentState>
void PosVelState<ParentState>::LaterTurnState::on_enter(RobotContext& ctx) {
    auto p = this->parent();
    double target_yaw = p->event_.yaw;
    auto current_pos = ctx.pos_enu.get();
    auto target_pos = state_utils::gps_to_enu(ctx.lon_lat_alt.get(),
                                              current_pos, p->event_.pos);
}

template <typename ParentState>
StateAction PosVelState<ParentState>::LaterTurnState::on_event(
    const dk::TickEvent& e, RobotContext& ctx) {
    auto p = this->parent();
    double current_yaw = ctx.yaw_enu;
    double target_yaw = p->event_.yaw;

    if (state_utils::get_yaw_diff(target_yaw, current_yaw) < YAW_TOLERANCE) {
        return p->get_next_state();
    }

    ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), target_yaw, std::nullopt,
                              CmdFrame::ENU);

    return StateAction::unhandled();  // 父状态处理超时
}