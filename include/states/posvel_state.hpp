#pragma once

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>  // 包含 std::clamp, std::min, std::max
#include <cmath>
#include <optional>
#include <string_view>

#include "../robot_context.hpp"
#include "dk/state.hpp"
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
        ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d::Zero(), std::nullopt,
                            std::nullopt, std::nullopt, CmdFrame::ENU);
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
};

template <typename ParentState>
class PosVelState<ParentState>::MoveState
    : public dk::BaseState<RobotContext, MoveState, PosVelState<ParentState>> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;

    // 父状态已接管初始化，这里可以保持为空，或者仅做特定子状态的日志等
    void on_enter(RobotContext& ctx) override {}

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "指点移动"; }
};

template <typename ParentState>
class PosVelState<ParentState>::LaterTurnState
    : public dk::BaseState<RobotContext, LaterTurnState,
                           PosVelState<ParentState>> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;
    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "指点转向"; }
};

const double POSVEL_TIMEOUT = 3.0;
const double POS_TOLERANCE = 1.0;
const double YAW_TOLERANCE = 0.08;  // Approx 4.5 degrees
inline double CLOSE_THRESH = 1.0;   // 两个点是否足够近

template <typename ParentState>
PosVelState<ParentState>::PosVelState(SetPosVelEvent e)
    : event_(e), start_time_(std::chrono::steady_clock::now()) {
    event_.yaw = state_utils::yaw_ned_to_enu(event_.yaw);
}

template <typename ParentState>
StateAction PosVelState<ParentState>::on_event(const SetPosVelEvent& e,
                                               RobotContext& ctx) {
    event_ = e;
    start_time_ = std::chrono::steady_clock::now();
    event_.yaw = state_utils::yaw_ned_to_enu(event_.yaw);

    // 由于目标更新，重置首次 tick 标志，使得下一次重新计算 dt
    // 此时不会归零当前速度 (current_xy_speed_ /
    // current_z_speed_)，保证切换目标时的平滑性
    is_first_tick_ = true;

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
        return get_next_state();
    }
    return StateAction::handled();
}

inline double YAW_FULL_SPEED_TOLERANCE = 0.174;
inline double YAW_ZERO_SPEED_TOLERANCE = 0.785;
inline double MAX_Z_VEL = 1.0;    // Z轴最大速度限制 1.0 m/s
inline double Z_KP = 1.0;         // Z轴比例系数
inline double XY_KP = 1.0;        // XY水平比例系数
inline double MAX_ACCEL = 0.5;    // 最大加速度 m/s^2
inline double MAX_DECEL = 1.0;    // 最大减速度 m/s^2
inline double MAX_ACCEL_Z = 0.5;  // Z轴加减速度限制

template <typename ParentState>
StateAction PosVelState<ParentState>::MoveState::on_event(
    const dk::TickEvent& e, RobotContext& ctx) {
    auto p = this->parent();
    auto current_pos = ctx.pos_enu.get();
    double current_yaw = ctx.yaw_enu;
    auto target_pos = state_utils::gps_to_enu(ctx, p->event_.pos);

    // 计算误差向量
    Eigen::Vector3d pos_err = target_pos - current_pos;
    Eigen::Vector2d xy_err = pos_err.head<2>();
    double dist_3d = pos_err.norm();
    double dist_xy = xy_err.norm();

    // 1. 检查是否到达
    if (dist_3d < POS_TOLERANCE) {
        if (p->event_.fix_yaw) {
            return p->get_next_state();
        } else {
            return this->template step<LaterTurnState>();
        }
    }

    // 2. 计算目标航向角
    double target_yaw =
        p->event_.fix_yaw ? p->event_.yaw
        : dist_xy < CLOSE_THRESH
            ? current_yaw
            : state_utils::get_heading(pos_err.x(), pos_err.y());

    // 3. 计算时间增量 (dt)
    auto now = std::chrono::steady_clock::now();
    if (p->is_first_tick_) {
        p->last_tick_time_ = now;
        p->is_first_tick_ = false;
    }
    double dt = std::chrono::duration<double>(now - p->last_tick_time_).count();
    p->last_tick_time_ = now;
    if (dt > 0.1) dt = 0.1;  // 防止跳跃
    // ---------------- [XY轴 向量轨迹规划] ----------------
    // 4. 计算 XY 期望速度大小 (Magnitude)
    double target_speed_mag = p->event_.vel;
    if (dist_xy >= CLOSE_THRESH) {
        double yaw_diff = state_utils::get_yaw_diff(target_yaw, current_yaw);
        if (yaw_diff <= YAW_FULL_SPEED_TOLERANCE) {
            // 死区内：不降速
        } else if (yaw_diff >= YAW_ZERO_SPEED_TOLERANCE) {
            target_speed_mag = 0.0;
        } else {
            double scale =
                1.0 - (yaw_diff - YAW_FULL_SPEED_TOLERANCE) /
                          (YAW_ZERO_SPEED_TOLERANCE - YAW_FULL_SPEED_TOLERANCE);
            target_speed_mag *= scale;
        }
    }
    // 基于 v^2 = 2ax 计算最大安全刹车速度，并与近距离 P 控制防抖融合
    double braking_speed_xy =
        std::sqrt(2.0 * MAX_DECEL * std::max(0.0, dist_xy));
    double linear_speed_xy = dist_xy * XY_KP;
    double max_safe_speed = std::min(braking_speed_xy, linear_speed_xy);

    target_speed_mag = std::min(target_speed_mag, max_safe_speed);
    // 5. 构造 期望目标速度向量 (Target Velocity Vector)
    Eigen::Vector2d target_vel_xy = Eigen::Vector2d::Zero();
    if (dist_xy > 0.001) {
        target_vel_xy = xy_err.normalized() * target_speed_mag;
    }
    // 6. 应用 2D 向量加减速限制 (Vector Rate Limiter)
    Eigen::Vector2d dv_xy =
        target_vel_xy - p->current_vel_xy_;  // 期望速度增量向量
    double dv_mag = dv_xy.norm();

    if (dv_mag > 1e-6) {  // 避免除以0
        // 利用点乘判断加速还是减速：
        // 如果速度增量 dv_xy 和当前速度 current_vel_xy_ 夹角大于 90 度 (点乘 <
        // 0)，说明在刹车或急转弯
        double limit_accel = MAX_ACCEL;
        if (p->current_vel_xy_.norm() > 0.01 &&
            p->current_vel_xy_.dot(dv_xy) < 0) {
            limit_accel =
                MAX_DECEL;  // 刹车或大角度转向时允许用更大的 MAX_DECEL
        }

        // 计算最大允许的速度增量模长
        double max_dv = limit_accel * dt;

        // 如果计算出的增量超过了物理极限，则进行等比例截断 (保持方向不变)
        if (dv_mag > max_dv) {
            dv_xy = dv_xy * (max_dv / dv_mag);
        }
    }
    // 更新真实的当前速度向量
    p->current_vel_xy_ += dv_xy;
    // ---------------- [Z轴 平滑轨迹规划] ----------------
    // Z 轴是1维的，标量符号自带方向，原逻辑已能处理方向性
    double dist_z = std::abs(pos_err.z());
    double braking_speed_z =
        std::sqrt(2.0 * MAX_ACCEL_Z * std::max(0.0, dist_z));
    double linear_speed_z = dist_z * Z_KP;

    double target_z_speed_mag = std::min(braking_speed_z, linear_speed_z);
    target_z_speed_mag = std::min(MAX_Z_VEL, target_z_speed_mag);

    double target_z_speed =
        (pos_err.z() >= 0 ? 1.0 : -1.0) * target_z_speed_mag;
    if (target_z_speed > p->current_z_speed_) {
        p->current_z_speed_ += MAX_ACCEL_Z * dt;
        p->current_z_speed_ = std::min(p->current_z_speed_, target_z_speed);
    } else if (target_z_speed < p->current_z_speed_) {
        p->current_z_speed_ -= MAX_ACCEL_Z * dt;
        p->current_z_speed_ = std::max(p->current_z_speed_, target_z_speed);
    }

    // 7. 发送最终指令
    Eigen::Vector3d vel_cmd = Eigen::Vector3d::Zero();
    vel_cmd.head<2>() = p->current_vel_xy_;  // 填入计算好的 2D 速度向量
    vel_cmd.z() = p->current_z_speed_;       // 填入 1D Z轴速度
    ctx.robot->send_cmd(std::nullopt, vel_cmd, std::nullopt, target_yaw,
                        std::nullopt, CmdFrame::ENU);
    return StateAction::unhandled();  // 让父状态处理超时
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

    ctx.robot->send_cmd(std::nullopt, Eigen::Vector3d::Zero(), std::nullopt,
                        target_yaw, std::nullopt, CmdFrame::ENU);

    return StateAction::unhandled();  // 父状态处理超时
}