#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>

#include "core/base_tracker.hpp"
#include "features/tracker/config.hpp"
#include "mavlink/imavlink.hpp"
#include "states/state_utils.hpp"
#include "utils/dirty_var.hpp"
#include "utils/thread_runner.hpp"

class ThreadedTracker : public IThreadRunner {
   private:
    enum class CtrlMode { NONE, POSITION, VELOCITY };

    // ==========================================
    // PID 状态暂存结构体
    // ==========================================
    struct PIDState {
        double integral = 0.0;
        double prev_error = 0.0;
        void reset() {
            integral = 0.0;
            prev_error = 0.0;
        }
    };

    // ==========================================
    // 指令暂存区结构体
    // ==========================================
    struct PendingCmd {
        CtrlMode mode = CtrlMode::NONE;
        CmdFrame frame = CmdFrame::ENU;
        Eigen::Vector3d vec3_cmd = Eigen::Vector3d::Zero();  // POS or VEL
        std::optional<double> yaw = std::nullopt;
        std::optional<double> yaw_rate =
            std::nullopt;  // Pos时的 ff_yaw_rate，Vel时的 yaw_rate
        std::optional<Eigen::Vector3d> ff_vel = Eigen::Vector3d::Zero();
        std::optional<double> fb_speed_limit_xy = std::nullopt;
        std::optional<double> fb_speed_limit_z = std::nullopt;
        std::optional<double> fb_speed_limit_yaw = std::nullopt;
        std::optional<double> max_acc_xy = std::nullopt;
        std::optional<double> max_decel_xy = std::nullopt;

        // [新增] 加加速度(Jerk)限制参数
        std::optional<double> max_jerk_xy = std::nullopt;
        std::optional<double> max_jerk_z = std::nullopt;

        Eigen::Vector3d gamma = {1.0, 1.0, 1.0};
    };

    const TrackerConfig& config_;
    ITrackerRuntime* runtime_;
    DirtyVar<Eigen::Vector3d>& pos_enu_;
    std::atomic<double>& yaw_enu_;

    std::mutex data_mutex_;

    // 控制运行状态
    CtrlMode current_mode_ = CtrlMode::NONE;
    bool ctrl_yaw_ = false;
    CmdFrame target_frame_ = CmdFrame::ENU;

    Eigen::Vector3d gamma_ = {1.0, 1.0, 1.0};
    Eigen::Vector4d target_pose_enu_;
    Eigen::Vector4d target_vel_;
    std::optional<double> max_acc_xy_ = std::nullopt;
    std::optional<double> max_decel_xy_ = std::nullopt;

    // [新增] 当前生效的 Jerk 限制
    std::optional<double> max_jerk_xy_ = std::nullopt;
    std::optional<double> max_jerk_z_ = std::nullopt;

    state_utils::AngleController angle_ctrl_;

    double fb_vel_limit_xy_ = 0.0;
    double fb_vel_limit_yaw_ = 0.0;
    double fb_vel_limit_z_ = 0.0;

    bool auto_heading_ = false;
    bool pos_cmd_finished_ = true;

    // 用来暂存外部下发的新指令
    PendingCmd pending_cmd_;

    double last_command_time_;

    Eigen::Vector4d last_cmd_vel_ = Eigen::Vector4d::Zero();
    Eigen::Vector3d last_cmd_acc_ =
        Eigen::Vector3d::Zero();  // [新增] 用于记录上一拍的加速度指令

    double last_yaw_ = 0.0;

    // 独立维护 X, Y, Z, Yaw 的 PID 状态
    PIDState pid_x_, pid_y_, pid_z_, pid_yaw_;

    std::shared_ptr<dk::ITimeProvider> time_provider_;

    bool is_pivoting_ = false;  // 用于记忆当前是否处于原地转向状态

   public:
    ThreadedTracker(const TrackerConfig& config, ITrackerRuntime* runtime,
                    DirtyVar<Eigen::Vector3d>& pos_enu,
                    std::atomic<double>& yaw_enu,
                    std::shared_ptr<dk::ITimeProvider> time_provider,
                    bool use_thread = true)
        : IThreadRunner(time_provider, use_thread),
          config_(config),
          runtime_(runtime),
          pos_enu_(pos_enu),
          yaw_enu_(yaw_enu),
          time_provider_(time_provider) {}

    ~ThreadedTracker() { stop(); }

    void on_start() override {
        last_cmd_vel_ = Eigen::Vector4d::Zero();
        last_cmd_acc_ = Eigen::Vector3d::Zero();  // [新增] 初始化加速度记录
        last_yaw_ = yaw_enu_.load();
        reset_all_pids();
    }

    void on_stop() override { send_zero_velocity(); }

    bool is_position_reached() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return pos_cmd_finished_;
    }

    // =========================================================================
    // 接口 1：位置控制 (仅仅写入暂存区)
    // gamma: KP系数衰减
    // =========================================================================
    // [修改] 增加 max_jerk_xy 和 max_jerk_z 传参
    void send_pos_cmd(const Eigen::Vector3d& pos, std::optional<double> yaw,
                      std::optional<double> ff_yaw_rate,
                      std::optional<Eigen::Vector3d> ff_vel,
                      std::optional<double> fb_speed_limit_xy,  // 限制反馈速度
                      std::optional<double> fb_speed_limit_z,
                      std::optional<double> fb_speed_limit_yaw, CmdFrame frame,
                      Eigen::Vector3d gamma = {1.0, 1.0, 1.0},
                      std::optional<double> max_acc_xy = std::nullopt,
                      std::optional<double> max_decel_xy = std::nullopt,
                      std::optional<double> max_jerk_xy = std::nullopt,
                      std::optional<double> max_jerk_z = std::nullopt) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_cmd_.mode = CtrlMode::POSITION;
        pending_cmd_.frame = frame;
        pending_cmd_.vec3_cmd = pos;
        pending_cmd_.yaw = yaw;
        pending_cmd_.yaw_rate = ff_yaw_rate;
        pending_cmd_.ff_vel = ff_vel;
        pending_cmd_.fb_speed_limit_xy = fb_speed_limit_xy;
        pending_cmd_.fb_speed_limit_z = fb_speed_limit_z;
        pending_cmd_.fb_speed_limit_yaw = fb_speed_limit_yaw;
        pending_cmd_.gamma = gamma;
        pending_cmd_.max_acc_xy = max_acc_xy;
        pending_cmd_.max_decel_xy = max_decel_xy;
        // [新增] 存入暂存区
        pending_cmd_.max_jerk_xy = max_jerk_xy;
        pending_cmd_.max_jerk_z = max_jerk_z;
    }

    // =========================================================================
    // 接口 2：速度控制 (仅仅写入暂存区)
    // =========================================================================
    void send_vel_cmd(const Eigen::Vector3d& vel, std::optional<double> yaw,
                      std::optional<double> yaw_rate, CmdFrame frame) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_cmd_.mode = CtrlMode::VELOCITY;
        pending_cmd_.frame = frame;
        pending_cmd_.vec3_cmd = vel;
        pending_cmd_.yaw = yaw;
        pending_cmd_.yaw_rate = yaw_rate;
    }

    void on_step(double dt) override {
        // ... (防止除0保护)
        if (dt < 1e-6) return;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (pending_cmd_.mode != CtrlMode::NONE) {
                apply_pending_command();
                pending_cmd_.mode = CtrlMode::NONE;
            }
        }

        if (current_mode_ == CtrlMode::NONE) return;

        Eigen::Vector4d current_pose;
        current_pose.head<3>() = pos_enu_.load();
        current_pose.w() = yaw_enu_.load();

        Eigen::Vector4d raw_cmd = Eigen::Vector4d::Zero();
        Eigen::Vector4d body_target_vel = target_vel_;

        if (target_frame_ == CmdFrame::ENU) {
            double cy = std::cos(-current_pose.w());
            double sy = std::sin(-current_pose.w());
            body_target_vel.x() = target_vel_.x() * cy - target_vel_.y() * sy;
            body_target_vel.y() = target_vel_.x() * sy + target_vel_.y() * cy;
        }

        if (current_mode_ == CtrlMode::POSITION) {
            double dx = target_pose_enu_.x() - current_pose.x();
            double dy = target_pose_enu_.y() - current_pose.y();
            double dz = target_pose_enu_.z() - current_pose.z();

            if (auto_heading_) {
                double dist_xy = std::hypot(dx, dy);
                if (dist_xy > config_.auto_heading_enable_dist_m.get()) {
                    target_pose_enu_.w() = std::atan2(dy, dx);
                }
                ctrl_yaw_ = true;
            }

            double dyaw = ctrl_yaw_
                              ? angle_ctrl_.get_distance(current_pose.w(),
                                                         target_pose_enu_.w())
                              : 0.0;
            double pos_error = std::sqrt(dx * dx + dy * dy + dz * dz);
            bool yaw_reached =
                (!ctrl_yaw_) || auto_heading_ ||
                (std::abs(dyaw) <= config_.yaw_tolerance_rad.get());

            if (pos_error <= config_.pos_tolerance_m.get() && yaw_reached) {
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    pos_cmd_finished_ = true;
                }
                current_mode_ = CtrlMode::NONE;
                send_zero_velocity();
                return;
            }

            raw_cmd = compute_pos_control(current_pose, target_pose_enu_,
                                          body_target_vel, ctrl_yaw_, gamma_,
                                          fb_vel_limit_xy_, fb_vel_limit_z_,
                                          fb_vel_limit_yaw_, dt);

        } else if (current_mode_ == CtrlMode::VELOCITY) {
            double now = time_provider_->now();
            double elapsed = now - last_command_time_;

            if (elapsed > config_.command_timeout_sec.get()) {
                current_mode_ = CtrlMode::NONE;
                send_zero_velocity();
                return;
            }

            raw_cmd = compute_vel_control(current_pose, target_pose_enu_,
                                          body_target_vel, ctrl_yaw_);
        }

        Eigen::Vector4d safe_cmd = apply_kinematic_constraints(
            raw_cmd, config_.max_vel_xy.get(), config_.max_vel_z.get(),
            config_.max_vel_yaw.get(), current_pose.w(), dt);

        runtime_->cmd_vel(safe_cmd);
    }

    void send_zero_velocity() {
        Eigen::Vector4d zero_cmd = Eigen::Vector4d::Zero();
        runtime_->cmd_vel(zero_cmd);
        last_cmd_vel_ = zero_cmd;
        last_cmd_acc_ = Eigen::Vector3d::Zero();  // [新增]
        last_yaw_ = yaw_enu_.load();
        reset_all_pids();
    }

   private:
    void reset_all_pids() {
        pid_x_.reset();
        pid_y_.reset();
        pid_z_.reset();
        pid_yaw_.reset();
        is_pivoting_ = false;
    }

    double calculate_pid(double error, PIDState& state, double kp, double ki,
                         double kd, double max_i, double dt, bool enable_i) {
        if (enable_i) {
            state.integral += error * dt;
            state.integral = std::clamp(state.integral, -max_i, max_i);
        }

        double derivative = (dt > 1e-6) ? (error - state.prev_error) / dt : 0.0;
        state.prev_error = error;

        return kp * error + ki * state.integral + kd * derivative;
    }

    void apply_pending_command() {
        if (pending_cmd_.mode == CtrlMode::POSITION &&
            current_mode_ != CtrlMode::POSITION) {
            reset_all_pids();
        }

        current_mode_ = pending_cmd_.mode;
        ctrl_yaw_ = pending_cmd_.yaw.has_value();
        target_frame_ = pending_cmd_.frame;
        gamma_ = pending_cmd_.gamma;
        last_command_time_ = time_provider_->now();
        max_acc_xy_ = pending_cmd_.max_acc_xy;
        max_decel_xy_ = pending_cmd_.max_decel_xy;

        // [新增] 提取外部传入的 Jerk 限制
        max_jerk_xy_ = pending_cmd_.max_jerk_xy;
        max_jerk_z_ = pending_cmd_.max_jerk_z;

        if (current_mode_ == CtrlMode::POSITION) {
            pos_cmd_finished_ = false;
            auto_heading_ = (!pending_cmd_.yaw.has_value() &&
                             !pending_cmd_.yaw_rate.has_value()) ||
                            !config_.is_omnidirectional.get();

            fb_vel_limit_xy_ = std::min(pending_cmd_.fb_speed_limit_xy.value_or(
                                            config_.max_vel_xy.get()),
                                        config_.max_vel_xy.get());
            fb_vel_limit_z_ = std::min(
                pending_cmd_.fb_speed_limit_z.value_or(config_.max_vel_z.get()),
                config_.max_vel_z.get());
            fb_vel_limit_yaw_ =
                std::min(pending_cmd_.fb_speed_limit_yaw.value_or(
                             config_.max_vel_yaw.get()),
                         config_.max_vel_yaw.get());

            if (target_frame_ == CmdFrame::BODY) {
                Eigen::Vector4d body_offset = Eigen::Vector4d::Zero();
                body_offset.head<3>() = pending_cmd_.vec3_cmd;
                if (ctrl_yaw_) body_offset.w() = pending_cmd_.yaw.value();

                Eigen::Vector4d current_odom;
                current_odom.head<3>() = pos_enu_.load();
                current_odom.w() = yaw_enu_.load();
                target_pose_enu_ =
                    state_utils::body_to_enu(body_offset, current_odom);
            } else {
                target_pose_enu_.head<3>() = pending_cmd_.vec3_cmd;
                if (ctrl_yaw_) target_pose_enu_.w() = pending_cmd_.yaw.value();
            }

            target_vel_.head<3>() =
                pending_cmd_.ff_vel.value_or(Eigen::Vector3d::Zero());
            target_vel_.w() = pending_cmd_.yaw_rate.value_or(0.0);

        } else if (current_mode_ == CtrlMode::VELOCITY) {
            pos_cmd_finished_ = true;
            auto_heading_ = false;

            target_vel_.head<3>() = pending_cmd_.vec3_cmd;
            target_vel_.w() = pending_cmd_.yaw_rate.value_or(0.0);

            if (ctrl_yaw_) {
                if (target_frame_ == CmdFrame::BODY) {
                    target_pose_enu_.w() = state_utils::norm_yaw(
                        yaw_enu_.load() + pending_cmd_.yaw.value());
                } else {
                    target_pose_enu_.w() = pending_cmd_.yaw.value();
                }
            }
        }
    }

    Eigen::Vector4d compute_pos_control(const Eigen::Vector4d& current_pose,
                                        const Eigen::Vector4d& target_pose,
                                        const Eigen::Vector4d& ff_vel_body,
                                        bool ctrl_yaw, Eigen::Vector3d gamma,
                                        double limit_fb_vel_xy,
                                        double limit_fb_vel_z,
                                        double limit_fb_vel_yaw, double dt) {
        Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
        double dx = target_pose.x() - current_pose.x();
        double dy = target_pose.y() - current_pose.y();
        double dz = target_pose.z() - current_pose.z();
        double current_yaw = current_pose.w();

        // ==========================================
        // 1. Z轴高度控制 (全向与阿克曼通用)
        // ==========================================
        double kp_z_active = config_.kp_z.get() * gamma.y();
        double fb_z =
            calculate_pid(dz, pid_z_, kp_z_active, config_.ki_z.get(),
                          config_.kd_z.get(), config_.max_i_z.get(), dt, true);
        fb_z = std::clamp(fb_z, -limit_fb_vel_z, limit_fb_vel_z);

        cmd.z() = fb_z + ff_vel_body.z();
        double max_v_brake_z =
            std::sqrt(2.0 * config_.max_decel_z.get() * std::abs(dz));
        cmd.z() = std::clamp(cmd.z(), -max_v_brake_z, max_v_brake_z);

        // ==========================================
        // 2. 平面控制 (按车型分支)
        // ==========================================
        if (config_.is_omnidirectional.get()) {
            // --- 全向车独立 PID 控制逻辑 ---
            double dyaw = ctrl_yaw ? angle_ctrl_.get_distance(current_yaw,
                                                              target_pose.w())
                                   : 0.0;

            // 航向误差过大时的降速逻辑
            double speed_scale = 1.0;
            if (auto_heading_ &&
                std::hypot(dx, dy) >= config_.pos_tolerance_m.get()) {
                double yaw_diff = std::abs(dyaw);
                if (yaw_diff >= config_.yaw_zero_speed_tol.get()) {
                    speed_scale = 0.0;
                } else if (yaw_diff > config_.yaw_full_speed_tol.get()) {
                    speed_scale =
                        1.0 - (yaw_diff - config_.yaw_full_speed_tol.get()) /
                                  (config_.yaw_zero_speed_tol.get() -
                                   config_.yaw_full_speed_tol.get());
                }
            }

            double kp_xy_active = config_.kp_xy.get() * gamma.x();
            bool enable_xy_i = (speed_scale > 1e-3);
            double pid_enu_x = calculate_pid(
                dx, pid_x_, kp_xy_active, config_.ki_xy.get(),
                config_.kd_xy.get(), config_.max_i_xy.get(), dt, enable_xy_i);
            double pid_enu_y = calculate_pid(
                dy, pid_y_, kp_xy_active, config_.ki_xy.get(),
                config_.kd_xy.get(), config_.max_i_xy.get(), dt, enable_xy_i);

            double fb_x = std::cos(current_yaw) * pid_enu_x +
                          std::sin(current_yaw) * pid_enu_y;
            double fb_y = -std::sin(current_yaw) * pid_enu_x +
                          std::cos(current_yaw) * pid_enu_y;

            double fb_speed_xy = std::hypot(fb_x, fb_y);
            if (fb_speed_xy > limit_fb_vel_xy && fb_speed_xy > 1e-4) {
                fb_x = (fb_x / fb_speed_xy) * limit_fb_vel_xy;
                fb_y = (fb_y / fb_speed_xy) * limit_fb_vel_xy;
                fb_speed_xy = limit_fb_vel_xy;
            }

            double dist_xy = std::hypot(dx, dy);
            if (dist_xy > 1e-4) {
                double max_v_brake_xy = std::sqrt(
                    2.0 * max_decel_xy_.value_or(config_.max_decel_xy.get()) *
                    dist_xy);
                if (fb_speed_xy > max_v_brake_xy) {
                    fb_x = (fb_x / fb_speed_xy) * max_v_brake_xy;
                    fb_y = (fb_y / fb_speed_xy) * max_v_brake_xy;
                }
            }

            double kp_yaw_active = config_.kp_yaw.get() * gamma.z();
            double fb_yaw = calculate_pid(
                dyaw, pid_yaw_, kp_yaw_active, config_.ki_yaw.get(),
                config_.kd_yaw.get(), config_.max_i_yaw.get(), dt, true);
            fb_yaw = std::clamp(fb_yaw, -limit_fb_vel_yaw, limit_fb_vel_yaw);

            cmd.x() = (fb_x + ff_vel_body.x()) * speed_scale;
            cmd.y() = (fb_y + ff_vel_body.y()) * speed_scale;
            cmd.w() = fb_yaw + ff_vel_body.w();

            // 全向车的 Yaw 刹车限幅
            if (ctrl_yaw) {
                double max_v_brake_yaw =
                    std::sqrt(2.0 * config_.max_acc_yaw.get() * std::abs(dyaw));
                cmd.w() =
                    std::clamp(cmd.w(), -max_v_brake_yaw, max_v_brake_yaw);
            }

        } else {
            // --- 阿克曼车纯追踪 (Pure Pursuit) 逻辑 ---
            double dist_to_target = std::hypot(dx, dy);

            if (dist_to_target < config_.pos_tolerance_m.get()) {
                // --- 1. 已到达目标点位置，平滑停车 ---
                cmd.x() = ff_vel_body.x();
                cmd.w() = 0.0;
                is_pivoting_ = false;  // 到达终点，重置状态

                if (ctrl_yaw) {
                    double dyaw =
                        angle_ctrl_.get_distance(current_yaw, target_pose.w());
                    double kp_yaw_active = config_.kp_yaw.get() * gamma.z();
                    cmd.w() = calculate_pid(dyaw, pid_yaw_, kp_yaw_active,
                                            config_.ki_yaw.get(),
                                            config_.kd_yaw.get(),
                                            config_.max_i_yaw.get(), dt, true);
                    cmd.w() = std::clamp(cmd.w(), -limit_fb_vel_yaw,
                                         limit_fb_vel_yaw);

                    double max_v_brake_yaw = std::sqrt(
                        2.0 * config_.max_acc_yaw.get() * std::abs(dyaw));
                    cmd.w() =
                        std::clamp(cmd.w(), -max_v_brake_yaw, max_v_brake_yaw);
                }
            } else {
                // --- 2. 未到达目标点，计算运动学 ---
                double target_angle = std::atan2(dy, dx);

                // 【修复1：规范化角度到 0~2pi】
                double alpha =
                    state_utils::norm_yaw(target_angle - current_yaw);

                // 【修复2：强制转换到 -pi ~ pi，获取最短转角误差】
                if (alpha > M_PI) {
                    alpha -= 2.0 * M_PI;
                }

                // ==========================================
                // 【双阈值滞回状态机：只处理前进与原地转向】
                // ==========================================
                const double PIVOT_ENTER_TOL =
                    30.0 * M_PI / 180.0;  // 误差 > 30度，进入原地转向
                const double PIVOT_EXIT_TOL =
                    5.0 * M_PI / 180.0;  // 误差 < 5度，退出原地转向，开始直行

                if (!is_pivoting_ && std::abs(alpha) > PIVOT_ENTER_TOL) {
                    is_pivoting_ = true;
                } else if (is_pivoting_ && std::abs(alpha) < PIVOT_EXIT_TOL) {
                    is_pivoting_ = false;
                }

                if (is_pivoting_) {
                    // ==========================================
                    // 状态 A：原地转向对齐目标
                    // ==========================================
                    cmd.x() = 0.0;  // 线速度死锁为0，触发底层 steering_mode = 1

                    double kp_yaw_active = config_.kp_yaw.get() * gamma.z();
                    // 此时 alpha 已经是正确的 [-pi, pi] 误差，直接送入 PID
                    cmd.w() = calculate_pid(alpha, pid_yaw_, kp_yaw_active,
                                            config_.ki_yaw.get(),
                                            config_.kd_yaw.get(),
                                            config_.max_i_yaw.get(), dt, true);
                    cmd.w() = std::clamp(cmd.w(), -limit_fb_vel_yaw,
                                         limit_fb_vel_yaw);

                } else {
                    // ==========================================
                    // 状态 B：纯追踪前行 (此时 abs(alpha) 必定 < 30 度)
                    // ==========================================
                    double target_v = config_.kp_xy.get() * dist_to_target;
                    // 取消倒车逻辑，强制速度下限为 0.0
                    target_v = std::clamp(target_v, 0.0, limit_fb_vel_xy);

                    double min_lookahead = config_.min_lookahead.get();
                    double max_lookahead = config_.max_lookahead.get();
                    double Ld = std::clamp(dist_to_target, min_lookahead,
                                           max_lookahead);

                    // 纯追踪计算角速度
                    double target_w = (2.0 * target_v * std::sin(alpha)) / Ld;

                    cmd.x() = target_v;
                    cmd.w() = std::clamp(target_w, -limit_fb_vel_yaw,
                                         limit_fb_vel_yaw);
                }

                cmd.x() += ff_vel_body.x();
            }

            cmd.y() = 0.0;
            cmd.w() += ff_vel_body.w();
        }
        return cmd;
    }

    Eigen::Vector4d compute_vel_control(const Eigen::Vector4d& current_pose,
                                        const Eigen::Vector4d& target_pose,
                                        const Eigen::Vector4d& target_vel_body,
                                        bool ctrl_yaw) {
        Eigen::Vector4d cmd = target_vel_body;
        if (ctrl_yaw) {
            double dyaw =
                angle_ctrl_.get_distance(current_pose.w(), target_pose.w());
            cmd.w() = config_.kp_yaw.get() * dyaw;
        }
        if (!config_.is_omnidirectional.get()) cmd.y() = 0.0;
        return cmd;
    }

    // =========================================================================
    // [修改核心区] 运动学约束（加入了 Jerk 限制和加速度积分逻辑）
    // =========================================================================
    Eigen::Vector4d apply_kinematic_constraints(Eigen::Vector4d raw_cmd,
                                                double limit_v_xy,
                                                double limit_v_z,
                                                double limit_v_yaw,
                                                double current_yaw, double dt) {
        // 1. 速度硬约束 (限速)
        double speed_xy =
            std::sqrt(raw_cmd.x() * raw_cmd.x() + raw_cmd.y() * raw_cmd.y());
        if (speed_xy > limit_v_xy && speed_xy > 1e-4) {
            raw_cmd.x() = (raw_cmd.x() / speed_xy) * limit_v_xy;
            raw_cmd.y() = (raw_cmd.y() / speed_xy) * limit_v_xy;
        }
        raw_cmd.z() = std::clamp(raw_cmd.z(), -limit_v_z, limit_v_z);
        raw_cmd.w() = std::clamp(raw_cmd.w(), -limit_v_yaw, limit_v_yaw);

        // 2.
        // 坐标系旋转处理：当机体发生偏航时，上一拍的速度和加速度必须旋转到新机体系下
        double delta_yaw = current_yaw - last_yaw_;
        double cy = std::cos(delta_yaw);
        double sy = std::sin(delta_yaw);

        double rotated_last_vx =
            last_cmd_vel_.x() * cy + last_cmd_vel_.y() * sy;
        double rotated_last_vy =
            -last_cmd_vel_.x() * sy + last_cmd_vel_.y() * cy;

        // [新增] 同样旋转上一拍的加速度
        double rotated_last_ax =
            last_cmd_acc_.x() * cy + last_cmd_acc_.y() * sy;
        double rotated_last_ay =
            -last_cmd_acc_.x() * sy + last_cmd_acc_.y() * cy;

        last_cmd_vel_.x() = rotated_last_vx;
        last_cmd_vel_.y() = rotated_last_vy;
        last_cmd_acc_.x() = rotated_last_ax;
        last_cmd_acc_.y() = rotated_last_ay;
        last_yaw_ = current_yaw;

        // ---------------------------------------------------------------------
        // 3. XY轴的加速度与加加速度(Jerk)限制
        // ---------------------------------------------------------------------
        // 计算如果不受任何限制，到达 raw_cmd 需要的“期望加速度”
        double desired_ax = (raw_cmd.x() - last_cmd_vel_.x()) / dt;
        double desired_ay = (raw_cmd.y() - last_cmd_vel_.y()) / dt;

        // 3.1 限制加速度大小 (Accel Limit)
        double limit_accel = max_acc_xy_.value_or(config_.max_acc_xy.get());
        // 判断是否为减速状态（点乘小于0且速度较大）
        if (last_cmd_vel_.head<2>().norm() > 0.01 &&
            (last_cmd_vel_.x() * desired_ax + last_cmd_vel_.y() * desired_ay) <
                0) {
            limit_accel = max_decel_xy_.value_or(config_.max_decel_xy.get());
        }

        double desired_a_mag = std::hypot(desired_ax, desired_ay);
        if (desired_a_mag > limit_accel && desired_a_mag > 1e-6) {
            desired_ax = (desired_ax / desired_a_mag) * limit_accel;
            desired_ay = (desired_ay / desired_a_mag) * limit_accel;
        }

        // 3.2 限制加加速度大小 (Jerk Limit)
        // [新增] 如果没有传入限制，给定一个极大值(1000.0)相当于不受限制
        double limit_jerk_xy = max_jerk_xy_.value_or(1000.0);

        double dax = desired_ax - last_cmd_acc_.x();
        double day = desired_ay - last_cmd_acc_.y();
        double da_mag = std::hypot(dax, day);
        double max_da = limit_jerk_xy * dt;  // 本周期内允许的最大加速度变化量

        if (da_mag > max_da && da_mag > 1e-6) {
            desired_ax = last_cmd_acc_.x() + (dax / da_mag) * max_da;
            desired_ay = last_cmd_acc_.y() + (day / da_mag) * max_da;
        }

        // 3.3 更新XY轴加速度和速度记录
        last_cmd_acc_.x() = desired_ax;
        last_cmd_acc_.y() = desired_ay;
        raw_cmd.x() = last_cmd_vel_.x() + desired_ax * dt;
        raw_cmd.y() = last_cmd_vel_.y() + desired_ay * dt;

        // ---------------------------------------------------------------------
        // 4. Z轴的加速度与加加速度(Jerk)限制
        // ---------------------------------------------------------------------
        double desired_az = (raw_cmd.z() - last_cmd_vel_.z()) / dt;
        double limit_acc_z = config_.max_acc_z.get();
        desired_az = std::clamp(desired_az, -limit_acc_z, limit_acc_z);

        double limit_jerk_z = max_jerk_z_.value_or(1000.0);
        double max_daz = limit_jerk_z * dt;
        desired_az = std::clamp(desired_az, last_cmd_acc_.z() - max_daz,
                                last_cmd_acc_.z() + max_daz);

        last_cmd_acc_.z() = desired_az;
        raw_cmd.z() = last_cmd_vel_.z() + desired_az * dt;

        // ---------------------------------------------------------------------
        // 5. Yaw轴 保持原有的速率限制 (通常Yaw直接限制角加速度即可)
        // ---------------------------------------------------------------------
        auto rate_limit = [](double current, double target, double max_acc,
                             double dt) {
            double max_delta = max_acc * dt;
            return std::clamp(target, current - max_delta, current + max_delta);
        };
        raw_cmd.w() = rate_limit(last_cmd_vel_.w(), raw_cmd.w(),
                                 config_.max_acc_yaw.get(), dt);

        last_cmd_vel_ = raw_cmd;
        return raw_cmd;
    }
};