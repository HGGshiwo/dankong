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
#include "dk/report.hpp"
#include "features/tracker/config.hpp"
#include "mavlink/imavlink.hpp"
#include "states/state_utils.hpp"
#include "utils/dirty_var.hpp"
#include "utils/thread_runner.hpp"

class ThreadedTracker : public IThreadRunner {
   private:
    enum class CtrlMode { NONE, POSITION, VELOCITY };

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
        std::optional<double> cruise_speed_xy = std::nullopt;
        std::optional<double> cruise_speed_z = std::nullopt;
        std::optional<double> max_acc_xy = std::nullopt;
        std::optional<double> max_decel_xy = std::nullopt;
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
    state_utils::AngleController angle_ctrl_;

    double active_max_vel_xy_ = 0.0;
    double active_max_vel_z_ = 0.0;

    bool auto_heading_ = false;
    bool pos_cmd_finished_ = true;

    // 用来暂存外部下发的新指令
    PendingCmd pending_cmd_;

    std::chrono::steady_clock::time_point last_command_time_;
    Eigen::Vector4d last_cmd_vel_ = Eigen::Vector4d::Zero();
    double last_yaw_ = 0.0;

   public:
    ThreadedTracker(const TrackerConfig& config, ITrackerRuntime* runtime,
                    DirtyVar<Eigen::Vector3d>& pos_enu,
                    std::atomic<double>& yaw_enu, bool use_thread = true)
        : IThreadRunner(use_thread),
          config_(config),
          runtime_(runtime),
          pos_enu_(pos_enu),
          yaw_enu_(yaw_enu) {}

    ~ThreadedTracker() { stop(); }

    void on_start() override {
        last_cmd_vel_ = Eigen::Vector4d::Zero();
        last_yaw_ = yaw_enu_.load();
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
    void send_pos_cmd(const Eigen::Vector3d& pos, std::optional<double> yaw,
                      std::optional<double> ff_yaw_rate,
                      std::optional<Eigen::Vector3d> ff_vel,
                      std::optional<double> cruise_speed_xy,
                      std::optional<double> cruise_speed_z, CmdFrame frame,
                      Eigen::Vector3d gamma = {1.0, 1.0, 1.0},
                      std::optional<double> max_acc_xy = std::nullopt,
                      std::optional<double> max_decel_xy = std::nullopt) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_cmd_.mode = CtrlMode::POSITION;
        pending_cmd_.frame = frame;
        pending_cmd_.vec3_cmd = pos;
        pending_cmd_.yaw = yaw;
        pending_cmd_.yaw_rate = ff_yaw_rate;
        pending_cmd_.ff_vel = ff_vel;
        pending_cmd_.cruise_speed_xy = cruise_speed_xy;
        pending_cmd_.cruise_speed_z = cruise_speed_z;
        pending_cmd_.gamma = gamma;
        pending_cmd_.max_acc_xy = max_acc_xy;
        pending_cmd_.max_decel_xy = max_decel_xy;
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

            // [修改点 1] 将 active_max_vel_xy_ 传入
            // compute_pos_control，仅用于限制纠偏反馈
            raw_cmd = compute_pos_control(current_pose, target_pose_enu_,
                                          body_target_vel, ctrl_yaw_, gamma_,
                                          active_max_vel_xy_);

        } else if (current_mode_ == CtrlMode::VELOCITY) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_command_time_;

            if (elapsed.count() > config_.command_timeout_sec.get()) {
                current_mode_ = CtrlMode::NONE;
                send_zero_velocity();
                return;
            }

            raw_cmd = compute_vel_control(current_pose, target_pose_enu_,
                                          body_target_vel, ctrl_yaw_);
        }

        // [修改点 2] 前馈已经加到 raw_cmd 里了。这里调用底层运动学约束时，
        // 必须放开给物理绝对极限
        // config_.max_vel_xy.get()，确保前馈+反馈不被误砍！
        Eigen::Vector4d safe_cmd = apply_kinematic_constraints(
            raw_cmd, config_.max_vel_xy.get(), active_max_vel_z_,
            current_pose.w(), dt);

        runtime_->cmd_vel(safe_cmd);
    }

   private:
    void apply_pending_command() {
        current_mode_ = pending_cmd_.mode;
        ctrl_yaw_ = pending_cmd_.yaw.has_value();
        target_frame_ = pending_cmd_.frame;
        gamma_ = pending_cmd_.gamma;
        last_command_time_ = std::chrono::steady_clock::now();
        gamma_ = pending_cmd_.gamma;
        max_acc_xy_ = pending_cmd_.max_acc_xy;
        max_decel_xy_ = pending_cmd_.max_decel_xy;

        if (current_mode_ == CtrlMode::POSITION) {
            pos_cmd_finished_ = false;
            auto_heading_ = (!pending_cmd_.yaw.has_value() &&
                             !pending_cmd_.yaw_rate.has_value()) ||
                            !config_.is_omnidirectional.get();

            active_max_vel_xy_ = std::min(
                pending_cmd_.cruise_speed_xy.value_or(config_.max_vel_xy.get()),
                config_.max_vel_xy.get());
            active_max_vel_z_ = std::min(
                pending_cmd_.cruise_speed_z.value_or(config_.max_vel_z.get()),
                config_.max_vel_z.get());

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

            active_max_vel_xy_ = config_.max_vel_xy.get();
            active_max_vel_z_ = config_.max_vel_z.get();

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

    // [修改点 3] 增加 limit_fb_vel_xy 参数
    Eigen::Vector4d compute_pos_control(const Eigen::Vector4d& current_pose,
                                        const Eigen::Vector4d& target_pose,
                                        const Eigen::Vector4d& ff_vel_body,
                                        bool ctrl_yaw, Eigen::Vector3d gamma,
                                        double limit_fb_vel_xy) {
        Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
        double dx = target_pose.x() - current_pose.x();
        double dy = target_pose.y() - current_pose.y();
        double dz = target_pose.z() - current_pose.z();
        double dyaw = ctrl_yaw ? angle_ctrl_.get_distance(current_pose.w(),
                                                          target_pose.w())
                               : 0.0;

        double current_yaw = current_pose.w();
        double e_body_x =
            std::cos(current_yaw) * dx + std::sin(current_yaw) * dy;
        double e_body_y =
            -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy;

        double speed_scale = 1.0;
        if ((!config_.is_omnidirectional.get() || auto_heading_) &&
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
        double kp_z_active = config_.kp_z.get() * gamma.y();
        double kp_yaw_active = config_.kp_yaw.get() * gamma.z();

        // ==========================================
        // [核心修改区] 纯反馈速度的计算与限幅
        // ==========================================
        double fb_x = kp_xy_active * e_body_x;
        double fb_y = kp_xy_active * e_body_y;

        // 1. 保护 FOV：用下发的 limit_fb_vel_xy
        // 截断单纯因为位置误差产生的追击速度
        double fb_speed_xy = std::hypot(fb_x, fb_y);
        if (fb_speed_xy > limit_fb_vel_xy && fb_speed_xy > 1e-4) {
            fb_x = (fb_x / fb_speed_xy) * limit_fb_vel_xy;
            fb_y = (fb_y / fb_speed_xy) * limit_fb_vel_xy;
            fb_speed_xy = limit_fb_vel_xy;  // 更新一下当前的 feedback speed
        }

        // 2. 相对目标刹车防过冲：刹车减速曲线也必须只针对反馈通道
        // 如果把刹车算在带有 ff_vel
        // 的总速度上，到了目标正上方飞机会强行悬停，直接放跑移动目标！
        double dist_xy = std::hypot(dx, dy);
        if (dist_xy > 1e-4) {
            double max_v_brake_xy =
                std::sqrt(2.0 * config_.max_decel_xy.get() * dist_xy);
            if (fb_speed_xy > max_v_brake_xy) {
                fb_x = (fb_x / fb_speed_xy) * max_v_brake_xy;
                fb_y = (fb_y / fb_speed_xy) * max_v_brake_xy;
            }
        }

        // ==========================================
        // 前馈融合组装指令
        // ==========================================
        if (config_.is_omnidirectional.get()) {
            // 前馈与截断后的反馈相加
            cmd.x() = fb_x + ff_vel_body.x();
            cmd.y() = fb_y + ff_vel_body.y();
            cmd.z() = kp_z_active * dz + ff_vel_body.z();
            cmd.w() = kp_yaw_active * dyaw + ff_vel_body.w();
        } else {
            cmd.y() = 0.0;
            cmd.z() = kp_z_active * dz + ff_vel_body.z();
            cmd.x() = ff_vel_body.x() * std::cos(dyaw) + fb_x;

            if (ff_vel_body.norm() < 1e-3) {
                if (dist_xy > config_.pos_tolerance_m.get()) {
                    cmd.x() = fb_x;
                    cmd.w() = kp_yaw_active * dyaw;
                } else {
                    cmd.x() = 0.0;
                    cmd.w() = ctrl_yaw ? kp_yaw_active * dyaw : 0.0;
                }
            } else {
                cmd.w() = ff_vel_body.w() +
                          std::abs(cmd.x()) * kp_xy_active * e_body_y;
                if (ctrl_yaw) cmd.w() += kp_yaw_active * std::sin(dyaw);
            }
        }

        // Z 轴由于通常是世界系绝对高度，依然保留原来的独立刹车逻辑
        double max_v_brake_z =
            std::sqrt(2.0 * config_.max_decel_z.get() * std::abs(dz));
        cmd.z() = std::clamp(cmd.z(), -max_v_brake_z, max_v_brake_z);

        if (ctrl_yaw) {
            double max_v_brake_yaw =
                std::sqrt(2.0 * config_.max_acc_yaw.get() * std::abs(dyaw));
            cmd.w() = std::clamp(cmd.w(), -max_v_brake_yaw, max_v_brake_yaw);
        }

        cmd.x() *= speed_scale;
        cmd.y() *= speed_scale;

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

    Eigen::Vector4d apply_kinematic_constraints(Eigen::Vector4d raw_cmd,
                                                double limit_v_xy,
                                                double limit_v_z,
                                                double current_yaw, double dt) {
        double speed_xy =
            std::sqrt(raw_cmd.x() * raw_cmd.x() + raw_cmd.y() * raw_cmd.y());
        if (speed_xy > limit_v_xy && speed_xy > 1e-4) {
            raw_cmd.x() = (raw_cmd.x() / speed_xy) * limit_v_xy;
            raw_cmd.y() = (raw_cmd.y() / speed_xy) * limit_v_xy;
        }
        raw_cmd.z() = std::clamp(raw_cmd.z(), -limit_v_z, limit_v_z);
        raw_cmd.w() = std::clamp(raw_cmd.w(), -config_.max_vel_yaw.get(),
                                 config_.max_vel_yaw.get());

        double delta_yaw = current_yaw - last_yaw_;
        double cy = std::cos(delta_yaw);
        double sy = std::sin(delta_yaw);

        double rotated_last_vx =
            last_cmd_vel_.x() * cy + last_cmd_vel_.y() * sy;
        double rotated_last_vy =
            -last_cmd_vel_.x() * sy + last_cmd_vel_.y() * cy;

        last_cmd_vel_.x() = rotated_last_vx;
        last_cmd_vel_.y() = rotated_last_vy;
        last_yaw_ = current_yaw;

        double dvx = raw_cmd.x() - last_cmd_vel_.x();
        double dvy = raw_cmd.y() - last_cmd_vel_.y();
        double dv_xy = std::sqrt(dvx * dvx + dvy * dvy);

        double limit_accel = max_acc_xy_.value_or(config_.max_acc_xy.get());
        if (last_cmd_vel_.head<2>().norm() > 0.01 &&
            (last_cmd_vel_.x() * dvx + last_cmd_vel_.y() * dvy) < 0) {
            limit_accel = max_decel_xy_.value_or(config_.max_decel_xy.get());
        }

        double max_dv_xy = limit_accel * dt;
        if (dv_xy > max_dv_xy && dv_xy > 1e-6) {
            raw_cmd.x() = last_cmd_vel_.x() + (dvx / dv_xy) * max_dv_xy;
            raw_cmd.y() = last_cmd_vel_.y() + (dvy / dv_xy) * max_dv_xy;
        }

        auto rate_limit = [](double current, double target, double max_acc,
                             double dt) {
            double max_delta = max_acc * dt;
            return std::clamp(target, current - max_delta, current + max_delta);
        };

        raw_cmd.z() = rate_limit(last_cmd_vel_.z(), raw_cmd.z(),
                                 config_.max_acc_z.get(), dt);
        raw_cmd.w() = rate_limit(last_cmd_vel_.w(), raw_cmd.w(),
                                 config_.max_acc_yaw.get(), dt);

        last_cmd_vel_ = raw_cmd;
        return raw_cmd;
    }

    void send_zero_velocity() {
        Eigen::Vector4d zero_cmd = Eigen::Vector4d::Zero();
        runtime_->cmd_vel(zero_cmd);
        last_cmd_vel_ = zero_cmd;
        last_yaw_ = yaw_enu_.load();
    }
};
