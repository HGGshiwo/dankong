#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "core/base_tracker.hpp"
#include "dk/report.hpp"
#include "mavlink/imavlink.hpp"
#include "states/state_utils.hpp"

struct TrackingConfig {
    bool is_omnidirectional = true;
    double kp_xy = 1.5;
    double kp_z = 1.0;
    double kp_yaw = 2.0;
    double command_timeout_sec = 0.5;
    int loop_rate_hz = 50;
    double pos_tolerance_m = 0.05;
    double auto_heading_enable_dist_m =
        1.0;  // autoheading的时候，距离目标x米位置固定朝向
    double yaw_tolerance_rad = 0.05;

    double max_vel_xy = 2.0;
    double max_vel_z = 1.0;
    double max_vel_yaw = 1.5;

    double max_acc_xy = 1.0;
    double max_acc_z = 0.8;
    double max_acc_yaw = 2.0;

    double max_decel_xy = 1.0;
    double max_decel_z = 0.8;          // XY最大减速度 (刹车/转弯用)
    double yaw_full_speed_tol = 0.17;  // 约10度 (在此偏差内满速)
    double yaw_zero_speed_tol = 0.78;  // 约45度 (偏差超过此值时完全停车原地转)
};

class ThreadedTracker {
   private:
    enum class CtrlMode { NONE, POSITION, VELOCITY };

    // ==========================================
    // 【新增】指令暂存区结构体
    // ==========================================
    struct PendingCmd {
        CtrlMode mode = CtrlMode::NONE;
        CmdFrame frame = CmdFrame::ENU;
        Eigen::Vector3d vec3_cmd = Eigen::Vector3d::Zero();  // POS or VEL
        std::optional<double> yaw;
        std::optional<double>
            yaw_rate;  // Pos时的 ff_yaw_rate，Vel时的 yaw_rate
        std::optional<Eigen::Vector3d> ff_vel;
        std::optional<double> cruise_speed_xy;
        std::optional<double> cruise_speed_z;
    };

    TrackingConfig config_;
    ITrackerRuntime* runtime_;
    dk::TrackedVar<Eigen::Vector3d>& pos_enu_;
    std::atomic<double>& yaw_enu_;

    std::thread control_thread_;
    std::atomic<bool> is_running_;
    std::mutex data_mutex_;

    // 控制运行状态
    CtrlMode current_mode_ = CtrlMode::NONE;
    bool ctrl_yaw_ = false;
    CmdFrame target_frame_ = CmdFrame::ENU;

    Eigen::Vector4d target_pose_enu_;
    Eigen::Vector4d target_vel_;
    state_utils::AngleController angle_ctrl_;

    double active_max_vel_xy_ = 0.0;
    double active_max_vel_z_ = 0.0;

    bool auto_heading_ = false;
    bool pos_cmd_finished_ = true;

    // 【新增】用来暂存外部下发的新指令
    PendingCmd pending_cmd_;

    std::chrono::steady_clock::time_point last_command_time_;
    Eigen::Vector4d last_cmd_vel_ = Eigen::Vector4d::Zero();
    double last_yaw_ = 0.0;

   public:
    ThreadedTracker(const TrackingConfig& config, ITrackerRuntime* runtime,
                    dk::TrackedVar<Eigen::Vector3d>& pos_enu,
                    std::atomic<double>& yaw_enu)
        : config_(config),
          runtime_(runtime),
          is_running_(false),
          pos_enu_(pos_enu),
          yaw_enu_(yaw_enu) {}

    ~ThreadedTracker() { stop(); }

    void start(bool start_thread = true) {
        if (is_running_) return;
        is_running_ = true;
        last_cmd_vel_ = Eigen::Vector4d::Zero();
        last_yaw_ = yaw_enu_.load();
        if (start_thread) {
            control_thread_ = std::thread(&ThreadedTracker::control_loop, this);
        }
    }

    void stop() {
        is_running_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        send_zero_velocity();
    }

    bool is_position_reached() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return pos_cmd_finished_;
    }

    // =========================================================================
    // 接口 1：位置控制 (仅仅写入暂存区)
    // =========================================================================
    void send_pos_cmd(const Eigen::Vector3d& pos, std::optional<double> yaw,
                      std::optional<double> ff_yaw_rate,
                      std::optional<Eigen::Vector3d> ff_vel,
                      std::optional<double> cruise_speed_xy,
                      std::optional<double> cruise_speed_z, CmdFrame frame) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_cmd_.mode = CtrlMode::POSITION;
        pending_cmd_.frame = frame;
        pending_cmd_.vec3_cmd = pos;
        pending_cmd_.yaw = yaw;
        pending_cmd_.yaw_rate = ff_yaw_rate;
        pending_cmd_.ff_vel = ff_vel;
        pending_cmd_.cruise_speed_xy = cruise_speed_xy;
        pending_cmd_.cruise_speed_z = cruise_speed_z;
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

    void process_control_step(double dt) {
        CtrlMode mode_copy;
        bool ctrl_yaw_copy;
        bool auto_heading_copy;
        CmdFrame frame_copy;
        Eigen::Vector4d target_pose_copy;
        Eigen::Vector4d target_vel_copy;
        std::chrono::steady_clock::time_point time_copy;
        double limit_v_xy, limit_v_z;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            // ===========================================================
            // 【核心处理】统一在此处“消耗”暂存区的新指令
            // ===========================================================
            if (pending_cmd_.mode != CtrlMode::NONE) {
                apply_pending_command();  // 应用指令，覆盖当前状态
                pending_cmd_.mode = CtrlMode::NONE;  // 消耗完毕，清空暂存区
            }

            mode_copy = current_mode_;

            // 无效/空闲状态，不要接着发指令了，直接 return
            if (mode_copy == CtrlMode::NONE) return;

            ctrl_yaw_copy = ctrl_yaw_;
            auto_heading_copy = auto_heading_;
            frame_copy = target_frame_;
            target_pose_copy = target_pose_enu_;
            target_vel_copy = target_vel_;
            time_copy = last_command_time_;
            limit_v_xy = active_max_vel_xy_;
            limit_v_z = active_max_vel_z_;
        }

        Eigen::Vector4d current_pose;
        current_pose.head<3>() = pos_enu_.get();
        current_pose.w() = yaw_enu_.load();

        Eigen::Vector4d raw_cmd = Eigen::Vector4d::Zero();
        Eigen::Vector4d body_target_vel = target_vel_copy;

        if (frame_copy == CmdFrame::ENU) {
            double cy = std::cos(-current_pose.w());
            double sy = std::sin(-current_pose.w());
            body_target_vel.x() =
                target_vel_copy.x() * cy - target_vel_copy.y() * sy;
            body_target_vel.y() =
                target_vel_copy.x() * sy + target_vel_copy.y() * cy;
        }

        if (mode_copy == CtrlMode::POSITION) {
            double dx = target_pose_copy.x() - current_pose.x();
            double dy = target_pose_copy.y() - current_pose.y();
            double dz = target_pose_copy.z() - current_pose.z();

            if (auto_heading_copy) {
                double dist_xy = std::hypot(dx, dy);
                if (dist_xy > config_.auto_heading_enable_dist_m) {
                    double new_target_yaw = std::atan2(dy, dx);
                    target_pose_copy.w() = new_target_yaw;

                    std::lock_guard<std::mutex> lock(data_mutex_);
                    target_pose_enu_.w() = new_target_yaw;
                }
                ctrl_yaw_copy = true;
            }

            double dyaw = ctrl_yaw_copy
                              ? angle_ctrl_.get_distance(current_pose.w(),
                                                         target_pose_copy.w())
                              : 0.0;
            double pos_error = std::sqrt(dx * dx + dy * dy + dz * dz);

            bool yaw_reached = (!ctrl_yaw_copy) || auto_heading_copy ||
                               (std::abs(dyaw) <= config_.yaw_tolerance_rad);

            // ==============================================================
            // 判断是否到达
            // ==============================================================
            if (pos_error <= config_.pos_tolerance_m && yaw_reached) {
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    current_mode_ = CtrlMode::NONE;
                    pos_cmd_finished_ = true;
                }
                // 正常到达，发一次 0 然后 return（不再下发）
                send_zero_velocity();
                return;
            }

            raw_cmd = compute_pos_control(current_pose, target_pose_copy,
                                          body_target_vel, ctrl_yaw_copy);

        } else if (mode_copy == CtrlMode::VELOCITY) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - time_copy;

            // ==============================================================
            // 判断是否超时
            // ==============================================================
            if (elapsed.count() > config_.command_timeout_sec) {
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    current_mode_ = CtrlMode::NONE;
                }
                // 速度超时，发一次 0 然后 return（不再下发）
                send_zero_velocity();
                return;
            }

            raw_cmd = compute_vel_control(current_pose, target_pose_copy,
                                          body_target_vel, ctrl_yaw_copy);
        }

        // 正常控制态：应用限制并发指令
        Eigen::Vector4d safe_cmd = apply_kinematic_constraints(
            raw_cmd, limit_v_xy, limit_v_z, current_pose.w(), dt);

        runtime_->cmd_vel(safe_cmd);
    }

   private:
    // =========================================================================
    // 【新增私有方法】应用暂存指令（必须在 data_mutex_ 锁定期间调用）
    // =========================================================================
    void apply_pending_command() {
        current_mode_ = pending_cmd_.mode;
        ctrl_yaw_ = pending_cmd_.yaw.has_value();
        target_frame_ = pending_cmd_.frame;
        last_command_time_ = std::chrono::steady_clock::now();

        if (current_mode_ == CtrlMode::POSITION) {
            pos_cmd_finished_ = false;  // 新位置指令，未完成
            auto_heading_ = (!pending_cmd_.yaw.has_value() &&
                             !pending_cmd_.yaw_rate.has_value()) ||
                            !config_.is_omnidirectional;

            active_max_vel_xy_ = std::min(
                pending_cmd_.cruise_speed_xy.value_or(config_.max_vel_xy),
                config_.max_vel_xy);
            active_max_vel_z_ = std::min(
                pending_cmd_.cruise_speed_z.value_or(config_.max_vel_z),
                config_.max_vel_z);

            if (target_frame_ == CmdFrame::BODY) {
                Eigen::Vector4d body_offset = Eigen::Vector4d::Zero();
                body_offset.head<3>() = pending_cmd_.vec3_cmd;
                if (ctrl_yaw_) body_offset.w() = pending_cmd_.yaw.value();

                Eigen::Vector4d current_odom;
                current_odom.head<3>() = pos_enu_.get();
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
            pos_cmd_finished_ =
                true;  // 被速度控制覆盖，旧的位置任务直接抛弃(算作完成)
            auto_heading_ = false;

            active_max_vel_xy_ = config_.max_vel_xy;
            active_max_vel_z_ = config_.max_vel_z;

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

    void control_loop() {
        const auto period =
            std::chrono::milliseconds(1000 / config_.loop_rate_hz);
        while (is_running_) {
            auto start_time = std::chrono::steady_clock::now();
            process_control_step(period.count() / 1000.0);
            std::this_thread::sleep_until(start_time + period);
        }
    }

    Eigen::Vector4d compute_pos_control(const Eigen::Vector4d& current_pose,
                                        const Eigen::Vector4d& target_pose,
                                        const Eigen::Vector4d& ff_vel_body,
                                        bool ctrl_yaw) {
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
        if ((!config_.is_omnidirectional || auto_heading_) &&
            std::hypot(dx, dy) >= config_.pos_tolerance_m) {
            double yaw_diff = std::abs(dyaw);
            if (yaw_diff >= config_.yaw_zero_speed_tol) {
                speed_scale = 0.0;
            } else if (yaw_diff > config_.yaw_full_speed_tol) {
                speed_scale = 1.0 - (yaw_diff - config_.yaw_full_speed_tol) /
                                        (config_.yaw_zero_speed_tol -
                                         config_.yaw_full_speed_tol);
            }
        }

        if (config_.is_omnidirectional) {
            cmd.x() = config_.kp_xy * e_body_x + ff_vel_body.x();
            cmd.y() = config_.kp_xy * e_body_y + ff_vel_body.y();
            cmd.z() = config_.kp_z * dz + ff_vel_body.z();
            cmd.w() = config_.kp_yaw * dyaw + ff_vel_body.w();
        } else {
            cmd.y() = 0.0;
            cmd.z() = config_.kp_z * dz + ff_vel_body.z();
            cmd.x() =
                ff_vel_body.x() * std::cos(dyaw) + config_.kp_xy * e_body_x;

            if (ff_vel_body.norm() < 1e-3) {
                double dist = std::hypot(dx, dy);
                if (dist > config_.pos_tolerance_m) {
                    cmd.x() = e_body_x * config_.kp_xy;
                    cmd.w() = config_.kp_yaw * dyaw;
                } else {
                    cmd.x() = 0.0;
                    cmd.w() = ctrl_yaw ? config_.kp_yaw * dyaw : 0.0;
                }
            } else {
                cmd.w() = ff_vel_body.w() +
                          std::abs(cmd.x()) * config_.kp_xy * e_body_y;
                if (ctrl_yaw) cmd.w() += config_.kp_yaw * std::sin(dyaw);
            }
        }

        double dist_xy = std::hypot(dx, dy);
        if (dist_xy > 1e-4) {
            double max_v_brake_xy =
                std::sqrt(2.0 * config_.max_decel_xy * dist_xy);
            double cmd_xy_norm = std::hypot(cmd.x(), cmd.y());
            if (cmd_xy_norm > max_v_brake_xy) {
                cmd.x() = (cmd.x() / cmd_xy_norm) * max_v_brake_xy;
                cmd.y() = (cmd.y() / cmd_xy_norm) * max_v_brake_xy;
            }
        }

        double max_v_brake_z =
            std::sqrt(2.0 * config_.max_decel_z * std::abs(dz));
        cmd.z() = std::clamp(cmd.z(), -max_v_brake_z, max_v_brake_z);

        if (ctrl_yaw) {
            double max_v_brake_yaw =
                std::sqrt(2.0 * config_.max_acc_yaw * std::abs(dyaw));
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
            cmd.w() = config_.kp_yaw * dyaw;
        }
        if (!config_.is_omnidirectional) cmd.y() = 0.0;
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
        raw_cmd.w() =
            std::clamp(raw_cmd.w(), -config_.max_vel_yaw, config_.max_vel_yaw);

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

        double limit_accel = config_.max_acc_xy;
        if (last_cmd_vel_.head<2>().norm() > 0.01 &&
            (last_cmd_vel_.x() * dvx + last_cmd_vel_.y() * dvy) < 0) {
            limit_accel = config_.max_decel_xy;
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

        raw_cmd.z() =
            rate_limit(last_cmd_vel_.z(), raw_cmd.z(), config_.max_acc_z, dt);
        raw_cmd.w() =
            rate_limit(last_cmd_vel_.w(), raw_cmd.w(), config_.max_acc_yaw, dt);

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