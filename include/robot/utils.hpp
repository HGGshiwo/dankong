#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "robot_context.hpp"
#include "states/state_utils.hpp"

struct TrackingConfig {
    bool is_omnidirectional = false;
    double kp_xy = 1.5;
    double kp_z = 1.0;
    double kp_yaw = 2.0;
    double command_timeout_sec = 0.5;  // Safety watchdog
    int loop_rate_hz = 50;             // Internal thread frequency
};

class ITrackerRuntime {
   public:
    virtual RobotContext get_context() = 0;
    virtual void cmd_vel(Eigen::Vector4d) = 0;  // body x,y,z + yaw_rate
};

// ITrackerRuntime, TrackingConfig and some unmodified members of
// ThreadedTracker are omitted.

class ThreadedTracker {
   private:
    TrackingConfig config_;
    std::shared_ptr<ITrackerRuntime> runtime_;

    std::thread control_thread_;
    std::atomic<bool> is_running_;
    std::mutex data_mutex_;

    bool has_active_command_;
    Eigen::Vector4d target_enu_pose_;
    Eigen::Vector4d target_ff_vel_;
    std::chrono::steady_clock::time_point last_command_time_;

    // Flags to indicate whether a specific dimension is actively controlled
    bool ctrl_pos_ = false;
    bool ctrl_yaw_ = false;

   public:
    ThreadedTracker(const TrackingConfig& config,
                    std::shared_ptr<ITrackerRuntime> runtime)
        : config_(config),
          runtime_(runtime),
          is_running_(false),
          has_active_command_(false) {}

    ~ThreadedTracker() { stop(); }

    void start() {
        if (is_running_) return;
        is_running_ = true;
        control_thread_ = std::thread(&ThreadedTracker::control_loop, this);
    }

    void stop() {
        is_running_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        send_zero_velocity();
    }

    void send_cmd(std::optional<Eigen::Vector3d> pos,
                  std::optional<Eigen::Vector3d> vel, std::optional<double> yaw,
                  std::optional<double> yaw_rate, CmdFrame frame) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        ctrl_pos_ = pos.has_value();
        ctrl_yaw_ = yaw.has_value();

        if (frame == CmdFrame::BODY) {
            Eigen::Vector4d body_pose_offset = Eigen::Vector4d::Zero();
            if (ctrl_pos_) body_pose_offset.head<3>() = pos.value();
            if (ctrl_yaw_) body_pose_offset.w() = yaw.value();

            Eigen::Vector4d current_odom;
            current_odom.head<3>() = runtime_->get_context().pos_enu.get();
            current_odom.w() = runtime_->get_context().yaw_enu.load();
            target_enu_pose_ =
                state_utils::body_to_enu(body_pose_offset, current_odom);
        } else {
            if (ctrl_pos_) target_enu_pose_.head<3>() = pos.value();
            if (ctrl_yaw_) target_enu_pose_.w() = yaw.value();
        }

        if (vel.has_value()) {
            target_ff_vel_.head<3>() = vel.value();
        } else {
            target_ff_vel_.head<3>().setZero();
        }

        if (yaw_rate.has_value()) {
            target_ff_vel_.w() = yaw_rate.value();
        } else {
            target_ff_vel_.w() = 0.0;
        }

        last_command_time_ = std::chrono::steady_clock::now();
        has_active_command_ = true;
    }

   private:
    void control_loop() {
        const auto period =
            std::chrono::milliseconds(1000 / config_.loop_rate_hz);

        while (is_running_) {
            auto start_time = std::chrono::steady_clock::now();
            process_control_step();
            std::this_thread::sleep_until(start_time + period);
        }
    }

    void process_control_step() {
        bool is_active;
        Eigen::Vector4d target_pose_copy;
        Eigen::Vector4d ff_vel_copy;
        std::chrono::steady_clock::time_point time_copy;
        bool ctrl_pos_copy;
        bool ctrl_yaw_copy;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            is_active = has_active_command_;
            target_pose_copy = target_enu_pose_;
            ff_vel_copy = target_ff_vel_;
            time_copy = last_command_time_;
            ctrl_pos_copy = ctrl_pos_;
            ctrl_yaw_copy = ctrl_yaw_;
        }

        if (!is_active) {
            return;
        }

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - time_copy;
        if (elapsed.count() > config_.command_timeout_sec) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                has_active_command_ = false;
            }
            send_zero_velocity();
            return;
        }

        Eigen::Vector4d current_pose;

        current_pose.head<3>() = runtime_->get_context().pos_enu.get();
        current_pose.w() = runtime_->get_context().yaw_enu.load();
        // Dynamically zero out position errors if not controlled
        if (!ctrl_pos_copy) {
            target_pose_copy.x() = current_pose.x();
            target_pose_copy.y() = current_pose.y();
            target_pose_copy.z() = current_pose.z();
        }
        if (!ctrl_yaw_copy) {
            target_pose_copy.w() = current_pose.w();
        }

        Eigen::Vector4d output_vel =
            compute_tracker(current_pose, target_pose_copy, ff_vel_copy);
        runtime_->cmd_vel(output_vel);
    }

    Eigen::Vector4d compute_tracker(const Eigen::Vector4d& current_pose,
                                    const Eigen::Vector4d& target_pose,
                                    const Eigen::Vector4d& ff_vel) {
        Eigen::Vector4d raw_cmd = Eigen::Vector4d::Zero();

        double dx = target_pose.x() - current_pose.x();
        double dy = target_pose.y() - current_pose.y();
        double dz = target_pose.z() - current_pose.z();
        double dyaw = state_utils::norm_yaw(target_pose.w() - current_pose.w());

        if (config_.is_omnidirectional) {
            double vx_global = config_.kp_xy * dx;
            double vy_global = config_.kp_xy * dy;

            raw_cmd.x() = vx_global * std::cos(-current_pose.w()) -
                          vy_global * std::sin(-current_pose.w());
            raw_cmd.y() = vx_global * std::sin(-current_pose.w()) +
                          vy_global * std::cos(-current_pose.w());

            raw_cmd.x() += ff_vel.x();
            raw_cmd.y() += ff_vel.y();
            raw_cmd.z() = config_.kp_z * dz + ff_vel.z();
            raw_cmd.w() = config_.kp_yaw * dyaw + ff_vel.w();
        } else {
            double e_x = std::cos(current_pose.w()) * dx +
                         std::sin(current_pose.w()) * dy;
            double e_y = -std::sin(current_pose.w()) * dx +
                         std::cos(current_pose.w()) * dy;
            double e_yaw = dyaw;

            raw_cmd.x() = ff_vel.x() * std::cos(e_yaw) + config_.kp_xy * e_x;
            raw_cmd.y() = 0.0;
            raw_cmd.z() = config_.kp_z * dz + ff_vel.z();
            raw_cmd.w() =
                ff_vel.w() + ff_vel.x() * (config_.kp_xy * e_y +
                                           config_.kp_yaw * std::sin(e_yaw));
        }

        return raw_cmd;
    }

    void send_zero_velocity() {
        Eigen::Vector4d zero_cmd = Eigen::Vector4d::Zero();
        runtime_->cmd_vel(zero_cmd);
    }
};