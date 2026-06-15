#pragma once
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>

#include "./ilanding_controller.hpp"
#include "./utils.hpp"
#include "core/base_tracker.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "features/tracker/tracker.hpp"
#include "nav_msgs/Odometry.h"
#include "robot_context.hpp"
#include "ros/publisher.h"
#include "spdlog/spdlog.h"
#include "utils/thread_runner.hpp"

// ==========================================
// 二阶运动学轨迹生成器 (虚拟兔子)
// ==========================================
class KinematicTrajectoryGenerator2D {
   private:
    Eigen::Vector2d virtual_pos_;
    Eigen::Vector2d virtual_vel_;
    bool initialized_ = false;

   public:
    struct State {
        Eigen::Vector2d pos;
        Eigen::Vector2d vel;
    };

    void reset(const Eigen::Vector2d& current_pos) {
        virtual_pos_ = current_pos;
        virtual_vel_ = Eigen::Vector2d::Zero();
        initialized_ = true;
    }

    void force_set_state(const Eigen::Vector2d& new_pos,
                         const Eigen::Vector2d& new_vel) {
        virtual_pos_ = new_pos;
        virtual_vel_ = new_vel;
    }

    bool is_initialized() const { return initialized_; }

    State step(double dt, const Eigen::Vector2d& target_pos,
               const Eigen::Vector2d& target_vel, double max_vel,
               double max_acc) {
        if (!initialized_) return {target_pos, target_vel};

        double Kp_ = GlobalConfig.GetConfig().pland_kp.get();
        double Kv_ = GlobalConfig.GetConfig().pland_kv.get();

        Eigen::Vector2d pos_err = target_pos - virtual_pos_;
        Eigen::Vector2d rel_vel = pos_err * Kp_;
        if (rel_vel.norm() > max_vel) {
            rel_vel = rel_vel.normalized() * max_vel;
        }

        double distance = pos_err.norm();
        if (distance > 0.01) {
            double safe_brake_acc = max_acc * 0.8;
            double max_kinematic_vel =
                std::sqrt(2.0 * safe_brake_acc * distance);
            if (rel_vel.norm() > max_kinematic_vel) {
                rel_vel = rel_vel.normalized() * max_kinematic_vel;
            }
        }

        Eigen::Vector2d desired_vel = rel_vel + target_vel;
        Eigen::Vector2d vel_err = desired_vel - virtual_vel_;
        Eigen::Vector2d desired_acc = vel_err * Kv_;

        if (desired_acc.norm() > max_acc) {
            desired_acc = desired_acc.normalized() * max_acc;
        }

        virtual_pos_ += virtual_vel_ * dt + 0.5 * desired_acc * dt * dt;
        virtual_vel_ += desired_acc * dt;

        return {virtual_pos_, virtual_vel_};
    }
};

// [新增] 伺服模式枚举
enum class ServoingMode {
    GLOBAL_ENU,  // 原方案：基于全局坐标系的位置追踪
    PURE_VISUAL_BODY  // 新方案：基于机体坐标系的纯视觉相对速度控制
};

class PlandController : public IThreadRunner, public ILandingController {
   private:
    struct ZControlCmd {
        bool should_land;         // 是否应该执行触地停机
        bool should_disarm;       // 触地时是否直接上锁 (Disarm)
        bool is_blind_drop;       // 是否触发盲降
        double descent_vel;       // 计算出的下降速度限幅 (绝对值)
        double hold_dist_thresh;  // (Debug用) 当前高度对应的漏斗半径
    };

    ZControlCmd calculate_shared_z_descent(double current_z,
                                           double current_xy_error,
                                           double abs_yaw_err_deg,
                                           bool is_currently_blind_drop) {
        ZControlCmd cmd = {false, false, false, 0.0, 0.0};
        auto cfg = GlobalConfig.GetConfig();

        // 默认继承当前状态
        cmd.is_blind_drop = is_currently_blind_drop;

        // 1. 触地停机检测
        double land_height = cfg.pland_land_alt.get();
        if (current_z < land_height) {
            cmd.should_land = true;
            cmd.should_disarm = cfg.pland_use_disarm.get();
            return cmd;  // 已经触地，直接返回
        }

        // 2. 状态重置：高度迟滞保护 (防抖)
        double blind_alt = cfg.pland_blind_drop_alt.get();
        if (current_z > std::max(1.0, blind_alt + 0.5)) {
            cmd.is_blind_drop = false;
        }

        // 3. 盲降状态判定与执行
        double blind_xy = cfg.blind_drop_xy_thresh.get();
        if (current_z < blind_alt &&
            (current_xy_error < blind_xy || cmd.is_blind_drop)) {
            cmd.is_blind_drop = true;
        }

        // 如果最终判定处于盲降状态，直接返回盲降速度
        if (cmd.is_blind_drop) {
            cmd.descent_vel = cfg.touchdown_velocity + 0.2;
            return cmd;
        }

        // 4. 计算动态对齐漏斗 (Funnel)
        double align_dist_thresh = std::max(0.2, current_z * 0.2);
        double hold_dist_thresh_max = cfg.hold_dist_thresh_max.get();
        double hold_dist_thresh_min = cfg.hold_dist_thresh_min.get();
        double hold_dist_thresh_min_alt = cfg.hold_dist_thresh_min_alt.get();
        double hold_dist_height_base = 10.0;

        if (current_z <= hold_dist_thresh_min_alt) {
            cmd.hold_dist_thresh = hold_dist_thresh_min;
        } else {
            cmd.hold_dist_thresh =
                hold_dist_thresh_min +
                (std::min(current_z, hold_dist_height_base) -
                 hold_dist_thresh_min_alt) /
                    (hold_dist_height_base - hold_dist_thresh_min_alt) *
                    (hold_dist_thresh_max - hold_dist_thresh_min);
        }

        // 5. 计算下降意愿因子
        double xy_descent_factor = 1.0;
        if (current_xy_error > cmd.hold_dist_thresh) {
            xy_descent_factor = 0.0;  // 在漏斗外，完全不下降
        } else if (current_xy_error > align_dist_thresh) {
            xy_descent_factor =
                1.0 - (current_xy_error - align_dist_thresh) /
                          (cmd.hold_dist_thresh - align_dist_thresh);
        }

        double yaw_descent_min_deg = 15.0;
        double yaw_descent_max_deg = 35.0;
        double yaw_descent_factor = 1.0;
        if (abs_yaw_err_deg > yaw_descent_max_deg) {
            yaw_descent_factor = 0.0;
        } else if (abs_yaw_err_deg > yaw_descent_min_deg) {
            yaw_descent_factor =
                1.0 - (abs_yaw_err_deg - yaw_descent_min_deg) /
                          (yaw_descent_max_deg - yaw_descent_min_deg);
        }

        double descent_factor = xy_descent_factor * yaw_descent_factor;

        // 6. 根据意愿因子生成平滑的下降速度
        if (descent_factor > 0.05) {
            double takeoff_alt = 10.0;
            double alt_ratio =
                std::min(1.0, std::abs(current_z / std::max(takeoff_alt, 1.0)));
            double base_descent_vel = 0.2 + alt_ratio * 0.8;
            cmd.descent_vel = base_descent_vel * descent_factor;
        } else {
            cmd.descent_vel = 0.0;  // 未对准，严格保持高度
        }

        // 全局最大速度限幅
        cmd.descent_vel = std::min(cmd.descent_vel, cfg.pland_max_vel_z.get());

        return cmd;
    }

    RobotContext& ctx_;
    int invalid_time_ = 0;

    ros::Publisher pnp_pub_;
    ros::Publisher los_pub_;
    ros::Publisher fused_pub_;
    ros::Publisher vel_pub_;
    int log_idx_ = 0;
    double last_step_time_;

    KinematicTrajectoryGenerator2D traj_gen_;
    std::mutex state_mtx_;
    DetectorResult latest_obs_;
    bool is_blind_drop_ = false;  // 是否已经进入降落状态
    ServoingMode current_mode_ = ServoingMode::GLOBAL_ENU;  // 默认使用原方案

   public:
    PlandController(RobotContext& ctx)
        : IThreadRunner(ctx.engine->get_time_provider(), true), ctx_(ctx) {
        ros::NodeHandle nh;
        pnp_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/pnp", 10);
        los_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/los", 10);
        fused_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/fused", 10);
        vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/pland/vel", 10);
        log_idx_ = 0;
        last_step_time_ = 0.0;

        if (GlobalConfig.GetConfig().pure_vision.get()) {
            current_mode_ = ServoingMode::PURE_VISUAL_BODY;
        }
    }

    void publish_debug_data(RobotContext& ctx, DetectorResult result) {
        std::vector<std::optional<Eigen::Vector3d>> pose{
            result.pnp_pos, result.los_pos, result.fused_pos};
        std::vector<ros::Publisher> pub{pnp_pub_, los_pub_, fused_pub_};
        for (int i = 0; i < 3; ++i) {
            std::optional<Eigen::Vector3d> ps = pose[i];
            auto pb = pub[i];
            if (ps.has_value()) {
                Eigen::Vector3d p = ps.value();
                nav_msgs::Odometry pnp_odom;
                pnp_odom.header.frame_id = "map";
                pnp_odom.pose.pose.position.x = p.x();
                pnp_odom.pose.pose.position.y = p.y();
                pnp_odom.pose.pose.position.z = 0;
                pb.publish(pnp_odom);
                if (i == 1) {
                    geometry_msgs::TwistStamped vel_msg;
                    vel_msg.twist.linear.x = result.target_vel_enu.x();
                    vel_msg.twist.linear.y = result.target_vel_enu.y();
                    vel_msg.header.frame_id = "map";
                    vel_pub_.publish(vel_msg);
                }
            }
        }
    }

    void update_observation(const DetectorResult& result) override {
        std::lock_guard<std::mutex> lk(state_mtx_);
        if (result.is_valid) {
            latest_obs_ = result;
            publish_debug_data(ctx_, result);
        }
    }

    void start(int hz) override { IThreadRunner::start(hz); }

    void stop() override { IThreadRunner::stop(); }

    void on_start() override {
        traj_gen_.reset(
            Eigen::Vector2d(ctx_.pos_enu.load().x(), ctx_.pos_enu.load().y()));
    }

    void on_step(double dt) override {
        double now = ctx_.engine->get_time_provider()->now();

        DetectorResult obs;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            obs = latest_obs_;
        }

        // 动态读取配置，支持空中无缝切换
        current_mode_ = GlobalConfig.GetConfig().pure_vision.get()
                            ? ServoingMode::PURE_VISUAL_BODY
                            : ServoingMode::GLOBAL_ENU;

        // 根据模式进入不同的控制逻辑分支
        if (current_mode_ == ServoingMode::GLOBAL_ENU) {
            step_global_enu(dt, obs, now);
        } else {
            step_pure_visual(dt, obs, now);
        }
    }

   private:
    // =================================================================
    // 方案一：保留你原有的全局 ENU 逻辑
    // =================================================================
    void step_global_enu(double dt, const DetectorResult& obs, double now) {
        if (obs.stamp == 0.0) return;

        auto pos_enu = ctx_.pos_enu.load();
        Eigen::Quaterniond q = ctx_.orientation.load();
        q.normalize();
        Eigen::Matrix3d R_wb = q.toRotationMatrix();

        double current_drone_yaw = ctx_.yaw_enu.load();

        bool is_timeout = (obs.stamp == 0.0) ? true : (now - obs.stamp) > 1.0;
        if (!obs.is_valid || is_timeout) {
            invalid_time_ += 1;
        } else {
            invalid_time_ = 0;
        }

        if (invalid_time_ > 20) {
            if (!is_blind_drop_) {
                traj_gen_.reset(Eigen::Vector2d(pos_enu.x(), pos_enu.y()));
                ctx_.tracker->send_pos_cmd(
                    {pos_enu.x(), pos_enu.y(),
                     GlobalConfig.GetConfig().lost_target_alt},
                    std::nullopt, std::nullopt, std::nullopt, std::nullopt, 0.5,
                    std::nullopt, CmdFrame::ENU);
                return;
            }
        }

        double delay_sec = (now - obs.stamp);
        if (is_blind_drop_)
            delay_sec = std::clamp(delay_sec, 0.0, 2.0);
        else
            delay_sec = std::clamp(delay_sec, 0.0, 0.5);

        Eigen::Vector3d current_target_enu = obs.target_pos_enu;
        double yaw = obs.target_yaw_enu;
        double omega = obs.target_vel_enu.z();
        Eigen::Vector2d vel_xy = obs.target_vel_enu.template head<2>();

        // =======================================================
        // [修复 1] 剔除不稳定的除以 omega 的圆周模型，改为极度稳定的线性预测
        // =======================================================
        current_target_enu.x() += vel_xy.x() * delay_sec;
        current_target_enu.y() += vel_xy.y() * delay_sec;
        double current_target_yaw = yaw + omega * delay_sec;

        // 确保目标偏航角在 [-pi, pi]
        current_target_yaw = std::fmod(current_target_yaw + M_PI, 2.0 * M_PI);
        if (current_target_yaw < 0) current_target_yaw += 2.0 * M_PI;
        current_target_yaw -= M_PI;

        double current_xy_error =
            std::hypot(current_target_enu.x() - pos_enu.x(),
                       current_target_enu.y() - pos_enu.y());

        // 计算当前偏航误差 (最短路径)
        double yaw_diff = current_target_yaw - current_drone_yaw;
        yaw_diff = std::fmod(yaw_diff + M_PI, 2.0 * M_PI);
        if (yaw_diff < 0) yaw_diff += 2.0 * M_PI;
        yaw_diff -= M_PI;
        double abs_yaw_err_deg = std::abs(yaw_diff) * 180.0 / M_PI;

        // =======================================================
        // [修复 2] 平滑过渡的 Yaw 对齐逻辑，代替原本的阶跃 if/else
        // =======================================================
        double desired_yaw_enu = current_drone_yaw;
        double desired_omega = 0.0;
        double yaw_enable_min = 1.0;  // 1米以内完全跟随目标机头
        double yaw_enable_max = 2.5;  // 2.5米以外完全不转头（保持当前航向）

        if (current_xy_error > yaw_enable_max) {
            desired_yaw_enu = current_drone_yaw;
            desired_omega = 0.0;
        } else if (current_xy_error < yaw_enable_min) {
            desired_yaw_enu = current_target_yaw;
            desired_omega = omega;
        } else {
            // 在 1.0m 到 2.5m 之间做线性插值，防止指令突变导致机身剧烈晃动
            double ratio = (yaw_enable_max - current_xy_error) /
                           (yaw_enable_max - yaw_enable_min);
            desired_yaw_enu = current_drone_yaw + yaw_diff * ratio;
            desired_omega = omega * ratio;
        }

        double TARGET_PLATFORM_HEIGHT =
            GlobalConfig.GetConfig().platform_height.get();
        double drone_enu_z = get_current_z(ctx_);
        double current_z = std::max(0.0, drone_enu_z - TARGET_PLATFORM_HEIGHT);

        double pland_max_acc_xy =
            GlobalConfig.GetConfig().pland_max_acc_xy.get();
        double decay_start_z =
            GlobalConfig.GetConfig().pland_decay_start_z.get();

        // 相机 FOV 计算
        Eigen::Vector3d target_body_relative =
            R_wb.transpose() * (current_target_enu - pos_enu);
        double dist_xy =
            std::hypot(target_body_relative.x(), target_body_relative.y());
        double visual_angle_deg = std::atan2(dist_xy, current_z) * 180.0 / M_PI;

        double fov_penalty = 1.0;
        if (!is_blind_drop_ && visual_angle_deg > 30.0) {
            fov_penalty =
                std::max(0.1, 1.0 - 0.9 * (visual_angle_deg - 30.0) / 15.0);
        }

        double yaw_penalty = 1.0;
        if (!is_blind_drop_ && abs_yaw_err_deg > 20.0 &&
            current_xy_error <= yaw_enable_max) {
            yaw_penalty =
                std::max(0.1, 1.0 - 0.9 * (abs_yaw_err_deg - 20.0) / 40.0);
        }

        double combined_penalty = std::min(fov_penalty, yaw_penalty);
        double max_cruise_speed_xy =
            GlobalConfig.GetConfig().pland_cruise_speed_xy.get();
        double distance_speed_limit = std::max(0.3, current_xy_error * 0.5);
        double active_cruise_speed =
            std::min(max_cruise_speed_xy, distance_speed_limit + vel_xy.norm());

        // 为了防止龟速导致追不上目标，限制最低巡航速度和加速度
        double cruise_speed_xy =
            std::max(active_cruise_speed * combined_penalty, 0.5);
        double pland_acc_xy =
            std::max(pland_max_acc_xy * combined_penalty, 0.8);

        if (!traj_gen_.is_initialized()) {
            traj_gen_.reset(Eigen::Vector2d(pos_enu.x(), pos_enu.y()));
        }

        Eigen::Vector2d target_pos_xy(current_target_enu.x(),
                                      current_target_enu.y());
        auto virtual_state_enu = traj_gen_.step(dt, target_pos_xy, vel_xy,
                                                cruise_speed_xy, pland_acc_xy);

        // =======================================================
        // [修复 3] 牵引绳(Leash)保护：只限制位置，坚决不再将速度 *= 0.5
        // =======================================================
        Eigen::Vector2d current_pos_enu_xy(pos_enu.x(), pos_enu.y());
        Eigen::Vector2d tracking_err =
            virtual_state_enu.pos - current_pos_enu_xy;
        double max_leash_length = std::max(2.0, 1.0 + 0.5 * (current_z / 3.0));

        if (tracking_err.norm() > max_leash_length) {
            virtual_state_enu.pos =
                current_pos_enu_xy +
                tracking_err.normalized() * max_leash_length;
            // 保持原速度或限幅，不粗暴折半，消除 "冲刺-顿挫-冲刺" 的震荡
            if (virtual_state_enu.vel.norm() > cruise_speed_xy) {
                virtual_state_enu.vel =
                    virtual_state_enu.vel.normalized() * cruise_speed_xy;
            }
            traj_gen_.force_set_state(virtual_state_enu.pos,
                                      virtual_state_enu.vel);
        }

        // =======================================================
        // [修复 4] 全面废除 Body 坐标系，改用全局 ENU
        // 下发绝对指令，消除画圈效应
        // =======================================================
        Eigen::Vector3d pos_cmd_enu(virtual_state_enu.pos.x(),
                                    virtual_state_enu.pos.y(),
                                    0.0);  // Z轴下面单独计算
        Eigen::Vector3d ff_vel_enu(virtual_state_enu.vel.x(),
                                   virtual_state_enu.vel.y(), 0.0);
        if (!GlobalConfig.GetConfig().use_ff_vel.get()) {
            ff_vel_enu.setZero();
        }
        // ---------------- 降落阶段 Z 轴与下发逻辑 (已重构复用)
        // ----------------
        ZControlCmd z_cmd = calculate_shared_z_descent(
            current_z, current_xy_error, abs_yaw_err_deg, is_blind_drop_);

        if (z_cmd.should_land) {
            if (z_cmd.should_disarm)
                ctx_.robot->disarm();
            else
                ctx_.robot->land();
            return;
        }

        // 同步状态机
        is_blind_drop_ = z_cmd.is_blind_drop;
        double max_vel_z = z_cmd.descent_vel;

        // 目标位置下发逻辑
        double z_enu_target = drone_enu_z;  // 默认维持当前高度
        double pland_gamma_z = GlobalConfig.GetConfig().pland_gamma_z.get();
        double pland_gamma_yaw = GlobalConfig.GetConfig().pland_gamma_yaw.get();

        if (is_blind_drop_) {
            z_enu_target = TARGET_PLATFORM_HEIGHT - 0.5;  // 盲降往下按
            pland_gamma_z = 100.0;                        // 加大下压刚度
        } else if (max_vel_z > 0.0) {
            z_enu_target = TARGET_PLATFORM_HEIGHT - 0.5;  // 对准后提供下降梯度
        }

        double max_gamma = GlobalConfig.GetConfig().pland_gamma.get();
        double min_gamma = GlobalConfig.GetConfig().pland_min_gamma.get();
        double gamma = max_gamma;
        if (current_z > 0 && current_z <= decay_start_z) {
            double ratio = current_z / decay_start_z;
            gamma = min_gamma + (max_gamma - min_gamma) * ratio;
        } else if (current_z <= 0) {
            gamma = min_gamma;
        }

        pos_cmd_enu.z() = z_enu_target;
        double cruise_speed_yaw = 0.4;

        // =======================================================
        // [修复 6] 修正 P 增益，并发送 ENU 绝对指令
        // 原本写死的 1.0 被替换成了算好的 gamma
        // CmdFrame::BODY 替换成了 CmdFrame::ENU
        // =======================================================
        ctx_.tracker->send_pos_cmd(
            pos_cmd_enu,      // ENU绝对目标位置
            desired_yaw_enu,  // ENU绝对目标偏航角
            desired_omega,    // 目标偏航角速度
            ff_vel_enu,       // ENU前馈绝对速度
            cruise_speed_xy, max_vel_z, cruise_speed_yaw,
            CmdFrame::ENU,  // ★ 切断与姿态解算的耦合，交由底层飞控闭环
            {gamma, pland_gamma_yaw, pland_gamma_z},  // ★ 恢复正确的 XY 增益
            pland_acc_xy);

        // [Debug 日志]
        log_idx_ += 1;
        if (log_idx_ % 10 == 0) {
            double ekf_v = vel_xy.norm();
            double ff_vel = ff_vel_enu.norm();
            bool is_leash_clamped = tracking_err.norm() > max_leash_length;

            spdlog::info(
                "Z:{:.1f} | "
                "Angle:{:.1f}deg | ErrXY:{:.2f}m | "
                "ff_vel:{:.2f}m/s | max_z_v:{:.2f} | "
                "(blind_drop:{}) | delay:{:.2f} | "
                "(LeashClamp:{}) | Gamma:{:.2f} | "
                "AccLim:{:.2f} | xy_thresh:{:.3f}",
                current_z, visual_angle_deg, current_xy_error, ff_vel,
                max_vel_z, is_blind_drop_ ? "YES" : "NO", delay_sec,
                is_leash_clamped ? "YES" : "NO", gamma, pland_acc_xy,
                z_cmd.hold_dist_thresh);
        }
    }

    // =================================================================
    // 方案二：新增的纯视觉伺服逻辑 (PBVS 基于位置的视觉伺服)
    // =================================================================
    void step_pure_visual(double dt, const DetectorResult& obs, double now) {
        bool is_timeout = (obs.stamp == 0.0) || ((now - obs.stamp) > 1.0);
        if (!obs.is_valid || is_timeout) {
            invalid_time_ += 1;
        } else {
            invalid_time_ = 0;
        }

        // 1.
        // 目标丢失处理：纯视觉失去目标后，最安全的做法是机体系速度归零（悬停）
        if (invalid_time_ > 20) {
            if (!is_blind_drop_) {
                // 停止任何相对运动，等待目标重新进入视野
                ctx_.tracker->send_vel_cmd(Eigen::Vector3d::Zero(),
                                           std::nullopt, 0.0, CmdFrame::BODY);
                return;
            }
        }

        // 2. 提取视觉相对误差 (Body Frame)
        Eigen::Vector3d err_body = obs.target_pos_body;
        double err_yaw = obs.yaw_relative;

        // ==========================================
        // [核心修正] 3. 计算机体坐标系下的前馈速度
        // ==========================================
        Eigen::Vector2d ff_vel_body(0.0, 0.0);

        // 如果系统开启了前馈，且目标处于运动状态
        if (GlobalConfig.GetConfig().use_ff_vel.get()) {
            // 取出 CTRV 滤波器算出的目标全局速度 (这个速度受 GPS 漂移影响较小)
            Eigen::Vector2d target_vel_enu_xy = obs.target_vel_enu.head<2>();

            // 获取无人机当前的全局偏航角 (只用 Yaw，不用 XY 位置)
            double current_yaw = ctx_.yaw_enu.load();

            // 旋转矩阵：将 ENU 速度投影到当前的机体前方 (X) 和左方/右方 (Y)
            // 假设 Body 系为 FLU (前左上)
            double cos_y = std::cos(current_yaw);
            double sin_y = std::sin(current_yaw);

            // 坐标旋转：从 ENU 变回 Body (逆旋转)
            ff_vel_body.x() =
                target_vel_enu_xy.x() * cos_y + target_vel_enu_xy.y() * sin_y;
            ff_vel_body.y() =
                -target_vel_enu_xy.x() * sin_y + target_vel_enu_xy.y() * cos_y;

            // 可以在这里对 ff_vel_body 做一下限幅，防止异常脉冲
            double max_ff =
                GlobalConfig.GetConfig().pland_cruise_speed_xy.get() * 0.8;
            if (ff_vel_body.norm() > max_ff) {
                ff_vel_body = ff_vel_body.normalized() * max_ff;
            }
        }

        // 4. 计算水平速度指令 (P-Controller + Feedforward)
        double Kp_xy = GlobalConfig.GetConfig().vision_kp.get();
        double max_v_xy = GlobalConfig.GetConfig().pland_cruise_speed_xy.get();

        Eigen::Vector3d vel_cmd_body(0.0, 0.0, 0.0);

        // 【结合误差反馈与目标前馈】
        vel_cmd_body.x() = Kp_xy * err_body.x() + ff_vel_body.x();
        vel_cmd_body.y() = Kp_xy * err_body.y() + ff_vel_body.y();

        // 整体限幅
        if (vel_cmd_body.head<2>().norm() > max_v_xy) {
            vel_cmd_body.head<2>() =
                vel_cmd_body.head<2>().normalized() * max_v_xy;
        }

        // 5. 计算偏航角速度指令 (包含角速度前馈)
        double Kp_yaw = 1.0;
        double ff_omega = obs.target_vel_enu.z();  // 目标的角速度直接作为前馈

        double omega_z = Kp_yaw * err_yaw + ff_omega;
        omega_z = std::clamp(omega_z, -0.5, 0.5);

        // 6. 下降逻辑 (调用通用 Z 轴漏斗控制引擎)
        double current_z = get_current_z(ctx_);
        double xy_error_norm = err_body.head<2>().norm();
        double abs_yaw_err_deg = std::abs(err_yaw) * 180.0 / M_PI;

        ZControlCmd z_cmd = calculate_shared_z_descent(
            current_z, xy_error_norm, abs_yaw_err_deg, is_blind_drop_);

        if (z_cmd.should_land) {
            if (z_cmd.should_disarm)
                ctx_.robot->disarm();
            else
                ctx_.robot->land();
            return;
        }

        // 同步全局状态
        is_blind_drop_ = z_cmd.is_blind_drop;

        if (is_blind_drop_) {
            // 盲降模式：切断水平和偏航输出
            vel_cmd_body.head<2>().setZero();
            omega_z = 0.0;
        }

        // 机体系 (FLU) 中，Z 轴下降是负向速度
        vel_cmd_body.z() = -z_cmd.descent_vel;

        // 7. 最终下发纯视觉速度指令 (完全运行在 CmdFrame::BODY 机体坐标系)
        ctx_.tracker->send_vel_cmd(
            vel_cmd_body,   // 机体坐标系下的 [vx, vy, vz]
            std::nullopt,   // 不控制绝对偏航角
            omega_z,        // 控制偏航角速度
            CmdFrame::BODY  // ★ 关键：指令参考系为机体坐标系
        );

        // [Debug 日志] 打印机体系误差，方便调参
        log_idx_ += 1;
        if (log_idx_ % 10 == 0) {
            spdlog::info(
                "[PVS] err_body:[{:.2f}, {:.2f}] | vel_body:[{:.2f}, {:.2f}, "
                "{:.2f}] | "
                "err_yaw:{:.1f}deg | omega_z:{:.2f} | Z:{:.1f}",
                err_body.x(), err_body.y(), vel_cmd_body.x(), vel_cmd_body.y(),
                vel_cmd_body.z(), err_yaw * 180.0 / M_PI, omega_z, current_z);
        }
    }
};
