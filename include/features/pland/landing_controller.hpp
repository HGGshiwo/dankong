#pragma once
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>

#include "core/base_tracker.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "features/tracker/tracker.hpp"
#include "landing_detector.hpp"
#include "nav_msgs/Odometry.h"
#include "robot_context.hpp"
#include "ros/publisher.h"
#include "spdlog/spdlog.h"
#include "utils/thread_runner.hpp"

// ==========================================
// [新增] 二阶运动学轨迹生成器 (虚拟兔子)
// ==========================================
class KinematicTrajectoryGenerator2D {
   private:
    Eigen::Vector2d virtual_pos_;
    Eigen::Vector2d virtual_vel_;
    bool initialized_ = false;

    // 调参经验值：P 决定追踪紧密度，V 决定速度响应
    const double Kp_ = 2.5;
    const double Kv_ = 3.5;

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

    // [修改] 增加 target_vel 参数
    State step(double dt, const Eigen::Vector2d& target_pos,
               const Eigen::Vector2d& target_vel, double max_vel,
               double max_acc) {
        if (!initialized_) return {target_pos, target_vel};

        // 1. 位置 P 环计算基础期望速度
        Eigen::Vector2d pos_err = target_pos - virtual_pos_;
        // 2. 基础 P 环计算期望相对速度
        Eigen::Vector2d rel_vel = pos_err * Kp_;
        if (rel_vel.norm() > max_vel) {
            rel_vel = rel_vel.normalized() * max_vel;
        }

        // ==========================================
        // ★ 核心魔法：Square-Root 刹车防过冲曲线 ★
        // ==========================================
        double distance = pos_err.norm();
        if (distance > 0.01) {
            // 预留 20% 的加速度给目标自身转向，用 80% 的加速度来做刹车规划
            double safe_brake_acc = max_acc * 0.8;

            // 根据物理学 v = sqrt(2ax)
            // 算出当前距离下，理论上能刹住的最大安全速度
            double max_kinematic_vel =
                std::sqrt(2.0 * safe_brake_acc * distance);

            // 强行限幅：如果 P 环给的速度太快会刹不住，就压平到安全速度
            if (rel_vel.norm() > max_kinematic_vel) {
                rel_vel = rel_vel.normalized() * max_kinematic_vel;
            }
        }

        // ==========================================
        // ★ 核心物理魔法：目标速度前馈 ★
        // 兔子的期望速度 = 消除位置误差的速度 + 目标平台本身的移动速度
        // ==========================================
        Eigen::Vector2d desired_vel = rel_vel + target_vel;

        // 2. 速度 P 环计算期望加速度
        Eigen::Vector2d vel_err = desired_vel - virtual_vel_;
        Eigen::Vector2d desired_acc = vel_err * Kv_;

        if (desired_acc.norm() > max_acc) {
            desired_acc = desired_acc.normalized() * max_acc;
        }

        // 3. 运动学积分
        virtual_pos_ += virtual_vel_ * dt + 0.5 * desired_acc * dt * dt;
        virtual_vel_ += desired_acc * dt;

        return {virtual_pos_, virtual_vel_};
    }
};

class PlandController : public IThreadRunner {
   private:
    RobotContext& ctx_;
    int invalid_time_ = 0;

    ros::Publisher pnp_pub_;
    ros::Publisher los_pub_;
    ros::Publisher fused_pub_;
    ros::Publisher vel_pub_;
    int log_idx_ = 0;
    ros::Time last_step_time_;

    // [新增] 实例化的轨迹生成器
    KinematicTrajectoryGenerator2D traj_gen_;

    // ==========================================
    // [新增] 共享观测状态与互斥锁 (用于异步解耦)
    // ==========================================
    std::mutex state_mtx_;
    DetectorResult latest_obs_;

    bool is_blind_drop_ = false;  // 新增：盲降状态锁

   public:
    PlandController(RobotContext& ctx) : IThreadRunner(true), ctx_(ctx) {
        ros::NodeHandle nh;
        pnp_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/pnp", 10);
        los_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/los", 10);
        fused_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/fused", 10);
        vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/pland/vel", 10);
        log_idx_ = 0;
        last_step_time_ = ros::Time(0);
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

    // [新增] 仅供视觉回调更新状态 (耗时极短)
    void update_observation(const DetectorResult& result) {
        std::lock_guard<std::mutex> lk(state_mtx_);
        // ★ 核心防火墙：只有视觉解算成功且有效的数据，才有资格更新状态！
        // 这样即使视觉掉帧，latest_obs_ 里保存的依然是上一帧完美的 EKF
        // 速度和位置
        if (result.is_valid) {
            latest_obs_ = result;
            publish_debug_data(ctx_, result);
        }
    }

    void on_start() override {
        traj_gen_.reset(
            Eigen::Vector2d(ctx_.pos_enu.load().x(), ctx_.pos_enu.load().y()));
    }

    void on_step(double dt) override {
        ros::Time now = ros::Time::now();

        // 1. 安全读取最新观测状态
        DetectorResult obs;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            obs = latest_obs_;
        }

        // ★ 新增：如果从来没收到过有效图像（时间戳为0），直接待命，不要瞎飞！
        if (obs.stamp.isZero()) {
            return;
        }

        auto pos_enu = ctx_.pos_enu.load();
        Eigen::Quaterniond q = ctx_.orientation.load();
        q.normalize();
        Eigen::Matrix3d R_wb = q.toRotationMatrix();

        // 2. 丢失保护判断 (如果 1 秒没收到新图，或者数据 is_valid=false)
        bool is_timeout =
            obs.stamp.isZero() ? true : (now - obs.stamp).toSec() > 1.0;
        if (!obs.is_valid || is_timeout) {
            invalid_time_ += 1;
        } else {
            invalid_time_ = 0;
        }

        if (invalid_time_ > 20) {  // 大约丢失 0.4 秒后执行悬停或盲降
            if (!is_blind_drop_) {  // ★ 只有在非盲降状态下才允许放弃任务
                traj_gen_.reset(Eigen::Vector2d(pos_enu.x(), pos_enu.y()));
                ctx_.tracker->send_pos_cmd(
                    {pos_enu.x(), pos_enu.y(),
                     GlobalConfig.GetConfig().lost_target_alt},
                    std::nullopt, std::nullopt, std::nullopt, std::nullopt, 0.5,
                    CmdFrame::ENU);
                return;
            }
        }

        // =======================================================
        // ★ 预测补偿魔法：把 100ms 前的目标坐标“快进”到现在 ★
        // =======================================================
        double delay_sec = (now - obs.stamp).toSec();
        if (is_blind_drop_) {
            // 如果已经在盲降中丢帧，允许兔子利用最后的速度惯性，最多盲跑 2.0
            // 秒！ 这足够支撑无人机完成最后 0.8 米的下砸
            delay_sec = std::clamp(delay_sec, 0.0, 2.0);
        } else {
            delay_sec = std::clamp(delay_sec, 0.0, 0.5);
        }

        // 总预测时间 = 图像传输计算延迟
        double lookahead_time = delay_sec;

        Eigen::Vector3d predicted_target_enu = obs.target_pos_enu;
        double omega = obs.target_vel_enu.z();
        double v_norm = obs.target_vel_enu.template head<2>().norm();
        double yaw = obs.target_yaw_enu;  // EKF 估计的目标航向
        Eigen::Vector2d vel_xy = obs.target_vel_enu.template head<2>();

        // A. 真实当前位置 (仅补偿图像延迟) -> 用于逻辑判断
        Eigen::Vector3d current_target_enu = obs.target_pos_enu;
        if (std::abs(omega) < 0.05) {
            current_target_enu.x() += vel_xy.x() * delay_sec;
            current_target_enu.y() += vel_xy.y() * delay_sec;
        } else {
            current_target_enu.x() +=
                (v_norm / omega) *
                (std::sin(yaw + omega * delay_sec) - std::sin(yaw));
            current_target_enu.y() +=
                (v_norm / omega) *
                (-std::cos(yaw + omega * delay_sec) + std::cos(yaw));
        }

        // B. 飞控前瞻位置 (补偿 APM 位置环约 0.7 秒的物理滞后) -> 用于喂给兔子
        double fc_lag_sec = 0.0;  // 抵消 APM 的拖尾滞后
        Eigen::Vector3d future_target_enu = current_target_enu;

        if (std::abs(omega) < 0.05) {
            future_target_enu.x() += vel_xy.x() * fc_lag_sec;
            future_target_enu.y() += vel_xy.y() * fc_lag_sec;
        } else {
            double current_yaw = yaw + omega * delay_sec;
            future_target_enu.x() +=
                (v_norm / omega) * (std::sin(current_yaw + omega * fc_lag_sec) -
                                    std::sin(current_yaw));
            future_target_enu.y() +=
                (v_norm / omega) *
                (-std::cos(current_yaw + omega * fc_lag_sec) +
                 std::cos(current_yaw));
        }

        // ---------------------------------------------------------
        // 计算真实物理误差 (用于判断盲降)
        Eigen::Vector3d target_body_relative =
            R_wb.transpose() * (current_target_enu - pos_enu);
        double current_xy_error =
            std::hypot(target_body_relative.x(), target_body_relative.y());

        // 计算补偿后的误差 (用于判断是否处于平稳伴飞状态)
        Eigen::Vector3d future_body_relative =
            R_wb.transpose() * (future_target_enu - pos_enu);
        double compensated_error =
            std::hypot(future_body_relative.x(), future_body_relative.y());

        // 获取参数 (加速度，FOV 保护)
        // 假设你仿真的降落平台（车/船）的物理高度是 1.0 米
        // 如果平台就是地面，这个值就是 0.0
        double TARGET_PLATFORM_HEIGHT = 0.0;

        // 真实的无人机 ENU Z 轴高度
        double drone_enu_z = ctx_.pos_enu.load().z();

        // 计算真正用于降落逻辑的“相对高度”
        double current_z = drone_enu_z - TARGET_PLATFORM_HEIGHT;
        if (current_z > 1.0) {
            is_blind_drop_ = false;
        }

        // 防止气压计漂移导致出现负数影响后续的数学计算
        current_z = std::max(0.0, current_z);
        double pland_max_acc_xy =
            GlobalConfig.GetConfig().pland_max_acc_xy.get();
        double pland_min_acc_xy = 2.0;
        double decay_start_z =
            GlobalConfig.GetConfig().pland_decay_start_z.get();

        double pland_acc_xy = pland_max_acc_xy;

        // FOV 保护 (算当前视角的几何偏移)
        double dist_xy =
            std::hypot(target_body_relative.x(), target_body_relative.y());
        double visual_angle_deg = std::atan2(dist_xy, current_z) * 180.0 / M_PI;

        // 2. 设定 FOV 安全边界 (假设半视角是 49度，我们把警戒线设在 30度)
        double fov_warning_deg = 40.0;
        double fov_danger_deg = 55.0;

        // 3. 计算视野惩罚系数 (1.0 代表全速，0.2 代表极其小心)
        double fov_penalty = 1.0;

        // 计算速度限幅
        double limit_start_z = 5.0;
        double min_cruise_speed =
            GlobalConfig.GetConfig().pland_min_cruise_speed.get();
        double max_cruise_speed_xy =
            GlobalConfig.GetConfig().pland_cruise_speed_xy.get();
        double cruise_speed_xy = max_cruise_speed_xy;

        // if (current_z > 0 && current_z <= limit_start_z) {
        //     double ratio = current_z / limit_start_z;
        //     cruise_speed_xy = min_cruise_speed +
        //                       (max_cruise_speed_xy - min_cruise_speed) *
        //                       ratio;
        // } else if (current_z <= 0) {
        //     cruise_speed_xy = min_cruise_speed;
        // }

        if (!traj_gen_.is_initialized()) {
            traj_gen_.reset(Eigen::Vector2d(pos_enu.x(), pos_enu.y()));
        }

        // =======================================================
        // 生成平滑轨迹
        // =======================================================
        Eigen::Vector2d target_pos_xy(future_target_enu.x(),
                                      future_target_enu.y());
        auto virtual_state_enu = traj_gen_.step(dt, target_pos_xy, vel_xy,
                                                cruise_speed_xy, pland_acc_xy);

        // [牵引绳 Leash 逻辑]
        Eigen::Vector2d current_pos_enu_xy(pos_enu.x(), pos_enu.y());
        Eigen::Vector2d tracking_err =
            virtual_state_enu.pos - current_pos_enu_xy;

        double max_leash_length = std::max(2.0, 0.5 + 0.5 * (current_z / 3.0));

        if (tracking_err.norm() > max_leash_length) {
            virtual_state_enu.pos =
                current_pos_enu_xy +
                tracking_err.normalized() * max_leash_length;
            virtual_state_enu.vel *= 0.5;
            traj_gen_.force_set_state(virtual_state_enu.pos,
                                      virtual_state_enu.vel);
        }

        // 转换回机体系
        Eigen::Vector3d virtual_err_enu(virtual_state_enu.pos.x() - pos_enu.x(),
                                        virtual_state_enu.pos.y() - pos_enu.y(),
                                        0.0);
        Eigen::Vector3d virtual_err_body = R_wb.transpose() * virtual_err_enu;

        Eigen::Vector3d virtual_vel_enu(virtual_state_enu.vel.x(),
                                        virtual_state_enu.vel.y(), 0.0);
        Eigen::Vector3d virtual_vel_body = R_wb.transpose() * virtual_vel_enu;

        // 纯净的前馈速度 (内部已经包含了目标移动速度)
        Eigen::Vector3d total_ff_vel(virtual_vel_body.x(), virtual_vel_body.y(),
                                     0.0);

        // ---------------- 降落阶段 Z 轴与下发逻辑 ----------------
        double touchdown_vel = GlobalConfig.GetConfig().touchdown_velocity;
        double pland_gamma_z = GlobalConfig.GetConfig().pland_gamma_z.get();
        double pland_gamma_yaw = GlobalConfig.GetConfig().pland_gamma_yaw.get();

        double max_gamma = GlobalConfig.GetConfig().pland_gamma.get();
        double min_gamma = GlobalConfig.GetConfig().pland_min_gamma.get();
        double gamma = max_gamma;
        if (current_z > 0 && current_z <= decay_start_z) {
            double ratio = current_z / decay_start_z;
            gamma = min_gamma + (max_gamma - min_gamma) * ratio;
        } else if (current_z <= 0) {
            gamma = min_gamma;
        }

        double current_visual_angle_deg =
            std::atan2(dist_xy, current_z) * 180.0 / M_PI;

        double max_vel_z = 0.0;
        double z_body_target = 0.0;

        double terminal_deadzone_radius = 0.25;

        if (current_z < 0.8 && (current_xy_error < 0.4 || is_blind_drop_)) {
            is_blind_drop_ = true;  // 状态上锁！

            max_vel_z = touchdown_vel + 0.2;
            z_body_target = -current_z - 0.5;

            // 盲降时，保持向前的速度趋势
            // virtual_err_body.x() = 0.0;
            // virtual_err_body.y() = 0.0;
            // pland_acc_xy = 0.5;  // 防止乱晃
        } else {
            // 【漏斗下滑道逻辑】
            double hold_dist_thresh = 3.5;

            // ★ 动态对准阈值：高度越高允许的误差越大，高度越低要求越苛刻
            // 例如：Z=3m时允许 1.2m 的误差；Z=0.5m时必须进入 0.2m
            // 的误差内才准下砸！
            double align_dist_thresh = std::max(0.2, current_z * 0.4);

            if (current_xy_error > hold_dist_thresh) {
                max_vel_z = 0.0;
                z_body_target = 0.0;
            } else {
                double descent_factor = 1.0;
                if (current_xy_error > align_dist_thresh) {
                    // 如果没进入漏斗中心，按比例疯狂减速，在空中等待超前补偿把机身拉过来
                    descent_factor =
                        1.0 - (current_xy_error - align_dist_thresh) /
                                  (hold_dist_thresh - align_dist_thresh);
                    // 给一个极小的下降速度，几乎悬停等待
                    descent_factor = std::max(0.1, descent_factor);
                }

                double takeoff_alt = ctx_.takeoff_lon_lat_alt.load().z();
                double alt_ratio =
                    std::min(1.0, std::abs(current_z / takeoff_alt));
                double base_descent_vel = 0.2 + alt_ratio * 0.8;

                max_vel_z = base_descent_vel * descent_factor;
                z_body_target = -current_z - 0.5;
            }
        }
        ctx_.tracker->send_pos_cmd(
            {virtual_err_body.x(), virtual_err_body.y(), z_body_target},
            obs.yaw_relative, 0.0, total_ff_vel, cruise_speed_xy, max_vel_z,
            CmdFrame::BODY, {1.0, pland_gamma_yaw, pland_gamma_z},
            pland_acc_xy);
        // [Debug 日志]

        log_idx_ += 1;
        if (log_idx_ % 10 == 0) {
            // 1. 记录 EKF 估算的目标速度和角速度（这是前馈追踪的核心）
            double ekf_v = obs.target_vel_enu.template head<2>().norm();
            double ekf_omega = obs.target_vel_enu.z();

            // 2. 记录当前真实视角，看看是否真的超出了相机物理 FOV
            double current_visual_angle_deg =
                std::atan2(dist_xy, current_z) * 180.0 / M_PI;

            // 3. 记录 Leash (牵引绳) 是否处于饱和状态
            bool is_leash_clamped = tracking_err.norm() > max_leash_length;

            spdlog::info(
                "Z:{:.1f} | EKF_V:{:.2f}m/s, W:{:.2f}rad/s | Angle:{:.1f}deg "
                "(blind_drop:{}) | "
                "RawErr:[{:.2f}, {:.2f}] | VirtErr:[{:.2f}, {:.2f}] "
                "(LeashClamp:{}) | "
                "AccLim:{:.2f} | Delay:{:.3f}s",
                current_z, ekf_v, ekf_omega, current_visual_angle_deg,
                is_blind_drop_ ? "YES" : "NO", target_body_relative.x(),
                target_body_relative.y(), virtual_err_body.x(),
                virtual_err_body.y(), is_leash_clamped ? "YES" : "NO",
                pland_acc_xy, delay_sec);
        }
    }
};