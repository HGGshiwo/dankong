#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>

#include "./config.hpp"
// #include "./kalman_filter_imm_ukf.hpp"
// #include "./kalman_filter_imm.hpp"
// #include "./kalman_filter_ctrv_ukf.hpp"
#include "./kalman_filter_ctrv.hpp"
#include "./kalman_filter_yaw.hpp"
// #include "./tag_detector/tags_pattern1.hpp"
#include "./tag_detector/multiarray_tags_pattern.hpp"
#include "./target_tracker.hpp"
#include "./utils.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "core/global_config.hpp"
#include "robot_context.hpp"
#include "states/state_utils.hpp"
#include "utils/dirty_var.hpp"
#include "utils/thread_runner.hpp"
#include "utils/throttle.hpp"

struct DetectorResult {
    bool is_valid = false;
    double stamp;  // 图像拍摄的真实时间

    // ENU 导航系下的绝对目标位置和速度 (EKF 融合后)
    Eigen::Vector3d target_pos_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_vel_enu =
        Eigen::Vector3d::Zero();  // .z() is yaw_rate

    double target_yaw_enu = 0.0;
    double yaw_relative = 0.0;
    double current_z = 0.0;  // 相对高度

    std::optional<Eigen::Vector3d> pnp_pos;
    std::optional<Eigen::Vector3d> los_pos;
    std::optional<Eigen::Vector3d> fused_pos;
};

class LandingDetector : public IThreadRunner {
   private:
    PlandConfig &config_;
    std::unique_ptr<SafeAprilTagDetector> tag_detector_;
    std::unique_ptr<ITargetPattern> current_pattern_;

    RobotContext &ctx_;

    std::shared_ptr<KalmanFilterCTRV> kf_xy_;
    std::shared_ptr<KalmanFilterYaw> kf_yaw_;
    std::shared_ptr<KalmanFilterYaw> kf_abs_yaw_;
    std::shared_ptr<TargetTracker> target_tracker_;

    // --- 时间戳与缓存 ---
    std::function<void(const DetectorResult &, cv::Mat &)> set_target_;
    int target_id_ = -1;

    Eigen::Matrix3d camera_inner_matrix_;
    double last_ekf_stamp_;

    Eigen::Vector3d last_valid_enu_pos_;  // 记录上一次有效位置
    bool has_last_valid_pos_ = false;

   private:
    // ---------------------------------------------------------
    // [新增核心函数]：动态获取 相机光学系(C) 到 机体系(B) 的旋转矩阵
    // 完美融合了云台的实时 Roll/Pitch/Yaw，并彻底规避欧拉角死锁
    // ---------------------------------------------------------
    Eigen::Matrix3d get_dynamic_camera_to_body_rotation(
        const Eigen::Matrix3d &hist_R_wb, std::optional<double> gimbal_roll,
        std::optional<double> gimbal_pitch, std::optional<double> gimbal_yaw,
        bool is_gimbal_absolute) const {
        // 默认相机朝下 (-90度)
        double def_roll = 0.0;
        double def_pitch = -M_PI_2;
        double def_yaw = 0.0;

        double g_roll = gimbal_roll.value_or(def_roll);
        double g_yaw = gimbal_yaw.value_or(def_yaw);
        double g_pitch;

        if (is_gimbal_absolute) {
            // 如果是绝对模式：输入的 gimbal_pitch 是相对大地的绝对下视角度
            double abs_pitch = gimbal_pitch.value_or(def_pitch);

            // 提取无人机当前的真实低头角度 (Nose Down)
            // 原理：取出 FLU 坐标系 X 轴在 ENU 世界中的投影向量
            Eigen::Vector3d forward_enu = hist_R_wb.col(0);
            double drone_pitch_down = std::atan2(
                -forward_enu.z(), std::sqrt(forward_enu.x() * forward_enu.x() +
                                            forward_enu.y() * forward_enu.y()));

            // 云台需要相对机身转动的角度 = 绝对目标角 + 飞机自身的低头角补偿
            g_pitch = abs_pitch + drone_pitch_down;
        } else {
            g_pitch = gimbal_pitch.value_or(def_pitch);
        }

        // 云台电机旋转矩阵 (在 FRD 坐标系下 Z-Y-X 旋转)
        Eigen::Matrix3d R_motor =
            (Eigen::AngleAxisd(g_yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(g_pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(g_roll, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();

        // FRD 到 FLU 的静态转换
        Eigen::Matrix3d R_frd_to_flu;
        R_frd_to_flu << 1, 0, 0, 0, -1, 0, 0, 0, -1;

        // 相机光学系 -> FRD系 -> FLU机身系
        return R_frd_to_flu * R_motor * get_cam_to_frd_matrix();
    }

    // 获取 相机在机体系下的物理偏移向量
    Eigen::Vector3d get_camera_body_offset() const {
        return Eigen::Vector3d(config_.offset_x.get(), config_.offset_y.get(),
                               config_.offset_z.get());
    }

    Eigen::Matrix3d get_cam_to_frd_matrix() const {
        Eigen::Matrix3d R;
        R << 0, 0, 1, 1, 0, 0, 0, 1, 0;
        return R;
    }

    // ---------------------------------------------------------
    // [大幅精简的射线法]：直接依赖正向运动学
    // ---------------------------------------------------------
    Eigen::Vector3d solve_pose_by_ray(
        double img_x, double img_y, const Eigen::Vector3d &hist_drone_pos,
        const Eigen::Matrix3d &hist_R_wb, double platform_height = 0.0,
        std::optional<double> gimbal_roll = std::nullopt,
        std::optional<double> gimbal_pitch = std::nullopt,
        std::optional<double> gimbal_yaw = std::nullopt,
        bool is_gimbal_absolute = true) const {
        Eigen::Vector3d t_bc = get_camera_body_offset();
        Eigen::Vector3d cam_ray =
            camera_inner_matrix_.inverse() * Eigen::Vector3d(img_x, img_y, 1.0);

        // 核心：直接获取动态的 C 到 B 的旋转矩阵
        Eigen::Matrix3d R_cam_to_body = get_dynamic_camera_to_body_rotation(
            hist_R_wb, gimbal_roll, gimbal_pitch, gimbal_yaw,
            is_gimbal_absolute);

        Eigen::Matrix3d R_effective_cam_to_world = hist_R_wb * R_cam_to_body;
        Eigen::Vector3d world_ray = R_effective_cam_to_world * cam_ray;

        Eigen::Vector3d camera_pos_world = hist_drone_pos + hist_R_wb * t_bc;

        // 如果射线平行于地面或指向天空
        if (std::abs(world_ray.z()) < 1e-6) {
            return Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN());
        }
        double t = (platform_height - camera_pos_world.z()) / world_ray.z();
        if (t < 0) {
            return Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN());
        }
        return camera_pos_world + t * world_ray;
    }

    // ---------------------------------------------------------
    // [修复后的 PnP 解算]：注入云台动态补偿
    // ---------------------------------------------------------
    Eigen::Vector3d solve_pose_by_pnp(
        const TargetPose *safe_pose, const Eigen::Vector3d &hist_drone_pos,
        const Eigen::Matrix3d &hist_R_wb, std::optional<double> gimbal_roll,
        std::optional<double> gimbal_pitch, std::optional<double> gimbal_yaw,
        bool is_gimbal_absolute, double &out_relative_yaw,
        double &out_abs_yaw) {
        // 核心：用动态矩阵替代原本写死的静态 R_bc
        Eigen::Matrix3d R_bc = get_dynamic_camera_to_body_rotation(
            hist_R_wb, gimbal_roll, gimbal_pitch, gimbal_yaw,
            is_gimbal_absolute);

        Eigen::Vector3d t_bc = get_camera_body_offset();

        Eigen::Vector3d t_tag_cam = safe_pose->t;
        Eigen::Matrix3d R_tag_cam = safe_pose->R;

        Eigen::Vector3d P_tag_body = t_bc + R_bc * t_tag_cam;
        Eigen::Matrix3d R_tag_body = R_bc * R_tag_cam;

        out_relative_yaw = std::atan2(R_tag_body(1, 0), R_tag_body(0, 0));

        Eigen::Matrix3d R_tag_world = hist_R_wb * R_tag_body;
        out_abs_yaw = std::atan2(R_tag_world(1, 0), R_tag_world(0, 0));

        return hist_drone_pos + hist_R_wb * P_tag_body;
    }

    Eigen::Vector3d solve_fused_pose(Eigen::Vector3d pnp_result,
                                     Eigen::Vector3d los_result) {
        // [原有逻辑保持不变]
        double current_z = get_current_z(ctx_);

        double z_high = GlobalConfig.GetConfig().z_high.get();
        double z_low = GlobalConfig.GetConfig().z_low.get();

        double w_pnp = 0.0;
        if (current_z >= z_high) {
            w_pnp = 0.0;
        } else if (current_z <= z_low) {
            w_pnp = 1.0;
        } else {
            w_pnp = (z_high - current_z) / (z_high - z_low);
        }

        Eigen::Vector3d fused_result;
        fused_result.x() =
            (1 - w_pnp) * los_result.x() + w_pnp * pnp_result.x();
        fused_result.y() =
            (1 - w_pnp) * los_result.y() + w_pnp * pnp_result.y();
        fused_result.z() = 0;
        return fused_result;
    }

   public:
    LandingDetector(
        PlandConfig &config, RobotContext &ctx,
        std::function<void(const DetectorResult &, cv::Mat &)> set_target)
        : IThreadRunner(ctx.engine->get_time_provider()),
          config_(config),
          ctx_(ctx),
          set_target_(set_target) {
        tag_detector_ =
            std::make_unique<SafeAprilTagDetector>(config_.tag_famliy.get());

        kf_xy_ = std::make_shared<KalmanFilterCTRV>();
        kf_yaw_ = std::make_shared<KalmanFilterYaw>();
        kf_abs_yaw_ = std::make_shared<KalmanFilterYaw>();
        target_tracker_ = std::make_shared<TargetTracker>();

        camera_inner_matrix_ = config_.camera_inner_matrix.get();
        last_ekf_stamp_ = 0.0;

        // current_pattern_ = std::make_unique<ConcentricTagsPattern>(
        //     target_id_, config_.inner_tag_size.get(),
        //     config_.outter_tag_size.get());
        current_pattern_ = std::make_unique<MultiArrayTagsPattern>(
            target_id_, config_.tag_pos_map.get());
    }

    ~LandingDetector() {}

    /*
        pos_enu: 用来计算目标的真实位置，判断是否在移动
    */
    DetectorResult update(double dt_thread, cv::Mat &detected) {
        DetectorResult output;
        output.is_valid = false;

        // 1. 时间戳与历史状态获取 (时光机逻辑保持不变)
        auto stamp_tracker = ctx_.pland_image_stamp.load();
        if (stamp_tracker <= 0 ||
            std::abs(stamp_tracker - last_ekf_stamp_) < 1e-6)
            return output;

        output.stamp = stamp_tracker;
        double elapsed =
            ctx_.engine->get_time_provider()->now() - stamp_tracker;
        if (elapsed > 0.15) {
            spdlog::warn("[Pland] image stamp too late {:.3f}s", elapsed);
            return output;
        }

        Eigen::Vector3d hist_drone_pos;
        Eigen::Quaterniond hist_q;
        double min_diff = -1;
        if (!ctx_.pose_history.get_pose_at(stamp_tracker, hist_drone_pos,
                                           hist_q, min_diff)) {
            hist_drone_pos = ctx_.pos_enu.load();
            hist_q = ctx_.orientation.load();
        }
        Eigen::Matrix3d hist_R_wb = hist_q.toRotationMatrix();

        // 2. 图像预处理与基础检测
        cv::Mat current_img = ctx_.pland_image.load();
        if (current_img.empty()) return output;
        detected = current_img.clone();

        cv::Mat gray;
        cv::cvtColor(current_img, gray, cv::COLOR_BGR2GRAY);
        auto detections = tag_detector_->detect(gray);

        // =======================================================
        // ★ 核心重构点：调用策略类处理特定图案、绘制并返回标准化观测 ★
        // =======================================================
        TargetObservation obs = current_pattern_->process(
            detections, tag_detector_.get(), camera_inner_matrix_, detected);

        std::optional<Eigen::Vector3d> pland_target = ctx_.pland_target.load();
        bool use_target = pland_target.has_value();
        if (!obs.is_valid && !use_target) return output;

        double relative_yaw = 0.0;
        double abs_yaw = 0.0;
        Eigen::Vector3d raw_target_enu;
        Eigen::Vector3d pnp_enu, los_enu;

        // 3. 如果视觉有效，进行姿态歧义消除与坐标系转换
        if (obs.is_valid) {
            TargetPose *best_pose = &obs.pose1;  // 默认相信 pose1

            bool gimbal_abs = config_.pland_gimbal_abs.get();
            std::optional<double> gimbal_roll, gimbal_pitch, gimbal_yaw;
            if (!ctx_.pose_history.get_gimbal_at(stamp_tracker, gimbal_roll,
                                                 gimbal_pitch, gimbal_yaw)) {
                gimbal_roll = ctx_.gimbal_roll.load();
                gimbal_pitch = ctx_.gimbal_pitch.load();
                gimbal_yaw = ctx_.gimbal_yaw.load();
            }

            // --- 歧义消除逻辑 ---
            double time_since_last_valid = stamp_tracker - last_ekf_stamp_;
            bool is_prior_reliable =
                has_last_valid_pos_ && (time_since_last_valid < 0.5);

            if (obs.pose2.valid && is_prior_reliable) {
                Eigen::Matrix3d R_bc_dynamic =
                    get_dynamic_camera_to_body_rotation(hist_R_wb, gimbal_roll,
                                                        gimbal_pitch,
                                                        gimbal_yaw, gimbal_abs);

                auto get_yaw_from_pose = [&](const TargetPose &p) {
                    Eigen::Matrix3d R_tag_world =
                        hist_R_wb * (R_bc_dynamic * p.R);
                    return std::atan2(R_tag_world(1, 0), R_tag_world(0, 0));
                };

                double yaw1 = get_yaw_from_pose(obs.pose1);
                double yaw2 = get_yaw_from_pose(obs.pose2);
                double predicted_yaw = kf_abs_yaw_->get_yaw();

                double diff1 = std::abs(
                    KalmanFilterYaw::normalize_angle(yaw1 - predicted_yaw));
                double diff2 = std::abs(
                    KalmanFilterYaw::normalize_angle(yaw2 - predicted_yaw));

                if (diff2 < diff1) {
                    best_pose = &obs.pose2;
                    spdlog::info(
                        "[Pland] Pose Ambiguity Resolved: Chose Pose 2 based "
                        "on EKF prior.");
                }
            }

            // --- 坐标求解 ---
            pnp_enu = solve_pose_by_pnp(best_pose, hist_drone_pos, hist_R_wb,
                                        gimbal_roll, gimbal_pitch, gimbal_yaw,
                                        gimbal_abs, relative_yaw, abs_yaw);

            los_enu = solve_pose_by_ray(
                obs.center_pixel.x, obs.center_pixel.y, hist_drone_pos,
                hist_R_wb, config_.platform_height.get(), gimbal_roll,
                gimbal_pitch, gimbal_yaw, gimbal_abs);

            bool los_valid = !std::isnan(los_enu.x());
            bool pnp_valid = !std::isnan(pnp_enu.x());

            if (!los_valid && pnp_valid) {
                raw_target_enu = pnp_enu;
                output.pnp_pos = raw_target_enu;
            } else if (los_valid && !pnp_valid) {
                raw_target_enu = los_enu;
                output.los_pos = raw_target_enu;
            } else if (!los_valid && !pnp_valid) {
                return output;
            } else {
                output.pnp_pos = pnp_enu;
                output.los_pos = los_enu;
                raw_target_enu = solve_fused_pose(pnp_enu, los_enu);
            }
            output.is_valid = true;
        }

        static Throttle throttle{4};
        auto pos_enu = ctx_.pos_enu.load();

        if (output.is_valid || use_target) {
            Eigen::Vector2d target_err{0.0, 0.0};
            if (use_target) {
                Eigen::Vector3d target = pland_target.value();
                target_err = pos_enu.head<2>() - target.head<2>();
                raw_target_enu = {target.x(), target.y(), 0.0};
                abs_yaw = target.z();
                relative_yaw = abs_yaw - ctx_.yaw_enu.load();
                output.is_valid = true;
            }
            output.fused_pos = raw_target_enu;

            // 计算 EKF 时间间隔 (使用真实照片时间戳)
            double dt_ekf = 0.033;
            if (last_ekf_stamp_ > 0) {
                dt_ekf = (stamp_tracker - last_ekf_stamp_);
            }

            if (has_last_valid_pos_) {
                // 计算本次视觉结果与上一次有效结果的物理距离
                double jump_dist =
                    (raw_target_enu.head<2>() - last_valid_enu_pos_.head<2>())
                        .norm();

                // 物理学常识限制：假设降落垫(车/船)的最大移动速度是 20 m/s
                // dt_ekf 是与上一帧的时间差，那么理论上的最大合理位移是:
                double max_physical_jump =
                    20.0 * dt_ekf + 1.0;  // 加 1.0 米的容错冗余

                if (jump_dist > max_physical_jump) {
                    spdlog::warn(
                        "[Pland] Outlier Rejected! Jump dist {:.2f}m is "
                        "physically impossible.",
                        jump_dist);
                    // 剔除这个幽灵数据，强制按无效帧处理
                    output.is_valid = false;
                    return output;
                }
            }

            if (!has_last_valid_pos_ || dt_ekf <= 1e-4 || dt_ekf > 1.0) {
                reset_estimator();
                dt_ekf = 0.033;
                kf_xy_->force_set_state(raw_target_enu.x(), raw_target_enu.y());
                kf_yaw_->force_set_state(relative_yaw);
                kf_abs_yaw_->force_set_state(abs_yaw);
            }
            last_ekf_stamp_ = stamp_tracker;

            // 数据合法，更新记录并放行进入 EKF
            last_valid_enu_pos_ = raw_target_enu;
            has_last_valid_pos_ = true;

            TargetState target_state =
                target_tracker_->update(raw_target_enu.head<2>());

            Eigen::Quaterniond q = ctx_.orientation.load();
            q.normalize();
            Eigen::Matrix3d R_wb = q.toRotationMatrix();
            auto pos_enu = ctx_.pos_enu.load();
            Eigen::Vector3d target_body_relative =
                R_wb.transpose() * (raw_target_enu - pos_enu);

            double dist_xy =
                std::hypot(target_body_relative.x(), target_body_relative.y());
            double current_z = get_current_z(ctx_);
            double current_visual_angle_deg =
                std::atan2(dist_xy, current_z) * 180.0 / M_PI;

            // 3. 执行 KF 滤波 (由于已经是 ENU，不需要任何 R_wb 转换)
            double current_angular_rate = ctx_.vel_angular_body.load().norm();
            double epsilon = 0;
            kf_xy_->update(epsilon, raw_target_enu.x(), raw_target_enu.y(),
                           dt_ekf, current_z, current_angular_rate,
                           current_visual_angle_deg);
            kf_yaw_->update(relative_yaw, dt_ekf);
            kf_abs_yaw_->update(abs_yaw, dt_ekf);

            Eigen::Vector2d v_xy_enu = kf_xy_->get_vel();
            if (v_xy_enu.norm() < config_.velocity_deadzone.get()) {
                v_xy_enu.setZero();
            }

            output.current_z = current_z;
            output.yaw_relative = kf_yaw_->get_yaw();
            output.target_pos_enu.head<2>() = kf_xy_->get_pos();
            output.target_pos_enu.z() = raw_target_enu.z();
            output.target_yaw_enu = kf_abs_yaw_->get_yaw();

            double yaw_rate = kf_abs_yaw_->get_yaw_rate();

            if (target_state == TargetState::MOVING) {
                output.target_vel_enu << v_xy_enu.x(), v_xy_enu.y(), yaw_rate;
            } else {
                output.target_vel_enu.setZero();
                output.target_vel_enu.z() = yaw_rate;
            }
            // auto prob = kf_xy_->get_prob();
            if (throttle.shouldLog()) {
                spdlog::info(
                    "pnp_enu=[{:.2f}, {:.2f}] los_enu=[{:.2f}, {:.2f}] "
                    "drone_enu=[{:.2f}, {:.2f}, {:.2f}] "
                    "drone_rpy=[{:.2f}, {:.2f}, {:.2f}] "
                    "target_err=[{:.2f}, {:.2f}] "
                    "vel_enu=[{:.2f}, {:.2f}] rangefinder={:.2f} "
                    "epsilon={:.2f}",
                    pnp_enu.x(), pnp_enu.y(), los_enu.x(), los_enu.y(),
                    pos_enu.x(), pos_enu.y(), pos_enu.z(), ctx_.roll.load(),
                    ctx_.pitch.load(), ctx_.yaw_enu.load(), target_err.x(),
                    target_err.y(), output.target_vel_enu.x(),
                    output.target_vel_enu.y(), ctx_.rangefinder_alt.load(),
                    epsilon);
            }
        }

        return output;
    }

    void set_target_id(int target_id) {
        target_id_ = target_id;
        current_pattern_ = std::make_unique<MultiArrayTagsPattern>(
            target_id_, config_.tag_pos_map.get());
    }

    void on_start() { reset_estimator(); }

    void on_stop() {
        ctx_.pland_target.store(std::nullopt);  // 清空目标，避免污染
    }

    void on_step(double dt) {
        cv::Mat detected;
        auto result = update(dt, detected);
        set_target_(result, detected);
    }

    void reset_estimator() {
        kf_xy_->reset();
        kf_yaw_->reset();
        kf_abs_yaw_->reset();
        target_tracker_->reset();
        last_ekf_stamp_ = 0.0;
        has_last_valid_pos_ = false;
        last_valid_enu_pos_ = Eigen::Vector3d::Zero();
    }
};
