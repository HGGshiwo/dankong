#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "./config.hpp"
// #include "./kalman_filter_imm_ukf.hpp"
// #include "./kalman_filter_imm.hpp"
// #include "./kalman_filter_ctrv_ukf.hpp"
#include "./kalman_filter_ctrv.hpp"
#include "./kalman_filter_yaw.hpp"
#include "./target_tracker.hpp"
#include "robot_context.hpp"
#include "safe_aprialtag.hpp"
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
    // 获取 相机光学系(C) 到 机体系(B) 的旋转矩阵
    Eigen::Matrix3d get_camera_to_body_rotation() const {
        // 向下看：相机X(右)->机身-Y, 相机Y(下)->机身-X, 相机Z(前)->机身-Z
        static const Eigen::Matrix3d R_bc =
            (Eigen::Matrix3d() << 0, -1, 0, -1, 0, 0, 0, 0, -1).finished();
        return R_bc;
    }

    // 获取 相机在机体系下的物理偏移向量
    Eigen::Vector3d get_camera_body_offset() const {
        // config_.offset_z 如果没有，默认为 0.0
        return Eigen::Vector3d(config_.offset_x.get(), config_.offset_y.get(),
                               config_.offset_z.get());
    }

    // [射线法直接返回目标在世界地图上的绝对ENU坐标，带云台掩码补偿
    Eigen::Vector3d solve_pose_by_ray(
        double img_x, double img_y, const Eigen::Vector3d &hist_drone_pos,
        const Eigen::Matrix3d &hist_R_wb,
        double platform_height = 0.0,  // 平台的高度
        bool stab_roll = false,        // 是否有横滚增稳？
        bool stab_pitch = false,  // 是否有俯仰增稳？(最常见的单轴)
        bool stab_yaw = false) {  // 是否有航向增稳？

        Eigen::Matrix3d R_bc = get_camera_to_body_rotation();
        Eigen::Vector3d t_bc = get_camera_body_offset();

        // 1. 相机光学系下的射线方向
        Eigen::Vector3d cam_ray =
            camera_inner_matrix_.inverse() * Eigen::Vector3d(img_x, img_y, 1.0);

        // ==========================================
        // 2. 姿态解耦：构建云台增稳后的等效旋转矩阵
        // ==========================================
        // 从飞机的绝对姿态矩阵中，提取 Z-Y-X (Yaw-Pitch-Roll) 欧拉角
        double roll = atan2(hist_R_wb(2, 1), hist_R_wb(2, 2));
        double pitch = asin(-hist_R_wb(2, 0));
        double yaw = atan2(hist_R_wb(1, 0), hist_R_wb(0, 0));

        // 云台掩码
        // (Mask)：如果该轴被云台增稳（始终朝下），则剥离该轴的飞机旋转干扰
        if (stab_roll) roll = 0.0;
        if (stab_pitch) pitch = 0.0;
        if (stab_yaw) yaw = 0.0;  // 普通下视云台通常为 false，让其跟随飞机偏航

        // 重组增稳后的世界系旋转矩阵 R_wb_stabilized
        Eigen::Matrix3d R_wb_stabilized;
        R_wb_stabilized = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        // ==========================================
        // 3. 机体系到世界系的射线转换 (核心区分点)
        // ==========================================
        // 射线方向：使用【剥离了特定轴的增稳矩阵】
        Eigen::Vector3d body_ray = R_bc * cam_ray;
        Eigen::Vector3d world_ray = R_wb_stabilized * body_ray;

        // 射线起点：必须使用【飞机的全量原始矩阵】，因为云台转动不改变它的物理铰链点位置
        Eigen::Vector3d camera_pos_world = hist_drone_pos + hist_R_wb * t_bc;

        // ==========================================
        // 4. 求与 Z = platform_height 平面的交点 (已修改)
        // ==========================================

        // 如果射线与水平面几乎平行，则认为无交点
        if (std::abs(world_ray.z()) < 1e-6) {
            return Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN());
        }

        // 计算射线步长 t: t = (目标平面Z - 起点Z) / 射线方向Z
        double t = (platform_height - camera_pos_world.z()) / world_ray.z();

        // t < 0 表示射线背离了目标平面（例如飞机在平台上方，但相机朝天上看）
        if (t < 0) {
            return Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN());
        }

        return camera_pos_world + t * world_ray;  // 直接返回带高度的 ENU 交点
    }

    // [重大修改] PnP 法直接返回目标在世界地图上的【绝对ENU坐标】
    Eigen::Vector3d solve_pose_by_pnp(const SafeTagPose *safe_pose,
                                      const Eigen::Vector3d &hist_drone_pos,
                                      const Eigen::Matrix3d &hist_R_wb,
                                      double &out_relative_yaw,
                                      double &out_abs_yaw) {
        Eigen::Matrix3d R_bc = get_camera_to_body_rotation();
        Eigen::Vector3d t_bc = get_camera_body_offset();
        auto pose = &(safe_pose->pose);
        // 1. 提取 Tag 在相机光学系 (C) 下的位姿
        Eigen::Vector3d t_tag_cam(pose->t->data[0], pose->t->data[1],
                                  pose->t->data[2]);
        Eigen::Matrix3d R_tag_cam;
        R_tag_cam << pose->R->data[0], pose->R->data[1], pose->R->data[2],
            pose->R->data[3], pose->R->data[4], pose->R->data[5],
            pose->R->data[6], pose->R->data[7], pose->R->data[8];

        // 2. 转换到无人机机体坐标系 (B)
        // 相对位置 = 相机偏移 + (机身视角看过去的相对坐标)
        Eigen::Vector3d P_tag_body = t_bc + R_bc * t_tag_cam;

        // 相对姿态 = 相机旋转 * 目标旋转
        Eigen::Matrix3d R_tag_body = R_bc * R_tag_cam;

        // 3. 提取偏航角 Yaw (Z-Y-X 欧拉角中的 Z 旋转)
        out_relative_yaw = std::atan2(R_tag_body(1, 0), R_tag_body(0, 0));

        // 转换到绝对 ENU 坐标
        Eigen::Matrix3d R_tag_world = hist_R_wb * R_tag_body;
        out_abs_yaw = std::atan2(R_tag_world(1, 0), R_tag_world(0, 0));

        return hist_drone_pos + hist_R_wb * P_tag_body;
    }

    // 传入 PnP 需要的 pose 结构体，以及 LOS 需要的像素坐标
    Eigen::Vector3d solve_fused_pose(Eigen::Vector3d pnp_result,
                                     Eigen::Vector3d los_result) {
        // 3. 基于当前高度计算权重
        // 这里的 alt_ 假设是无人机相对地面的绝对高度 (正数)
        // double current_z = std::abs(ctx_.rangefinder_alt.load());
        double current_z = ctx_.pos_enu.load().z();

        // ---- 【关键配置参数】 ----
        double z_high = 3.0;  // 高于 1 米，完全信任 LOS 射线法
        double z_low = 1.0;   // 低于 0 米，完全信任 PnP 视觉解算
        // ------------------------

        // 4. 计算 PnP 的权重 (w_pnp 从 0.0 平滑过渡到 1.0)
        double w_pnp = 0.0;
        if (current_z >= z_high) {
            w_pnp = 0.0;  // 高空纯 LOS
        } else if (current_z <= z_low) {
            w_pnp = 1.0;  // 低空纯 PnP
        } else {
            // 过渡区：线性插值
            w_pnp = (z_high - current_z) / (z_high - z_low);
        }
        double w_los = 1.0 - w_pnp;

        // 5. 执行融合操作
        Eigen::Vector3d fused_result;

        // X 和 Y 进行加权平均平滑过渡
        // fused_result.x() = (w_pnp * pnp_result.x()) + (w_los *
        // los_result.x()); fused_result.y() = (w_pnp * pnp_result.y()) + (w_los
        // * los_result.y());
        fused_result.x() = los_result.x();
        fused_result.y() = los_result.y();
        fused_result.z() = 0;  // los没有yaw，只能用pnp

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
        tag_detector_ = std::make_unique<SafeAprilTagDetector>();

        kf_xy_ = std::make_shared<KalmanFilterCTRV>();
        kf_yaw_ = std::make_shared<KalmanFilterYaw>();
        kf_abs_yaw_ = std::make_shared<KalmanFilterYaw>();
        target_tracker_ = std::make_shared<TargetTracker>();

        camera_inner_matrix_ = config_.camera_inner_matrix.get();
        last_ekf_stamp_ = 0.0;
    }

    ~LandingDetector() {}

    /*
        pos_enu: 用来计算目标的真实位置，判断是否在移动
    */
    DetectorResult update(double dt_thread, cv::Mat &detected) {
        DetectorResult output;
        output.is_valid = false;

        // 1. 获取图片真实时间戳
        auto stamp_tracker = ctx_.pland_image_stamp.load();
        if (stamp_tracker <= 0) {
            return output;  // 没有获取到图片
        }

        // 如果图像时间戳没有更新，直接返回（防止重复处理同一帧）
        if (std::abs(stamp_tracker - last_ekf_stamp_) < 1e-6) return output;

        output.stamp = stamp_tracker;

        double elapsed =
            ctx_.engine->get_time_provider()->now() - stamp_tracker;
        if (elapsed > 0.15) {
            spdlog::warn("[Pland] image stamp too late {:.3f}s", elapsed);
            return output;
        }

        // =======================================================
        // ★ 时光机核心：提取拍照那一瞬间的历史无人机姿态 ★
        // =======================================================
        Eigen::Vector3d hist_drone_pos;
        Eigen::Quaterniond hist_q;
        double min_diff = -1;
        if (!ctx_.pose_history.get_pose_at(stamp_tracker, hist_drone_pos,
                                           hist_q, min_diff)) {
            // 如果找不到历史状态（比如刚启动），用当前状态凑合
            hist_drone_pos = ctx_.pos_enu.load();
            hist_q = ctx_.orientation.load();
        }
        auto rpy = state_utils::orientation_to_euler(hist_q);
        // spdlog::info(
        //     "min_diff={:.2f} pos_x={:.2f} pos_y={:.2f} r={:.2f} p={:.2f} "
        //     "y={:.2f}",
        //     min_diff, hist_drone_pos.x(), hist_drone_pos.y(), rpy.x(),
        //     rpy.y(), rpy.z());

        Eigen::Matrix3d hist_R_wb = hist_q.toRotationMatrix();

        // 【修改点 1】: 获取当前帧，保存到 detected
        // 用于可视化输出，并用于灰度转换
        cv::Mat current_img = ctx_.pland_image.load();
        if (current_img.empty()) return output;
        detected = current_img.clone();  // 保障 RTSP 内存安全

        cv::Mat gray;
        cv::cvtColor(current_img, gray, cv::COLOR_BGR2GRAY);
        auto detections = tag_detector_->detect(gray);

        apriltag_detection_t *inner_result = nullptr;
        double inner_dist = -1;
        apriltag_detection_t *outter_result = nullptr;
        double outter_dist = -1;

        for (int i = 0; i < detections.size(); i++) {
            apriltag_detection_t *det = detections[i];

            double dx = det->c[0] / ((double)gray.cols) - 0.5;
            double dy = det->c[1] / ((double)gray.rows) - 0.5;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (det->id == target_id_ * 2 + 1) {
                if (inner_dist < 0 || dist < inner_dist) {
                    inner_dist = dist;
                    inner_result = det;
                }
            } else if (det->id == target_id_ * 2) {
                if (outter_dist < 0 || dist < outter_dist) {
                    outter_dist = dist;
                    outter_result = det;
                }
            }
        }

        apriltag_detection_t *result = nullptr;
        bool is_inner = false;  // 初始化

        if (inner_dist >= 0) {
            result = inner_result;
            is_inner = true;
        } else if (outter_dist >= 0) {
            result = outter_result;
            is_inner = false;
        }

        double current_z = std::abs(ctx_.pos_enu.load().z());
        if (result != nullptr) {
            // 绘制绿色边框
            for (int i = 0; i < 4; i++) {
                cv::Point pt1(result->p[i][0], result->p[i][1]);
                cv::Point pt2(result->p[(i + 1) % 4][0],
                              result->p[(i + 1) % 4][1]);
                cv::line(detected, pt1, pt2, cv::Scalar(0, 255, 0), 2);
            }
            // 绘制红色中心点
            cv::circle(detected, cv::Point(result->c[0], result->c[1]), 4,
                       cv::Scalar(0, 0, 255), -1);
            // 标注 ID 文本
            std::string tag_label = "ID: " + std::to_string(result->id);
            cv::putText(detected, tag_label,
                        cv::Point(result->c[0] + 10, result->c[1] - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0),
                        2);

            // 后续姿态解算
            double tag_size =
                is_inner ? config_.inner_tag_size : config_.outter_tag_size;

            SafeTagPose pose1, pose2;

            tag_detector_->estimatePose(
                result, tag_size, camera_inner_matrix_(0, 0),
                camera_inner_matrix_(1, 1), camera_inner_matrix_(0, 2),
                camera_inner_matrix_(1, 2), pose1, pose2);

            if (!pose1.isValid()) {
                output.is_valid = false;
                return output;
            }

            SafeTagPose *best_pose =
                &pose1;  // 默认相信误差较小的那个 (通常是 pose1)

            // 如果算法确实产生了歧义(算出了两个解)，且我们的 EKF 已经收敛
            double time_since_last_valid = stamp_tracker - last_ekf_stamp_;
            bool is_prior_reliable =
                has_last_valid_pos_ && (time_since_last_valid < 0.5);
            if (pose2.isValid() && is_prior_reliable) {
                // 提取姿态 1 的旋转矩阵和 Yaw
                Eigen::Matrix3d R1 = pose1.getRotation();
                Eigen::Matrix3d R_tag_body1 =
                    get_camera_to_body_rotation() * R1;
                Eigen::Matrix3d R_tag_world1 = hist_R_wb * R_tag_body1;
                double yaw1 =
                    std::atan2(R_tag_world1(1, 0), R_tag_world1(0, 0));

                // 提取姿态 2 的旋转矩阵和 Yaw
                Eigen::Matrix3d R2 = pose2.getRotation();
                Eigen::Matrix3d R_tag_body2 =
                    get_camera_to_body_rotation() * R2;
                Eigen::Matrix3d R_tag_world2 = hist_R_wb * R_tag_body2;
                double yaw2 =
                    std::atan2(R_tag_world2(1, 0), R_tag_world2(0, 0));

                // 引入 EKF 的先验知识当裁判
                double predicted_yaw = kf_abs_yaw_->get_yaw();

                // 计算两个解与先验的差异 (注意 normalize_angle 处理 -pi 到 pi
                // 的环绕)
                double diff1 = std::abs(
                    KalmanFilterYaw::normalize_angle(yaw1 - predicted_yaw));
                double diff2 = std::abs(
                    KalmanFilterYaw::normalize_angle(yaw2 - predicted_yaw));

                // 哪一个更符合物理运动的连续性，我们就强制采用哪一个！
                if (diff2 < diff1) {
                    best_pose = &pose2;
                    spdlog::info(
                        "[Pland] Pose Ambiguity Resolved: Chose Pose 2 based "
                        "on EKF prior.");
                }
            }

            double px = result->c[0];
            double py = result->c[1];

            double relative_yaw = 0.0;
            double abs_yaw = 0.0;

            // 调用高度融合算法，传入历史位姿解算绝对坐标
            Eigen::Vector3d pnp_enu = solve_pose_by_pnp(
                best_pose, hist_drone_pos, hist_R_wb, relative_yaw, abs_yaw);
            Eigen::Vector3d los_enu = solve_pose_by_ray(
                px, py, hist_drone_pos, hist_R_wb,
                config_.platform_height.get(), config_.pland_fix_roll.get(),
                config_.pland_fix_pitch.get(), config_.pland_fix_yaw.get());

            bool los_valid = !std::isnan(los_enu.x());
            bool pnp_valid = !std::isnan(pnp_enu.x());

            Eigen::Vector3d raw_target_enu;
            output.is_valid = true;

            // 异常处理：如果由于某种原因只剩一个有效，直接返回有效的那个
            if (!los_valid && pnp_valid) {
                raw_target_enu = pnp_enu;
                output.pnp_pos = raw_target_enu;
            } else if (los_valid && !pnp_valid) {
                raw_target_enu = los_enu;
                output.los_pos = raw_target_enu;
            } else if (!los_valid && !pnp_valid) {
                output.is_valid = false;
                return output;  // 没有解算出结果，直接丢弃
            } else {
                output.pnp_pos = pnp_enu;
                output.los_pos = los_enu;
                raw_target_enu = solve_fused_pose(pnp_enu, los_enu);
            }

            static Throttle throttle{4};
            auto pos_enu = ctx_.pos_enu.load();

            if (output.is_valid) {
                output.fused_pos = raw_target_enu;

                // 计算 EKF 时间间隔 (使用真实照片时间戳)
                double dt_ekf = 0.033;
                if (last_ekf_stamp_ > 0) {
                    dt_ekf = (stamp_tracker - last_ekf_stamp_);
                }

                if (has_last_valid_pos_) {
                    // 计算本次视觉结果与上一次有效结果的物理距离
                    double jump_dist = (raw_target_enu.head<2>() -
                                        last_valid_enu_pos_.head<2>())
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
                    kf_xy_->force_set_state(raw_target_enu.x(),
                                            raw_target_enu.y());
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

                double dist_xy = std::hypot(target_body_relative.x(),
                                            target_body_relative.y());

                double current_visual_angle_deg =
                    std::atan2(dist_xy, current_z) * 180.0 / M_PI;

                // 3. 执行 KF 滤波 (由于已经是 ENU，不需要任何 R_wb 转换)
                double current_angular_rate =
                    ctx_.vel_angular_body.load().norm();
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
                    output.target_vel_enu << v_xy_enu.x(), v_xy_enu.y(),
                        yaw_rate;
                } else {
                    output.target_vel_enu.setZero();
                    output.target_vel_enu.z() = yaw_rate;
                }
                // auto prob = kf_xy_->get_prob();
                if (throttle.shouldLog()) {
                    spdlog::info(
                        "ekf_x={:.2f} ekf_y={:.2f} drone_x={:.2f} "
                        "drone_y={:.2f} "
                        "drone_roll={:.2f} drone_pitch={:.2f} "
                        "drone_yaw={:.2f} epsilon={:.2f}",
                        raw_target_enu.x(), raw_target_enu.y(), pos_enu.x(),
                        pos_enu.y(), ctx_.roll.load(), ctx_.pitch.load(),
                        ctx_.yaw_enu.load(), epsilon);
                }
            }
        }
        return output;
    }

    void set_target_id(int target_id) { target_id_ = target_id; }

    void on_start() { reset_estimator(); }

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
