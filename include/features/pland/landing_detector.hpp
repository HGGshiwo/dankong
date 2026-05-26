#pragma once
#include <chrono>
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/tagCustom48h12.h"
}
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "./config.hpp"
#include "./kalman_filter_2d.hpp"
#include "./kalman_filter_yaw.hpp"
#include "utils/dirty_var.hpp"
#include "utils/thread_runner.hpp"
#include "utils/time_tracker.hpp"

struct DetectorResult {
    bool is_valid = false;
    Eigen::Vector4d relative_pos =
        Eigen::Vector4d::Zero();  // [X(前), Y(左), Z(高度差), Yaw偏差]
    Eigen::Vector3d velocity =
        Eigen::Vector3d::Zero();  // [Vx, Vy, Yaw_Rate] (机体系)

    std::optional<Eigen::Vector3d> pnp_pos;
    std::optional<Eigen::Vector3d> los_pos;
    std::optional<Eigen::Vector3d> fused_pos;
};

class LandingDetector : public IThreadRunner {
   private:
    PlandConfig &config_;
    apriltag_detector_t *td_;
    apriltag_family_t *tf_;

    DirtyVar<Eigen::Quaternion<double>> &drone_orientation_;
    std::atomic<double> &alt_;

    DirtyVar<cv::Mat> &image_;
    DirtyVar<TimeTracker> &stamp_;

    std::shared_ptr<KalmanFilter2D> kf_xy_;
    std::shared_ptr<KalmanFilterYaw> kf_yaw_;

    // --- 时间戳与缓存 ---
    std::function<void(const DetectorResult &, cv::Mat &)> set_target_;
    int target_id_ = -1;

    Eigen::Matrix3d camera_inner_matrix_;

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
                               0.0);
    }

    // 直接进行检测
    // 返回值: Vector3d(相对于机身的 X, 相对于机身的 Y, Yaw = 0.0)
    Eigen::Vector3d solve_pose_by_ray(double img_x, double img_y) {
        Eigen::Matrix3d R_bc = get_camera_to_body_rotation();
        Eigen::Vector3d t_bc = get_camera_body_offset();

        // 1. 相机光学系下的射线方向
        Eigen::Vector3d cam_ray =
            camera_inner_matrix_.inverse() * Eigen::Vector3d(img_x, img_y, 1.0);

        // 2. 机体系下的射线方向
        Eigen::Vector3d body_ray = R_bc * cam_ray;

        // 3. 转到世界系下求地面交点 (考虑无人机的 Pitch 和 Roll)
        Eigen::Vector3d drone_pos{0.0, 0.0, alt_.load()};
        Eigen::Matrix3d R_wb = drone_orientation_.load().toRotationMatrix();
        Eigen::Vector3d world_ray = R_wb * body_ray;
        Eigen::Vector3d camera_pos_world = drone_pos + R_wb * t_bc;

        // 4. 求与 Z=0 平面的交点
        if (world_ray.z() > -1e-6) {
            // 射线没指向地面，返回 NaN 代表无效
            return Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN());
        }
        double t = -camera_pos_world.z() / world_ray.z();
        Eigen::Vector3d ground_point_world = camera_pos_world + t * world_ray;

        // 5. 关键步：将世界系的地面点，反向转换回相对于无人机重心的机体坐标系
        // 公式: P_body_relative = R_wb^T * (P_world - drone_pos)
        Eigen::Vector3d target_body_relative =
            R_wb.transpose() * (ground_point_world - drone_pos);

        // 射线法没有目标偏航角信息，Yaw 填 0.0
        return Eigen::Vector3d(target_body_relative.x(),
                               target_body_relative.y(), 0.0);
    }

    // 输入: apriltag 算出来的位姿
    // 返回值: Vector3d(相对于机身的 X, 相对于机身的 Y, 相对于机身的 Yaw)
    Eigen::Vector3d solve_pose_by_pnp(const apriltag_pose_t &pose) {
        Eigen::Matrix3d R_bc = get_camera_to_body_rotation();
        Eigen::Vector3d t_bc = get_camera_body_offset();

        // 1. 提取 Tag 在相机光学系 (C) 下的位姿
        Eigen::Vector3d t_tag_cam(pose.t->data[0], pose.t->data[1],
                                  pose.t->data[2]);
        Eigen::Matrix3d R_tag_cam;
        R_tag_cam << pose.R->data[0], pose.R->data[1], pose.R->data[2],
            pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.R->data[6],
            pose.R->data[7], pose.R->data[8];

        // 2. 转换到无人机机体坐标系 (B)
        // 相对位置 = 相机偏移 + (机身视角看过去的相对坐标)
        Eigen::Vector3d P_tag_body = t_bc + R_bc * t_tag_cam;

        // 相对姿态 = 相机旋转 * 目标旋转
        Eigen::Matrix3d R_tag_body = R_bc * R_tag_cam;

        // 3. 提取偏航角 Yaw (Z-Y-X 欧拉角中的 Z 旋转)
        double yaw = std::atan2(R_tag_body(1, 0), R_tag_body(0, 0));

        // 组装返回结果: [相对于机身重心前方的X, 左方的Y, 偏航角Yaw]
        return Eigen::Vector3d(P_tag_body.x(), P_tag_body.y(), yaw);
    }

    // 传入 PnP 需要的 pose 结构体，以及 LOS 需要的像素坐标
    Eigen::Vector3d solve_fused_pose(Eigen::Vector3d pnp_result,
                                     Eigen::Vector3d los_result) {
        // 3. 基于当前高度计算权重
        // 这里的 alt_ 假设是无人机相对地面的绝对高度 (正数)
        double current_z = std::abs(alt_.load());

        // ---- 【关键配置参数】 ----
        double z_high = 3.0;  // 高于 3 米，完全信任 LOS 射线法
        double z_low = 1.0;   // 低于 1 米，完全信任 PnP 视觉解算
        // ------------------------

        // 4. 计算 PnP 的权重 (w_pnp 从 0.0 平滑过渡到 1.0)
        double w_pnp = 0.0;
        // if (current_z >= z_high) {
        //     w_pnp = 0.0;  // 高空纯 LOS
        // } else if (current_z <= z_low) {
        //     w_pnp = 1.0;  // 低空纯 PnP
        // } else {
        //     // 过渡区：线性插值
        //     w_pnp = (z_high - current_z) / (z_high - z_low);
        // }
        double w_los = 1.0 - w_pnp;

        // 5. 执行融合操作
        Eigen::Vector3d fused_result;

        // X 和 Y 进行加权平均平滑过渡
        fused_result.x() = (w_pnp * pnp_result.x()) + (w_los * los_result.x());
        fused_result.y() = (w_pnp * pnp_result.y()) + (w_los * los_result.y());

        // Yaw 的特殊处理：LOS 法没有 Yaw (默认0)，所以权重乘以 PnP 的 Yaw，
        // 这在物理上的表现是：无人机在高空不修正 Yaw，随着高度下降，Yaw
        // 修正指令平滑地越来越大
        fused_result.z() = w_pnp * pnp_result.z();
        return fused_result;
    }

   public:
    LandingDetector(
        PlandConfig &config,
        DirtyVar<Eigen::Quaternion<double>> &drone_orientation,
        std::atomic<double> &alt, DirtyVar<cv::Mat> &image,
        DirtyVar<TimeTracker> &stamp,
        std::function<void(const DetectorResult &, cv::Mat &)> set_target)
        : IThreadRunner(),
          config_(config),
          set_target_(set_target),
          drone_orientation_(drone_orientation),
          alt_(alt),
          image_(image),
          stamp_(stamp) {
        td_ = apriltag_detector_create();
        tf_ = tagCustom48h12_create();
        apriltag_detector_add_family(td_, tf_);

        kf_xy_ = std::make_shared<KalmanFilter2D>();
        kf_yaw_ = std::make_shared<KalmanFilterYaw>();

        camera_inner_matrix_ = config_.camera_inner_matrix.get();
    }

    ~LandingDetector() {
        tagCustom48h12_destroy(tf_);
        apriltag_detector_destroy(td_);
    }

    DetectorResult update(double dt, cv::Mat &detected) {
        double elapsed = stamp_.load().elapsed_seconds();
        if (elapsed > 0.1) {
            spdlog::warn("[Pland] image stamp too late {}", elapsed);
            return DetectorResult();
        }
        DetectorResult output;

        // 【修改点 1】: 获取当前帧，保存到 detected
        // 用于可视化输出，并用于灰度转换
        cv::Mat current_img = image_.load();
        current_img.copyTo(detected);

        cv::Mat gray;
        cv::cvtColor(current_img, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.step[0],
                         .buf = gray.data};

        zarray_t *detections = apriltag_detector_detect(td_, &im);

        // 【修复潜在Bug】: 必须初始化为 nullptr，否则没检测到时会导致野指针崩溃
        apriltag_detection_t *inner_result = nullptr;
        double inner_dist = -1;
        apriltag_detection_t *outter_result = nullptr;
        double outter_dist = -1;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            double dx = det->c[0] / ((double)im.width) - 0.5;
            double dy = det->c[1] / ((double)im.height) - 0.5;
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

        double current_z = std::abs(alt_.load());
        if (result != nullptr) {
            // 【修改点 2】: 绘制检测到的 AprilTag 到 detected 图像上
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
            apriltag_detection_info_t info = {
                .det = result,
                .tagsize =
                    is_inner ? config_.inner_tag_size : config_.outter_tag_size,
                .fx = camera_inner_matrix_(0, 0),
                .fy = camera_inner_matrix_(1, 1),
                .cx = camera_inner_matrix_(0, 2),
                .cy = camera_inner_matrix_(1, 2)};

            apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);

            double px = result->c[0];
            double py = result->c[1];
            // 调用高度融合算法
            Eigen::Vector3d pnp_result = solve_pose_by_pnp(pose);
            Eigen::Vector3d los_result = solve_pose_by_ray(px, py);

            Eigen::Vector3d raw_pose;

            // 检查 LOS 结果的有效性 (防止射线指天等异常情况)
            bool los_valid = !std::isnan(los_result.x());
            bool pnp_valid = !std::isnan(pnp_result.x());

            output.is_valid = true;

            // 异常处理：如果由于某种原因只剩一个有效，直接返回有效的那个
            if (!los_valid && pnp_valid) {
                raw_pose = pnp_result;
                output.pnp_pos = raw_pose;
            } else if (los_valid && !pnp_valid) {
                raw_pose = los_result;
                output.los_pos = raw_pose;
            } else if (!los_valid && !pnp_valid) {
                output.is_valid = false;
            } else {
                output.pnp_pos = pnp_result;
                output.los_pos = los_result;
                raw_pose = solve_fused_pose(pnp_result, los_result);
            }
            output.fused_pos = raw_pose;
            // 记得释放内存
            matd_destroy(pose.t);
            matd_destroy(pose.R);

            // 2. 时间差与滤波器更新逻辑

            // 如果 dt 异常（如刚启动或丢失太久），重置状态机
            if (dt <= 0.0 || dt > 1.0) {
                reset_estimator();

                output.relative_pos << raw_pose.x(), raw_pose.y(), current_z,
                    raw_pose.z();
                output.velocity.setZero();  // 刚看到目标，无法求导，速度输出0
            } else {
                // 3. 执行 KF 滤波
                Eigen::Vector2d v_xy =
                    kf_xy_->update(raw_pose.x(), raw_pose.y(), dt);
                Eigen::Vector2d yaw_res = kf_yaw_->update(
                    raw_pose.z(), dt);  // 返回 [滤波后的yaw, yaw_rate]

                // 4. 死区处理（去噪）
                if (v_xy.norm() < config_.velocity_deadzone.get()) {
                    v_xy.setZero();
                }

                // 5. 组装最终结果
                output.relative_pos << raw_pose.x(), raw_pose.y(), current_z,
                    yaw_res.x();
                output.velocity << v_xy.x(), v_xy.y(), yaw_res.y();
            }
        }

        apriltag_detections_destroy(detections);

        return output;
    }

    void set_target_id(int target_id) { target_id_ = target_id; }

    void on_step(double dt) {
        cv::Mat detected;
        auto result = update(dt, detected);
        set_target_(result, detected);
    }

    void reset_estimator() {
        kf_xy_->reset();
        kf_yaw_->reset();
    }
};