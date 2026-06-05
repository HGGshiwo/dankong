#pragma once

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tagCustom48h12.h"
}

#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>

// ==========================================
// 1. 姿态资源的 RAII 包装器 (解决野指针和内存泄漏)
// ==========================================
struct SafeTagPose {
    apriltag_pose_t pose;
    double error;

    // 默认构造：强制零初始化，彻底干掉野指针！
    SafeTagPose() {
        pose = {nullptr, nullptr};
        error = 0.0;
    }

    ~SafeTagPose() {
        if (pose.R != nullptr) matd_destroy(pose.R);
        if (pose.t != nullptr) matd_destroy(pose.t);
    }

    // 禁用拷贝，防止重复释放
    SafeTagPose(const SafeTagPose&) = delete;
    SafeTagPose& operator=(const SafeTagPose&) = delete;

    // 允许移动
    SafeTagPose(SafeTagPose&& other) noexcept {
        pose = other.pose;
        error = other.error;
        other.pose = {nullptr, nullptr};
    }

    Eigen::Matrix3d getRotation() const {
        Eigen::Matrix3d R;
        if (pose.R) {
            R << pose.R->data[0], pose.R->data[1], pose.R->data[2],
                pose.R->data[3], pose.R->data[4], pose.R->data[5],
                pose.R->data[6], pose.R->data[7], pose.R->data[8];
        } else {
            R.setIdentity();
        }
        return R;
    }

    bool isValid() const { return pose.R != nullptr; }
};

// ==========================================
// 2. 检测结果数组的 RAII 包装器
// ==========================================
class SafeDetections {
   private:
    zarray_t* detections_ = nullptr;

   public:
    explicit SafeDetections(zarray_t* d) : detections_(d) {}

    ~SafeDetections() {
        if (detections_ != nullptr) {
            apriltag_detections_destroy(detections_);
        }
    }

    SafeDetections(const SafeDetections&) = delete;
    SafeDetections& operator=(const SafeDetections&) = delete;

    SafeDetections(SafeDetections&& other) noexcept
        : detections_(other.detections_) {
        other.detections_ = nullptr;
    }

    int size() const { return detections_ ? zarray_size(detections_) : 0; }

    // 重载 [] 操作符，像用 std::vector 一样用它
    apriltag_detection_t* operator[](int i) const {
        apriltag_detection_t* det = nullptr;
        zarray_get(detections_, i, &det);
        return det;
    }
};

// ==========================================
// 3. 安全检测器核心类
// ==========================================
class SafeAprilTagDetector {
   private:
    apriltag_family_t* tf_ = nullptr;
    apriltag_detector_t* td_ = nullptr;

   public:
    SafeAprilTagDetector(int threads = 4) {
        tf_ = tagCustom48h12_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);
        td_->nthreads = threads;
    }

    ~SafeAprilTagDetector() {
        if (td_ != nullptr) apriltag_detector_destroy(td_);
        if (tf_ != nullptr) tagCustom48h12_destroy(tf_);
    }

    // 第一步：只管检测，不管姿态。返回的 RAII 对象会在离开作用域时自动销毁
    SafeDetections detect(const cv::Mat& gray_img) {
        if (gray_img.empty() || gray_img.type() != CV_8UC1) {
            return SafeDetections(nullptr);
        }
        image_u8_t im = {.width = gray_img.cols,
                         .height = gray_img.rows,
                         .stride = static_cast<int32_t>(gray_img.step[0]),
                         .buf = gray_img.data};
        return SafeDetections(apriltag_detector_detect(td_, &im));
    }

    // 第二步：只针对你挑出来的那一个 Tag 算姿态！
    void estimatePose(apriltag_detection_t* det, double tag_size, double fx,
                      double fy, double cx, double cy, SafeTagPose& out_pose1,
                      SafeTagPose& out_pose2) {
        apriltag_detection_info_t info = {.det = det,
                                          .tagsize = tag_size,
                                          .fx = fx,
                                          .fy = fy,
                                          .cx = cx,
                                          .cy = cy};

        estimate_tag_pose_orthogonal_iteration(
            &info, &out_pose1.error, &out_pose1.pose, &out_pose2.error,
            &out_pose2.pose, 50);
    }
};
