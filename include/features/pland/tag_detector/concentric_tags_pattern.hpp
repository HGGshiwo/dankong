#pragma once
#include <opencv2/opencv.hpp>

#include "./safe_aprialtag.hpp"
#include "target_observation.hpp"

class ConcentricTagsPattern : public ITargetPattern {
   private:
    int target_id_;
    double inner_tag_size_;
    double outter_tag_size_;

   public:
    ConcentricTagsPattern(int target_id, double inner_size, double outter_size)
        : target_id_(target_id),
          inner_tag_size_(inner_size),
          outter_tag_size_(outter_size) {}

    TargetObservation process(const SafeDetections& detections,
                              SafeAprilTagDetector* detector,
                              const Eigen::Matrix3d& cam_K,
                              cv::Mat& draw_img) override {
        TargetObservation obs;

        apriltag_detection_t* inner_result = nullptr;
        double inner_dist = -1;
        apriltag_detection_t* outter_result = nullptr;
        double outter_dist = -1;

        // 1. 寻找内外圈 Tag
        for (int i = 0; i < detections.size(); i++) {
            apriltag_detection_t* det = detections[i];
            double dx = det->c[0] / ((double)draw_img.cols) - 0.5;
            double dy = det->c[1] / ((double)draw_img.rows) - 0.5;
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

        apriltag_detection_t* result = nullptr;
        bool is_inner = false;

        // 优先使用内圈
        if (inner_dist >= 0) {
            result = inner_result;
            is_inner = true;
        } else if (outter_dist >= 0) {
            result = outter_result;
            is_inner = false;
        }

        if (result == nullptr) {
            return obs;  // 未检测到，直接返回无效的 obs
        }

        // 2. 绘制可视化元素
        for (int i = 0; i < 4; i++) {
            cv::Point pt1(result->p[i][0], result->p[i][1]);
            cv::Point pt2(result->p[(i + 1) % 4][0], result->p[(i + 1) % 4][1]);
            cv::line(draw_img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(draw_img, cv::Point(result->c[0], result->c[1]), 4,
                   cv::Scalar(0, 0, 255), -1);
        std::string tag_label = "ID: " + std::to_string(result->id);
        cv::putText(draw_img, tag_label,
                    cv::Point(result->c[0] + 10, result->c[1] - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

        // 3. 计算位姿并封装输出
        double tag_size = is_inner ? inner_tag_size_ : outter_tag_size_;

        detector->estimatePose(result, tag_size, cam_K(0, 0), cam_K(1, 1),
                               cam_K(0, 2), cam_K(1, 2), obs.pose1, obs.pose2);

        if (obs.pose1.valid) {
            obs.is_valid = true;
            obs.center_pixel = cv::Point2d(result->c[0], result->c[1]);
        }

        return obs;
    }
};