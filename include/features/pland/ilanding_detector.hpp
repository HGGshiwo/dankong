#pragma once
#include <Eigen/Dense>
#include <optional>

struct DetectorResult {
    bool is_valid = false;
    double stamp;  // 图像拍摄的真实时间

    // ENU 导航系下的绝对目标位置和速度 (EKF 融合后)
    Eigen::Vector3d target_pos_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_vel_enu =
        Eigen::Vector3d::Zero();  // .z() is yaw_rate

    double target_yaw_enu = 0.0;
    double yaw_relative = 0.0;
    Eigen::Vector3d target_pos_body = Eigen::Vector3d::Zero();
    double current_z = 0.0;  // 相对高度

    std::optional<Eigen::Vector3d> pnp_pos;
    std::optional<Eigen::Vector3d> los_pos;
    std::optional<Eigen::Vector3d> fused_pos;
};

class ILandingDetector {
   public:
    virtual ~ILandingDetector() = default;
    virtual void start(int hz) = 0;
    virtual void start(int hz, bool estimate) = 0;
    virtual void stop() = 0;
    virtual std::string stop(bool save) = 0;
};
