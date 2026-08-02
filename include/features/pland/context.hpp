#pragma once
#ifdef USE_ROS
#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>

#include "dk/adapters/udp/udp_client.hpp"
#include "ilanding_controller.hpp"
#include "ilanding_detector.hpp"
#include "mavlink/mavsdk_drone.hpp"
#include "utils/dirty_var.hpp"
#include "utils/state_registry.hpp"

class ILandingController;
class ILandingDetector;

struct PlandContext {
   public:
    DirtyVar<cv::Mat> pland_image;
    DirtyVar<bool> do_pland{false};
    DirtyVar<double> pland_image_stamp{-1.0};
    DirtyVar<std::optional<double>> gimbal_roll{std::nullopt};
    DirtyVar<std::optional<double>> gimbal_pitch{std::nullopt};
    DirtyVar<std::optional<double>> gimbal_yaw{std::nullopt};
    DirtyVar<std::optional<Eigen::Vector3d>> pland_target{
        std::nullopt};  // 用来作弊的精准降落目标

    std::shared_ptr<ILandingDetector> land_detector;
    std::shared_ptr<ILandingController> land_controller;

    explicit PlandContext(StateRegistry& reg) {
        reg.bind("do_pland", do_pland, 2.0);
    }
};
#endif