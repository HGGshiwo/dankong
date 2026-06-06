#pragma once
#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>

#include "dk/adapters/udp/udp_client.hpp"
#include "dk/report.hpp"
#include "mavlink/mavros.hpp"
#include "utils/state_registry.hpp"

class PlandController;
class LandingDetector;

struct PlandContext {
   public:
    DirtyVar<cv::Mat> pland_image;
    DirtyVar<int> land_target_id{-1};
    DirtyVar<double> pland_image_stamp{-1.0};
    std::shared_ptr<LandingDetector> land_detector;
    std::shared_ptr<PlandController> land_controller;

    explicit PlandContext(StateRegistry& reg) {
        reg.bind("land_target", land_target_id, 2.0);
    }
};