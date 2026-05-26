#pragma once
#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>

#include "./landing_detector.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/report.hpp"
#include "mavlink/mavros.hpp"
#include "utils/state_registry.hpp"
#include "utils/time_tracker.hpp"

struct PlandContext {
   public:
    DirtyVar<cv::Mat> pland_image;
    DirtyVar<int> land_target_id{-1};
    DirtyVar<TimeTracker> pland_image_stamp;
    std::shared_ptr<LandingDetector> land_detector;

    explicit PlandContext(StateRegistry& reg) {
        reg.bind("land_target", land_target_id, 2.0);
    }
};