#pragma once
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <optional>

#include "./landing_controller.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "features/algo/events.hpp"
#include "features/tracker/tracker.hpp"
#include "landing_detector.hpp"
#include "nav_msgs/Odometry.h"
#include "robot_context.hpp"
#include "ros/publisher.h"
#include "spdlog/spdlog.h"

class PlandFeature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        ros::NodeHandle nh;
        ros::Publisher image_pub_ = nh.advertise<sensor_msgs::Image>(
            GlobalConfig.GetConfig().pland_detect_topic.get(), 10);

        ctx.land_controller = std::make_shared<PlandController>(ctx);

        ctx.land_detector = std::make_shared<LandingDetector>(
            GlobalConfig.GetConfig(), ctx,
            [&ctx, image_pub_](const DetectorResult& result, cv::Mat& detect) {
                sensor_msgs::ImagePtr msg =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8", detect)
                        .toImageMsg();
                image_pub_.publish(msg);

                // 只更新观测状态，不执行控制！
                ctx.land_controller->update_observation(result);
            });
    }

    template <typename RosAdapter>
    static void setup(TagRos, std::shared_ptr<RosAdapter>& ros) {
        ros->bind_context(
            GlobalConfig.GetConfig().pland_image_topic,
            [](const sensor_msgs::ImageConstPtr& msg, RobotContext& ctx) {
                try {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                        msg, sensor_msgs::image_encodings::BGR8);

                    ctx.pland_image.store(cv_ptr->image);
                    ctx.pland_image_stamp.emplace(msg->header.stamp);
                } catch (cv_bridge::Exception& e) {
                    spdlog::error("[Pland] convert image error: {}", e.what());
                }
            });
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        // start/stop 在 land_state.cpp 中已经处理了 detector，
        // 这里我们可以根据需要添加更多的监听。
    }
};
