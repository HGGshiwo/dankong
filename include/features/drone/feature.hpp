#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/Range.h>

#include "./context.hpp"
#include "core/engine.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "robot/drone.hpp"
#include "robot_context.hpp"

// 修复 1：使用 struct 使其默认 public，或者在 class 内加上 public:
struct DroneFeature {
    static void init(RobotContext& ctx) {
        // 这里现在会被正确调用了！
        ctx.robot = std::make_shared<Drone>(std::make_shared<MavRos>());
    }

    // 修复 2：函数名改为 register_ros
    // 修复 3：参数签名与 SFINAE (std::shared_ptr<Adapter>&) 保持一致
    template <typename RosAdapterType>
    static void register_ros(std::shared_ptr<RosAdapterType>& ros) {
        ros->bind_context("/mavros/vfr_hud",
                          [](const mavros_msgs::VFR_HUD::ConstPtr& vfr_hud_msg,
                             RobotContext& ctx) -> auto {
                              ctx.throttle = vfr_hud_msg->throttle;
                          });

        ros->bind_context("/mavros/distance_sensor/rangefinder_pub",
                          [](const sensor_msgs::Range::ConstPtr& range_msg,
                             RobotContext& ctx) -> auto {
                              ctx.rangefinder_alt = range_msg->range;
                          });
    }
};