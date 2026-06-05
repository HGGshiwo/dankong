#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/Range.h>

#include "./context.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "robot/car.hpp"
#include "robot_context.hpp"

struct CarFeature {
    static void setup(TagInit, RobotContext& ctx) {
        ctx.robot = std::make_shared<Car>(std::make_shared<MavRos>());
    }
};