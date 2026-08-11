#pragma once
#include <memory>

#include "./context.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "move_base_tracker.hpp"
#include "robot_context.hpp"
#include "tracker.hpp"

class TrackerFeature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        auto& config = GlobalConfig.GetConfig();
        if (config.tracker_type.get() == "move_base") {
#ifdef USE_ROS1
            ctx.tracker = std::make_shared<MoveBaseTracker>(
                ctx.nh, config, ctx.robot.get(), ctx.pos_enu, ctx.yaw_enu,
                ctx.engine->get_time_provider(), "move_base_simple/goal",
                "move_base/cancel", "map");
#elif defined(USE_ROS2)
            ctx.tracker = std::make_shared<MoveBaseTracker>(
                ctx.node, config, ctx.robot.get(), ctx.pos_enu, ctx.yaw_enu,
                ctx.engine->get_time_provider(), "goal_pose",
                "navigate_to_pose/_action/cancel_goal", "map");
#else
            ctx.tracker = std::make_shared<MoveBaseTracker>(
                config, ctx.robot.get(), ctx.pos_enu, ctx.yaw_enu,
                ctx.engine->get_time_provider(), "map");
#endif
            ctx.tracker->start(config.loop_rate_hz.get());
        } else {
            ctx.tracker = std::make_shared<ThreadedTracker>(
                config, ctx.robot.get(), ctx.pos_enu, ctx.yaw_enu,
                ctx.engine->get_time_provider());
            ctx.tracker->start(config.loop_rate_hz.get());
        }
    }
};