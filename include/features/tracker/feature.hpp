#pragma once
#include <memory>

#include "./context.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "robot_context.hpp"
#include "tracker.hpp"

class TrackerFeature {
   public:
    static void init(RobotContext& ctx) {
        auto& config = GlobalConfig.GetConfig();
        ctx.tracker = std::make_shared<ThreadedTracker>(
            config, ctx.robot.get(), ctx.pos_enu, ctx.yaw_enu);
        ctx.tracker->start(config.loop_rate_hz.get());
    }
};