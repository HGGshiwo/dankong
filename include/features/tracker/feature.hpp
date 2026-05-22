#pragma once
#include <memory>

#include "./context.hpp"
#include "context_config.hpp"
#include "core/engine.hpp"
#include "tracker.hpp"

class TrackerFeature {
   public:
    static void init(RobotContext& ctx) {
        TrackingConfig config;
        ctx.tracker = std::make_shared<ThreadedTracker>(
            config, ctx.robot.get(), ctx.pos_enu, ctx.yaw_enu);
        ctx.tracker->start();
    }
};