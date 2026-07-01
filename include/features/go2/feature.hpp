#pragma once
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "./event_listener.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "robot/go2.hpp"
#include "robot_context.hpp"

class Go2Feature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        auto mavlink = std::make_shared<MavsdkDrone>(
            ctx.engine->get_context().mavsdk_system);
        auto& cfg = GlobalConfig.GetConfig();
        ctx.robot = std::make_shared<Go2>(ctx);
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        engine->add_listener(
            std::make_shared<Go2EventListener>(engine->get_context()));
    }
};