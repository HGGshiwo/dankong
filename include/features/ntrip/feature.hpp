#pragma once
#include "./event_listener.hpp"
#include "./ntrip_client.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "robot_context.hpp"

class NtripFeature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        ctx.ntrip_client = std::make_shared<NtripClient>(
            ctx.engine->get_time_provider(), ctx.ntrip_running);
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<NtripEventListener>();
        engine->add_listener(listener);
    }
};