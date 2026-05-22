#pragma once
#include <memory>

#include "./event_listener.hpp"
#include "core/empty_context.hpp"
#include "core/engine.hpp"

// 数据上报
class ReportFeature {
   public:
    static void register_listeners(std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<ReportEventListener>(
            engine->get_context().ws_manager);
        engine->add_listener(listener);
    }
};