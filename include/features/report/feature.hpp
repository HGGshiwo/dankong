#pragma once
#include <memory>

#include "./event_listener.hpp"
#include "core/empty_context.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"

// 数据上报
class ReportFeature {
   public:
    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<ReportEventListener>(
            engine->get_context().ws_manager);
        engine->add_listener(listener);
    }
};