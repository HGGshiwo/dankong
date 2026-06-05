#pragma once
#include <memory>

#include "./context.hpp"
#include "./event_listener.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"

class TemplateFeature {
   public:
    template <typename RosAdapter>
    static void setup(TagRos, std::shared_ptr<RosAdapter>& ros) {
        // ...
    }
    template <typename WebAdapter>
    static void setup(TagWeb, std::shared_ptr<WebAdapter>& web) {
        // ...
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<TemplateEventListener>();
        engine->add_listener(listener);
    }
};