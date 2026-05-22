#pragma once
#include <memory>

#include "./context.hpp"
#include "./event_listener.hpp"
#include "core/engine.hpp"

class TemplateFeature {
   public:
    // 1. 声明属于自己的 Context（如果没有，就写 EmptyContext）
    using Context = TemplateContext;

    template <typename RosAdapter>
    static void register_ros(std::shared_ptr<RosAdapter>& ros) {
        // ...
    }
    template <typename WebAdapter>
    static void register_web(std::shared_ptr<WebAdapter>& web) {
        // ...
    }

    static void register_listeners(std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<TemplateEventListener>();
        engine->add_listener(listener);
    }
};