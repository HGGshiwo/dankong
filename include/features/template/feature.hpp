#pragma once
#include <memory>

#include "./context.hpp"
#include "./event_listener.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/ros.hpp"
#include "dk/adapters/web/adapter.hpp"

class TemplateFeature {
   public:
    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
        // ...
    }
    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        // ...
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<TemplateEventListener>();
        engine->add_listener(listener);
    }
};