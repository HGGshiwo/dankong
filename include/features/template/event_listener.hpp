#pragma once
// 注意类的声明中坚决不要使用State
// 否则出现 feature -> listener -> state -> state_common -> context -> feature

#include "./events.hpp"
#include "dk/engine.hpp"
#include "dk/event_listener.hpp"
#include "robot_context.hpp"

class TemplateEventListener
    : public dk::BaseEventListener<RobotContext, TemplateEventListener> {
   public:
    using AllowedEvents = std::tuple<TemplateEvent>;

    TemplateEventListener() {}

    void on_tick(double dt, RobotContext& ctx) {}
};