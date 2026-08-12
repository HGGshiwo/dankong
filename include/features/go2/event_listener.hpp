#pragma once

#include "dk/event_listener.hpp"
#include "robot_context.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "utils/state_registry.hpp"

class Go2EventListener
    : public dk::BaseEventListener<RobotContext, Go2EventListener> {
   public:
    using AllowedEvents = std::tuple<>;

    void on_tick(double dt, RobotContext& ctx) override {}
};
