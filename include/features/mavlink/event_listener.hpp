#pragma once

#include "dk/event_listener.hpp"
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "utils/state_registry.hpp"

class MavlinkEventListener
    : public dk::BaseEventListener<RobotContext, MavlinkEventListener> {
   public:
    using AllowedEvents =
        std::tuple<dk::TickEvent, StatusTextEvent, FcuConnectedEvent>;

    void on_event(const dk::TickEvent& event, RobotContext& ctx);
    void on_event(const StatusTextEvent& event, RobotContext& ctx);
    void on_event(const FcuConnectedEvent& event, RobotContext& ctx);

   private:
    RateLimiter rate_{1.0};
};
