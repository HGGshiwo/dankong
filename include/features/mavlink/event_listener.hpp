#pragma once

#include "dk/event_listener.hpp"
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "utils/state_registry.hpp"

class MavlinkEventListener
    : public dk::BaseEventListener<RobotContext, MavlinkEventListener> {
   public:
    using AllowedEvents = std::tuple<StatusTextEvent, FcuConnectedEvent>;

    void on_tick(double dt, RobotContext& ctx);
    void on_event(const StatusTextEvent& event, RobotContext& ctx);
    void on_event(const FcuConnectedEvent& event, RobotContext& ctx);
};
