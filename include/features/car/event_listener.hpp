#pragma once
#include <memory>

#include "./events.hpp"
#include "dk/adapters/can/can_client.hpp"
#include "dk/engine.hpp"
#include "dk/event_listener.hpp"
#include "ipc_vcu_zrd.h"
#include "robot/car.hpp"
#include "robot_context.hpp"
#include "utils/state_registry.hpp"

class CarListener : public dk::BaseEventListener<RobotContext, CarListener> {
   public:
    using AllowedEvents = std::tuple<LightEvent, EStopEvent, HornEvent>;

    CarListener(RobotContext& ctx) {}

    void on_event(const LightEvent& event, RobotContext& ctx) {
        auto car = std::static_pointer_cast<Car>(ctx.robot);
        car->set_light(event);
    }

    void on_event(const HornEvent& event, RobotContext& ctx) {
        auto car = std::static_pointer_cast<Car>(ctx.robot);
        car->set_horn(event);
    }

    void on_event(const EStopEvent& event, RobotContext& ctx) {
        auto car = std::static_pointer_cast<Car>(ctx.robot);
        car->set_estop(event.set_estop);
    }
};