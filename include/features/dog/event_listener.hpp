#pragma once
#include <memory>

#include "./command.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/engine.hpp"
#include "dk/event_listener.hpp"
#include "robot_context.hpp"
#include "utils/state_registry.hpp"

class DogListener : public dk::BaseEventListener<RobotContext, DogListener> {
   public:
    using AllowedEvents = std::tuple<>;

    DogListener(RobotContext& ctx) : udp_client_(ctx.udp_client) {}

    void on_tick(double dt, RobotContext& ctx) {
        if (is_hz(3)) {
            auto data = pack_cmd(CommandType::MANUAL_HEARTBEAT);
            udp_client_->send(data);
        }
    }

   private:
    std::shared_ptr<UdpClient> udp_client_;
};