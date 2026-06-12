#pragma once
#include <memory>

#include "./events.hpp"
#include "dk/adapters/can/can_client.hpp"
#include "dk/engine.hpp"
#include "dk/event_listener.hpp"
#include "ipc_vcu_zrd.h"
#include "robot_context.hpp"
#include "utils/state_registry.hpp"

class CarListener : public dk::BaseEventListener<RobotContext, CarListener> {
   public:
    using AllowedEvents = std::tuple<LightEvent>;

    CarListener(RobotContext& ctx) : can_client_(ctx.can_client) {}

    void on_event(const LightEvent& event, RobotContext& ctx) {
        struct ipc_vcu_zrd_ipc_210_t cmd;
        memset(&cmd, 0, sizeof(cmd));

        cmd.ipc_en = 1;

        // 灯光喇叭赋值
        cmd.turn_lamp = event.turn;
        cmd.dipped_lamp = event.dipped ? 1 : 0;
        cmd.far_lamp = event.far ? 1 : 0;
        cmd.out_line_lamp = event.outline ? 1 : 0;
        cmd.alarm_lamp = event.alarm ? 1 : 0;  // 双闪
        cmd.horn = event.horn ? 1 : 0;

        uint8_t buffer[8] = {0};  // CAN 报文固定 8 字节
        ipc_vcu_zrd_ipc_210_pack(buffer, &cmd, sizeof(buffer));
        can_client_->send_frame(0x210, buffer, 8);
    }

   private:
    std::shared_ptr<CanClient> can_client_;
};