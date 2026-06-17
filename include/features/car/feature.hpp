#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/Range.h>
#include <spdlog/spdlog.h>

#include <string>

#include "./context.hpp"
#include "core/engine.hpp"
#include "core/event_result.hpp"
#include "core/tag.hpp"
#include "dk/RosTimeProvider.hpp"
#include "dk/adapters/can/can_adapter.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "event_listener.hpp"
#include "events.hpp"
#include "ipc_vcu_zrd.h"
#include "robot/car.hpp"
#include "robot_context.hpp"
#include "utils/fixed_string64.hpp"

FixedString64 gear_to_str(int gear) {
    switch (gear) {
        case 0:
            return "P";
        case 1:
            return "R";
        case 2:
            return "N";
        case 3:
            return "D";
    }
    return std::string("UNKNOWN") + std::to_string(gear);
}

FixedString64 drive_mode_to_str(int drive_mode) {
    switch (drive_mode) {
        case 0:
            return "手动控制";
        case 1:
            return "程序控制";
        case 2:
            return "遥控器控制";
    }
    return std::string("UNKNOWN") + std::to_string(drive_mode);
}

FixedString64 car_state_to_str(int car_state) {
    switch (car_state) {
        case 0:
            return "未知";
        case 1:
            return "就绪";
        case 2:
            return "关机";
    }
    return std::string("UNKNOWN") + std::to_string(car_state);
}

struct CarFeature {
    static void setup(TagInit, RobotContext& ctx) {
        ctx.robot =
            std::make_shared<Car>(std::make_shared<MavRos>(), ctx.can_client,
                                  ctx.engine->get_time_provider());
    }

    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        web->template register_route<LightEvent, EventResult>(
            boost::beast::http::verb::post, "/light/set");
        web->template register_route<HornEvent, EventResult>(
            boost::beast::http::verb::post, "/horn/set");
    }

    static void setup(
        TagCan, std::shared_ptr<dk::CanAdapter<RobotContext, Engine>>& can) {
        can->bind_context(
            0x201, [](const std::vector<uint8_t>& data, auto& ctx) -> void {
                struct ipc_vcu_zrd_ipc_201_t unpacked_data;
                if (data.size() != 8) {
                    spdlog::error("Error 201 data size: {}", data.size());
                    return;
                }
                ipc_vcu_zrd_ipc_201_unpack(&unpacked_data, data.data(),
                                           data.size());

                ctx.gear.store(unpacked_data.gear);
                ctx.gear_str.store(gear_to_str(unpacked_data.gear));
                ctx.speed.store(unpacked_data.car_speed * 0.1);
                ctx.angle.store(unpacked_data.angle);
                ctx.drive_mode.store(
                    drive_mode_to_str(unpacked_data.drive_mode));
                ctx.estop_status.store(unpacked_data.epo_sts != 0);
            });

        can->bind_context(
            0x203, [](const std::vector<uint8_t>& data, auto& ctx) -> void {
                struct ipc_vcu_zrd_ipc_203_t unpacked_data;
                if (data.size() != 8) {
                    spdlog::error("Error 203 data size: {}", data.size());
                    return;
                }
                ipc_vcu_zrd_ipc_203_unpack(&unpacked_data, data.data(),
                                           data.size());

                ctx.battery_level.store((double)(unpacked_data.soc) / 255.0 *
                                        100);
            });

        can->bind_context(
            0x260, [](const std::vector<uint8_t>& data, auto& ctx) -> void {
                struct ipc_vcu_zrd_ipc_260_t unpacked_data;
                if (data.size() != 8) {
                    spdlog::error("Error 260 data size: {}", data.size());
                    return;
                }
                ipc_vcu_zrd_ipc_260_unpack(&unpacked_data, data.data(),
                                           data.size());

                ctx.car_state.store(
                    car_state_to_str(unpacked_data.car_start_state));
            });
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        engine->add_listener(
            std::make_shared<CarListener>(engine->get_context()));
    }
};