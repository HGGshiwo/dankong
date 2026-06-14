#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/Range.h>
#include <spdlog/spdlog.h>

#include "./context.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/can/can_adapter.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "events.hpp"
#include "ipc_vcu_zrd.h"
#include "robot/car.hpp"
#include "robot_context.hpp"

struct CarFeature {
    static void setup(TagInit, RobotContext& ctx) {
        ctx.robot =
            std::make_shared<Car>(std::make_shared<MavRos>(), ctx.can_client);
    }

    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        web->template register_route<LightEvent, EventResult>(
            boost::beast::http::verb::post, "/light/set");
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
                std::string gear_str = "UNKNOWN";
                switch (unpacked_data.gear) {
                    case 0:
                        gear_str = "P";
                        break;
                    case 1:
                        gear_str = "R";
                        break;
                    case 2:
                        gear_str = "N";
                        break;
                    case 3:
                        gear_str = "D";
                        break;
                }
                ctx.gear_str.store(gear_str);
                ctx.speed.store(unpacked_data.car_speed * 0.1);
                ctx.angle.store(unpacked_data.angle * 0.1);
                ctx.drive_mode.store(unpacked_data.drive_mode);
                ctx.estop_status.store(unpacked_data.epo_sts);
            });
    }
};