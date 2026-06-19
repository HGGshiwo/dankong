#pragma once
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "./context.hpp"
#include "./event_listener.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "features/dog/command.hpp"
#include "robot/dog.hpp"
#include "robot_context.hpp"

const std::unordered_map<uint8_t, std::string> state_map = {
    {0, "趴下状态"},        {1, "正在起立状态"}, {2, "初始站立状态"},
    {3, "力控站立状态"},    {4, "踏步状态"},     {5, "正在趴下状态"},
    {6, "软急停/摔倒状态"}, {0x10, "L模式"}};

class DogFeature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        auto mavlink = std::make_shared<MavsdkDrone>(
            ctx.engine->get_context().mavsdk_system);
        auto& cfg = GlobalConfig.GetConfig();
        ctx.robot =
            std::make_shared<Dog>(mavlink, cfg.udp_host, cfg.udp_port, ctx);
    }

    static void setup(
        TagUdp,
        std::shared_ptr<dk::UdpAdapter<RobotContext, Engine, CommandType>>&
            udp) {
        udp->bind_context(
            CommandType::BATTERY_LEVEL_REPORT,
            [](const std::vector<uint8_t>& data, RobotContext& ctx) {
                UdpPacketView msg(data);
                ctx.battery_remaining.store(msg.get_battery_level());
            });

        udp->bind_context(
            CommandType::MOTION_STATE_REPORT,
            [](const std::vector<uint8_t>& data, RobotContext& ctx) {
                UdpPacketView msg(data);
                auto payload = msg.get_payload<MotionStateData>();
                ctx.gait_state = payload->gait_state;

                auto it = state_map.find(payload->basic_state);
                if (it == state_map.end()) {
                    ctx.dog_state_name.store("未知状态");
                    ctx.dog_state = -1;
                } else {
                    ctx.dog_state_name.store(it->second);
                    ctx.dog_state = it->first;
                }
            });
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        engine->add_listener(
            std::make_shared<DogListener>(engine->get_context()));
    }
};