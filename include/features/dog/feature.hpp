#pragma once
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "./context.hpp"
#include "./event_listener.hpp"
#include "core/engine.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "robot/dog.hpp"

const std::unordered_map<uint8_t, std::string> state_map = {
    {0, "趴下状态"},        {1, "正在起立状态"}, {2, "初始站立状态"},
    {3, "力控站立状态"},    {4, "踏步状态"},     {5, "正在趴下状态"},
    {6, "软急停/摔倒状态"}, {0x10, "L模式"}};

const std::string UDP_CLIENT_HOST = "127.0.0.1";
const unsigned int UDP_CLIENT_PORT = 9112;

class DogFeature {
   public:
    static void init(RobotContext& ctx) {
        auto mavlink = std::make_shared<MavRos>();
        ctx.robot = std::make_shared<Dog>(mavlink, UDP_CLIENT_HOST,
                                          UDP_CLIENT_PORT, ctx);
    }

    template <typename UdpAdapter>
    static void register_udp(std::shared_ptr<UdpAdapter>& udp) {
        udp->bind_context(
            CommandType::BATTERY_LEVEL_REPORT,
            [](const std::vector<uint8_t>& data, RobotContext& ctx) {
                UdpPacketView msg(data);
                ctx.battery_remaining.set(msg.get_battery_level());
            });

        udp->bind_context(
            CommandType::MOTION_STATE_REPORT,
            [](const std::vector<uint8_t>& data, RobotContext& ctx) {
                UdpPacketView msg(data);
                auto payload = msg.get_payload<MotionStateData>();
                ctx.gait_state = payload->gait_state;

                auto it = state_map.find(payload->basic_state);
                if (it == state_map.end()) {
                    ctx.dog_state_name.set("未知状态");
                    ctx.dog_state = -1;
                } else {
                    ctx.dog_state_name.set(it->second);
                    ctx.dog_state = it->first;
                }
            });
    }

    static void register_listeners(std::shared_ptr<Engine>& engine) {
        engine->add_listener(
            std::make_shared<DogListener>(engine->get_context()));
    }
};