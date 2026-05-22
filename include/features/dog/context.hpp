#pragma once
#include <cstdint>
#include <memory>

#include "./events.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/report.hpp"
#include "mavlink/mavros.hpp"
#include "robot/irobot.hpp"
#include "utils/fixed_string64.hpp"

const std::string TARGET_IP = "127.0.0.1";
const int TARGET_PORT = 9112;

struct DogContext {
    std::shared_ptr<IRobot> robot;
    dk::StateRegistry& reg;

    std::shared_ptr<UdpClient> udp_client =
        std::make_shared<UdpClient>(TARGET_IP, TARGET_PORT);

    dk::ThreadVar<MotionState> motion_state =
        MotionState::WALK;  // 当前的运动状态，CRAWL, WALK, ...

    dk::TrackedVar<FixedString64> dog_state_name{
        reg, "basic_state", 5.0,
        FixedString64("未知状态")};  // 消息上报，dog_state的文字

    std::atomic<uint8_t> dog_state = 0;  // 消息上报, dog状态（趴下、起立、...）
    std::atomic<uint8_t> gait_state = 0;  // 消息上报-步态

   public:
    DogContext(dk::StateRegistry& r) : reg(r) {};
    static constexpr std::string_view ROBOT_TYPE = "DOG";
};