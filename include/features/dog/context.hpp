#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <string_view>

#include "./events.hpp"
#include "core/global_config.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/report.hpp"
#include "mavlink/mavros.hpp"
#include "robot/irobot.hpp"
#include "utils/dirty_var.hpp"
#include "utils/state_registry.hpp"

struct DogContext {
    std::shared_ptr<IRobot> robot;

    std::shared_ptr<UdpClient> udp_client = std::make_shared<UdpClient>(
        GlobalConfig.GetConfig().udp_host, GlobalConfig.GetConfig().udp_port);

    // =========================================================================
    // 数据模型定义（纯净数据载体）
    // =========================================================================

    // 替换原来的 dirty<T> 为统一的 DirtyVar<T>
    DirtyVar<MotionState> motion_state{
        MotionState::WALK};  // 当前的运动状态，CRAWL, WALK, ...

    // 替换原来的 TrackedVar 为 DirtyVar
    DirtyVar<FixedString64> dog_state_name{
        FixedString64("未知状态")};  // 消息上报，dog_state的文字

    std::atomic<uint8_t> dog_state{0};  // 消息上报, dog状态（趴下、起立、...）
    std::atomic<uint8_t> gait_state{0};  // 消息上报-步态

   public:
    // =========================================================================
    // 外部上报绑定
    // =========================================================================
    explicit DogContext(StateRegistry& reg) {
        // 在构造时进行绑定映射，结构体自身不再持有 reg 成员
        reg.bind("basic_state", dog_state_name, 5.0);

        // （注：原代码中 dog_state 和 gait_state 是
        // atomic，如果你后续也想让它们自动上报， 可以将它们也改为
        // DirtyVar，并在这里加两行 reg.bind(...) 即可）
    }

    static constexpr std::string_view ROBOT_TYPE = "DOG";
};