#pragma once
#include <atomic>
#include <memory>
#include <string_view>

#include "dk/adapters/can/can_client.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "mavlink/mavros.hpp"
#include "robot_context.hpp"
#include "utils/dirty_var.hpp"
#include "utils/fixed_string64.hpp"

struct CarContext {
    std::shared_ptr<IRobot> robot;

    std::shared_ptr<CanClient> can_client =
        std::make_shared<CanClient>(GlobalConfig.GetConfig().can_name);

    DirtyVar<int> gear{0};  // 挡位 (0:P, 1:R, 2:N, 3:D)
    DirtyVar<FixedString64> gear_str{"UNKNOWN"};

    DirtyVar<double> speed{0.0};                    // 车速 km/h
    DirtyVar<double> angle{0.0};                    // 转向角度 °
    DirtyVar<FixedString64> drive_mode{"UNKNOWN"};  // 驾驶模式
    DirtyVar<bool> estop_status{false};  // 急停状态反馈 (0:未触发)
    DirtyVar<FixedString64> car_state{"UNKNOW"};  // 车辆启动/关机

    explicit CarContext(StateRegistry& reg) {
        reg.bind("gear", gear_str, 2);
        reg.bind("speed", speed, 2);
        reg.bind("angle", angle, 2);
        reg.bind("drive_mode", drive_mode, 2);
        reg.bind("estop", estop_status, 2);
        reg.bind("car_state", car_state, 2);
    }

    static constexpr std::string_view ROBOT_TYPE = "CAR";
};