#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct DroneConfig {
    static constexpr const char* __group_name = "Drone";

    dk::Param<double> throttle_thresh =
        INIT_PARAM("throttle_thresh", 0.1, "判定起飞/降落状态的油门阈值");
};