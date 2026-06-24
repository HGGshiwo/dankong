#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct CarConfig {
    static constexpr const char* __group_name = "Car";
    dk::Param<double> wheelbase = INIT_PARAM("wheelbase", 0.82, "前后轮的轴距");
    dk::Param<double> max_speed_kmh =
        INIT_PARAM("max_speed_kmh", 3.6, "最大速度(Km/h)");
};