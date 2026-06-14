#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct CarConfig {
    static constexpr const char* __group_name = "Car";
    dk::Param<double> wheelbase = INIT_PARAM("wheelbase", 0.82, "前后轮的轴距");
    dk::Param<std::string> can_name =
        INIT_PARAM("can_name", "can0", "can总线对应网口名称");
};