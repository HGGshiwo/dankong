#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct CarConfig {
    static constexpr const char* __group_name = "Car";
    dk::Param<std::string> can_name =
        INIT_PARAM("can_name", "can0", "can总线对应网口名称");
};