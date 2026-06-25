#pragma once
#include <string>

#include "utils/config_param.hpp"

//@JSON_ENABLE
struct MavlinkConfig {
    static constexpr const char* __group_name = "Mavlink";

    dk::Param<std::string> mavsdk_url = INIT_PARAM(
        "mavsdk_url", "udp://:14540", "MAVSDK连接地址(如 udp://:14540)");
    dk::Param<int> fcu_data_rate =
        INIT_PARAM("fcu_data_rate", 10, "设置MAVSDK数据上报的频率(Hz)");
    dk::Param<std::string> pdef_path = INIT_PARAM(
        "pdef_path", "config/apm.pdef.xml", "APM参数描述XML文件路径");
    dk::Param<int> msg_interval_rate =
        INIT_PARAM("msg_interval_rate", 30, "数据上报频率");
};
