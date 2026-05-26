#pragma once
#include <string>

#include "utils/config_param.hpp"

//@JSON_ENABLE
struct DogConfig {
    static constexpr const char* __group_name = "Dog";

    dk::Param<std::string> udp_host =
        INIT_PARAM("udp_host", "127.0.0.1", "机器狗UDP通信地址");
    dk::Param<int> udp_port = INIT_PARAM("udp_port", 9112, "机器狗UDP通信端口");
};