#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct Go2Config {
    static constexpr const char* __group_name = "Go2";

    dk::Param<std::string> go2_server_url =
        INIT_PARAM("go2_server_url", "https://localhost:8444/api/push",
                   "狗控制的后端接口");
};