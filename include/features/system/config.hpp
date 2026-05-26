#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct SystemConfig {
    static constexpr const char* __group_name = "System";

    dk::Param<std::string> static_dir =
        INIT_PARAM("static_dir", "dist", "Web前端文件所在路径");
    dk::Param<std::string> ui_config =
        INIT_PARAM("ui_config", "config/ui.yaml", "ui.yaml的存储路径");
    dk::Param<std::string> json_path =
        INIT_PARAM("json_path", "config.json", "生成后供前端使用的json路径");

    dk::Param<int> server_port = INIT_HIDDEN_PARAM("server_port", 8000);
    dk::Param<int> udp_server_port = INIT_HIDDEN_PARAM("udp_server_port", 9111);
};