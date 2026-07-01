#pragma once
#include <string>

#include "utils/config_param.hpp"

//@JSON_ENABLE
struct NtripConfig {
    static constexpr const char* __group_name = "Ntrip";
    dk::Param<int> ntrip_nstats =
        INIT_PARAM("ntrip_nstats", 15, "ntrip开启的最小星数");
    dk::Param<std::string> ntrip_ip = INIT_PARAM("ntrip_ip", "", "ntrip账号ip");
    dk::Param<int> ntrip_port = INIT_PARAM("ntrip_port", 8000, "ntrip账号port");
    dk::Param<std::string> ntrip_mountpoint =
        INIT_PARAM("ntrip_mountpoint", "", "ntrip挂载点");
    dk::Param<std::string> ntrip_username =
        INIT_PARAM("ntrip_username", "", "ntrip用户名");
    dk::Param<std::string> ntrip_password =
        INIT_PARAM("ntrip_password", "", "ntrip密码");
    dk::Param<bool> ntrip_enable =
        INIT_PARAM("ntrip_enable", false, "开启ntrip");
};