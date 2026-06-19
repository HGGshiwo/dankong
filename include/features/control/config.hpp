#pragma once
#include <string>

#include "utils/config_param.hpp"

//@JSON_ENABLE
struct ControlConfig {
    static constexpr const char* __group_name = "Control";

    dk::Param<std::string> mavsdk_url = INIT_PARAM(
        "mavsdk_url", "udp://:14540", "MAVSDK连接地址(如 udp://:14540)");
    dk::Param<int> fcu_data_rate =
        INIT_PARAM("fcu_data_rate", 10, "设置MAVSDK数据上报的频率(Hz)");
    dk::Param<std::string> pdef_path = INIT_PARAM(
        "pdef_path", "config/apm.pdef.xml", "APM参数描述XML文件路径");

    dk::Param<double> follow_timeout =
        INIT_PARAM("follow_timeout", 1.0, "跟随指令失效时间(s)");
    dk::Param<double> prearm_timeout =
        INIT_PARAM("prearm_timeout", 3.0, "等待解锁前检查通过的超时时间(s)");
    dk::Param<double> takeoff_timeout =
        INIT_PARAM("takeoff_timeout", 100.0, "起飞过程最大允许时间(s)");
    dk::Param<double> posvel_timeout =
        INIT_PARAM("posvel_timeout", 3.0, "指点指令失效保护时间(s)");

    dk::Param<double> pos_tolerance =
        INIT_PARAM("pos_tolerance", 1.0, "判定到达目标点的距离偏差(m)");
    dk::Param<double> yaw_tolerance =
        INIT_PARAM("yaw_tolerance", 0.08, "判定到达目标点的角度偏差(rad)");
    dk::Param<double> z_tolerance =
        INIT_PARAM("z_tolerance", 1.0, "判定起飞到达高度的偏差(m)");
    dk::Param<double> fix_yaw_dist =
        INIT_PARAM("fix_yaw_dist", 1.0, "距离目标点小于此值时锁定朝向(m)");
    dk::Param<int> msg_interval_rate =
        INIT_PARAM("msg_interval_rate", 30, "数据上报频率");
};