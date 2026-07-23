#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct ReportConfig {
    static constexpr const char* __group_name = "Report";

    dk::Param<double> report_hz =
        INIT_PARAM("report_hz", 10.0, "Websocket状态广播的频率(Hz)");
    dk::Param<double> report_mission_hz =
        INIT_PARAM("report_mission_hz", 1.0, "任务进度上报的频率(Hz)");
    dk::Param<double> heartbeat_hz =
        INIT_PARAM("heartbeat_hz", 0.5, "Websocket心跳包发送频率(Hz)");
    dk::Param<double> report_pos_threshold =
        INIT_PARAM("report_pos_threshold", 0.05, "位置上报变化阈值(m)");
    dk::Param<double> report_alt_threshold =
        INIT_PARAM("report_alt_threshold", 0.05, "高度上报变化阈值(m)");
    dk::Param<double> report_ang_threshold =
        INIT_PARAM("report_ang_threshold", 1.0, "姿态角变化上报阈值(度)");
    dk::Param<double> report_vel_threshold =
        INIT_PARAM("report_vel_threshold", 0.1, "速度上报变化阈值(m/s)");
};