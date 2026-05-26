#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct ReportConfig {
    static constexpr const char* __group_name = "Report";

    dk::Param<double> report_hz =
        INIT_PARAM("report_hz", 10.0, "Websocket状态广播的频率(Hz)");
    dk::Param<double> report_mission_hz =
        INIT_PARAM("report_mission_hz", 1.0, "任务进度上报的频率(Hz)");
};