#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct DroneConfig {
    static constexpr const char* __group_name = "Drone";

    dk::Param<double> throttle_thresh =
        INIT_PARAM("throttle_thresh", 0.1, "判定起飞/降落状态的油门阈值");

    // rsos 云台服务桥: MAVLink 机型 (MavDrone) 上由 dankong 提供云台服务,
    // 串口云台机型保持 false, service 由各自的云台驱动节点提供
    dk::Param<bool> gimbal_bridge_enable =
        INIT_PARAM("gimbal_bridge_enable", false,
                   "是否由 dankong 提供 rsos 云台服务(MAVLink 云台)");
    dk::Param<std::string> gimbal_service_topic = INIT_PARAM(
        "gimbal_service_topic", "/UAV0/sensor/serial_gimbal/set_gimbal_angle",
        "rsos 云台服务名称");
    // 云台 pitch 可控范围 (DJI 约定: 负值向下, 单位度)。
    // body 闭环的目标世界角 = 飞机 pitch - rsos角度, 机体抬头时会越过
    // 下俯极限, 需按机型 clamp (Mini4Pro: -90~+35, 按实际机型配置)
    dk::Param<double> gimbal_pitch_min = INIT_PARAM(
        "gimbal_pitch_min", -90.0, "云台 pitch 可控下限(deg, DJI约定)");
    dk::Param<double> gimbal_pitch_max = INIT_PARAM(
        "gimbal_pitch_max", 35.0, "云台 pitch 可控上限(deg, DJI约定)");
};