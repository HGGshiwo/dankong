#pragma once
#include "utils/config_param.hpp"

//@JSON_ENABLE
struct TrackerConfig {
    static constexpr const char* __group_name = "Tracker";

    dk::Param<std::string> tracker_type =
        INIT_PARAM("tracker_type", std::string("threaded"),
                   "追踪器类型: threaded (内部PID) 或 move_base (ROS导航)");
    dk::Param<bool> is_omnidirectional =
        INIT_PARAM("is_omnidirectional", true, "是否开启全向移动模式");
    dk::Param<double> kp_xy = INIT_PARAM("kp_xy", 1.5, "水平位置控制比例系数");
    dk::Param<double> kp_z = INIT_PARAM("kp_z", 1.0, "高度控制比例系数");
    dk::Param<double> kp_yaw = INIT_PARAM("kp_yaw", 2.0, "偏航角控制比例系数");

    dk::Param<double> ki_xy = INIT_PARAM("ki_xy", 0.01, "水平位置控制比例系数");
    dk::Param<double> ki_z = INIT_PARAM("ki_z", 0.01, "高度控制比例系数");
    dk::Param<double> ki_yaw = INIT_PARAM("ki_yaw", 0.01, "偏航角控制比例系数");

    dk::Param<double> kd_xy = INIT_PARAM("kd_xy", 0.1, "水平位置控制比例系数");
    dk::Param<double> kd_z = INIT_PARAM("kd_z", 0.1, "高度控制比例系数");
    dk::Param<double> kd_yaw = INIT_PARAM("kd_yaw", 0.1, "偏航角控制比例系数");

    dk::Param<double> max_i_xy = INIT_PARAM("max_i_xy", 1.0, "积分抗饱和");
    dk::Param<double> max_i_z = INIT_PARAM("max_i_z", 1.0, "积分抗饱和");
    dk::Param<double> max_i_yaw = INIT_PARAM("max_i_yaw", 1.0, "积分抗饱和");

    dk::Param<double> command_timeout_sec =
        INIT_PARAM("command_timeout_sec", 0.5, "控制指令失效保护阈值(s)");
    dk::Param<int> loop_rate_hz =
        INIT_PARAM("loop_rate_hz", 50, "跟踪器内部控制循环频率(Hz)");
    dk::Param<double> pos_tolerance_m =
        INIT_PARAM("pos_tolerance_m", 0.05, "判定目标到达的位置偏差阈值(m)");
    dk::Param<double> auto_heading_enable_dist_m = INIT_PARAM(
        "auto_heading_enable_dist_m", 1.0, "距离目标多远时开始自动调整朝向(m)");
    dk::Param<double> yaw_tolerance_rad = INIT_PARAM(
        "yaw_tolerance_rad", 0.05, "判定目标到达的角度偏差阈值(rad)");

    dk::Param<double> max_vel_xy =
        INIT_PARAM("max_vel_xy", 3.0, "XY轴最大允许速度(m/s)");
    dk::Param<double> max_vel_z =
        INIT_PARAM("max_vel_z", 1.0, "Z轴最大允许速度(m/s)");
    dk::Param<double> max_vel_yaw =
        INIT_PARAM("max_vel_yaw", 1.5, "最大旋转速度(rad/s)");

    dk::Param<double> max_acc_xy =
        INIT_PARAM("max_acc_xy", 2.0, "XY轴最大加速度(m/s^2)");
    dk::Param<double> max_acc_z =
        INIT_PARAM("max_acc_z", 0.8, "Z轴最大加速度(m/s^2)");
    dk::Param<double> max_acc_yaw =
        INIT_PARAM("max_acc_yaw", 2.0, "最大旋转加速度(rad/s^2)");

    dk::Param<double> max_decel_xy =
        INIT_PARAM("max_decel_xy", 1.0, "XY轴最大减速度(m/s^2)");
    dk::Param<double> max_decel_z =
        INIT_PARAM("max_decel_z", 0.8, "Z轴最大减速度(m/s^2)");
    dk::Param<double> yaw_full_speed_tol =
        INIT_PARAM("yaw_full_speed_tol", 0.17, "在此偏差内允许满速移动(rad)");
    dk::Param<double> yaw_zero_speed_tol = INIT_PARAM(
        "yaw_zero_speed_tol", 0.78, "偏差超过此值时完全停车原地转(rad)");

    dk::Param<double> min_lookahead =
        INIT_PARAM("min_lookahead", 0.5, "最小前视距离");

    dk::Param<double> max_lookahead =
        INIT_PARAM("max_lookahead", 2.5, "最大前视距离");
};