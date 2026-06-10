#pragma once
#include <Eigen/Dense>

#include "utils/config_param.hpp"

//@JSON_ENABLE
struct PlandConfig {
    static constexpr const char* __group_name = "Pland";
    dk::Param<double> inner_tag_size =
        INIT_PARAM("inner_tag_size", 0.8, "内部tag的边长，单位m(黑边之内)");
    dk::Param<double> outter_tag_size =
        INIT_PARAM("outter_tag_size", 0.15, "外部tag的边长，单位m(黑边之内)");
    dk::Param<double> offset_x =
        INIT_PARAM("offset_x", 0, "相机相对于机身的x轴偏移");
    dk::Param<double> offset_y =
        INIT_PARAM("offset_y", 0, "相机相对于机身的y轴偏移");
    dk::Param<double> offset_z =
        INIT_PARAM("offset_z", 0, "相机相对于机身的z轴偏移");

    dk::Param<Eigen::Matrix3d> camera_inner_matrix =
        INIT_HIDDEN_PARAM("camera_inner_matrix", Eigen::Matrix3d::Identity());
    dk::Param<std::string> pland_image_topic =
        INIT_HIDDEN_PARAM("pland_image_topic", "/pland/image");
    dk::Param<std::string> pland_detect_topic =
        INIT_HIDDEN_PARAM("pland_detect_topic", "/pland/detect");

    dk::Param<double> velocity_deadzone =
        INIT_PARAM("velocity_deadzone", 0.2, "如果速度小于0.2则认为没有移动");

    dk::Param<double> touchdown_z_thresh = INIT_PARAM(
        "touchdown_z_thresh", 0.3,
        "低于该高度不再使用多项式计算降落速度，直接使用touchdown_velocity");
    dk::Param<double> xy_align_thresh =
        INIT_PARAM("xy_align_thresh", 0.45, "xy误差低于该阈值允许降低高度");
    dk::Param<double> yaw_align_thresh =
        INIT_PARAM("yaw_align_thresh", 0.2, "yaw误差低于该阈值允许降低高度");
    dk::Param<double> touchdown_velocity = INIT_PARAM(
        "touchdown_velocity", 0.4, "低于touchdown_z_thresh时使用固定降落速度");
    dk::Param<double> lost_target_alt =
        INIT_PARAM("lost_target_alt", 15.0, "丢失目标后的悬停高度(m)");

    dk::Param<double> pland_max_acc_xy =
        INIT_PARAM("pland_max_acc_xy", 1.0, "精准降落使用的加速度");
    dk::Param<double> pland_max_devel_xy =
        INIT_PARAM("pland_max_devel_xy", 1.0, "精准降落使用的减加速度");

    dk::Param<double> pland_limit_start_z =
        INIT_PARAM("pland_limit_start_z", 3.0, "该高度以下开始收紧反馈速度");
    dk::Param<double> pland_min_cruise_speed = INIT_PARAM(
        "pland_min_curise_speed", 0.4, "靠近地面时反馈速度限制的最小值");
    dk::Param<double> pland_cruise_speed_xy = INIT_PARAM(
        "pland_cruise_speed_xy", 3.0, "靠近地面时反馈速度限制的最大值");

    dk::Param<double> pland_decay_start_z =
        INIT_PARAM("pland_decay_start_z", 3.0, "位置反馈系数开始衰减的高度");

    dk::Param<double> pland_gamma_yaw =
        INIT_PARAM("pland_gamma_yaw", 0.2, "位置反馈系数");
    dk::Param<double> pland_gamma =
        INIT_PARAM("pland_gamma", 0.25, "位置反馈系数线性衰减开始值");
    dk::Param<double> pland_min_gamma =
        INIT_PARAM("pland_min_gamma", 0.1, "位置反馈系数线性衰减最终值");
    dk::Param<double> pland_gamma_z =
        INIT_PARAM("pland_gamma_z", 0.5, "位置反馈系数线性衰减开始值");
    dk::Param<double> pland_blind_drop_alt =
        INIT_PARAM("pland_blind_drop_alt", 0.1, "切降落的高度");
    dk::Param<double> platform_height =
        INIT_PARAM("platform_height", 0.0, "降落平台的高度");
    dk::Param<double> pland_max_vel_z =
        INIT_PARAM("pland_max_vel_z", 1.0, "最大降落速度");

    dk::Param<bool> pland_gimbal_abs =
        INIT_PARAM("pland_gimbal_abs", false, "云台固定角模式(垂直地面)");
    dk::Param<std::string> gimbal_roll_topic =
        INIT_PARAM("gimbal_roll_topic", "/gimbal/roll", "云台回传数据");
    dk::Param<std::string> gimbal_pitch_topic =
        INIT_PARAM("gimbal_pitch_topic", "/gimbal/pitch", "云台回传数据");
    dk::Param<std::string> gimbal_yaw_topic =
        INIT_PARAM("gimbal_yaw_topic", "/gimbal/yaw", "云台回传数据");
};