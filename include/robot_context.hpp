#pragma once
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "dk/engine.hpp"
#include "dk/utils.hpp"
#include "mavlink/imavlink.hpp"
#include "mavlink/mavros.hpp"
#include "robot/robot_base.hpp"
#include "robot_event.hpp"

// struct WpData {
//     std::vector<Eigen::Vector3d> wp_list;
//     std::optional<std::vector<std::vector<std::string>>> event_list;
//     bool land;                // 任务结束后是否降落
//     int wp_idx = 0;           // 当前点的序号
//     double wp_dist = 0;       // 到下一个目标的距离
//     Eigen::Vector3d wp_goal;  // 点在enu坐标系下的目标
// };

struct RobotContext : public dk::BaseContext<RobotContext> {
    // 这两个需要小心，没有加锁
    std::shared_ptr<IRobot<MavRos, RobotContext>> robot;
    // dk::thread_safe<std::optional<WpData>> wp_data;

    std::atomic<bool> odom_ok = false;
    std::atomic<bool> arm = false;
    std::atomic<int> sensor_health = 0;

    std::atomic<double> throttle = -1.0;         // 当前的电机推力
    std::atomic<double> rangefinder_alt = -1.0;  // 测距仪的高度
    std::atomic<bool> planner_enable = false;

    std::atomic<FixedString64> mode = FixedString64("UNKNOWN");

    dk::thread_safe<Eigen::Vector3d> pos_enu;  // ENU坐标系下的位置
    std::atomic<double> yaw_enu;
    std::atomic<double> yaw_ned;
    // 融合后的纬经高，来源于global_position，
    // 注意rel_alt其实和这里的z是同一个
    dk::thread_safe<Eigen::Vector3d> lon_lat_alt;
    dk::thread_safe<Eigen::Vector3d> takeoff_lon_lat_alt;  // 起飞的纬经高
};