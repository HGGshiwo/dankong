#pragma once
#include <cstring>
#include <string>
#include <vector>

#include "dk/core.hpp"
#include "dk/utils.hpp"
#include "mavlink/imavlink.hpp"
#include "robot/robot_base.hpp"
#include "robot_event.hpp"

struct RobotContext : public dk::BaseContext<RobotEvent, RobotContext> {
    // 这两个需要小心，没有加锁
    std::shared_ptr<IRobot> robot;
    std::shared_ptr<IMavlink> mavlink;

    std::atomic<bool> odom_ok = false;
    std::atomic<bool> arm = false;
    std::atomic<int> sensor_health = 0;
    std::atomic<int> wp_idx = 0;      // 当前点的序号
    std::atomic<double> wp_dist = 0;  // 到目标的之下距离

    std::atomic<double> throttle = -1.0;         // 当前的电机推力
    std::atomic<double> rangefinder_alt = -1.0;  // 测距仪的高度
    std::atomic<bool> planner_enable = false;

    std::atomic<FixedString64> mode = FixedString64("UNKNOWN");

    dk::thread_safe<Eigen::Vector3d> wp_goal;  // 当前enu坐标系下的目标
    dk::thread_safe<std::vector<Eigen::Vector3d>> waypoint;
    dk::thread_safe<std::vector<std::vector<std::string>>> event_list;
    dk::thread_safe<Eigen::Vector3d> pos;
    dk::thread_safe<Eigen::Vector3d> takeoff_pos;
};