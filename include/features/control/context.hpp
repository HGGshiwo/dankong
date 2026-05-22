#pragma once
#include <Eigen/Dense>

#include "dk/report.hpp"
#include "utils/fixed_string64.hpp"

struct ControlContext {
    dk::StateRegistry& reg;

    // --- 以下变量定义时，直接指定了 Key、上报频率、初始值 ---
    dk::TrackedVar<bool> arm{reg, "arm", 5.0, false};
    dk::TrackedVar<bool> fcu_connected{reg, "connected", 2.0, false};
    dk::TrackedVar<int> gps_fix_type{reg, "gps_fix_type", 1.0, 0};
    dk::TrackedVar<int> gps_nsats{reg, "gps_nsats", 1.0, 0};
    dk::TrackedVar<double> battery_remaining{reg, "battery_remaining", 1.0,
                                             -1.0};
    dk::TrackedVar<double> battery_level{reg, "battery_level", 1.0, -1.0};

    dk::TrackedVar<FixedString64> mode{reg, "mode", 5.0,
                                       FixedString64("未知飞控模式")};
    // 航点到下一个目标点的距离
    dk::TrackedVar<double> dist_to_target{reg, "dist", 5.0, 0.0};
    dk::TrackedVar<int> wp_idx{reg, "wp_idx", 5.0, 0};  // 当前执行的航点序号

    dk::TrackedVar<Eigen::Vector3d> pos_enu{reg, "pos_enu", 0.0,
                                            Eigen::Vector3d::Zero()};
    dk::TrackedVar<Eigen::Vector3d> takeoff_lon_lat_alt{
        reg, "takeoff_lon_lat_alt", 0.0, Eigen::Vector3d::Zero()};

    dk::TrackedVar<double> yaw_ned{reg, "yaw", 10.0, 0.0};
    dk::TrackedVar<nlohmann::json> mission_data{reg, "mission_data", 1.0,
                                                nlohmann::json::array()};

    // --- 对于复合类型，传入 Lambda 直接定义 JSON 展开逻辑 ---
    dk::TrackedVar<Eigen::Vector3d> lon_lat_alt{
        reg, 10.0, Eigen::Vector3d::Zero(),
        [](nlohmann::json& j, const Eigen::Vector3d& data) {
            j["lon"] = data.x();
            j["lat"] = data.y();
            j["rel_alt"] = data.z();
        }};
    dk::TrackedVar<Eigen::Vector3d> vel_enu{
        reg, 10.0, Eigen::Vector3d::Zero(),
        [](nlohmann::json& j, const Eigen::Vector3d& data) {
            j["x_vel"] = data.x();
            j["y_vel"] = data.y();
        }};
    dk::TrackedVar<Eigen::Vector3d> vel_body{
        reg, 10.0, Eigen::Vector3d::Zero(),
        [](nlohmann::json& j, const Eigen::Vector3d& data) {
            j["x_vel_body"] = data.x();
            j["y_vel_body"] = data.y();
        }};

    dk::ThreadVar<std::chrono::steady_clock::time_point>
        stop_follow_stamp;  // stop_follow被调用的时间

    std::atomic<bool> odom_ok = false;
    std::atomic<double> current_battery = -1.0;
    std::atomic<double> voltage_battery = -1.0;

    std::atomic<int> sensor_health = 0;
    std::atomic<double> yaw_enu = 0.0;

   public:
    explicit ControlContext(dk::StateRegistry& r) : reg(r) {};
};