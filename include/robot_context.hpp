#pragma once
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "dk/adapters/web/protocal.hpp"
#include "dk/engine.hpp"
#include "dk/report.hpp"
#include "dk/utils.hpp"
#include "mavlink/imavlink.hpp"
#include "mavlink/mavros.hpp"
#include "nlohmann/json.hpp"
#include "robot/robot_base.hpp"
#include "robot_event.hpp"
#include "utils.hpp"

struct IContext {
    virtual ~IContext() = default;
    // 这两个需要小心，没有加锁
    std::shared_ptr<dk::ConnectionManager> ws_manager;
    std::shared_ptr<IRobot> robot;

    dk::StateRegistry state_registry;
    // --- 以下变量定义时，直接指定了 Key、上报频率、初始值 ---
    dk::TrackedVar<bool> arm{state_registry, "arm", 5.0, false};
    dk::TrackedVar<bool> fcu_connected{state_registry, "connected", 2.0, false};
    dk::TrackedVar<int> gps_fix_type{state_registry, "gps_fix_type", 1.0, 0};
    dk::TrackedVar<int> gps_nsats{state_registry, "gps_nsats", 1.0, 0};
    dk::TrackedVar<double> battery_remaining{state_registry,
                                             "battery_remaining", 1.0, -1.0};
    dk::TrackedVar<double> battery_level{state_registry, "battery_level", 1.0,
                                         -1.0};

    dk::TrackedVar<bool> planner_enable{state_registry, "planner", 2.0, false};
    dk::TrackedVar<bool> pland_enable{state_registry, "pland", 2.0, false};
    dk::TrackedVar<FixedString64> version{state_registry, "version", 0.5,
                                          FixedString64("未知")};
    dk::TrackedVar<FixedString64> mode{state_registry, "mode", 5.0,
                                       FixedString64("未知飞控模式")};
    // 航点到下一个目标点的距离
    dk::TrackedVar<double> dist_to_target{state_registry, "dist", 5.0, 0.0};
    dk::TrackedVar<int> wp_idx{state_registry, "wp_idx", 5.0,
                               0};  // 当前执行的航点序号

    dk::TrackedVar<Eigen::Vector3d> pos_enu{state_registry, "pos_enu", 0.0,
                                            Eigen::Vector3d::Zero()};
    dk::TrackedVar<Eigen::Vector3d> takeoff_lon_lat_alt{
        state_registry, "takeoff_lon_lat_alt", 0.0, Eigen::Vector3d::Zero()};

    dk::TrackedVar<double> yaw_ned{state_registry, "yaw", 10.0, 0.0};
    dk::TrackedVar<nlohmann::json> mission_data{state_registry, "mission_data",
                                                1.0, nlohmann::json::array()};
    dk::TrackedVar<FixedString64> detect_type{state_registry, "detect_type",
                                              1.0, "Disabled"};
    dk::TrackedVar<bool> recording{state_registry, "recording", 1.0, false};

    // --- 对于复合类型，传入 Lambda 直接定义 JSON 展开逻辑 ---
    dk::TrackedVar<Eigen::Vector3d> lon_lat_alt{
        state_registry, 10.0, Eigen::Vector3d::Zero(),
        [](nlohmann::json& j, const Eigen::Vector3d& data) {
            j["lon"] = data.x();
            j["lat"] = data.y();
            j["rel_alt"] = data.z();
        }};
    dk::TrackedVar<Eigen::Vector3d> vel_enu{
        state_registry, 10.0, Eigen::Vector3d::Zero(),
        [](nlohmann::json& j, const Eigen::Vector3d& data) {
            j["x_vel"] = data.x();
            j["y_vel"] = data.y();
        }};
    dk::TrackedVar<Eigen::Vector3d> vel_body{
        state_registry, 10.0, Eigen::Vector3d::Zero(),
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
    std::atomic<double> throttle = -1.0;         // 当前的电机推力
    std::atomic<double> rangefinder_alt = -1.0;  // 测距仪的高度
    std::atomic<double> yaw_enu = 0.0;
};

// 在抽象层持有 Engine！
template <typename DerivedContext>
struct BaseRobotContext : public IContext {
    std::shared_ptr<dk::IEngine<DerivedContext>> engine;
};

struct DogContext : BaseRobotContext<DogContext> {
    std::atomic<uint> dog_state;
    dk::TrackedVar<FixedString64> dog_state_name{
        state_registry, "basic_state", 5.0, FixedString64("未知Dog模式")};
};

struct DroneContext : BaseRobotContext<DroneContext> {};

// 4. 根据编译目标，将 Context 绑定到具体的类型上
#ifdef BUILD_DRONE
using RobotContext = DroneContext;
#else
using RobotContext = DogContext;
#endif