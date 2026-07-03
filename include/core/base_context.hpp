#pragma once
#include <atomic>
#include <chrono>
#include <memory>

#include "dk/adapters/web/websocket.hpp"
#include "dk/engine.hpp"
#include "robot/irobot.hpp"
#include "utils/ros.hpp"
#include "utils/state_registry.hpp"

// 所有机器人 Context 共享的基础数据结构
struct IContext {
    virtual ~IContext() = default;
    // 这两个需要小心，没有加锁
    std::shared_ptr<dk::ConnectionManager> ws_manager;
    StateRegistry state_registry;
    std::shared_ptr<mavsdk::System> mavsdk_system;
#ifdef USE_ROS1
    ros::NodeHandle nh;
#elif defined(USE_ROS2)
    std::shared_ptr<rclcpp::Node> node;
#endif
};

// 在抽象层持有 Engine！
template <typename DerivedContext>
struct BaseRobotContext : public IContext {
    std::shared_ptr<dk::IEngine<DerivedContext>> engine;
};