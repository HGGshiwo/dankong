#pragma once
#include <atomic>
#include <chrono>
#include <memory>

#include "dk/adapters/web/websocket.hpp"
#include "dk/engine.hpp"
#include "dk/report.hpp"
#include "robot/irobot.hpp"

// 所有机器人 Context 共享的基础数据结构
struct IContext {
    virtual ~IContext() = default;
    // 这两个需要小心，没有加锁
    std::shared_ptr<dk::ConnectionManager> ws_manager;
    dk::StateRegistry state_registry;
};

// 在抽象层持有 Engine！
template <typename DerivedContext>
struct BaseRobotContext : public IContext {
    std::shared_ptr<dk::IEngine<DerivedContext>> engine;
};