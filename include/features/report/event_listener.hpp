#pragma once
#include <nlohmann/json.hpp>

#include "core/global_config.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/event_listener.hpp"
#include "robot_context.hpp"

// 和ws事件相关的状态监听器
class ReportEventListener
    : public dk::BaseEventListener<RobotContext, ReportEventListener> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent, dk::WsOpenEvent>;

    std::shared_ptr<dk::ConnectionManager> manager_;

    ReportEventListener(std::shared_ptr<dk::ConnectionManager> manager)
        : manager_(manager) {}

    // 当有新客户端接入时的处理逻辑：
    void on_event(const dk::WsOpenEvent& event, RobotContext& ctx) {
        // 1. 无视脏位，一键提取所有变量当前的最新快照！
        nlohmann::json full_state = ctx.state_registry.get_full_state();

        // 2. 补齐静态/基础字段
        full_state["type"] = "state";
        if (ctx.engine) {
            full_state["state"] = ctx.engine->get_state_name();
        }

        // 3. 仅发给这一个新连进来的客户端
        event.conn->send(full_state);
    }

    void on_event(const dk::TickEvent& event, RobotContext& ctx) {
        nlohmann::json j;
        double now = ctx.engine->get_time_provider()->now();
        ctx.state_registry.report_all(j, now);
        // 如果没有任何数据改变或达到频率条件，j 将是空的，直接 return
        if (j.empty()) {
            return;
        }
        // 补充动态的基本状态
        j["type"] = "state";
        if (ctx.engine) {
            j["state"] = ctx.engine->get_state_name();
        }
        manager_->publish(j);
    }
};