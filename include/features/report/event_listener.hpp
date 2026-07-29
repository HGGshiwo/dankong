#pragma once

#include <cmath>
#include <nlohmann/json.hpp>

#include "core/global_config.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/event_listener.hpp"
#include "robot_context.hpp"
#include "utils/logger/fg_logger.hpp"

// 和ws事件相关的状态监听器
class ReportEventListener
    : public dk::BaseEventListener<RobotContext, ReportEventListener> {
   public:
    using AllowedEvents = std::tuple<dk::WsOpenEvent>;

    std::shared_ptr<dk::ConnectionManager> manager_;
    double last_send_time_ = 0.0;

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

    void publish_pos(RobotContext& ctx) {
        auto pos_enu = ctx.pos_enu.load();
        double px = std::isnan(pos_enu.x()) || std::isinf(pos_enu.x())
                        ? 0.0
                        : pos_enu.x();
        double py = std::isnan(pos_enu.y()) || std::isinf(pos_enu.y())
                        ? 0.0
                        : pos_enu.y();
        double pz = std::isnan(pos_enu.z()) || std::isinf(pos_enu.z())
                        ? 0.0
                        : pos_enu.z();
        nlohmann::json pose_msg = {px, py, pz};
        fglog::publish("/drone/current_pose", pose_msg);
    }

    void publish_lla(RobotContext& ctx) {
        auto lla = ctx.lon_lat_alt.load();
        double lat = std::isnan(lla.y()) || std::isinf(lla.y()) ? 0.0 : lla.y();
        double lon = std::isnan(lla.x()) || std::isinf(lla.x()) ? 0.0 : lla.x();
        double alt = std::isnan(lla.z()) || std::isinf(lla.z()) ? 0.0 : lla.z();
        nlohmann::json gps_msg = {lat, lon, alt};  // lat, lon, alt
        fglog::publish("/drone/gps/fix", gps_msg);
    }

    void on_tick(double dt, RobotContext& ctx) {
        // 先尝试记录到FoxGlove
        if (is_hz(10)) {
            publish_pos(ctx);
            publish_lla(ctx);
        }
        nlohmann::json j;
        double now = ctx.engine->get_time_provider()->now();
        ctx.state_registry.report_all(j, now);

        bool is_heartbeat = false;
        if (j.empty()) {
            double hz = GlobalConfig.GetConfig().heartbeat_hz.get();
            if (hz > 0) {
                double interval = 1.0 / hz;
                if (last_send_time_ <= 0.0) {
                    last_send_time_ = now;
                } else if (now - last_send_time_ >= interval) {
                    is_heartbeat = true;
                }
            }
        }

        // 如果没有任何数据改变且不需要心跳，直接 return
        if (j.empty() && !is_heartbeat) {
            return;
        }

        // 补充动态的基本状态
        j["type"] = "state";
        // if (ctx.engine) {
        //     j["state"] = ctx.engine->get_state_name();
        // }
        manager_->publish(j);
        last_send_time_ = now;
    }
};