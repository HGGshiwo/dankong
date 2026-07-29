#pragma once
#include <foxglove/LocationFix.pb.h>
#include <foxglove/Pose.pb.h>

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
        foxglove::Pose pose_msg;

        // 2. 填充位置 (Vector3)
        auto* position = pose_msg.mutable_position();
        auto pos_enu = ctx.pos_enu.load();
        position->set_x(pos_enu.x());
        position->set_y(pos_enu.y());
        position->set_z(pos_enu.z());

        // 3. 填充姿态 (Quaternion)
        auto* orientation = pose_msg.mutable_orientation();
        auto orient = ctx.orientation.load();
        orientation->set_x(orient.x());
        orientation->set_y(orient.y());
        orientation->set_z(orient.z());
        orientation->set_w(orient.w());

        // 4. 直接通过泛型接口发布！
        fglog::publish("/drone/current_pose", pose_msg);
    }

    void publish_lla(RobotContext& ctx) {
        // 1. 创建官方的 LocationFix 消息
        foxglove::LocationFix gps_msg;

        auto lla = ctx.lon_lat_alt.load();

        // 2. 填入经纬度和高度 (强制必须是 double 类型)
        gps_msg.set_latitude(lla.y());
        gps_msg.set_longitude(lla.x());
        gps_msg.set_altitude(lla.z());

        auto* meta_fix = gps_msg.add_metadata();
        meta_fix->set_key("Fix Type");
        switch (ctx.gps_fix_type.load()) {
            case 0:
                meta_fix->set_value("No Fix");
                break;
            case 2:
                meta_fix->set_value("2D Fix");
                break;
            case 3:
                meta_fix->set_value("3D Fix");
                break;
            case 4:
                meta_fix->set_value("DGPS");
                break;
            case 5:
                meta_fix->set_value("RTK Float");
                break;
            case 6:
                meta_fix->set_value("RTK Fixed");
                break;
            default:
                meta_fix->set_value("Unknown");
                break;
        }

        // 3. 极速扔给泛型队列发走
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