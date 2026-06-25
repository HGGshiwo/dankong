#include "features/mavlink/event_listener.hpp"

#include "core/global_config.hpp"
#include "robot_context.hpp"

void MavlinkEventListener::on_event(const dk::TickEvent& event,
                                    RobotContext& ctx) {
    // 405飞控在连接上之前必须一直请求，否则不会主动发送心跳
    if (rate_.check_and_update(ctx.engine->get_time_provider()->now()) &&
        !ctx.fcu_connected.load()) {
        spdlog::info("[Mavlink] call pull param!");
        ctx.engine->post_background_task<bool>([robot = ctx.robot]() {
            robot->pull_params();
            return true;
        });
    }
}

void MavlinkEventListener::on_event(const StatusTextEvent& event,
                                    RobotContext& ctx) {
    if (event.should_report) {
        ctx.ws_manager->publish_reliable(
            nlohmann::json{{"type", "error"}, {"error", event.text}});
    }
}

void MavlinkEventListener::on_event(const FcuConnectedEvent& event,
                                    RobotContext& ctx) {
    if (event.connected) {
        ctx.engine->post_background_task<bool>([robot = ctx.robot]() {
            robot->pull_params();
            return true;
        });
        ctx.engine->get_context().robot->set_stream_rate(
            0, GlobalConfig.GetConfig().fcu_data_rate);
        int msg_interval_rate =
            GlobalConfig.GetConfig().msg_interval_rate.get();
        ctx.engine->get_context().robot->set_msg_interval(
            32, msg_interval_rate);  // position
        ctx.engine->get_context().robot->set_msg_interval(
            30, msg_interval_rate);  // attitude
        ctx.engine->get_context().robot->set_msg_interval(
            132,
            msg_interval_rate);  // rangefinder
    }
}
