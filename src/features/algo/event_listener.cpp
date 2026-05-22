#include "features/algo/event_listener.hpp"

#include "states/follow_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "states/waypoint_state.hpp"

// 构造函数实现
AlgoEventListener::AlgoEventListener() : nh_() {
    start_record_client_ =
        std::make_shared<ServiceClient<rsos_msgs::StartBagRecord>>(
            "/data_recorder/start_recording",
            [](rsos_msgs::StartBagRecord srv) -> bool {
                return srv.response.success;
            });

    stop_record_client_ = std::make_shared<ServiceClient<std_srvs::Trigger>>(
        "/data_recorder/stop_recording",
        [](std_srvs::Trigger srv) { return true; });

    set_gimbal_client_ =
        std::make_shared<ServiceClient<rsos_msgs::SetGimbalAngle>>(
            "/UAV0/sensor/serial_gimbal/set_gimbal_angle",
            [](rsos_msgs::SetGimbalAngle srv) { return srv.response.success; });
    set_exposure_client_ =
        std::make_shared<ServiceClient<rsos_msgs::SetCameraExposure>>(
            "/UAV0/sensor/video11_camera/set_exposure",
            [](rsos_msgs::SetCameraExposure srv) {
                return srv.response.success;
            });
}

// on_event 各重载实现
void AlgoEventListener::on_event(const EnableDetectEvent& event,
                                 RobotContext& ctx) {
    auto target_it = detect_map_.find(event.type);
    if (target_it == detect_map_.end()) {
        event.reject("can not find event: " + event.type);
        return;
    }
    for (auto it = detect_map_.begin(); it != detect_map_.end(); ++it) {
        if (it == target_it)
            nh_.setParam(it->second, true);
        else
            nh_.setParam(it->first, false);
    }
    ctx.detect_type.set(event.type);
    nh_.setParam(
        "/UAV0/perception/object_location/object_location_node/enable_send",
        true);
    event.resolve({"success", "OK"});
}

void AlgoEventListener::on_event(const DisableDetectEvent& event,
                                 RobotContext& ctx) {
    for (auto& [k, v] : detect_map_) {
        nh_.setParam(v, false);
    }
    ctx.detect_type.set("Disabled");
    nh_.setParam(
        "/UAV0/perception/object_location/object_location_node/enable_send",
        false);
    event.resolve({"success", "OK"});
}

void AlgoEventListener::on_event(const GetDetectEvent& event,
                                 RobotContext& ctx) {
    auto detect_type = ctx.detect_type.get();
    if (detect_type == "Disabled") {
        detect_type = "";
    }
    event.resolve({"success", detect_type});
}

void AlgoEventListener::on_event(const StartRecordEvent& event,
                                 RobotContext& ctx) {
    rsos_msgs::StartBagRecord srv;
    srv.request.prefix = event.bag_name;
    if (start_record_client_->call(srv)) {
        ctx.recording.set(true);
        event.resolve({"success", srv.response.message});
    } else {
        event.reject(srv.response.message);
    }
}

void AlgoEventListener::on_event(const StopRecordEvent& event,
                                 RobotContext& ctx) {
    std_srvs::Trigger srv;
    if (stop_record_client_->call(srv)) {
        event.resolve({"success", "OK"});
    } else {
        event.reject("stop call failed!");
    }
}

void AlgoEventListener::on_event(const GetGimbalEvent& event,
                                 RobotContext& ctx) {
    std::string mode;
    double angle;
    nh_.param<std::string>("/UAV0/sensor/serial_gimbal/angle_mode", mode, "");
    nh_.param<double>("/UAV0/sensor/serial_gimbal/gimbal_angle", angle, 0.0);
    auto j = nlohmann::json{"success", {{"mode", mode}, {"angle", angle}}};
    event.resolve({"success", j});
}

void AlgoEventListener::on_event(const SetGimbalEvent& event,
                                 RobotContext& ctx) {
    rsos_msgs::SetGimbalAngle srv;
    srv.request.mode = event.mode;
    srv.request.angle = event.angle;
    if (set_gimbal_client_->call(srv)) {
        event.resolve({"success", srv.response.message});
    } else {
        event.reject(srv.response.message);
    }
}

void AlgoEventListener::on_event(const GetExposureEvent& event,
                                 RobotContext& ctx) {
    double shutter, shutter_max, shutter_min, shutter_step;
    double sens, sens_max, sens_min, sens_step;
    nh_.param<double>("/UAV0/sensor/video11_camera/shutter", shutter, 50.0);
    nh_.param<double>("/UAV0/sensor/video11_camera/shutter_max", shutter_max,
                      100.0);
    nh_.param<double>("/UAV0/sensor/video11_camera/shutter_min", shutter_min,
                      0.0);
    nh_.param<double>("/UAV0/sensor/video11_camera/shutter_step", shutter_step,
                      1.0);
    nh_.param<double>("/UAV0/sensor/video11_camera/ISO", sens, 50.0);
    nh_.param<double>("/UAV0/sensor/video11_camera/ISO_max", sens_max, 100.0);
    nh_.param<double>("/UAV0/sensor/video11_camera/ISO_min", sens_min, 0.0);
    nh_.param<double>("/UAV0/sensor/video11_camera/ISO_step", sens_step, 2.0);

    nlohmann::json json_data{{"shutter",
                              {{"value", shutter},
                               {"max", shutter_max},
                               {"min", shutter_min},
                               {"step", shutter_step}}},
                             {"sensitivity",
                              {{"value", sens},
                               {"max", sens_max},
                               {"min", sens_min},
                               {"step", sens_step}}}};
    event.resolve({"success", json_data});
}

void AlgoEventListener::on_event(const SetExposureEvent& event,
                                 RobotContext& ctx) {
    rsos_msgs::SetCameraExposure srv;
    srv.request.shutter = event.shutter;
    srv.request.sensitivity = event.sensitivity;

    if (set_exposure_client_->call(srv))
        event.resolve({"success", srv.response.message});
    else
        event.reject(srv.response.message);
}

void AlgoEventListener::on_event(const DetectEvent& event, RobotContext& ctx) {
    double FOLLOW_COOL_TIME = 100.0;  // 100秒内不响应
    if (state_utils::get_time_span(ctx.stop_follow_stamp.get()) <
        FOLLOW_COOL_TIME) {
        return;
    }

    if (ctx.engine->template is_active_state<FollowState>()) {
        return;  // 已经在跟随，则不进行捕获，注意跟随中可能仍然在Hover
    }
    if (ctx.engine->template is_active_state<HoverState>()) {
        ctx.engine->template step<FollowState<HoverState>>(
            std::forward_as_tuple(event, ctx));
    } else if (ctx.engine->template is_active_state<WaypointState>()) {
        ctx.engine->template step<FollowState<WaypointState>>(
            std::forward_as_tuple(event, ctx));
    }
    // 不支持的类型
    return;
}

void AlgoEventListener::on_event(const StopFollowEvent& event,
                                 RobotContext& ctx) {
    auto cur_name = ctx.engine->get_state_name();
    if (cur_name != "跟随模式") {
        event.reject("非跟随模式调用无效! 当前状态：" + cur_name);
        return;
    }
    event.resolve({"success", "OK"});
    ctx.stop_follow_stamp.set(std::chrono::steady_clock::now());
    if (ctx.engine->is_active_state<HoverState>()) {
        ctx.engine->step<HoverState>();
    } else if (ctx.engine->is_active_state<WaypointState>()) {
        ctx.engine->step<WaypointState::LiftingState>();
    } else {
        spdlog::warn("unknow state: {}, switch to hover", cur_name);
        ctx.engine->step<HoverState>();
    }
}

void AlgoEventListener::on_event(const EnablePlandEvent& event,
                                 RobotContext& ctx) {
    ctx.pland_enable.set(true);
    event.resolve({"success", "OK"});
}

void AlgoEventListener::on_event(const DisablePlandEvent& event,
                                 RobotContext& ctx) {
    ctx.pland_enable.set(false);
    event.resolve({"success", "OK"});
}

void AlgoEventListener::on_event(const EnablePlannerEvent& event,
                                 RobotContext& ctx) {
    ctx.planner_enable.set(true);
    event.resolve({"success", "OK"});
}

void AlgoEventListener::on_event(const DisablePlannerEvent& event,
                                 RobotContext& ctx) {
    ctx.planner_enable.set(false);
    event.resolve({"success", "OK"});
}
