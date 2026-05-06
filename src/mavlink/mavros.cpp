#include "mavlink/mavros.hpp"

#include <ros/ros.h>

#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"

bool MavRos::set_stream_rate(int rate) {
    // 填充请求
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = 0;
    srv.request.message_rate = rate;
    srv.request.on_off = true;
    return set_rate_client_->call(srv);
}

bool MavRos::set_mode(const FixedString64& mode) {
    // 构造服务请求
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 0;       // 基础模式通常设为 0
    srv.request.custom_mode = mode;  // 自定义模式字符串，如 "GUIDED"
    return set_mode_client_->call(srv);
}

// MAV_CMD_RUN_PREARM_CHECKS
bool MavRos::run_prearm_checks() {
    mavros_msgs::CommandLong srv;

    // 对应 Python: command=401, confirmation=0, param1=0...
    srv.request.command = 401;  // MAV_CMD_RUN_PREARM_CHECKS
    srv.request.confirmation = 0;
    srv.request.param1 = 0.0;
    srv.request.param2 = 0.0;
    srv.request.param3 = 0.0;
    srv.request.param4 = 0.0;
    srv.request.param5 = 0.0;
    srv.request.param6 = 0.0;
    srv.request.param7 = 0.0;
    // 调用服务 (如果服务存在且通信成功，返回 true)
    return cmd_client_->call(srv);
}

bool MavRos::arm() {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    return arm_client_->call(srv);
}

bool MavRos::takeoff(double alt) {
    mavros_msgs::CommandTOL srv;
    srv.request.altitude = alt;
    srv.request.latitude = 0;
    srv.request.longitude = 0;
    srv.request.min_pitch = 0;
    srv.request.yaw = 0;
    return takeoff_client_->call(srv);
}

bool MavRos::land() {
    return set_mode("LAND");
}

ApmParam MavRos::get_param(std::string name, ApmParam value) {
    ApmParam result;
    // std::visit 会根据 value 中实际存储的类型（int/double/string）实例化出对应的代码分支
    std::visit(
        [this, &name, &result](auto&& default_val) {
            // 获取当前分支的底层类型 T
            using T = std::decay_t<decltype(default_val)>;

            T fetched_val;
            // 调用 ROS API: nh_.param<T>("参数名", 存储变量, 默认值)
            this->nh_.param<T>("/mavros/param/" + name, fetched_val, default_val);

            // 将获取到的强类型值重新赋给 variant
            result = fetched_val;
        },
        value);
    return result;
}
