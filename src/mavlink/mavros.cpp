#include "mavlink/mavros.hpp"

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "ros/time.h"

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

bool MavRos::send_cmd(std::optional<Eigen::Vector3d> pos, std::optional<Eigen::Vector3d> vel,
                      std::optional<Eigen::Vector3d> acc, std::optional<double> yaw, std::optional<double> yaw_rate,
                      CmdFrame frame) {
    auto mav_frame = frame == CmdFrame::BODY ? mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED
                                             : mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "local_ned";
    target.coordinate_frame = mav_frame;
    uint32_t type_mask = 0;
    if (!pos.has_value()) {
        type_mask = type_mask | mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ;
        pos = Eigen::Vector3d::Zero();
    }

    if (!vel.has_value()) {
        type_mask = type_mask | mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ;
        vel = Eigen::Vector3d::Zero();
    }
    if (!acc.has_value()) {
        type_mask = type_mask | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ;
        acc = Eigen::Vector3d::Zero();
    }
    if (!yaw.has_value()) {
        type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
        yaw = 0;
    }
    if (!yaw_rate.has_value()) {
        type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        yaw_rate = 0;
    }
    target.position.x = pos->x();
    target.position.y = pos->y();
    target.position.z = pos->z();
    target.velocity.x = vel->x();
    target.velocity.y = vel->y();
    target.velocity.z = vel->z();
    target.acceleration_or_force.x = acc->x();
    target.acceleration_or_force.y = acc->y();
    target.acceleration_or_force.z = acc->z();
    target.yaw = yaw.value();
    target.yaw_rate = yaw_rate.value();
    target.type_mask = type_mask;
    setpoint_pub_->publish(target);
    return true;
}
