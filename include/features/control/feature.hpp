#pragma once
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/SysStatus.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <rsos_msgs/PointObj.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <Eigen/Dense>
#include <memory>

#include "./context.hpp"
#include "Eigen/src/Geometry/Quaternion.h"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "features/control/events.hpp"
#include "states/state_utils.hpp"

class ControlFeature {
   public:
    // 1. 声明属于自己的 Context（如果没有，就写 EmptyContext）
    using Context = ControlContext;

    template <typename RosAdapter>
    static void setup(TagRos, std::shared_ptr<RosAdapter>& ros) {
        ros->bind_context(
            "/mavros/state",
            [](const mavros_msgs::State::ConstPtr& state_ptr,
               RobotContext& ctx) -> void {
                auto armed = ctx.arm.load();
                auto cur_armed = state_ptr->armed != 0;
                if (armed != cur_armed) {
                    ctx.arm.store(cur_armed);
                    ctx.engine->dispatch_internal(ArmEvent{cur_armed});
                }
                if (ctx.mode.load() != state_ptr->mode) {
                    ctx.engine->dispatch_internal(
                        FlightModeEvent{ctx.mode.load(), state_ptr->mode});
                    ctx.mode.store(state_ptr->mode);
                }
                auto connected = state_ptr->connected != 0;
                if (ctx.fcu_connected.load() != connected) {
                    ctx.fcu_connected.store(connected);
                    ctx.engine->dispatch_internal(FcuConnectedEvent{connected});
                }
            });

        ros->bind_context("/mavros/local_position/velocity_body",
                          [](const geometry_msgs::TwistStamped::ConstPtr& msg,
                             RobotContext& ctx) -> void {
                              ctx.vel_body.store(Eigen::Vector3d{
                                  msg->twist.linear.x, msg->twist.linear.y,
                                  msg->twist.linear.z});
                              ctx.vel_angular_body.store(Eigen::Vector3d{
                                  msg->twist.angular.x, msg->twist.angular.y,
                                  msg->twist.angular.z});
                          });

        ros->bind_context("/mavros/local_position/velocity_local",
                          [](const geometry_msgs::TwistStamped::ConstPtr& msg,
                             RobotContext& ctx) -> void {
                              ctx.vel_enu.store(Eigen::Vector3d{
                                  msg->twist.linear.x, msg->twist.linear.y,
                                  msg->twist.linear.z});
                          });

        ros->bind_context(
            "/mavros/local_position/odom",
            [](const nav_msgs::Odometry::ConstPtr& odom,
               RobotContext& ctx) -> void {
                auto pos = odom->pose.pose.position;
                ctx.pos_enu.emplace(pos.x, pos.y, pos.z);

                auto orientation = odom->pose.pose.orientation;
                auto rpy = state_utils::orientation_to_euler(
                    orientation.x, orientation.y, orientation.z, orientation.w);

                ctx.yaw_enu = rpy.z();
                ctx.pitch.store(rpy.y());
                ctx.roll.store(rpy.x());
                ctx.orientation.store(
                    Eigen::Quaterniond(orientation.w, orientation.x,
                                       orientation.y, orientation.z));
                ctx.yaw_ned.store(state_utils::yaw_enu_to_ned(rpy.z()));

                // [新增] 将当前位姿存入历史记录
                ctx.pose_history.push(
                    odom->header.stamp, Eigen::Vector3d(pos.x, pos.y, pos.z),
                    Eigen::Quaterniond(orientation.w, orientation.x,
                                       orientation.y, orientation.z));
            });

        ros->bind_context(
            "/mavros/global_position/global",
            [](const sensor_msgs::NavSatFix::ConstPtr& msg,
               RobotContext& ctx) -> void {
                ctx.lon_lat_alt.write([msg](Eigen::Vector3d& data) {
                    data.x() = msg->longitude;
                    data.y() = msg->latitude;
                });
                ctx.odom_ok = true;
            });

        // 该值是相对于home的高度, gps的alt是相对于海平面高度(MSL)
        ros->bind_context(
            "/mavros/global_position/rel_alt",
            [](const std_msgs::Float64::ConstPtr& msg,
               RobotContext& ctx) -> void {
                ctx.lon_lat_alt.write(
                    [msg](Eigen::Vector3d& data) { data.z() = msg->data; });
            });

        ros->bind_context("/mavros/gpsstatus/gps1/raw",
                          [](const mavros_msgs::GPSRAW::ConstPtr& msg,
                             RobotContext& ctx) -> void {
                              ctx.gps_fix_type.store(msg->fix_type);
                          });

        ros->bind_context(
            "/mavros/sys_status",
            [](const mavros_msgs::SysStatus::ConstPtr& status,
               RobotContext& ctx) -> void {
                ctx.battery_remaining.store(status->battery_remaining);
            });

        ros->bind_context(
            "/mavros/global_position/raw/satellites",
            [](const std_msgs::UInt32::ConstPtr& msg,
               RobotContext& ctx) -> void { ctx.gps_nsats.store(msg->data); });

        ros->bind_event(
            "/mavros/sys_status",
            [](const mavros_msgs::SysStatus::ConstPtr status)
                -> SysStatusEvent { return {status->sensors_health}; });

        ros->bind_event(
            "/mavros/statustext/recv",
            [](mavros_msgs::StatusText::ConstPtr data) -> StatusTextEvent {
                return {data->text};
            });

        ros->bind_event(
            "/restart",
            [](std_msgs::String::ConstPtr data) -> RestartEvent { return {}; });

        ros->bind_event("/UAV0/perception/object_location/location_vel",
                        [](rsos_msgs::PointObj::ConstPtr msg) -> DetectEvent {
                            DetectEvent e;
                            e.score = msg->score;
                            e.target_pos.x() = msg->pos.x;
                            e.target_pos.y() = msg->pos.y;
                            e.target_pos.z() = msg->pos.z;
                            e.cmd_vel.x() = msg->velocity.x;
                            e.cmd_vel.y() = msg->velocity.y;
                            e.cmd_vel.z() = msg->velocity.z;
                            return e;
                        });
    }
    template <typename WebAdapter>
    static void setup(TagWeb, std::shared_ptr<WebAdapter>& web) {
        web->template register_route<PrearmEvent, EventResult>(
            boost::beast::http::verb::get, "/prearms", 5000);

        web->template register_route<TakeoffEvent, EventResult>(
            boost::beast::http::verb::post, "/takeoff", 5000);

        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/set_waypoint", 5000);

        web->template register_route<SetModeEvent, EventResult>(
            boost::beast::http::verb::post, "/set_mode", 5000);

        web->template register_route<SetModeEvent, EventResult>(
            boost::beast::http::verb::post, "/loiter", 5000,
            [](SetModeEvent& event) { event.mode = "LOITER"; });

        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/return", 5000,
            [](SetWaypointEvent& event) {
                event.finish_action = FinishAction::RETURN;
            });

        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/land", 5000,
            [](SetWaypointEvent& event) {
                event.finish_action = FinishAction::LAND;
            });

        web->template register_route<RebootFcuEvent, EventResult>(
            boost::beast::http::verb::post, "/reboot_fcu");

        web->template register_route<GetWpEvent, EventResult>(
            boost::beast::http::verb::get, "/get_waypoint");

        web->template register_route<GetGpsEvent, EventResult>(
            boost::beast::http::verb::get, "/get_gps");

        web->template register_route<GetParamEvent, EventResult>(
            boost::beast::http::verb::get, "/params");

        web->template register_route<SetParamEvent, EventResult>(
            boost::beast::http::verb::post, "/set_param");

        web->template register_route<DisarmEvent, EventResult>(
            boost::beast::http::verb::post, "/disarm");

        // 该接口只是调试时候使用
        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/pland", 5000,
            [](SetWaypointEvent& event) {
                event.land_target_id = 0;
                event.finish_action = FinishAction::LAND;
            });

        web->template register_route<SetPosVelEvent, EventResult>(
            boost::beast::http::verb::post, "/set_posvel", 5000);
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine);
};

#include "./event_listener.hpp"
inline void ControlFeature::setup(TagListeners,
                                  const std::shared_ptr<Engine>& engine) {
    auto listener = std::make_shared<ControlEventListener>();
    engine->add_listener(listener);
}