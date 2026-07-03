#pragma once
#include "core/event_result.hpp"

#ifdef USE_ROS
#include "dk/adapters/ros.hpp"
#endif

#ifdef USE_ROS1
#include <rsos_msgs/PointObj.h>
#include <std_msgs/String.h>
#endif

#include <Eigen/Dense>
#include <memory>

#include "./context.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "features/control/events.hpp"
#include "robot_context.hpp"
#include "states/state_utils.hpp"

class ControlFeature {
   public:
#ifdef USE_ROS
    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
#ifdef USE_ROS1
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

#endif
    }
#endif

    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        web->template register_route<PrearmEvent, EventResult>(
            boost::beast::http::verb::get, "/prearms", 5000);

        web->template register_route<TakeoffEvent, EventResult>(
            boost::beast::http::verb::post, "/takeoff", 5000);

        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/set_waypoint", 5000);

        web->template register_route<SetModeEvent, EventResult>(
            boost::beast::http::verb::post, "/set_mode", 5000);

        // Web 端发送 LOITER 模式，APM 完全兼容这个名称
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

        web->template register_route<TestEvent, EventResult>(
            boost::beast::http::verb::post, "/test", 5000);

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

        web->template register_route<SetPosVelEvent, EventResult>(
            boost::beast::http::verb::post, "/set_posvel", 5000);

        web->template register_route<JoystickEvent, EventResult>(
            boost::beast::http::verb::post, "/cmd_vel", 5000);

        web->template register_route<EnableJoystickEvent, EventResult>(
            boost::beast::http::verb::post, "/joystick/enable", 5000);
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine);
};

#include "./event_listener.hpp"
inline void ControlFeature::setup(TagListeners,
                                  const std::shared_ptr<Engine>& engine) {
    auto listener = std::make_shared<ControlEventListener>();
    engine->add_listener(listener);
}