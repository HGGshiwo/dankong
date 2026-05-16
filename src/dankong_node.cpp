#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/SysStatus.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rsos_msgs/PointObj.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <boost/beast/http/verb.hpp>
#include <chrono>
#include <dk_auto_json.hpp>
#include <memory>

#include "./utils.hpp"
#include "dk/adapters/ros.hpp"
#include "dk/adapters/web.hpp"
#include "dk/engine.hpp"
#include "dk/logger.hpp"
#include "dk/utils.hpp"
#include "event_listener/control_event_listener.hpp"
#include "event_listener/report_event_listener.hpp"
#include "event_listener/ros_event_listener.hpp"
#include "mavlink/mavros.hpp"
#include "robot/drone.hpp"
#include "robot_context.hpp"
#include "robot_event.hpp"
#include "states/ground_state.hpp"
#include "states/init_state.hpp"
#include "states/state_utils.hpp"

namespace fs = boost::filesystem;
const std::string STAIC_DIR =
    "/home/hgg/catkin_ws/src/dankong/dk/frontend/dist";
const std::string UI_CONFIG_PATH =
    "/home/hgg/catkin_ws/src/dankong/config/ui.yaml";
const std::string JSON_PATH = (fs::current_path() / "config.json").string();

class Engine : public dk::BaseEngine<RobotContext, Engine> {};

class DKNode {
    using RosAdapterType = dk::RosAdapter<RobotContext, Engine>;
    using WebAdapterType = dk::WebAdapter<RobotContext, Engine>;

   private:
    std::shared_ptr<Engine> engine_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::shared_ptr<RosAdapterType> ros_adapter_;
    std::shared_ptr<WebAdapterType> web_adapter_;

   private:
    void setup_ros_config() {}

    void setup_ros_adapter() {
        ros_adapter_ = std::make_shared<RosAdapterType>(engine_, nh_);
        ros_adapter_->bind_context(
            "/mavros/state",
            [](const mavros_msgs::State::ConstPtr& state_ptr,
               RobotContext& ctx) -> void {
                auto armed = ctx.arm.get();
                auto cur_armed = state_ptr->armed != 0;
                if (armed != cur_armed) {
                    ctx.arm.set(cur_armed);
                    ctx.engine->dispatch_internal(ArmEvent{cur_armed});
                }
                if (ctx.mode.get() != state_ptr->mode) {
                    ctx.engine->dispatch_internal(
                        FlightModeEvent{ctx.mode.get(), state_ptr->mode});
                    ctx.mode.set(state_ptr->mode);
                }
                auto connected = state_ptr->connected != 0;
                if (ctx.fcu_connected.get() != connected) {
                    ctx.fcu_connected.set(connected);
                    ctx.engine->dispatch_internal(FcuConnectedEvent{connected});
                }
            });

        ros_adapter_->bind_context(
            "/mavros/vfr_hud",
            [](const mavros_msgs::VFR_HUD::ConstPtr& vfr_hud_msg,
               RobotContext& ctx) -> auto {
                ctx.throttle = vfr_hud_msg->throttle;
            });

        ros_adapter_->bind_context(
            "/mavros/distance_sensor/rangefinder_pub",
            [](const sensor_msgs::Range::ConstPtr& range_msg, RobotContext& ctx)
                -> auto { ctx.rangefinder_alt = range_msg->range; });

        ros_adapter_->bind_context(
            "/mavros/local_position/odom",
            [](const nav_msgs::Odometry::ConstPtr& odom,
               RobotContext& ctx) -> void {
                auto pos = odom->pose.pose.position;
                ctx.pos_enu.set({pos.x, pos.y, pos.z});

                auto orientation = odom->pose.pose.orientation;
                auto yaw_enu = state_utils::orientation_to_euler(
                                   orientation.x, orientation.y, orientation.z,
                                   orientation.w)
                                   .z();

                ctx.yaw_enu = yaw_enu;
                ctx.yaw_ned.set(state_utils::yaw_enu_to_ned(yaw_enu));
            });

        ros_adapter_->bind_context(
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
        ros_adapter_->bind_context(
            "/mavros/global_position/rel_alt",
            [](const std_msgs::Float64::ConstPtr& msg,
               RobotContext& ctx) -> void {
                ctx.lon_lat_alt.write(
                    [msg](Eigen::Vector3d& data) { data.z() = msg->data; });
            });

        ros_adapter_->bind_context("/mavros/gpsstatus/gps1/raw",
                                   [](const mavros_msgs::GPSRAW::ConstPtr& msg,
                                      RobotContext& ctx) -> void {
                                       ctx.gps_fix_type.set(msg->fix_type);
                                   });

        ros_adapter_->bind_context(
            "/mavros/sys_status",
            [](const mavros_msgs::SysStatus::ConstPtr& status,
               RobotContext& ctx) -> void {
                ctx.battery_remaining.set(status->battery_remaining);
            });

        ros_adapter_->bind_context(
            "/mavros/global_position/raw/satellites",
            [](const std_msgs::UInt32::ConstPtr& msg,
               RobotContext& ctx) -> void { ctx.gps_nsats.set(msg->data); });

        ros_adapter_->bind_event(
            "/mavros/sys_status",
            [](const mavros_msgs::SysStatus::ConstPtr status)
                -> SysStatusEvent { return {status->sensors_health}; });

        ros_adapter_->bind_event(
            "/mavros/statustext/recv",
            [](mavros_msgs::StatusText::ConstPtr data) -> StatusTextEvent {
                return {data->text};
            });

        ros_adapter_->bind_event(
            "/restart",
            [](std_msgs::String::ConstPtr data) -> RestartEvent { return {}; });

        ros_adapter_->bind_event(
            "/UAV0/perception/object_location/location_vel",
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

    void setup_web_adapter() {
        // 生成配置文件
        const std::string json_path =
            (fs::current_path() / "config.json").string();
        dk::generate_json_file(UI_CONFIG_PATH, JSON_PATH);

        web_adapter_ = std::make_shared<WebAdapterType>(engine_, 8000);
        web_adapter_->enable_cors();

        engine_->get_context().ws_manager = web_adapter_->get_manager();

        web_adapter_->register_file_route(boost::beast::http::verb::get,
                                          "/page_config", JSON_PATH);
        web_adapter_->register_static_dir("/", STAIC_DIR);
        web_adapter_->register_static_dir("/home", STAIC_DIR);
        web_adapter_->register_managed_ws_route("/ws", [](auto, auto&) {});
        web_adapter_->register_route<PrearmEvent, EventResult>(
            boost::beast::http::verb::get, "/prearms", 5000);
        web_adapter_->register_route<TakeoffEvent, EventResult>(
            boost::beast::http::verb::post, "/takeoff", 5000);
        web_adapter_->register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/set_waypoint", 5000);
        web_adapter_->register_route<SetModeEvent, EventResult>(
            boost::beast::http::verb::post, "/set_mode", 5000);
        web_adapter_->register_route<SetModeEvent, EventResult>(
            boost::beast::http::verb::post, "/loiter", 5000,
            [](SetModeEvent& event) { event.mode = "LOITER"; });
        web_adapter_->register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/return", 5000,
            [](SetWaypointEvent& event) {
                event.finish_action = FinishAction::RETURN;
            });
        web_adapter_->register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/land", 5000,
            [](SetWaypointEvent& event) {
                event.finish_action = FinishAction::LAND;
            });

        web_adapter_->register_route<RebootFcuEvent, EventResult>(
            boost::beast::http::verb::post, "/reboot_fcu");

        web_adapter_->register_route<GetWpEvent, EventResult>(
            boost::beast::http::verb::get, "/get_waypoint");

        web_adapter_->register_route<GetGpsEvent, EventResult>(
            boost::beast::http::verb::get, "/get_gps");

        web_adapter_->register_route<StopFollowEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_follow");

        web_adapter_->register_route<EnablePlandEvent, EventResult>(
            boost::beast::http::verb::post, "/start_pland");

        web_adapter_->register_route<DisablePlandEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_pland");

        web_adapter_->register_route<EnablePlannerEvent, EventResult>(
            boost::beast::http::verb::post, "/start_planner");

        web_adapter_->register_route<DisablePlannerEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_planner");

        web_adapter_->register_route<StartRecordEvent, EventResult>(
            boost::beast::http::verb::post, "/start_record");

        web_adapter_->register_route<StopRecordEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_record");

        web_adapter_->register_route<GetGimbalEvent, EventResult>(
            boost::beast::http::verb::get, "/get_gimbal");

        web_adapter_->register_route<SetGimbalEvent, EventResult>(
            boost::beast::http::verb::post, "/set_gimbal");

        web_adapter_->register_route<GetExposureEvent, EventResult>(
            boost::beast::http::verb::get, "/get_exposure");

        web_adapter_->register_route<SetExposureEvent, EventResult>(
            boost::beast::http::verb::post, "/set_exposure");

        web_adapter_->register_route<GetParamEvent, EventResult>(
            boost::beast::http::verb::get, "/params");

        web_adapter_->register_route<SetParamEvent, EventResult>(
            boost::beast::http::verb::post, "/set_param");

        web_adapter_->register_route<DisarmEvent, EventResult>(
            boost::beast::http::verb::post, "/disarm");

        // 该接口只是调试时候使用
        web_adapter_->register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/pland", 5000,
            [](SetWaypointEvent& event) {
                event.land_target_id = 0;
                event.finish_action = FinishAction::LAND;
            });

        web_adapter_->register_route<EnableDetectEvent, EventResult>(
            boost::beast::http::verb::post, "/start_detect");

        web_adapter_->register_route<DisableDetectEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_detect");

        web_adapter_->register_route<GetDetectEvent, EventResult>(
            boost::beast::http::verb::get, "/get_detect");

        web_adapter_->register_route<SetPosVelEvent, EventResult>(
            boost::beast::http::verb::post, "/set_posvel", 5000);
    }

    void setup_event_listener() {
        auto control_event_listener = std::make_shared<ControlEventListener>();
        engine_->add_listener(control_event_listener);

        auto report_listener =
            std::make_shared<ReportEventListener>(web_adapter_->get_manager());
        engine_->add_listener(report_listener);

        auto ros_event_listener = std::make_shared<RosEventListener>();
        engine_->add_listener(ros_event_listener);
    }

   public:
    using AllowedEvents = std::tuple<>();

    DKNode() : nh_(), pnh_("~") {
        engine_ = std::make_shared<Engine>();
        setup_ros_adapter();
        setup_web_adapter();
        setup_ros_config();
        setup_event_listener();
        engine_->get_context().robot =
            std::make_shared<Drone>(std::make_shared<MavRos>());
        engine_->get_context().engine = engine_;
        engine_->start<InitState>(std::chrono::milliseconds(50));
    }

    ~DKNode() { engine_->stop(); }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dk_node");
    dk::init_logger();
    DKNode node;

    // 启动 ROS 异步线程池 (处理网络 IO)
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // 阻塞主线程
    ros::waitForShutdown();
    return 0;
}