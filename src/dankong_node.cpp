#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/SysStatus.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <chrono>
#include <memory>

#include "dk/adapters/ros.hpp"
#include "dk/adapters/web.hpp"
#include "dk/engine.hpp"
#include "dk/logger.hpp"
#include "dk_auto_json.hpp"
#include "event_listener.hpp"
#include "mavlink/mavros.hpp"
#include "robot/drone.hpp"
#include "robot_context.hpp"
#include "robot_event.hpp"
#include "states/ground_state.hpp"
#include "states/init_state.hpp"
#include "states/state_utils.hpp"

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
        ros_adapter_->bind_context("/mavros/state",
                                   [](const mavros_msgs::State::ConstPtr& state_ptr, RobotContext& ctx) -> void {
                                       ctx.arm = state_ptr->armed;
                                       ctx.mode = state_ptr->mode;
                                   });
        ros_adapter_->bind_context("/mavros/vfr_hud",
                                   [](const mavros_msgs::VFR_HUD::ConstPtr& vfr_hud_msg, RobotContext& ctx) -> auto {
                                       ctx.throttle = vfr_hud_msg->throttle;
                                   });
        ros_adapter_->bind_context("/mavros/distance_sensor/rangefinder_pub",
                                   [](const sensor_msgs::Range::ConstPtr& range_msg, RobotContext& ctx) -> auto {
                                       ctx.rangefinder_alt = range_msg->range;
                                   });
        ros_adapter_->bind_context(
            "/mavros/local_position/odom", [](const nav_msgs::Odometry::ConstPtr& odom, RobotContext& ctx) -> void {
                auto pos = odom->pose.pose.position;
                ctx.pos_enu.set({pos.x, pos.y, pos.z});

                auto orientation = odom->pose.pose.orientation;
                auto yaw_enu =
                    state_utils::orientation_to_euler(orientation.x, orientation.y, orientation.z, orientation.w).z();

                ctx.yaw_enu = yaw_enu;
                ctx.yaw_ned = state_utils::yaw_enu_to_ned(yaw_enu);
            });
        ros_adapter_->bind_context("/mavros/global_position/global",
                                   [](const sensor_msgs::NavSatFix::ConstPtr& msg, RobotContext& ctx) -> void {
                                       ctx.lon_lat_alt.write([msg](Eigen::Vector3d& data) {
                                           data.x() = msg->longitude;
                                           data.y() = msg->latitude;
                                       });
                                       ctx.odom_ok = true;
                                   });
        ros_adapter_->bind_context("/mavros/global_position/rel_alt",
                                   [](const std_msgs::Float64::ConstPtr& msg, RobotContext& ctx) -> void {
                                       ctx.lon_lat_alt.write([msg](Eigen::Vector3d& data) { data.z() = msg->data; });
                                   });
        ros_adapter_->bind_event(
            "/mavros/sys_status",
            [](const mavros_msgs::SysStatus::ConstPtr status) -> SysStatusEvent { return {status->sensors_health}; });

        ros_adapter_->bind_event(
            "/mavros/statustext/recv",
            [](mavros_msgs::StatusText::ConstPtr data) -> StatusTextEvent { return {data->text}; });

        ros_adapter_->bind_event("/mavros/state",
                                 [](mavros_msgs::State::ConstPtr data) -> ArmEvent { return {data->armed != 0}; });
    }

    void setup_web_adapter() {
        web_adapter_ = std::make_shared<WebAdapterType>(engine_, 8000);
        dk::WsEndpoint endpoint;
        web_adapter_->register_ws_route("/ws", endpoint);
        web_adapter_->register_route<PrearmEvent, EventResult>(boost::beast::http::verb::get, "/prearms", 5000);
        web_adapter_->register_route<TakeoffEvent, EventResult>(boost::beast::http::verb::post, "/takeoff", 5000);
        web_adapter_->register_route<SetWaypointEvent, EventResult>(boost::beast::http::verb::post, "/set_waypoint",
                                                                    5000);
    }

    void setup_evente_listener() {
        auto event_listener = std::make_shared<EventListener>();
        engine_->add_listener(event_listener);
    }

   public:
    using AllowedEvents = std::tuple<>();

    DKNode() : nh_(), pnh_("~") {
        engine_ = std::make_shared<Engine>();
        setup_ros_adapter();
        setup_web_adapter();
        setup_ros_config();
        setup_evente_listener();
        engine_->get_context().robot = std::make_shared<Drone<MavRos>>();
        engine_->get_context().robot->set_stream_rate(10);
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