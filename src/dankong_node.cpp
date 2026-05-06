#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/SysStatus.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

#include <chrono>
#include <memory>

#include "dk/adapters/ros.hpp"
#include "dk/adapters/web.hpp"
#include "dk/core.hpp"
#include "dk/logger.hpp"
#include "dk_auto_json.hpp"
#include "event_listener.hpp"
#include "mavlink/mavros.hpp"
#include "robot/drone.hpp"
#include "robot_context.hpp"
#include "robot_event.hpp"
#include "states/ground_state.hpp"
#include "states/init_state.hpp"

class Engine : public dk::BaseEngine<RobotEvent, RobotContext, Engine> {};

class DKNode {
    using RosAdapterType = dk::RosAdapter<RobotEvent, RobotContext, Engine>;
    using WebAdapterType = dk::WebAdapter<RobotEvent, RobotContext, Engine>;

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
        ros_adapter_->bind_context("/mavros/local_position/odom",
                                   [](const nav_msgs::Odometry::ConstPtr& odom, RobotContext& ctx) -> void {
                                       auto pos = odom->pose.pose.position;
                                       ctx.pos.set({pos.x, pos.y, pos.z});
                                   });
        ros_adapter_->bind_event(
            "/mavros/sys_status",
            [](const mavros_msgs::SysStatus::ConstPtr status) -> SysStatusEvent { return {status->sensors_health}; });

        ros_adapter_->bind_event(
            "/mavros/statustext/recv",
            [](mavros_msgs::StatusText::ConstPtr data) -> StatusTextEvent { return {data->text}; });
    }

    void setup_web_adapter() {
        web_adapter_ = std::make_shared<WebAdapterType>(engine_, 8000);
        dk::WsEndpoint endpoint;
        web_adapter_->register_ws_route("/ws", endpoint);
        web_adapter_->register_route<PrearmEvent, EventResult>(boost::beast::http::verb::get, "/prearms", 5000);
        web_adapter_->register_route<TakeoffEvent, EventResult>(boost::beast::http::verb::post, "/takeoff", 5000);
    }

    void setup_evente_listener() {
        auto event_listener = std::make_shared<EventListener>();
        engine_->add_listener(event_listener);
    }

   public:
    DKNode() : nh_(), pnh_("~") {
        engine_ = std::make_shared<Engine>();
        setup_ros_adapter();
        setup_web_adapter();
        setup_ros_config();
        setup_evente_listener();
        engine_->get_context().robot = std::make_shared<Drone>();
        engine_->get_context().mavlink = std::make_shared<MavRos>();
        engine_->get_context().mavlink->set_stream_rate(10);
        engine_->start(InitState::instance(), std::chrono::milliseconds(50));
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