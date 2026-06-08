#pragma once
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/Param.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/StreamRate.h>
#include <memory.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <spdlog/spdlog.h>

#include <functional>
#include <memory>

#include "core/global_config.hpp"
#include "imavlink.hpp"
#include "mavros_msgs/MessageInterval.h"

template <typename MsgType>
class ServiceClient {
    ros::NodeHandle nh_;
    ros::ServiceClient srv_client_;
    std::string srv_name_;
    std::function<bool(MsgType)> check_;

   public:
    ServiceClient(std::string srv_name, std::function<bool(MsgType)> check)
        : srv_name_(srv_name), check_(check) {
        srv_client_ = nh_.serviceClient<MsgType>(srv_name);
    }

    bool call(MsgType& srv) {
        // if (!ros::service::waitForService(srv_name_, ros::Duration(3.0))) {
        //     spdlog::error("Service {} not available after 3 seconds",
        //     srv_name_); return false;
        // }
        if (!srv_client_.call(srv)) {
            spdlog::error("{} service call failed!", srv_name_);
            return false;
        }
        if (check_(srv)) {
            spdlog::info("{} service call success!", srv_name_);
            return true;
        }
        spdlog::info("{} service call return false!", srv_name_);
        return false;
    }
};

class MavRos : public IMavlink {
    ros::NodeHandle nh_;

    nlohmann::json pdef_;

    std::shared_ptr<ServiceClient<mavros_msgs::CommandLong>> cmd_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::SetMode>> set_mode_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::StreamRate>> set_rate_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::MessageInterval>>
        set_msg_interval_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::CommandTOL>> takeoff_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::CommandBool>> arm_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::ParamPull>> param_pull_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::ParamSet>> param_set_client_;

    std::shared_ptr<ros::Publisher> setpoint_pub_;
    ros::Subscriber param_sub_;

   private:
    void load_pdef(const std::string& path);

   public:
    MavRos() {
        cmd_client_ = std::make_shared<ServiceClient<mavros_msgs::CommandLong>>(
            "/mavros/cmd/command", [](mavros_msgs::CommandLong srv) -> bool {
                return srv.response.success;
            });

        set_mode_client_ =
            std::make_shared<ServiceClient<mavros_msgs::SetMode>>(
                "/mavros/set_mode", [](mavros_msgs::SetMode srv) -> bool {
                    return srv.response.mode_sent != 0;
                });

        set_rate_client_ =
            std::make_shared<ServiceClient<mavros_msgs::StreamRate>>(
                "/mavros/set_stream_rate",
                [](mavros_msgs::StreamRate srv) -> bool { return true; });

        set_msg_interval_client_ =
            std::make_shared<ServiceClient<mavros_msgs::MessageInterval>>(
                "/mavros/set_message_interval",
                [](mavros_msgs::MessageInterval srv) -> bool { return true; });

        takeoff_client_ =
            std::make_shared<ServiceClient<mavros_msgs::CommandTOL>>(
                "/mavros/cmd/takeoff", [](mavros_msgs::CommandTOL srv) -> bool {
                    spdlog::info("takeoff result: {}", srv.response.result);
                    return srv.response.success;
                });

        arm_client_ = std::make_shared<ServiceClient<mavros_msgs::CommandBool>>(
            "/mavros/cmd/arming", [](mavros_msgs::CommandBool srv) -> bool {
                return srv.response.success;
            });

        param_pull_client_ =
            std::make_shared<ServiceClient<mavros_msgs::ParamPull>>(
                "mavros/param/pull", [](mavros_msgs::ParamPull srv) -> bool {
                    return srv.response.success;
                });

        param_set_client_ =
            std::make_shared<ServiceClient<mavros_msgs::ParamSet>>(
                "mavros/param/set", [](mavros_msgs::ParamSet srv) -> bool {
                    return srv.response.success;
                });

        setpoint_pub_ = std::make_shared<ros::Publisher>(
            nh_.advertise<mavros_msgs::PositionTarget>(
                "/mavros/setpoint_raw/local", 1));

        // 需要把topic的数据更新到参数服务器，之后可以直接用
        param_sub_ = nh_.subscribe<mavros_msgs::Param>(
            "/mavros/param/param_value", 1, &MavRos::param_callback, this);

        load_pdef(GlobalConfig.GetConfig().pdef_path);
    }

    bool arm() override;

    bool disarm() override;

    bool takeoff(double alt) override;

    bool set_mode(const FixedString64& mode) override;

    // MAV_CMD_RUN_PREARM_CHECKS
    bool run_prearm_checks() override;

    bool set_stream_rate(int stream_id, int rate) override;

    bool set_msg_interval(int stream_id, int rate) override;

    ApmParam get_param(const std::string& name, const ApmParam& value) override;

    bool set_param(const std::string& name, const ApmParam& value) override;

    bool pull_params() override;

    nlohmann::json get_all_params() override;  // 加载全部参数

    bool reboot_fcu() override;

    bool cmd_vel(Eigen::Vector4d vel) override;

    void param_callback(const mavros_msgs::Param::ConstPtr& msg);
};