#pragma once
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/StreamRate.h>
#include <memory.h>
#include <ros/ros.h>

#include <functional>
#include <memory>

#include "./imavlink.hpp"
#include "dk/logger.hpp"
#include "spdlog/spdlog.h"

template <typename MsgType>
class ServiceClient {
    ros::NodeHandle nh_;
    ros::ServiceClient srv_client_;
    std::string srv_name_;
    std::function<bool(MsgType)> check_;

   public:
    ServiceClient(std::string srv_name, std::function<bool(MsgType)> check) : srv_name_(srv_name), check_(check) {
        srv_client_ = nh_.serviceClient<MsgType>(srv_name);
    }

    bool call(MsgType& srv) {
        // if (!ros::service::waitForService(srv_name_, ros::Duration(3.0))) {
        //     spdlog::error("Service {} not available after 3 seconds", srv_name_);
        //     return false;
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
    std::shared_ptr<ServiceClient<mavros_msgs::CommandLong>> cmd_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::SetMode>> set_mode_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::StreamRate>> set_rate_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::CommandTOL>> takeoff_client_;
    std::shared_ptr<ServiceClient<mavros_msgs::CommandBool>> arm_client_;

   public:
    MavRos() {
        cmd_client_ = std::make_shared<ServiceClient<mavros_msgs::CommandLong>>(
            "/mavros/cmd/command", [](mavros_msgs::CommandLong srv) -> bool { return srv.response.success; });
        set_mode_client_ = std::make_shared<ServiceClient<mavros_msgs::SetMode>>(
            "/mavros/set_mode", [](mavros_msgs::SetMode srv) -> bool { return srv.response.mode_sent != 0; });
        set_rate_client_ = std::make_shared<ServiceClient<mavros_msgs::StreamRate>>(
            "/mavros/set_stream_rate", [](mavros_msgs::StreamRate srv) -> bool { return true; });

        takeoff_client_ = std::make_shared<ServiceClient<mavros_msgs::CommandTOL>>(
            "/mavros/cmd/takeoff", [](mavros_msgs::CommandTOL srv) -> bool { return srv.response.success; });
        arm_client_ = std::make_shared<ServiceClient<mavros_msgs::CommandBool>>(
            "/mavros/cmd/arming", [](mavros_msgs::CommandBool srv) -> bool { return srv.response.success; });
    }

    bool arm() override;

    bool takeoff(double alt) override;

    bool land() override;

    bool set_mode(const FixedString64& mode) override;

    // MAV_CMD_RUN_PREARM_CHECKS
    bool run_prearm_checks() override;

    bool set_stream_rate(int rate) override;

    ApmParam get_param(std::string name, ApmParam value = 0) override;
};