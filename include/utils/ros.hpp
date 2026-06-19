#include <ros/ros.h>
#include <spdlog/spdlog.h>

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
