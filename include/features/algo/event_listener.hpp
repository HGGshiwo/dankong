#pragma once

#include <ros/ros.h>
#include <rsos_msgs/SetCameraExposure.h>
#include <rsos_msgs/SetGimbalAngle.h>
#include <rsos_msgs/StartBagRecord.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <unordered_map>

#include "./events.hpp"
#include "context_config.hpp"
#include "dk/event_listener.hpp"
#include "mavlink/mavros.hpp"
#include "nlohmann/json.hpp"

// 和ROS相关的操作的事件走这个监听器
class AlgoEventListener
    : public dk::BaseEventListener<RobotContext, AlgoEventListener> {
   private:
    ros::NodeHandle nh_;
    std::shared_ptr<ServiceClient<rsos_msgs::StartBagRecord>>
        start_record_client_;
    std::shared_ptr<ServiceClient<std_srvs::Trigger>> stop_record_client_;

    std::shared_ptr<ServiceClient<rsos_msgs::SetGimbalAngle>>
        set_gimbal_client_;

    std::shared_ptr<ServiceClient<rsos_msgs::SetCameraExposure>>
        set_exposure_client_;

    std::unordered_map<std::string, std::string> detect_map_ = {
        {"nohardhat", "/UAV0/perception/yolo_detection/enable_detection"},
        {"smoke", "/UAV0/perception/yolo_detection_smoke/enable_detection"}};

   public:
    using AllowedEvents =
        std::tuple<EnableDetectEvent, DisableDetectEvent, GetDetectEvent,
                   StartRecordEvent, StopRecordEvent, SetGimbalEvent,
                   GetGimbalEvent, GetExposureEvent, SetExposureEvent,
                   DetectEvent, StopFollowEvent, EnablePlandEvent,
                   DisablePlandEvent, EnablePlannerEvent, DisablePlannerEvent>;

    AlgoEventListener();

    void on_event(const EnableDetectEvent& event, RobotContext& ctx);
    void on_event(const DisableDetectEvent& event, RobotContext& ctx);
    void on_event(const GetDetectEvent& event, RobotContext& ctx);
    void on_event(const StartRecordEvent& event, RobotContext& ctx);
    void on_event(const StopRecordEvent& event, RobotContext& ctx);
    void on_event(const GetGimbalEvent& event, RobotContext& ctx);
    void on_event(const SetGimbalEvent& event, RobotContext& ctx);
    void on_event(const GetExposureEvent& event, RobotContext& ctx);
    void on_event(const SetExposureEvent& event, RobotContext& ctx);
    void on_event(const DetectEvent& event, RobotContext& ctx);
    void on_event(const StopFollowEvent& event, RobotContext& ctx);
    void on_event(const EnablePlandEvent& event, RobotContext& ctx);
    void on_event(const DisablePlandEvent& event, RobotContext& ctx);
    void on_event(const EnablePlannerEvent& event, RobotContext& ctx);
    void on_event(const DisablePlannerEvent& event, RobotContext& ctx);
};
