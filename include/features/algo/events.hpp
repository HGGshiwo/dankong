#pragma once
#include <Eigen/Dense>

#include "core/event_result.hpp"
#include "dk/engine.hpp"

//@JSON_ENABLE
struct StopFollowEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct EnablePlannerEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct DisablePlannerEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct EnableDetectEvent : dk::AsyncEvent<EventResult> {
    std::string type;
};  // 开始检测

// @JSON_ENABLE
struct DisableDetectEvent : dk::AsyncEvent<EventResult> {};  // 结束检测

//@JSON_ENABLE
struct GetDetectEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct StartRecordEvent : dk::AsyncEvent<EventResult> {
    std::string bag_name;
};  // 开始录制

// @JSON_ENABLE
struct StopRecordEvent : dk::AsyncEvent<EventResult> {};  // 结束录制

// @JSON_ENABLE
struct GetGimbalEvent : dk::AsyncEvent<EventResult> {};  // 获取云台数据

// @JSON_ENABLE
struct SetGimbalEvent : dk::AsyncEvent<EventResult> {
    std::string mode;
    double angle;
};  // 设置云台

//@JSON_ENABLE
struct SetExposureEvent : dk::AsyncEvent<EventResult> {
    double shutter;
    double sensitivity;
};  // 设置曝光

//@JSON_ENABLE
struct GetExposureEvent : dk::AsyncEvent<EventResult> {};

struct DetectEvent {
    double score;
    Eigen::Vector3d target_pos;
    Eigen::Vector3d cmd_vel;
};

struct DetectTargetEvent {
    Eigen::Vector3d pos;
};

struct ExcutePlandEvent {
    int target_tag_id;
};