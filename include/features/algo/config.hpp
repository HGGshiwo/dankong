#pragma once
#ifdef USE_ROS
#include <string>
#include <unordered_map>

#include "utils/config_param.hpp"

typedef std::unordered_map<std::string, std::string> DetectMap;

//@JSON_ENABLE
struct AlgoConfig {
    static constexpr const char* __group_name = "Algo";

    dk::Param<DetectMap> detect_map = {
        (DetectMap{
            {"nohardhat", "/UAV0/perception/yolo_detection/enable_detection"},
            {"smoke",
             "/UAV0/perception/yolo_detection_smoke/enable_detection"}}),
        "detect_map",
        "检测映射表",
        "配置检测类型到ROS参数的映射关系",
        __group_name,
        false};
};
#endif