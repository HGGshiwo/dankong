#pragma once
#ifdef USE_ROS
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/config_param.hpp"

typedef std::unordered_map<std::string, std::string> DetectMap;

//@JSON_ENABLE
struct AlgoConfig {
    static constexpr const char* __group_name = "Algo";

    // 注意: 字段初始化器统一用 INIT_PARAM 宏 (以 ");" 结尾)。
    // gen_json.py 的正则会把字段结尾的大括号加分号 (右大括号紧贴分号)
    // 误认为 struct 的闭合, 导致该字段以及其后的字段全部丢失序列化
    dk::Param<DetectMap> detect_map = INIT_PARAM(
        "detect_map",
        (DetectMap{
            {"nohardhat", "/UAV0/perception/yolo_detection/enable_detection"},
            {"smoke",
             "/UAV0/perception/yolo_detection_smoke/enable_detection"}}),
        "配置检测类型到ROS参数的映射关系");

    dk::Param<std::string> record_path =
        INIT_PARAM("record_path", "bags",
                   "rosbag 录制文件的输出目录 (不配置时默认 ~/.ros/bags, "
                   "配置绝对路径则直接使用)");

    dk::Param<std::string> record_compression = INIT_PARAM(
        "record_compression", "lz4",
        "bag 压缩方式: lz4 (快, 实时推荐) / bz2 (压缩率高但慢, 高码率会丢帧) / "
        "其他值 = 不压缩");

    dk::Param<int> record_split_mb = INIT_PARAM(
        "record_split_mb", 0,
        "bag 分片大小 (MB), 0 = 不分片; 录制内容大时防止单文件过大/磁盘写满");

    dk::Param<std::vector<std::string>> record_topics =
        INIT_PARAM("record_topics", {},
                   "开始录制时订阅并写入 bag 的话题, 为空则无法开始录制");
};
#endif