#pragma once
#include <optional>
#include <vector>

#include "Eigen/Dense"
#include "nlohmann/json.hpp"

// @JSON_ENABLE
struct TaskEvent {
    nlohmann::json deviceId;
    nlohmann::json pathEventInfo;
    std::vector<Eigen::Vector3d> pathInfo;
    std::optional<nlohmann::json> mapCode = std::nullopt;
    nlohmann::json serialNo;
    nlohmann::json taskId;
};

struct TaskProgressEvent {
    int cur;
    int total;
};

struct TaskDoneEvent {};

// @JSON_ENABLE
struct SpeakEvent {};

// @JSON_ENABLE
struct VideoEvent {};

// @JSON_ENABLE
struct RegisterEvent {
    std::string deviceCode;
};