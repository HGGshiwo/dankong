#pragma once
#include <string>

#include "nlohmann/json.hpp"

// @JSON_ENABLE
struct EventResult {
    std::string status = "OK";
    nlohmann::json msg;
};