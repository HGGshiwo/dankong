#pragma once
#include <memory>

#include "utils/dirty_var.hpp"
#include "utils/state_registry.hpp"

struct ReportContext {
    nlohmann::json taskId;
    nlohmann::json serialNo;
    std::optional<nlohmann::json> mapCode;

    ReportContext() = default;
    explicit ReportContext(StateRegistry& reg) {}
};