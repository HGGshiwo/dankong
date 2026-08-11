#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <stdexcept>

#include "utils/fixed_string64.hpp"

struct FlightMode {
    mavsdk::Telemetry::FlightMode mode_raw =
        mavsdk::Telemetry::FlightMode::Unknown;
    FixedString64 mode_str = "UNKNOWN";
    bool operator==(const FlightMode& other) const {
        return mode_raw == other.mode_raw;
    }

    bool operator!=(const FlightMode& other) const { return !(*this == other); }

    FlightMode() {};

    FlightMode(mavsdk::Telemetry::FlightMode mode) {
        mode_str = mode_raw_to_string(mode);
        mode_raw = mode;
    }

    static FlightMode create(std::string str) {
        auto mode = string_to_mode_raw(str);
        if (!mode.has_value()) {
            throw std::runtime_error("Unknow Mode: " + str);
        }
        return FlightMode(mode.value());
    }

    // ---------- 静态转换辅助函数 ----------
    static std::string mode_raw_to_string(mavsdk::Telemetry::FlightMode mode) {
        switch (mode) {
            case mavsdk::Telemetry::FlightMode::Unknown:
                return "UNKNOWN";
            case mavsdk::Telemetry::FlightMode::Ready:
                return "READY";
            case mavsdk::Telemetry::FlightMode::Takeoff:
                return "GUIDED";
            case mavsdk::Telemetry::FlightMode::Hold:
                return "LOITER";
            case mavsdk::Telemetry::FlightMode::Mission:
                return "AUTO";
            case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
                return "RTL";
            case mavsdk::Telemetry::FlightMode::Land:
                return "LAND";
            case mavsdk::Telemetry::FlightMode::Offboard:
                return "GUIDED";
            case mavsdk::Telemetry::FlightMode::FollowMe:
                return "FOLLOW";
            case mavsdk::Telemetry::FlightMode::Manual:
                return "MANUAL";
            case mavsdk::Telemetry::FlightMode::Altctl:
                return "ALT_HOLD";
            case mavsdk::Telemetry::FlightMode::Posctl:
                return "POSHOLD";
            case mavsdk::Telemetry::FlightMode::Acro:
                return "ACRO";
            case mavsdk::Telemetry::FlightMode::Stabilized:
                return "STABILIZE";
            default:
                return "UNKNOWN";
        }
    }

    static std::optional<mavsdk::Telemetry::FlightMode> string_to_mode_raw(
        const std::string& str) {
        // 统一转为大写比较
        std::string upper = str;
        for (auto& c : upper) c = std::toupper(c);

        // 注意：多个枚举可映射到同一字符串（如 "GUIDED" 对应 Takeoff 和
        // Offboard） 这里按优先级返回第一个匹配，你可以根据需求调整
        static const std::unordered_map<std::string,
                                        mavsdk::Telemetry::FlightMode>
            map = {
                {"UNKNOWN", mavsdk::Telemetry::FlightMode::Unknown},
                {"READY", mavsdk::Telemetry::FlightMode::Ready},
                {"GUIDED", mavsdk::Telemetry::FlightMode::Offboard},
                {"LOITER", mavsdk::Telemetry::FlightMode::Hold},
                {"AUTO", mavsdk::Telemetry::FlightMode::Mission},
                {"RTL", mavsdk::Telemetry::FlightMode::ReturnToLaunch},
                {"LAND", mavsdk::Telemetry::FlightMode::Land},
                {"FOLLOW", mavsdk::Telemetry::FlightMode::FollowMe},
                {"MANUAL", mavsdk::Telemetry::FlightMode::Manual},
                {"ALT_HOLD", mavsdk::Telemetry::FlightMode::Altctl},
                {"POSHOLD", mavsdk::Telemetry::FlightMode::Posctl},
                {"ACRO", mavsdk::Telemetry::FlightMode::Acro},
                {"STABILIZE", mavsdk::Telemetry::FlightMode::Stabilized},
            };

        auto it = map.find(upper);
        if (it != map.end()) {
            return it->second;
        }
        return std::nullopt;
    }
};