#pragma once
#include <cstdint>
#include <string>

struct FcuConnectedEvent {
    bool connected;
};

struct SysStatusEvent {
    uint32_t data;
};

struct StatusTextEvent {
    std::string text;
    bool should_report = false;
};
