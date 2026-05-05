#pragma once
#include <stdexcept>
#include <string>
#include <variant>

#include "./utils.hpp"

using ApmParam = std::variant<int, double, std::string>;
class IMavlink {
   public:
    virtual bool set_mode(const FixedString64& mode) = 0;

    // MAV_CMD_RUN_PREARM_CHECKS
    virtual bool run_prearm_checks() = 0;
    virtual bool set_stream_rate(int rate) = 0;
    virtual ApmParam get_param(std::string name, ApmParam value = 0) = 0;

    // 获取实际的数据
    template <typename Type>
    static Type unpack(const ApmParam& param) {
        return std::get<Type>(param);
    }
};
