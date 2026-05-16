#pragma once
#include <Eigen/Dense>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>

#include "utils.hpp"

enum CmdFrame { BODY, ENU };
using ApmParam = std::variant<int, double, std::string>;
class IMavlink {
   public:
    virtual bool set_mode(const FixedString64& mode) = 0;
    virtual bool send_cmd(std::optional<Eigen::Vector3d> pos,
                          std::optional<Eigen::Vector3d> vel,
                          std::optional<Eigen::Vector3d> acc,
                          std::optional<double> yaw,
                          std::optional<double> yaw_rate, CmdFrame frame) = 0;
    // MAV_CMD_RUN_PREARM_CHECKS
    virtual bool run_prearm_checks() = 0;
    virtual bool reboot_fcu() = 0;
    virtual bool set_stream_rate(int rate) = 0;
    virtual bool arm() = 0;
    virtual bool disarm() = 0;
    virtual bool takeoff(double alt) = 0;
    virtual ApmParam get_param(const std::string& name,
                               const ApmParam& value) = 0;
    virtual bool set_param(const std::string& name, const ApmParam& value) = 0;
    virtual nlohmann::json get_all_params() = 0;  // 加载全部参数
    virtual bool pull_params() = 0;

    // 获取实际的数据
    template <typename Type>
    static Type unpack(const ApmParam& param) {
        return std::get<Type>(param);
    }
};
