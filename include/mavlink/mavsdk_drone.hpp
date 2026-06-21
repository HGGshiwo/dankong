#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/rtk/rtk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <tinyxml2.h>

#include <memory>
#include <nlohmann/json.hpp>

#include "imavlink.hpp"

class MavsdkDrone : public IMavlink {
   private:
    std::shared_ptr<mavsdk::System> system_;
    std::shared_ptr<mavsdk::Action> action_;
    std::shared_ptr<mavsdk::Param> param_;
    std::shared_ptr<mavsdk::Offboard> offboard_;
    std::shared_ptr<mavsdk::Telemetry> telemetry_;
    std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough_;
    std::shared_ptr<mavsdk::Rtk> rtk_;
    nlohmann::json pdef_;

    void load_pdef(const std::string& path);

   public:
    MavsdkDrone(std::shared_ptr<mavsdk::System> system);
    bool arm() override;
    bool disarm() override;
    bool takeoff(double alt) override;
    bool set_mode(const FixedString64& mode) override;
    bool run_prearm_checks() override;
    bool set_stream_rate(int stream_id, int rate) override;
    bool set_msg_interval(int stream_id, int rate) override;
    ApmParam get_param(const std::string& name, const ApmParam& value) override;
    bool set_param(const std::string& name, const ApmParam& value) override;
    bool pull_params() override;
    nlohmann::json get_all_params() override;
    bool reboot_fcu() override;
    bool cmd_vel(Eigen::Vector4d vel) override;
    void send_rtcm_data(const uint8_t* data, size_t size) override;
};