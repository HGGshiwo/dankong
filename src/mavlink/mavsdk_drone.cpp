#include "mavlink/mavsdk_drone.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <regex>
#include <unordered_map>

#include "core/global_config.hpp"
#include "utils/get_executable_path.hpp"

// 初始化所有需要的 MAVSDK 插件
MavsdkDrone::MavsdkDrone(std::shared_ptr<mavsdk::System> system)
    : system_(system) {
    action_ = std::make_shared<mavsdk::Action>(system_);
    offboard_ = std::make_shared<mavsdk::Offboard>(system_);
    param_ = std::make_shared<mavsdk::Param>(system_);
    telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
    passthrough_ = std::make_shared<mavsdk::MavlinkPassthrough>(system_);
    rtk_ = std::make_shared<mavsdk::Rtk>(system_);

    load_pdef(
        get_config_dir(GlobalConfig.GetConfig().pdef_path.get()).string());
}

bool MavsdkDrone::arm() {
    mavsdk::Action::Result result = action_->arm();
    if (result != mavsdk::Action::Result::Success) {
        spdlog::error("Arm failed: {}", static_cast<int>(result));
        return false;
    }
    return true;
}

bool MavsdkDrone::disarm() {
    mavsdk::Action::Result result = action_->disarm();
    if (result != mavsdk::Action::Result::Success) {
        spdlog::error("Disarm failed: {}", static_cast<int>(result));
        return false;
    }
    return true;
}

bool MavsdkDrone::takeoff(double alt) {
    action_->set_takeoff_altitude(alt);
    // APM 要求在 GUIDED 模式下才能通过外部指令起飞
    mavsdk::Action::Result result = action_->takeoff();
    if (result != mavsdk::Action::Result::Success) {
        spdlog::error("Takeoff failed: {}", static_cast<int>(result));
        return false;
    }
    return true;
}

bool MavsdkDrone::set_mode(const FixedString64& mode) {
    std::string mode_str = mode;
    std::transform(mode_str.begin(), mode_str.end(), mode_str.begin(),
                   ::toupper);

    // APM 旋翼机 (ArduCopter) 模式枚举映射
    static const std::unordered_map<std::string, float> apm_copter_modes = {
        {"STABILIZE", 0.0f}, {"ACRO", 1.0f},   {"ALT_HOLD", 2.0f},
        {"AUTO", 3.0f},      {"GUIDED", 4.0f}, {"LOITER", 5.0f},
        {"RTL", 6.0f},       {"CIRCLE", 7.0f}, {"LAND", 9.0f},
        {"POSHOLD", 16.0f},  {"BRAKE", 17.0f}};

    if (mode_str == "RTL" || mode_str == "RETURN_TO_LAUNCH") {
        return action_->return_to_launch() == mavsdk::Action::Result::Success;
    } else if (mode_str == "LAND") {
        return action_->land() == mavsdk::Action::Result::Success;
    }

    auto it = apm_copter_modes.find(mode_str);
    if (it != apm_copter_modes.end()) {
        float custom_mode = it->second;

        // 构建 MAVLink 模式切换指令 (MAV_CMD_DO_SET_MODE)
        mavsdk::MavlinkPassthrough::CommandLong cmd{};
        cmd.target_sysid = passthrough_->get_our_sysid();
        cmd.target_compid = 0;
        cmd.command = 176;  // 176 = MAV_CMD_DO_SET_MODE
        cmd.param1 = 1.0f;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = custom_mode;

        auto result = passthrough_->send_command_long(cmd);
        if (result != mavsdk::MavlinkPassthrough::Result::Success) {
            spdlog::error("Failed to set APM mode {} (Code {})", mode_str,
                          custom_mode);
            return false;
        }
        spdlog::info("Successfully set APM mode to {}", mode_str);
        return true;
    }

    spdlog::error("Unsupported APM flight mode: {}", mode_str);
    return false;
}

bool MavsdkDrone::run_prearm_checks() {
    if (!passthrough_) {
        spdlog::error("MavlinkPassthrough plugin is not initialized.");
        return false;
    }
    mavsdk::MavlinkPassthrough::CommandLong cmd{};
    // 💡 避坑提示：目标 sysid 应该是飞控的 ID，而不是我们自己的 ID
    cmd.target_sysid = passthrough_->get_target_sysid();
    cmd.target_compid = passthrough_->get_target_compid();

    // 401 = MAV_CMD_RUN_PREARM_CHECKS
    // 这个指令会让飞控立即执行一次完整的起飞检查，并通过 STATUSTEXT
    // 广播结果
    cmd.command = 401;

    // 该命令不需要参数，默认全填 0
    cmd.param1 = 0.0f;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;

    auto result = passthrough_->send_command_long(cmd);

    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        spdlog::info("Successfully triggered prearm checks request.");
        return true;
    } else {
        spdlog::error("Failed to trigger prearm checks: {}",
                      static_cast<int>(result));
        return false;
    }
}

bool MavsdkDrone::set_stream_rate(int stream_id, int rate) {
    if (!passthrough_) {
        spdlog::error("MavlinkPassthrough plugin is not initialized.");
        return false;
    }

    mavlink_message_t msg;
    // 构造 REQUEST_DATA_STREAM 消息
    // rate > 0 时 start_stop 位设为 1（启动），否则设为 0（停止）
    mavlink_msg_request_data_stream_pack(
        passthrough_->get_our_sysid(), passthrough_->get_our_compid(), &msg,
        passthrough_->get_target_sysid(), passthrough_->get_target_compid(),
        static_cast<uint8_t>(stream_id), static_cast<uint16_t>(rate),
        rate > 0 ? 1 : 0);

    auto result = passthrough_->send_message(msg);
    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        spdlog::error("Failed to set stream rate for ID: {}", stream_id);
        return false;
    }

    spdlog::info("Successfully requested stream ID {} at {} Hz", stream_id,
                 rate);
    return true;
}

bool MavsdkDrone::set_msg_interval(int msg_id, int rate_hz) {
    // APM 推荐使用 MAV_CMD_SET_MESSAGE_INTERVAL (511)
    // 来设置指定消息的发送频率
    int interval_us =
        (rate_hz > 0) ? (1000000 / rate_hz) : -1;  // -1 意味着禁用该消息

    mavsdk::MavlinkPassthrough::CommandLong cmd{};
    cmd.target_sysid = passthrough_->get_our_sysid();
    cmd.target_compid = 0;
    cmd.command = 511;  // MAV_CMD_SET_MESSAGE_INTERVAL
    cmd.param1 =
        static_cast<float>(msg_id);  // 消息 ID (例如 33 是 GLOBAL_POSITION_INT)
    cmd.param2 = static_cast<float>(interval_us);  // 间隔 (微秒)

    auto result = passthrough_->send_command_long(cmd);
    return result == mavsdk::MavlinkPassthrough::Result::Success;
}

ApmParam MavsdkDrone::get_param(const std::string& name,
                                const ApmParam& value) {
    ApmParam result = value;
    std::visit(
        [this, &name, &result](auto&& val) {
            using T = std::decay_t<decltype(val)>;
            if constexpr (std::is_same_v<T, int>) {
                auto res = param_->get_param_int(name);
                if (res.first == mavsdk::Param::Result::Success)
                    result = res.second;
            } else if constexpr (std::is_same_v<T, double>) {
                auto res = param_->get_param_float(name);
                if (res.first == mavsdk::Param::Result::Success)
                    result = static_cast<double>(res.second);
            }
        },
        value);
    return result;
}

bool MavsdkDrone::set_param(const std::string& name, const ApmParam& value) {
    bool success = false;
    std::visit(
        [this, &name, &success](auto&& val) {
            using T = std::decay_t<decltype(val)>;
            if constexpr (std::is_same_v<T, int>) {
                auto res = param_->set_param_int(name, val);
                success = (res == mavsdk::Param::Result::Success);
            } else if constexpr (std::is_same_v<T, double>) {
                auto res =
                    param_->set_param_float(name, static_cast<float>(val));
                success = (res == mavsdk::Param::Result::Success);
            }
        },
        value);
    return success;
}

bool MavsdkDrone::pull_params() {
    if (!passthrough_) {
        spdlog::error("MavlinkPassthrough plugin is not initialized.");
        return false;
    }

    mavlink_message_t msg;
    // 构造 PARAM_REQUEST_LIST 消息，强制飞控重新发送所有参数
    mavlink_msg_param_request_list_pack(
        passthrough_->get_our_sysid(), passthrough_->get_our_compid(), &msg,
        passthrough_->get_target_sysid(), passthrough_->get_target_compid());

    auto result = passthrough_->send_message(msg);
    if (result == mavsdk::MavlinkPassthrough::Result::Success) {
        spdlog::info(
            "Successfully sent PARAM_REQUEST_LIST to force parameter "
            "refresh.");
        return true;
    }

    spdlog::error("Failed to pull params via Passthrough.");
    return false;
}

void MavsdkDrone::load_pdef(const std::string& path) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return;
    }
    pdef_ = nlohmann::json::object();
    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) {
        return;
    }
    std::vector<tinyxml2::XMLElement*> stack;
    stack.push_back(root);
    while (!stack.empty()) {
        tinyxml2::XMLElement* curr = stack.back();
        stack.pop_back();
        if (std::string(curr->Name()) == "param") {
            std::string full_name =
                curr->Attribute("name") ? curr->Attribute("name") : "";
            std::string name = full_name;
            size_t colon_pos = full_name.find_last_of(':');
            if (colon_pos != std::string::npos) {
                name = full_name.substr(colon_pos + 1);
            }
            nlohmann::json help_text = nlohmann::json::array();
            const char* doc_attr = curr->Attribute("documentation");
            help_text.push_back(doc_attr ? doc_attr : "");
            for (tinyxml2::XMLElement* field = curr->FirstChildElement("field");
                 field != nullptr; field = field->NextSiblingElement("field")) {
                std::string field_name =
                    field->Attribute("name") ? field->Attribute("name") : "";
                std::string field_text =
                    field->GetText() ? field->GetText() : "";
                help_text.push_back(field_name + ": " + field_text);
            }
            pdef_[name] = {{"default", nullptr}, {"help", help_text}};
        }
        for (tinyxml2::XMLElement* child = curr->FirstChildElement();
             child != nullptr; child = child->NextSiblingElement()) {
            stack.push_back(child);
        }
    }
}

nlohmann::json MavsdkDrone::get_all_params() {
    auto all_params = param_->get_all_params();
    nlohmann::json data = nlohmann::json::array();

    auto process_param = [&](const std::string& key, const auto& value) {
        nlohmann::json pdef_data;
        if (pdef_.contains(key)) {
            pdef_data = pdef_[key];
        } else {
            pdef_data = {{"help", {""}}, {"default", nullptr}};
        }
        std::string prefix = key.substr(0, key.find('_'));
        std::regex digit_re("\\d");
        std::string result = std::regex_replace(prefix, digit_re, "X");
        nlohmann::json item = {{"key", key},
                               {"name", key},
                               {"value", value},
                               {"help", pdef_data["help"]},
                               {"group", {result, key}}};
        data.push_back(item);
    };

    for (const auto& p : all_params.int_params) {
        process_param(p.name, p.value);
    }
    for (const auto& p : all_params.float_params) {
        process_param(p.name, static_cast<double>(p.value));
    }
    return data;
}

bool MavsdkDrone::reboot_fcu() {
    auto res = action_->reboot();
    return res == mavsdk::Action::Result::Success;
}

bool MavsdkDrone::cmd_vel(Eigen::Vector4d vel) {
    // 注意这里的vel是FLU坐标系的
    mavsdk::Offboard::VelocityBodyYawspeed cmd{};
    cmd.forward_m_s = vel[0];
    cmd.right_m_s = -vel[1];
    cmd.down_m_s = -vel[2];
    cmd.yawspeed_deg_s = -vel[3] * (180.0 / M_PI);

    // 在 APM 中，Offboard 接口发送的设定值(Setpoint)会在飞控处于 GUIDED
    // 模式时生效。
    mavsdk::Offboard::Result result = offboard_->set_velocity_body(cmd);
    if (result != mavsdk::Offboard::Result::Success) {
        // MAVSDK 的 offboard_->start() 会尝试将模式切换为 Offboard
        // (APM端映射为 GUIDED 模式)
        offboard_->start();
        result = offboard_->set_velocity_body(cmd);
    }
    return result == mavsdk::Offboard::Result::Success;
}

void MavsdkDrone::send_rtcm_data(const uint8_t* data, size_t size) {
    // 180 字节防爆切片逻辑
    const size_t MAX_CHUNK = 180;
    for (size_t i = 0; i < size; i += MAX_CHUNK) {
        size_t chunk_size = std::min(MAX_CHUNK, size - i);

        mavsdk::Rtk::RtcmData rtcm_data;
        // 将 uint8_t 转换为 MAVSDK 需要的 std::string
        rtcm_data.data_base64.assign(reinterpret_cast<const char*>(data + i),
                                     chunk_size);

        // 最终调用！将切片后的差分数据发给飞控
        rtk_->send_rtcm_data(rtcm_data);
    }
}