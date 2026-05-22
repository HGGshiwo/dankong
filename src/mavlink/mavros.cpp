#include "mavlink/mavros.hpp"

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tinyxml2.h>

#include <optional>
#include <regex>

#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandTOL.h"
#include "ros/time.h"

bool MavRos::set_stream_rate(int rate) {
    // 填充请求
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = 0;
    srv.request.message_rate = rate;
    srv.request.on_off = true;
    return set_rate_client_->call(srv);
}

bool MavRos::reboot_fcu() {
    mavros_msgs::CommandLong srv;
    srv.request.command = 246;
    srv.request.param1 = 1.0;
    srv.request.param2 = 0.0;
    srv.request.param3 = 0.0;
    srv.request.param4 = 0.0;
    srv.request.confirmation = 0.0;
    return cmd_client_->call(srv);
}

bool MavRos::set_mode(const FixedString64& mode) {
    // 构造服务请求
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 0;       // 基础模式通常设为 0
    srv.request.custom_mode = mode;  // 自定义模式字符串，如 "GUIDED"
    return set_mode_client_->call(srv);
}

// MAV_CMD_RUN_PREARM_CHECKS
bool MavRos::run_prearm_checks() {
    mavros_msgs::CommandLong srv;

    // 对应 Python: command=401, confirmation=0, param1=0...
    srv.request.command = 401;  // MAV_CMD_RUN_PREARM_CHECKS
    srv.request.confirmation = 0;
    srv.request.param1 = 0.0;
    srv.request.param2 = 0.0;
    srv.request.param3 = 0.0;
    srv.request.param4 = 0.0;
    srv.request.param5 = 0.0;
    srv.request.param6 = 0.0;
    srv.request.param7 = 0.0;
    // 调用服务 (如果服务存在且通信成功，返回 true)
    return cmd_client_->call(srv);
}

bool MavRos::arm() {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    return arm_client_->call(srv);
}

bool MavRos::disarm() {
    mavros_msgs::CommandBool srv;
    srv.request.value = false;
    return arm_client_->call(srv);
}

bool MavRos::takeoff(double alt) {
    mavros_msgs::CommandTOL srv;
    srv.request.altitude = alt;
    srv.request.latitude = 0;
    srv.request.longitude = 0;
    srv.request.min_pitch = 0;
    srv.request.yaw = 0;
    return takeoff_client_->call(srv);
}

ApmParam MavRos::get_param(const std::string& name, const ApmParam& value) {
    ApmParam result;

    // std::visit will instantiate branches based on the actual type in value
    std::visit(
        [this, &name, &result](auto&& default_val) {
            // Get underlying type T
            using T = std::decay_t<decltype(default_val)>;
            T fetched_val;
            // Fetch parameter from ROS parameter server
            this->nh_.param<T>("/mavros/param/" + name, fetched_val,
                               default_val);
            // Re-assign the strongly typed value to variant
            result = fetched_val;
        },
        value);

    return result;
}

bool MavRos::set_param(const std::string& name, const ApmParam& value) {
    // Visit the variant to extract the underlying value
    return std::visit(
        [this, &name](auto&& val) {
            // ros::NodeHandle::setParam has overloads for int, double, string,
            // bool, etc. The compiler will automatically choose the correct
            // overload based on 'val'
            this->nh_.setParam("/mavros/param/" + name, val);
            return true;
        },
        value);
}

bool MavRos::pull_params() {
    mavros_msgs::ParamPull srv;
    srv.request.force_pull = true;
    if (param_pull_client_->call(srv)) {
        spdlog::info("[mavros] received {} params",
                     srv.response.param_received);
        return true;
    };
    spdlog::error("[mavros] pull param failed!");
    return false;
}

bool MavRos::cmd_vel(Eigen::Vector4d vel) {
    auto mav_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "local_ned";
    target.coordinate_frame = mav_frame;
    uint32_t type_mask = 0;
    type_mask |= mavros_msgs::PositionTarget::IGNORE_PX |
                 mavros_msgs::PositionTarget::IGNORE_PY |
                 mavros_msgs::PositionTarget::IGNORE_PZ;

    type_mask |= mavros_msgs::PositionTarget::IGNORE_AFX |
                 mavros_msgs::PositionTarget::IGNORE_AFY |
                 mavros_msgs::PositionTarget::IGNORE_AFZ;

    type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;

    target.position.x = 0;
    target.position.y = 0;
    target.position.z = 0;
    target.velocity.x = vel.x();
    target.velocity.y = vel.y();
    target.velocity.z = vel.z();
    target.acceleration_or_force.x = 0;
    target.acceleration_or_force.y = 0;
    target.acceleration_or_force.z = 0;
    target.yaw = 0;
    target.yaw_rate = vel.w();
    target.type_mask = type_mask;
    setpoint_pub_->publish(target);
    return true;
}

void MavRos::load_pdef(const std::string& path) {
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

// 获取所有参数
nlohmann::json MavRos::get_all_params() {
    XmlRpc::XmlRpcValue rpc_params;
    nlohmann::json data = nlohmann::json::array();
    if (ros::param::get("/mavros/param", rpc_params) &&
        rpc_params.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for (auto const& param : rpc_params) {
            std::string key = param.first;
            XmlRpc::XmlRpcValue value = param.second;
            nlohmann::json json_value;
            if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                json_value = static_cast<int>(value);
            } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                json_value = static_cast<double>(value);
            } else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
                json_value = static_cast<std::string>(value);
            } else if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
                json_value = static_cast<bool>(value);
            }
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
                                   {"help", pdef_data["help"]},
                                   {"value", json_value},
                                   {"group", {result, key}}};
            data.push_back(item);
        }
    }
    return data;
}

void MavRos::param_callback(const mavros_msgs::Param::ConstPtr& msg) {
    std::string param_path = "/mavros/param/" + msg->param_id;
    // In MAVROS, parameter values are typically split into integer and real
    // (double) Sync the appropriate value to the ROS parameter server Note: You
    // might need to adjust the condition based on your specific message
    // definition
    if (msg->value.integer != 0) {
        // Sync as int
        this->nh_.setParam(param_path, static_cast<int>(msg->value.integer));
    } else {
        // Sync as double
        this->nh_.setParam(param_path, static_cast<double>(msg->value.real));
    }
}