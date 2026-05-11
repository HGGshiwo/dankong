#include "utils.hpp"

// 获取当前运行的绝对路径 (工具方法)
fs::path get_current_run_path() {
    return fs::current_path();
}

// 递归函数：将 YAML::Node 转换为 nlohmann::json
json yaml_to_json(const YAML::Node& yaml_node) {
    json j;
    switch (yaml_node.Type()) {
        case YAML::NodeType::Null:
            j = nullptr;
            break;
        case YAML::NodeType::Scalar: {
            // YAML的标量需要推断类型。按 bool -> int -> double -> string 的顺序尝试解析

            // 尝试解析为布尔值 (YAML 支持 true, false, yes, no, on, off)
            try {
                return yaml_node.as<bool>();
            } catch (const YAML::BadConversion&) {
            }
            // 尝试解析为整数 (int64_t 防止溢出)
            try {
                return yaml_node.as<int64_t>();
            } catch (const YAML::BadConversion&) {
            }
            // 尝试解析为浮点数
            try {
                return yaml_node.as<double>();
            } catch (const YAML::BadConversion&) {
            }
            // 如果都不是，则作为普通字符串处理
            return yaml_node.as<std::string>();
        }
        case YAML::NodeType::Sequence: {
            // 处理数组 (List/Array)
            j = json::array();
            for (const auto& item : yaml_node) {
                j.push_back(yaml_to_json(item));
            }
            break;
        }
        case YAML::NodeType::Map: {
            // 处理对象 (Dictionary/Object)
            j = json::object();
            for (const auto& it : yaml_node) {
                // JSON的键必须是字符串，而YAML的键可以是复杂类型
                // 这里强制将YAML的键转换为字符串形式
                std::string key = it.first.as<std::string>();
                j[key] = yaml_to_json(it.second);
            }
            break;
        }
        case YAML::NodeType::Undefined:
        default:
            break;
    }
    return j;
}