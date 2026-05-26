#pragma once
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <shared_mutex>
#include <string>

using json = nlohmann::json;
namespace fs = boost::filesystem;

inline int strict_stoi(const std::string& str) {
    size_t pos = 0;
    int val =
        std::stoi(str, &pos);  // 可能抛出 invalid_argument 或 out_of_range

    // 如果解析停止的位置不等于字符串长度，说明有残留字符
    if (pos != str.length()) {
        throw std::invalid_argument("字符串包含无法解析为整数的后缀字符: " +
                                    str);
    }
    return val;
}

inline double strict_stod(const std::string& str) {
    size_t pos = 0;
    // std::stod 可能会抛出 invalid_argument 或 out_of_range
    double val = std::stod(str, &pos);

    // 如果没有解析到字符串末尾，说明有残留的非法字符
    if (pos != str.length()) {
        throw std::invalid_argument("字符串包含无法解析为浮点数的后缀字符: " +
                                    str);
    }
    return val;
}

// ==========================================
// 模块 1: YAML 核心工具层 (满足你复用的需求)
// ==========================================
class YamlHelper {
   public:
    static YAML::Node load_with_base(const std::string& filepath) {
        return load_with_base(fs::path{filepath});
    }
    // 递归处理基类继承加载
    static YAML::Node load_with_base(const fs::path& filepath) {
        if (!fs::exists(filepath)) {
            throw std::runtime_error("File not found: " + filepath.string());
        }

        YAML::Node current = YAML::LoadFile(filepath.string());

        // 如果存在 base 字段，则先加载 base，然后将 current 覆盖上去
        if (current["base"]) {
            std::string base_filename = current["base"].as<std::string>();
            fs::path base_path = filepath.parent_path() /
                                 base_filename;  // 基于当前文件目录找 base

            YAML::Node base_node = load_with_base(base_path);  // 递归加载
            deep_merge(base_node, current);  // current 覆盖 base

            base_node.remove("base");  // 移除 base 标记，保持最终数据纯净
            return base_node;
        }
        return current;
    }

    // 纯工具：将 YAML Node 转为 nlohmann::json
    static json yaml_to_json(const YAML::Node& root) {
        json j{};
        switch (root.Type()) {
            case YAML::NodeType::Null:
                j = nullptr;
                break;
            case YAML::NodeType::Scalar: {
                std::string val = root.as<std::string>();
                // 尝试智能推导类型
                if (val == "true")
                    j = true;
                else if (val == "false")
                    j = false;
                else if (val == "~" || val == "null")
                    j = nullptr;
                else {
                    try {
                        j = strict_stoi(val);
                    }  // 尝试转为 int
                    catch (...) {
                        try {
                            j = strict_stod(val);
                        }  // 尝试转为 double
                        catch (...) {
                            j = val;
                        }  // 兜底为 string
                    }
                }
                break;
            }
            case YAML::NodeType::Sequence:
                j = json::array();
                for (auto&& node : root) j.push_back(yaml_to_json(node));
                break;
            case YAML::NodeType::Map:
                j = json::object();
                for (auto&& it : root)
                    j[it.first.as<std::string>()] = yaml_to_json(it.second);
                break;
            case YAML::NodeType::Undefined:
                break;
        }
        return j;
    }

    static void save(const nlohmann::json& j, const std::string& p) {
        save(j, fs::path{p});
    }

    static void save(const nlohmann::json& j, const fs::path& p) {
        std::ofstream file(p.string());

        // 3. 检查文件是否成功打开
        if (file.is_open()) {
            file << std::setw(4) << j << std::endl;
            file.close();  // 关闭流
            std::cout << "JSON 文件保存成功！\n";
        } else {
            std::cerr << "[Config.save] " << p.string()
                      << " 无法打开文件进行写入！\n";
        }
    }

   private:
    // 深度合并 YAML 节点：source 覆盖 target
    static void deep_merge(YAML::Node target, const YAML::Node& source) {
        if (target.IsMap() && source.IsMap()) {
            for (const auto& kv : source) {
                std::string key = kv.first.as<std::string>();
                std::string append_suffix = "__append";

                // 检查是否带有 __append 后缀
                if (key.size() > append_suffix.size() &&
                    key.compare(key.size() - append_suffix.size(),
                                append_suffix.size(), append_suffix) == 0) {
                    // 提取真实键名：比如 "flags__append" -> "flags"
                    std::string real_key =
                        key.substr(0, key.size() - append_suffix.size());

                    if (target[real_key] && target[real_key].IsSequence() &&
                        kv.second.IsSequence()) {
                        for (const auto& el : kv.second) {
                            target[real_key].push_back(el);
                        }
                    } else {
                        target[real_key] =
                            kv.second;  // 如果 base 中没有，直接赋值为新列表
                    }
                } else {
                    // 默认行为：如果是普通键名，依然进行递归（但列表走覆盖逻辑）
                    if (target[key] && target[key].IsMap() &&
                        kv.second.IsMap()) {
                        deep_merge(target[key], kv.second);
                    } else {
                        target[key] = kv.second;  // 列表或标量直接【覆盖】
                    }
                }
            }
        } else {
            target = source;
        }
    }
};