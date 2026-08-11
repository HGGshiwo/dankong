#pragma once
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <shared_mutex>
#include <string>

#include "robot_config.hpp"
#include "utils/config_param.hpp"
#include "utils/yaml_helper.hpp"

using AppConfigData = RobotConfig;

class ConfigManager {
   public:
    static ConfigManager& GetInstance() {
        static ConfigManager instance;
        return instance;
    }

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    bool load(const boost::filesystem::path& filepath) {
        return load(filepath.string());
    }

    bool load(const std::string& filepath) {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        try {
            YAML::Node final_yaml = YamlHelper::load_with_base(filepath);
            spdlog::info("Config loaded from: {}", filepath);

            std::stringstream ss;
            ss << final_yaml;                   // 将节点内容写入流
            std::string yamlString = ss.str();  // 获取字符串

            spdlog::info("Config content: {}", yamlString);
            nlohmann::json final_json = YamlHelper::yaml_to_json(final_yaml);
            data_ = final_json.get<AppConfigData>();
            last_filepath_ = filepath;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Config Load Error: " << e.what() << std::endl;
            return false;
        }
    }

    bool save() const {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        if (last_filepath_.empty()) {
            std::cerr << "Config Save Error: No filepath specified."
                      << std::endl;
            return false;
        }
        try {
            nlohmann::json current_json;
            nlohmann::to_json(current_json, data_);

            // Helper function to convert JSON to YAML Node
            std::function<YAML::Node(const nlohmann::json&)> json_to_yaml =
                [&](const nlohmann::json& j) -> YAML::Node {
                YAML::Node node;
                if (j.is_null()) {
                    node = YAML::Null;
                } else if (j.is_boolean()) {
                    node = j.get<bool>();
                } else if (j.is_number_integer()) {
                    node = j.get<int64_t>();
                } else if (j.is_number_float()) {
                    node = j.get<double>();
                } else if (j.is_string()) {
                    node = j.get<std::string>();
                } else if (j.is_array()) {
                    for (const auto& item : j) {
                        node.push_back(json_to_yaml(item));
                    }
                } else if (j.is_object()) {
                    for (auto it = j.begin(); it != j.end(); ++it) {
                        node[it.key()] = json_to_yaml(it.value());
                    }
                }
                return node;
            };

            YAML::Node yaml_node = json_to_yaml(current_json);
            std::ofstream fout(last_filepath_);
            fout << yaml_node;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Config Save Error: " << e.what() << std::endl;
            return false;
        }
    }

    AppConfigData& GetConfig() { return data_; }

    const AppConfigData& GetConfig() const { return data_; }

    template <typename T>
    bool UpdateParam(const std::string& key, const T& value) {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        try {
            nlohmann::json j;
            nlohmann::to_json(j, data_);

            if (!j.contains(key)) {
                std::cerr << "Config Update Warning: Key [" << key
                          << "] not found." << std::endl;
                return false;
            }

            j[key] = value;
            data_ = j.get<AppConfigData>();
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Config Update Error: " << e.what() << std::endl;
            return false;
        }
    }

    nlohmann::json GetAllParams() const {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        nlohmann::json data = nlohmann::json::array();

        nlohmann::json values_json;
        nlohmann::to_json(values_json, data_);

        auto& meta_map = dk::ParamRegistry::GetMap();

        for (auto it = values_json.begin(); it != values_json.end(); ++it) {
            std::string key = it.key();
            auto meta_it = meta_map.find(key);

            if (meta_it != meta_map.end() && meta_it->second.hidden) {
                continue;
            }

            nlohmann::json item;
            item["key"] = key;
            item["value"] = it.value();

            if (meta_it != meta_map.end()) {
                item["name"] = meta_it->second.name;
                item["help"] = nlohmann::json::array({meta_it->second.help});
                item["group"] =
                    nlohmann::json::array({meta_it->second.group, key});
            } else {
                item["name"] = key;
                item["help"] = nlohmann::json::array({""});
                item["group"] = nlohmann::json::array({"Default", key});
            }
            data.push_back(item);
        }
        return data;
    }

   private:
    ConfigManager() = default;
    ~ConfigManager() = default;

    AppConfigData data_;
    std::string last_filepath_;
    mutable std::shared_mutex rw_mutex_;
};

#define GlobalConfig ConfigManager::GetInstance()
