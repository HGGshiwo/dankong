#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

/**
 * @brief 生成带时间戳的唯一日志文件名
 *
 * @param log_dir  日志保存的目录 (例如 "./logs")
 * @param prefix   文件名前缀 (例如 "flight")
 * @param ext      文件扩展名 (例如 ".mcap")
 * @return std::string 最终可用的绝对或相对路径
 */
inline std::string generate_unique_filename(
    const std::string& log_dir = "./logs", const std::string& prefix = "flight",
    const std::string& ext = ".mcap") {
    // 1. 确保日志目录存在，如果不存在则自动创建
    fs::path dir_path(log_dir);
    if (!fs::exists(dir_path)) {
        fs::create_directories(dir_path);
    }

    // 2. 获取当前时间并格式化为 YYYYMMDD_HHMMSS
    auto now = std::chrono::system_clock::now();
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    std::string timestamp = ss.str();

    // 3. 拼接基础文件名： "flight_20260728_111530"
    std::string base_name = prefix + "_" + timestamp;

    // 4. 组装初始完整路径： "./logs/flight_20260728_111530.mcap"
    fs::path file_path = dir_path / (base_name + ext);

    // 5. 检查文件是否已存在。如果存在，自动追加 _1, _2 ...
    int counter = 1;
    while (fs::exists(file_path)) {
        std::string new_name = base_name + "_" + std::to_string(counter) + ext;
        file_path = dir_path / new_name;
        counter++;
    }

    return file_path.string();
}