#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <magic_enum.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <queue>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"
#include "core/global_config.hpp"
#include "spdlog/spdlog.h"
#include "utils/get_executable_path.hpp"

// Type traits for detecting Eigen matrices/vectors
template <typename T, typename = void>
struct is_eigen_matrix : std::false_type {};

template <typename T>
struct is_eigen_matrix<
    T, std::void_t<typename std::decay_t<T>::Scalar,
                   decltype(std::decay_t<T>::RowsAtCompileTime)>>
    : std::true_type {};

namespace fglog {
enum class LogLevel { Info = 0, Warn = 1, Error = 2 };

struct fglog_conn_registry {
    static fglog_conn_registry& get() {
        static fglog_conn_registry instance;
        return instance;
    }
    void enable(size_t id) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_enabled_ids.insert(id);
    }
    void disable(size_t id) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_enabled_ids.erase(id);
    }
    bool is_enabled(size_t id) {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_enabled_ids.count(id) > 0;
    }

   private:
    std::mutex m_mutex;
    std::unordered_set<size_t> m_enabled_ids;
};
}  // namespace fglog

class FGLoggerBackend {
   public:
    using WsSender = std::function<void(const std::string&)>;

    static FGLoggerBackend& getInstance() {
        static FGLoggerBackend instance;
        return instance;
    }

    ~FGLoggerBackend() { shutdown(); }

    void set_ws_sender(WsSender sender) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        m_ws_sender = sender;
    }

    bool init(uint16_t port = 8765, const std::string& mcap_filename = "") {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        m_start_time = std::chrono::steady_clock::now();

        if (!m_running) {
            m_running = true;
            m_worker_thread = std::thread(&FGLoggerBackend::worker_loop, this);
        }

        if (!mcap_filename.empty()) {
            namespace fs = std::filesystem;
            fs::path p(mcap_filename);
            fs::path dir = p.parent_path();
            if (!dir.empty() && !fs::exists(dir)) {
                try {
                    fs::create_directories(dir);
                } catch (const std::exception& e) {
                    spdlog::error(
                        "[fglog] Failed to create directories for path {}: {}",
                        mcap_filename, e.what());
                }
            }

            m_log_path = mcap_filename;
            if (m_log_file.is_open()) {
                m_log_file.close();
            }
            m_log_file.open(m_log_path, std::ios::out | std::ios::app);
            if (m_log_file.is_open()) {
                spdlog::info("[fglog] Local log file opened successfully at {}",
                             m_log_path);
            } else {
                spdlog::error("[fglog] Failed to open local log file at {}",
                              m_log_path);
            }
        }
        return true;
    }

    void shutdown() {
        m_running = false;
        m_queue_cv.notify_all();
        if (m_worker_thread.joinable()) {
            m_worker_thread.join();
        }
        std::lock_guard<std::mutex> lock(m_state_mutex);
        if (m_log_file.is_open()) {
            m_log_file.flush();
            m_log_file.close();
            spdlog::info("[fglog] Log writer closed cleanly.");
        }
    }

    void post_log(const std::string& topic, nlohmann::json value) {
        double elapsed = get_elapsed_time();
        post_task([this, elapsed, topic, val = std::move(value)]() {
            write_log_entry(elapsed, topic, val);
        });
    }

    template <typename EnumType>
    void publish_enum(const std::string& topic, EnumType enum_value) {
        double elapsed = get_elapsed_time();
        std::string enum_name_str = get_enum_name(enum_value);
        post_task([this, elapsed, topic, name = std::move(enum_name_str)]() {
            write_log_entry(elapsed, topic, name);
        });
    }

    void publish_dynamic_state(const std::string& topic, int state_id) {
        double elapsed = get_elapsed_time();
        post_task([this, elapsed, topic, state_id]() {
            std::string name;
            {
                std::lock_guard<std::mutex> lock(m_state_mutex);
                auto it = m_enum_maps.find(topic);
                if (it != m_enum_maps.end()) {
                    auto map_it = it->second.find(state_id);
                    if (map_it != it->second.end()) {
                        name = map_it->second;
                    }
                }
            }
            if (name.empty()) {
                name = std::to_string(state_id);
            }
            write_log_entry(elapsed, topic, name);
        });
    }

    void publish_value(const std::string& topic, int value) {
        post_log(topic, value);
    }

    void publish_value(const std::string& topic, const std::string& value) {
        post_log(topic, value);
    }

    void log(const std::string& tag, fglog::LogLevel level,
             const std::string& message) {
        std::string lvl_str = "INFO";
        if (level == fglog::LogLevel::Warn)
            lvl_str = "WARN";
        else if (level == fglog::LogLevel::Error)
            lvl_str = "ERROR";

        nlohmann::json log_msg;
        log_msg["tag"] = tag;
        log_msg["level"] = lvl_str;
        log_msg["message"] = message;

        post_log("/system/logs", log_msg);
    }

    template <typename EnumType>
    void register_enum(const std::string& topic,
                       const std::string& enum_name = "") {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        std::map<int, std::string> id_name_map;
        for (const auto& [val, n] : magic_enum::enum_entries<EnumType>()) {
            id_name_map[static_cast<int>(val)] = std::string(n);
        }
        m_enum_maps[topic] = id_name_map;
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::map<int, std::string>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        m_enum_maps[topic] = map;
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::unordered_map<int, std::string>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        m_enum_maps[topic] = std::map<int, std::string>(map.begin(), map.end());
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::map<std::string, int>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        std::map<int, std::string> id_name_map;
        for (const auto& [name, id] : map) {
            id_name_map[id] = name;
        }
        m_enum_maps[topic] = id_name_map;
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::unordered_map<std::string, int>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        std::map<int, std::string> id_name_map;
        for (const auto& [name, id] : map) {
            id_name_map[id] = name;
        }
        m_enum_maps[topic] = id_name_map;
    }

   private:
    FGLoggerBackend() = default;

    double get_elapsed_time() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - m_start_time).count();
    }

    template <typename EnumType>
    std::string get_enum_name(EnumType enum_value) {
        auto name = magic_enum::enum_name(enum_value);
        if (!name.empty()) {
            return std::string(name);
        }
        return std::to_string(static_cast<int>(enum_value));
    }

    void post_task(std::function<void()> task) {
        if (!m_running) return;
        {
            std::lock_guard<std::mutex> lock(m_queue_mutex);
            m_task_queue.emplace(std::move(task));
        }
        m_queue_cv.notify_one();
    }

    void write_log_entry(double time, const std::string& topic,
                         const nlohmann::json& value) {
        nlohmann::json entry;
        entry["time"] = time;
        entry["topic"] = topic;
        entry["value"] = value;

        std::string entry_str = entry.dump();

        // 1. Write to local file
        std::lock_guard<std::mutex> lock(m_state_mutex);
        if (m_log_file.is_open()) {
            m_log_file << entry_str << "\n";
            m_log_file.flush();
        }

        // 2. Send via WebSocket using the main server channel
        if (m_ws_sender) {
            nlohmann::json ws_msg;
            ws_msg["type"] = "fglog";
            ws_msg["fglog"] = entry;
            m_ws_sender(ws_msg.dump());
        }
    }

    void worker_loop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(m_queue_mutex);
                m_queue_cv.wait(lock, [this]() {
                    return !m_task_queue.empty() || !m_running;
                });
                if (m_task_queue.empty() && !m_running) {
                    break;
                }
                task = std::move(m_task_queue.front());
                m_task_queue.pop();
            }
            if (task) {
                task();
            }
        }
    }

    std::mutex m_state_mutex;
    std::string m_log_path;
    std::ofstream m_log_file;
    std::chrono::steady_clock::time_point m_start_time;

    std::queue<std::function<void()>> m_task_queue;
    std::mutex m_queue_mutex;
    std::condition_variable m_queue_cv;
    std::atomic<bool> m_running{false};
    std::thread m_worker_thread;

    std::unordered_map<std::string, std::map<int, std::string>> m_enum_maps;
    WsSender m_ws_sender = nullptr;
};

namespace fglog {
inline void init(uint16_t port = 8765, const std::string& mcap_path = "") {
    FGLoggerBackend::getInstance().init(port, mcap_path);
}

inline void close() {
    FGLoggerBackend::getInstance().shutdown();
}

inline void info(const std::string& tag, const std::string& msg) {
    FGLoggerBackend::getInstance().log(tag, LogLevel::Info, msg);
}
inline void warn(const std::string& tag, const std::string& msg) {
    FGLoggerBackend::getInstance().log(tag, LogLevel::Warn, msg);
}
inline void error(const std::string& tag, const std::string& msg) {
    FGLoggerBackend::getInstance().log(tag, LogLevel::Error, msg);
}
inline void info(const std::string& msg) {
    info("SYS", msg);
}

template <typename EnumType>
inline void publish_enum(const std::string& topic, EnumType enum_value) {
    FGLoggerBackend::getInstance().publish_enum(topic, enum_value);
}

inline void publish_enum(const std::string& topic, int state_id) {
    FGLoggerBackend::getInstance().publish_dynamic_state(topic, state_id);
}

template <typename EnumType>
inline void register_enum(const std::string& topic,
                          const std::string& enum_name = "") {
    FGLoggerBackend::getInstance().register_enum<EnumType>(topic, enum_name);
}

inline void register_enum(const std::string& topic,
                          const std::string& enum_name,
                          const std::map<int, std::string>& map) {
    FGLoggerBackend::getInstance().register_enum(topic, enum_name, map);
}

inline void register_enum(const std::string& topic,
                          const std::string& enum_name,
                          const std::unordered_map<int, std::string>& map) {
    FGLoggerBackend::getInstance().register_enum(topic, enum_name, map);
}

inline void register_enum(const std::string& topic,
                          const std::string& enum_name,
                          const std::map<std::string, int>& map) {
    FGLoggerBackend::getInstance().register_enum(topic, enum_name, map);
}

inline void register_enum(const std::string& topic,
                          const std::string& enum_name,
                          const std::unordered_map<std::string, int>& map) {
    FGLoggerBackend::getInstance().register_enum(topic, enum_name, map);
}

template <typename T>
inline void publish(const std::string& topic, const T& value) {
    if constexpr (is_eigen_matrix<T>::value) {
        using CleanT = std::decay_t<T>;
        auto eval_vec = value.eval();
        if constexpr (CleanT::RowsAtCompileTime == 3 &&
                      CleanT::ColsAtCompileTime == 1) {
            std::vector<double> vec = {static_cast<double>(eval_vec.x()),
                                       static_cast<double>(eval_vec.y()),
                                       static_cast<double>(eval_vec.z())};
            FGLoggerBackend::getInstance().post_log(topic, vec);
        } else if constexpr (CleanT::RowsAtCompileTime == 2 &&
                             CleanT::ColsAtCompileTime == 1) {
            std::vector<double> vec = {static_cast<double>(eval_vec.x()),
                                       static_cast<double>(eval_vec.y())};
            FGLoggerBackend::getInstance().post_log(topic, vec);
        } else {
            std::vector<std::vector<double>> mat(
                eval_vec.rows(), std::vector<double>(eval_vec.cols()));
            for (int r = 0; r < eval_vec.rows(); ++r) {
                for (int c = 0; c < eval_vec.cols(); ++c) {
                    mat[r][c] = static_cast<double>(eval_vec(r, c));
                }
            }
            FGLoggerBackend::getInstance().post_log(topic, mat);
        }
    } else {
        FGLoggerBackend::getInstance().post_log(topic, value);
    }
}

inline void set_websocket_sender(
    std::function<void(const std::string&)> sender) {
    FGLoggerBackend::getInstance().set_ws_sender(sender);
}
}  // namespace fglog

template <typename EnumType>
inline void register_enum_to_fg(const std::string& enum_name) {
    fglog::register_enum<EnumType>(enum_name, enum_name);
}

inline void register_enum_to_fg(const std::string& enum_name,
                                const std::map<int, std::string>& map) {
    fglog::register_enum(enum_name, enum_name, map);
}

inline void register_enum_to_fg(
    const std::string& enum_name,
    const std::unordered_map<int, std::string>& map) {
    fglog::register_enum(enum_name, enum_name, map);
}

inline void register_enum_to_fg(const std::string& enum_name,
                                const std::map<std::string, int>& map) {
    fglog::register_enum(enum_name, enum_name, map);
}

inline void register_enum_to_fg(
    const std::string& enum_name,
    const std::unordered_map<std::string, int>& map) {
    fglog::register_enum(enum_name, enum_name, map);
}

inline void init_fg_logger() {
    try {
        boost::filesystem::path LOG_DIR =
            get_config_dir(GlobalConfig.GetConfig().fg_log_dir.get());
        unsigned int port = GlobalConfig.GetConfig().fg_log_port.get();
        fglog::init(port, LOG_DIR.string());
    } catch (const std::exception& ex) {
        spdlog::error("FoxGlove Log initialization failed: {}", ex.what());
    }
}