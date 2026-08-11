#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <queue>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"

// spdlog headers
#include "spdlog/async.h"
#include "spdlog/sinks/base_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/spdlog.h"

// Project headers
#include "core/global_config.hpp"
#include "utils/get_executable_path.hpp"

// Eigen traits detection
template <typename T, typename = void>
struct is_eigen_matrix : std::false_type {};

template <typename T>
struct is_eigen_matrix<
    T, std::void_t<typename std::decay_t<T>::Scalar,
                   decltype(std::decay_t<T>::RowsAtCompileTime)>>
    : std::true_type {};

namespace fglog {

// ============================================================================
// 1. 还原并增强你的 连接/通道 注册开关机制
// ============================================================================
class fglog_conn_registry {
   public:
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
    bool any_enabled() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return !m_enabled_ids.empty();
    }

   private:
    std::mutex m_mutex;
    std::unordered_set<size_t> m_enabled_ids;
};

// ============================================================================
// 2. 独立的 WebSocket 异步发送执行引擎（网络I/O不干扰文件持久化）
// ============================================================================
class WebSocketEngine {
   public:
    using WsSender = std::function<void(const nlohmann::json&)>;

    WebSocketEngine() : m_running(false) {}

    ~WebSocketEngine() { stop(); }

    void start(WsSender sender) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_ws_sender = std::move(sender);
        if (!m_running) {
            m_running = true;
            m_thread = std::thread(&WebSocketEngine::worker_loop, this);
        }
    }

    void stop() {
        if (m_running) {
            m_running = false;
            m_cv.notify_all();
            if (m_thread.joinable()) {
                m_thread.join();
            }
        }
    }

    // 非阻塞丢弃机制：如果网络卡了，队列超过上限（如 1024条），直接丢弃网络帧
    void try_push(nlohmann::json raw_json) {
        if (!m_running || !fglog_conn_registry::get().any_enabled()) return;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            // 这里设置最大 1024 缓冲帧。网络太差就丢帧，绝不积压内存
            if (m_queue.size() >= 1024) {
                m_queue.pop();  // 丢弃最老的一帧，保护内存
            }
            m_queue.emplace(std::move(raw_json));
        }
        m_cv.notify_one();
    }

   private:
    void worker_loop() {
        while (true) {
            nlohmann::json msg;
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_cv.wait(lock,
                          [this]() { return !m_queue.empty() || !m_running; });
                if (m_queue.empty() && !m_running) break;
                msg = std::move(m_queue.front());
                m_queue.pop();
            }

            // 真正进行网络 IO 发送
            if (m_ws_sender && fglog_conn_registry::get().any_enabled()) {
                nlohmann::json ws_msg;
                ws_msg["type"] = "fglog";
                ws_msg["fglog"] = msg;
                if (!ws_msg["fglog"].is_discarded()) {
                    m_ws_sender(ws_msg);
                }
            }
        }
    }

    std::atomic<bool> m_running{false};
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::queue<nlohmann::json> m_queue;
    std::thread m_thread;
    WsSender m_ws_sender{nullptr};
};

// ============================================================================
// 3. Spdlog 纯文件每日轮转 Sink（绝不包含任何 WebSocket 代码）
// ============================================================================
template <typename Mutex>
class fglog_daily_sink : public spdlog::sinks::base_sink<Mutex> {
   public:
    explicit fglog_daily_sink(const std::string& base_filename,
                              int rotation_hour = 23,
                              int rotation_minute = 59) {
        m_daily_sink = std::make_shared<spdlog::sinks::daily_file_sink<Mutex>>(
            base_filename, rotation_hour, rotation_minute);
        m_daily_sink->set_pattern("%v");
    }

   protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        // 只有且纯粹的 本地磁盘 IO 写入（由 spdlog
        // 的后台线程负责，带每日自动轮转）
        m_daily_sink->log(msg);
    }

    void flush_() override { m_daily_sink->flush(); }

   private:
    std::shared_ptr<spdlog::sinks::daily_file_sink<Mutex>> m_daily_sink;
};

using fglog_daily_sink_mt = fglog_daily_sink<std::mutex>;

// ============================================================================
// 4. 全局后备管理器：调度 spdlog文件日志 与 独立的WS发送线程
// ============================================================================
class Backend {
   public:
    static Backend& getInstance() {
        static Backend instance;
        return instance;
    }

    void init(const std::string& jsonl_log_path) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_initialized) return;

        // 创建专属的每日轮转文件 sink
        auto sink =
            std::make_shared<fglog_daily_sink_mt>(jsonl_log_path, 23, 59);
        sink->set_pattern("%v");

        // 绑定到 spdlog 的后台线程池 (专为磁盘 IO 优化)
        m_file_logger = std::make_shared<spdlog::async_logger>(
            "fglog_jsonl", sink, spdlog::thread_pool(),
            spdlog::async_overflow_policy::overrun_oldest);

        spdlog::register_logger(m_file_logger);
        m_initialized = true;
    }

    void set_websocket_sender(
        std::function<void(const nlohmann::json&)> sender) {
        m_ws_engine.start(std::move(sender));
    }

    double get_elapsed_time() const {
        auto now = std::chrono::system_clock::now();
        auto milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch());
        return milliseconds.count();
    }

    // 核心调用入口：将 JSON 同时分发至 [本地文件线程] 与 [网络网络线程]
    void submit_json(const std::string& type, const std::string& topic,
                     nlohmann::json value) {
        if (!m_file_logger) return;

        nlohmann::json entry;
        entry["time"] = get_elapsed_time();
        entry["type"] = type;  // "state" | "log" | "value"
        entry["topic"] = topic;
        entry["value"] = std::move(value);

        std::string payload = entry.dump();

        // 1. 发往本地文件系统（进 spdlog 无锁队列，极大保证落地可靠，不丢文件）
        m_file_logger->info(payload);

        // 2. 发往 WebSocket
        // 专属线程（带丢帧保护，和文件系统在硬件IO和线程上完全物理隔离）
        m_ws_engine.try_push(std::move(entry));
    }

   private:
    Backend() = default;
    std::mutex m_mutex;
    bool m_initialized{false};
    std::shared_ptr<spdlog::logger> m_file_logger;
    WebSocketEngine m_ws_engine;
};

// ============================================================================
// 5. 对外公开的控制 API
// ============================================================================

inline void init(const std::string& log_path = "") {
    Backend::getInstance().init(log_path);
}

inline void set_websocket_sender(
    std::function<void(const nlohmann::json&)> sender) {
    Backend::getInstance().set_websocket_sender(std::move(sender));
}

// 供网络相关业务启停通道
inline void enable_ws_connection(size_t id) {
    fglog_conn_registry::get().enable(id);
}
inline void disable_ws_connection(size_t id) {
    fglog_conn_registry::get().disable(id);
}

inline bool check_ws_connection(size_t id) {
    return fglog_conn_registry::get().is_enabled(id);
}

// ---- 类型一：log (控制台输出字符串) ----
inline void log(const std::string& tag, const std::string& message) {
    nlohmann::json log_msg;
    log_msg["tag"] = tag;
    log_msg["message"] = message;

    Backend::getInstance().submit_json("log", "/system/logs",
                                       std::move(log_msg));
}

// ---- 类型二：state (专属于状态机等，纯 string 数据) ----
inline void publish_state(const std::string& topic,
                          const std::string& state_value) {
    Backend::getInstance().submit_json("state", topic, state_value);
}

// ---- 类型三：value (原生数值、数组及 Eigen 矩阵类) ----
template <typename T>
inline void publish_value(const std::string& topic, const T& value) {
    if constexpr (is_eigen_matrix<T>::value) {
        using CleanT = std::decay_t<T>;
        auto eval_vec = value.eval();

        if constexpr (CleanT::RowsAtCompileTime == 3 &&
                      CleanT::ColsAtCompileTime == 1) {
            std::vector<double> vec = {static_cast<double>(eval_vec.x()),
                                       static_cast<double>(eval_vec.y()),
                                       static_cast<double>(eval_vec.z())};
            Backend::getInstance().submit_json("value", topic, vec);
        } else if constexpr (CleanT::RowsAtCompileTime == 2 &&
                             CleanT::ColsAtCompileTime == 1) {
            std::vector<double> vec = {static_cast<double>(eval_vec.x()),
                                       static_cast<double>(eval_vec.y())};
            Backend::getInstance().submit_json("value", topic, vec);
        } else {
            std::vector<std::vector<double>> mat(
                eval_vec.rows(), std::vector<double>(eval_vec.cols()));
            for (int r = 0; r < eval_vec.rows(); ++r) {
                for (int c = 0; c < eval_vec.cols(); ++c) {
                    mat[r][c] = static_cast<double>(eval_vec(r, c));
                }
            }
            Backend::getInstance().submit_json("value", topic, mat);
        }
    } else {
        Backend::getInstance().submit_json("value", topic, value);
    }
}

}  // namespace fglog