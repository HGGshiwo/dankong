#pragma once

#include <foxglove/Log.pb.h>
#include <foxglove/Vector2.pb.h>
#include <foxglove/Vector3.pb.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>  // 引入 Protobuf 反射基类
#include <google/protobuf/wrappers.pb.h>

#include <boost/filesystem/path.hpp>
#include <chrono>
#include <filesystem>
#include <foxglove/foxglove.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>
#include <iomanip>
#include <magic_enum.hpp>  // 引入神库
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <type_traits>  // 用于类型判断
#include <unordered_map>
#include <unordered_set>

#include "Eigen/Core"

// 新增异步所需头文件
#include <atomic>
#include <condition_variable>
#include <functional>
#include <queue>
#include <thread>

#include "core/global_config.hpp"
#include "spdlog/spdlog.h"
#include "utils/get_executable_path.hpp"

// 探测一个类型是否是 Eigen 矩阵/向量或其延迟计算表达式
template <typename T, typename = void>
struct is_eigen_matrix : std::false_type {};

template <typename T>
struct is_eigen_matrix<
    T, std::void_t<typename std::decay_t<T>::Scalar,
                   decltype(std::decay_t<T>::RowsAtCompileTime)>>
    : std::true_type {};

class FGLoggerBackend {
   private:
    // 缓存每个 Topic 的动态描述符，防止重复构建
    struct DynamicEnumContext {
        std::unique_ptr<google::protobuf::DescriptorPool> pool;
        std::unique_ptr<google::protobuf::DynamicMessageFactory> factory;
        const google::protobuf::Descriptor* msg_desc = nullptr;
        const google::protobuf::EnumDescriptor* enum_desc = nullptr;
    };
    std::unordered_map<std::string, DynamicEnumContext> m_enum_factories;

    std::string sanitize_identifier(const std::string& name) {
        if (name.empty()) return "UNKNOWN";
        std::string clean = name;
        for (char& c : clean) {
            if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
                c = '_';
            }
        }
        if (std::isdigit(static_cast<unsigned char>(clean[0]))) {
            clean = "_" + clean;
        }
        return clean;
    }

    std::string get_safe_value_name(const std::string& name, int id) {
        std::string cleaned = sanitize_identifier(name);
        bool has_special = false;
        for (char c : name) {
            if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
                has_special = true;
                break;
            }
        }
        if (has_special) {
            return cleaned + "_" + std::to_string(id);
        }
        return cleaned;
    }

    void build_custom_enum_schema(
        const std::string& topic, const std::string& enum_name,
        const std::map<int, std::string>& id_name_map) {
        google::protobuf::FileDescriptorProto file_proto;
        std::string safe_topic_name = topic;
        for (char& c : safe_topic_name) {
            if (c == '/') c = '_';
        }
        file_proto.set_name(safe_topic_name + "_schema.proto");

        std::string safe_enum_name = sanitize_identifier(enum_name);

        auto* enum_proto = file_proto.add_enum_type();
        enum_proto->set_name(safe_enum_name);

        for (const auto& [id, name] : id_name_map) {
            auto* proto_val = enum_proto->add_value();
            proto_val->set_name(get_safe_value_name(name, id));
            proto_val->set_number(id);
        }

        std::string msg_name = safe_enum_name + "Message";
        auto* msg_proto = file_proto.add_message_type();
        msg_proto->set_name(msg_name);
        auto* field = msg_proto->add_field();
        field->set_name("value");
        field->set_number(1);
        field->set_type(google::protobuf::FieldDescriptorProto::TYPE_ENUM);
        field->set_type_name(safe_enum_name);

        auto ctx = DynamicEnumContext{};
        ctx.pool = std::make_unique<google::protobuf::DescriptorPool>();
        ctx.pool->BuildFile(file_proto);
        ctx.factory = std::make_unique<google::protobuf::DynamicMessageFactory>(
            ctx.pool.get());
        ctx.msg_desc = ctx.pool->FindMessageTypeByName(msg_name);
        ctx.enum_desc = ctx.pool->FindEnumTypeByName(safe_enum_name);

        google::protobuf::FileDescriptorSet fd_set;
        *fd_set.add_file() = file_proto;
        std::string schema_bytes;
        fd_set.SerializeToString(&schema_bytes);

        foxglove::Schema fg_schema{
            msg_name, "protobuf",
            reinterpret_cast<const std::byte*>(schema_bytes.data()),
            schema_bytes.size()};
        auto res = foxglove::RawChannel::create(topic, "protobuf", fg_schema);
        if (res.has_value()) {
            m_channels.insert_or_assign(topic, std::move(res.value()));
        }

        m_enum_factories.emplace(topic, std::move(ctx));
    }

    template <typename EnumType>
    void build_dynamic_enum_schema(const std::string& topic) {
        std::string enum_name =
            std::string(magic_enum::enum_type_name<EnumType>());
        std::map<int, std::string> id_name_map;
        for (const auto& [val, name] : magic_enum::enum_entries<EnumType>()) {
            id_name_map[static_cast<int>(val)] = std::string(name);
        }
        build_custom_enum_schema(topic, enum_name, id_name_map);
    }

    // ==========================================
    // 后台真正执行序列化和写入的内部函数
    // ==========================================
    template <typename T>
    void publish_impl(const std::string& topic, const T& protobuf_msg) {
        std::lock_guard<std::mutex> lock(m_state_mutex);  // 保护 map 状态
        auto it = m_channels.find(topic);
        if (it == m_channels.end()) {
            auto desc = T::descriptor();
            std::string schema_bytes = get_file_descriptor_set_bytes(desc);
            foxglove::Schema schema{
                desc->full_name(), "protobuf",
                reinterpret_cast<const std::byte*>(schema_bytes.data()),
                schema_bytes.size()};
            auto res = foxglove::RawChannel::create(topic, "protobuf", schema);
            if (res.has_value()) {
                auto emplace_res =
                    m_channels.emplace(topic, std::move(res.value()));
                it = emplace_res.first;
            } else {
                spdlog::error("Failed to create channel for {}", topic);
                return;
            }
        }
        std::string serialized;
        protobuf_msg.SerializeToString(&serialized);
        it->second.log(reinterpret_cast<const std::byte*>(serialized.data()),
                       serialized.size());
    }

    void publish_value_locked_impl(const std::string& topic, int value) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        auto it = m_enum_factories.find(topic);
        if (it == m_enum_factories.end()) return;

        auto& ctx = it->second;
        std::unique_ptr<google::protobuf::Message> msg(
            ctx.factory->GetPrototype(ctx.msg_desc)->New());
        const auto* mode_field = ctx.msg_desc->FindFieldByName("value");
        const auto* enum_val_desc = ctx.enum_desc->FindValueByNumber(value);
        if (enum_val_desc) {
            msg->GetReflection()->SetEnum(msg.get(), mode_field, enum_val_desc);
        } else {
            msg->GetReflection()->SetEnumValue(msg.get(), mode_field, value);
        }

        std::string serialized;
        msg->SerializeToString(&serialized);
        auto chan_it = m_channels.find(topic);
        if (chan_it != m_channels.end()) {
            chan_it->second.log(
                reinterpret_cast<const std::byte*>(serialized.data()),
                serialized.size());
        }
    }

    void publish_value_locked_impl(const std::string& topic,
                                   const std::string& value) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        auto it = m_enum_factories.find(topic);
        if (it == m_enum_factories.end()) return;

        auto& ctx = it->second;
        std::unique_ptr<google::protobuf::Message> msg(
            ctx.factory->GetPrototype(ctx.msg_desc)->New());
        const auto* mode_field = ctx.msg_desc->FindFieldByName("value");

        std::string sanitized_query = sanitize_identifier(value);
        const auto* enum_val_desc =
            ctx.enum_desc->FindValueByName(sanitized_query);
        if (!enum_val_desc) {
            for (int i = 0; i < ctx.enum_desc->value_count(); ++i) {
                const auto* val = ctx.enum_desc->value(i);
                if (val->name() ==
                    sanitized_query + "_" + std::to_string(val->number())) {
                    enum_val_desc = val;
                    break;
                }
            }
        }
        if (!enum_val_desc) return;

        msg->GetReflection()->SetEnum(msg.get(), mode_field, enum_val_desc);

        std::string serialized;
        msg->SerializeToString(&serialized);
        auto chan_it = m_channels.find(topic);
        if (chan_it != m_channels.end()) {
            chan_it->second.log(
                reinterpret_cast<const std::byte*>(serialized.data()),
                serialized.size());
        }
    }

    // 修复 Bug：被移入类内部的 dynamic state 发布实现
    void publish_dynamic_state_impl(const std::string& topic, int state_id) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        auto it = m_enum_factories.find(topic);
        if (it == m_enum_factories.end()) {
            spdlog::warn("[fglog] enum topic: {} not found", topic);
            return;
        };

        auto& ctx = it->second;
        std::unique_ptr<google::protobuf::Message> msg(
            ctx.factory->GetPrototype(ctx.msg_desc)->New());
        const auto* mode_field = ctx.msg_desc->FindFieldByName("value");
        const auto* enum_val_desc = ctx.enum_desc->FindValueByNumber(state_id);

        if (enum_val_desc != nullptr) {
            msg->GetReflection()->SetEnum(msg.get(), mode_field, enum_val_desc);
            std::string serialized;
            msg->SerializeToString(&serialized);
            auto chan_it = m_channels.find(topic);
            if (chan_it != m_channels.end()) {
                chan_it->second.log(
                    reinterpret_cast<const std::byte*>(serialized.data()),
                    serialized.size());
            }
        }
    }

   public:
    static FGLoggerBackend& getInstance() {
        static FGLoggerBackend instance;
        return instance;
    }

    ~FGLoggerBackend() { shutdown(); }

    void shutdown() {
        m_running = false;
        m_queue_cv.notify_all();
        if (m_worker_thread.joinable()) {
            m_worker_thread.join();
        }
        std::lock_guard<std::mutex> lock(m_state_mutex);
        if (m_writer.has_value()) {
            m_writer->close();
            m_writer.reset();
            spdlog::info("[fglog] MCAP Writer closed cleanly.");
        }
    }

    bool init(uint16_t port = 8765, const std::string& mcap_filename = "") {
        std::lock_guard<std::mutex> lock(m_state_mutex);

        // 初始化并拉起后台异步工作线程
        if (!m_running) {
            m_running = true;
            m_worker_thread = std::thread(&FGLoggerBackend::worker_loop, this);
        }

        foxglove::WebSocketServerOptions ws_options;
        ws_options.port = port;
        ws_options.host = "0.0.0.0";
        auto serverResult =
            foxglove::WebSocketServer::create(std::move(ws_options));
        if (serverResult.has_value()) {
            m_server = std::move(serverResult.value());
            spdlog::info("[fglog] Live server on port: {}", port);
        } else {
            spdlog::error("[fglog] Live server start failed on port {}: {}",
                          port, foxglove::strerror(serverResult.error()));
        }

        if (!mcap_filename.empty()) {
            namespace fs = std::filesystem;
            fs::path input_path(mcap_filename);
            fs::path dir = input_path.parent_path();
            if (!dir.empty() && !fs::exists(dir)) {
                fs::create_directories(dir);
            }

            auto now = std::chrono::system_clock::now();
            std::time_t t_now = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&t_now), "%Y-%m-%d-%H-%M-%S");
            std::string ts = ss.str();

            std::string stem = input_path.stem().string();
            std::string ext = input_path.extension().string();
            std::string base_name = stem + "-" + ts;
            fs::path final_path = dir.empty() ? fs::path(base_name + ext)
                                              : (dir / (base_name + ext));

            int counter = 1;
            while (fs::exists(final_path)) {
                std::string new_name =
                    base_name + "_" + std::to_string(counter) + ext;
                final_path =
                    dir.empty() ? fs::path(new_name) : (dir / new_name);
                counter++;
            }

            // 此处兼容 boost::filesystem (或者可以直接换成 std::filesystem)
            std::filesystem::create_directories(final_path.parent_path());

            std::string final_path_str = final_path.string();
            foxglove::McapWriterOptions mcap_options;
            mcap_options.path = final_path_str;
            auto writerResult = foxglove::McapWriter::create(mcap_options);
            if (writerResult.has_value()) {
                m_writer = std::move(writerResult.value());
                spdlog::info("[fglog] Recording to {}", final_path.string());
            } else {
                spdlog::error("[fglog] Create log file failed for path {}: {}",
                              final_path.string(),
                              foxglove::strerror(writerResult.error()));
            }
        }

        auto desc = foxglove::Log::descriptor();
        std::string schema_bytes = get_file_descriptor_set_bytes(desc);
        foxglove::Schema log_schema{
            desc->full_name(), "protobuf",
            reinterpret_cast<const std::byte*>(schema_bytes.data()),
            schema_bytes.size()};
        auto logRes = foxglove::RawChannel::create("/system/logs", "protobuf",
                                                   log_schema);
        if (logRes.has_value()) {
            m_channels.insert_or_assign("/system/logs",
                                        std::move(logRes.value()));
        }

        return true;
    }

    // ==========================================
    // 异步投递 API (极速返回，不阻塞前端)
    // ==========================================
    template <typename T>
    void publish(const std::string& topic, T protobuf_msg) {
        // 注意：protobuf_msg 被按值捕获，安全跨线程传递
        post_task([this, topic, msg = std::move(protobuf_msg)]() {
            this->publish_impl(topic, msg);
        });
    }

    template <typename EnumType>
    void publish_enum(const std::string& topic, EnumType enum_value) {
        post_task([this, topic, val = static_cast<int>(enum_value)]() {
            {
                std::lock_guard<std::mutex> lock(m_state_mutex);
                if (m_enum_factories.find(topic) == m_enum_factories.end()) {
                    build_dynamic_enum_schema<EnumType>(topic);
                }
            }
            // 解锁后执行写入
            this->publish_value_locked_impl(topic, val);
        });
    }

    void publish_dynamic_state(const std::string& topic, int state_id) {
        post_task([this, topic, state_id]() {
            this->publish_dynamic_state_impl(topic, state_id);
        });
    }

    void publish_value(const std::string& topic, int value) {
        post_task([this, topic, value]() {
            this->publish_value_locked_impl(topic, value);
        });
    }

    void publish_value(const std::string& topic, const std::string& value) {
        post_task([this, topic, value]() {
            this->publish_value_locked_impl(topic, value);
        });
    }

    void log(const std::string& tag, foxglove::LogLevel level,
             const std::string& message) {
        foxglove::Log log_msg;
        log_msg.set_name(tag);
        log_msg.set_level(
            static_cast<foxglove::Log::Level>(static_cast<int>(level)));
        log_msg.set_message(message);
        // 直接复用泛型 publish，丢入异步队列
        publish("/system/logs", log_msg);
    }

    // 注册接口 (依然保持同步，因为只在系统启动期调用)
    template <typename EnumType>
    void register_enum(const std::string& topic,
                       const std::string& enum_name = "") {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        std::string name =
            enum_name.empty()
                ? std::string(magic_enum::enum_type_name<EnumType>())
                : enum_name;
        std::map<int, std::string> id_name_map;
        for (const auto& [val, n] : magic_enum::enum_entries<EnumType>()) {
            id_name_map[static_cast<int>(val)] = std::string(n);
        }
        build_custom_enum_schema(topic, name, id_name_map);
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::map<int, std::string>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        build_custom_enum_schema(topic, enum_name, map);
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::unordered_map<int, std::string>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        std::map<int, std::string> id_name_map(map.begin(), map.end());
        build_custom_enum_schema(topic, enum_name, id_name_map);
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::map<std::string, int>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        std::map<int, std::string> id_name_map;
        for (const auto& [name, id] : map) {
            id_name_map[id] = name;
        }
        build_custom_enum_schema(topic, enum_name, id_name_map);
    }

    void register_enum(const std::string& topic, const std::string& enum_name,
                       const std::unordered_map<std::string, int>& map) {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        std::map<int, std::string> id_name_map;
        for (const auto& [name, id] : map) {
            id_name_map[id] = name;
        }
        build_custom_enum_schema(topic, enum_name, id_name_map);
    }

   private:
    FGLoggerBackend() = default;

    void collect_dependencies(
        const google::protobuf::FileDescriptor* file,
        std::unordered_set<const google::protobuf::FileDescriptor*>& visited,
        google::protobuf::FileDescriptorSet& fd_set) {
        if (!file || !visited.insert(file).second) {
            return;
        }
        for (int i = 0; i < file->dependency_count(); ++i) {
            collect_dependencies(file->dependency(i), visited, fd_set);
        }
        file->CopyTo(fd_set.add_file());
    }

    std::string get_file_descriptor_set_bytes(
        const google::protobuf::Descriptor* desc) {
        google::protobuf::FileDescriptorSet fd_set;
        std::unordered_set<const google::protobuf::FileDescriptor*> visited;
        collect_dependencies(desc->file(), visited, fd_set);
        return fd_set.SerializeAsString();
    }

    // 将任务压入队列并唤醒后台线程
    void post_task(std::function<void()> task) {
        if (!m_running) {
            spdlog::warn("[fglog] post_task called before init! Data dropped.");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_queue_mutex);
            m_task_queue.emplace(std::move(task));
        }
        m_queue_cv.notify_one();
    }

    // 后台消费线程
    void worker_loop() {
        auto last_flush_time = std::chrono::steady_clock::now();
        while (true) {
            std::function<void()> task;
            bool is_idle = false;
            {
                std::unique_lock<std::mutex> lock(m_queue_mutex);
                // 每次最多等待 1 秒，超时则唤醒进行空闲检测并触发保存
                bool notified = m_queue_cv.wait_for(
                    lock, std::chrono::seconds(1),
                    [this]() { return !m_task_queue.empty() || !m_running; });

                if (m_task_queue.empty() && !m_running) {
                    break;
                }

                if (!notified && m_task_queue.empty()) {
                    is_idle = true;
                } else {
                    task = std::move(m_task_queue.front());
                    m_task_queue.pop();
                }
            }

            if (task) {
                task();  // 执行反序列化与 I/O 写入
            }

            // 执行自动保存 (空闲时，或者距离上次保存超过 1 秒)
            auto now = std::chrono::steady_clock::now();
            if (is_idle || (now - last_flush_time >= std::chrono::seconds(1))) {
                std::lock_guard<std::mutex> state_lock(m_state_mutex);
                if (m_writer.has_value()) {
                    m_writer->flush();
                }
                last_flush_time = now;
            }
        }
    }

    // 原 m_mutex 更名为 m_state_mutex，专用于保护 m_channels 和工厂映射
    std::mutex m_state_mutex;
    std::optional<foxglove::WebSocketServer> m_server;
    std::optional<foxglove::McapWriter> m_writer;
    std::unordered_map<std::string, foxglove::RawChannel> m_channels;

    // === 异步队列组件 ===
    std::queue<std::function<void()>> m_task_queue;
    std::mutex m_queue_mutex;
    std::condition_variable m_queue_cv;
    std::atomic<bool> m_running{false};
    std::thread m_worker_thread;
};

// ================= 对外暴露的极简 API =================
namespace fglog {
inline void init(uint16_t port = 8765, const std::string& mcap_path = "") {
    FGLoggerBackend::getInstance().init(port, mcap_path);
}

inline void close() {
    FGLoggerBackend::getInstance().shutdown();
}

inline void info(const std::string& tag, const std::string& msg) {
    FGLoggerBackend::getInstance().log(tag, foxglove::LogLevel::Info, msg);
}
inline void warn(const std::string& tag, const std::string& msg) {
    FGLoggerBackend::getInstance().log(tag, foxglove::LogLevel::Warn, msg);
}
inline void error(const std::string& tag, const std::string& msg) {
    FGLoggerBackend::getInstance().log(tag, foxglove::LogLevel::Error, msg);
}

inline void info(const std::string& msg) {
    info("SYS", msg);
}

template <typename EnumType>
inline void publish_enum(const std::string& topic, EnumType enum_value) {
    FGLoggerBackend::getInstance().publish_enum(topic, enum_value);
}

// 修复点：调用后端提供的安全方法，不再越界访问私有变量
inline void publish_enum(const std::string& topic, int state_id) {
    FGLoggerBackend::getInstance().publish_dynamic_state(topic, state_id);
}

// 注册等函数保留不变...
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
    // 场景 1：如果你传入的本身就是 Protobuf Message (比如 foxglove::Pose)
    if constexpr (std::is_base_of_v<google::protobuf::Message, T>) {
        FGLoggerBackend::getInstance().publish(topic, value);
    }
    // 场景 2：如果你传入的是布尔值 (bool)
    else if constexpr (std::is_same_v<T, bool>) {
        google::protobuf::BoolValue msg;
        msg.set_value(value);
        FGLoggerBackend::getInstance().publish(topic, msg);
    }
    // 场景 3：如果你传入的是浮点数 (float / double)
    else if constexpr (std::is_floating_point_v<T>) {
        google::protobuf::DoubleValue msg;
        msg.set_value(static_cast<double>(value));
        FGLoggerBackend::getInstance().publish(topic, msg);
    }
    // 场景 4：如果你传入的是整型 (int / uint32_t / long 等)
    else if constexpr (std::is_integral_v<T>) {
        google::protobuf::Int64Value msg;
        msg.set_value(static_cast<int64_t>(value));
        FGLoggerBackend::getInstance().publish(topic, msg);
    }
    // 场景 5：如果你传入的是字符串 (std::string / const char*)
    else if constexpr (std::is_convertible_v<T, std::string>) {
        google::protobuf::StringValue msg;
        msg.set_value(std::string(value));
        FGLoggerBackend::getInstance().publish(topic, msg);
    }
    // 场景 6：原生支持任意 Eigen::Vector2 / Vector3 及其数学表达式
    else if constexpr (is_eigen_matrix<T>::value) {
        using CleanT = std::decay_t<T>;

        // 匹配 3D 向量 (无论是 double 还是 float)
        if constexpr (CleanT::RowsAtCompileTime == 3 &&
                      CleanT::ColsAtCompileTime == 1) {
            foxglove::Vector3 msg;
            auto eval_vec =
                value.eval();  // 极其重要：强制计算可能存在的延迟表达式
            msg.set_x(eval_vec.x());
            msg.set_y(eval_vec.y());
            msg.set_z(eval_vec.z());
            FGLoggerBackend::getInstance().publish(topic, msg);
        }
        // 匹配 2D 向量
        else if constexpr (CleanT::RowsAtCompileTime == 2 &&
                           CleanT::ColsAtCompileTime == 1) {
            foxglove::Vector2 msg;
            auto eval_vec = value.eval();
            msg.set_x(eval_vec.x());
            msg.set_y(eval_vec.y());
            FGLoggerBackend::getInstance().publish(topic, msg);
        } else {
            static_assert(sizeof(T) == 0,
                          "[fglog] Only Eigen Vector2 and Vector3 are "
                          "supported for auto-publishing!");
        }
    } else {
        static_assert(sizeof(T) == 0,
                      "[fglog] Unsupported primitive type for publish!");
    }
}

}  // namespace fglog

// --- 下方的 register_enum_to_fg 保持不变，供你原有的初始化逻辑使用 ---

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
        fglog::init(port, LOG_DIR.string());  // 确保转为 std::string 传入

    } catch (const std::exception& ex) {
        spdlog::error("FoxGlove Log initialization failed: {}", ex.what());
    }
}