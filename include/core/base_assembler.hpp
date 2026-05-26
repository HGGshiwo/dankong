// core/robot_assembler.hpp
#pragma once
#include <nlohmann/json.hpp>

#include "core/base_context.hpp"

// 魔法1：自动提取所有的 Context 并利用多重继承扁平化组装
template <typename... Context>
struct ContextGenerator : public BaseRobotContext<ContextGenerator<Context...>>,
                          public Context... {
    // 自动把 registry 传给所有的子 Context
    ContextGenerator()
        : BaseRobotContext<ContextGenerator<Context...>>(),
          Context(this->state_registry)... {}
};

// 魔法1：自动提取所有的 Config 并利用多重继承扁平化组装
template <typename... Configs>
struct ConfigGenerator : public Configs... {
    friend void from_json(const nlohmann::json& j, ConfigGenerator& c) {
        // 定义一个泛型 lambda 拦截解析过程
        auto try_parse = [&j](auto& base_config) {
            using ConfigType = std::decay_t<decltype(base_config)>;
            try {
                // 去除 nlohmann:: 前缀，依赖 ADL 查找
                from_json(j, base_config);
            } catch (const nlohmann::json::exception& e) {
                // 捕获异常，提取出错的结构体类型名
                std::string type_name = typeid(ConfigType).name();
                std::string error_msg = "配置解析失败! 出错模块: [" +
                                        type_name + "], 报错信息: " + e.what();

                // 1. 你可以选择打印日志
                std::cerr << error_msg << std::endl;

                // 2. 建议将原始异常包装后抛出，防止程序带着错误配置继续运行
                throw std::runtime_error(error_msg);
            }
        };

        // 使用折叠表达式逐个调用 lambda
        (try_parse(static_cast<Configs&>(c)), ...);
    }

    friend void to_json(nlohmann::json& j, const ConfigGenerator& c) {
        (to_json(j, static_cast<const Configs&>(c)), ...);
    }
};

namespace detail {
// SFINAE detector for register_ros
template <typename T, typename Adapter, typename = void>
struct has_register_ros : std::false_type {};
template <typename T, typename Adapter>
struct has_register_ros<T, Adapter,
                        std::void_t<decltype(T::register_ros(
                            std::declval<std::shared_ptr<Adapter>&>()))>>
    : std::true_type {};

template <typename T, typename RobotContext, typename = void>
struct has_init : std::false_type {};
template <typename T, typename RobotContext>
struct has_init<T, RobotContext,
                std::void_t<decltype(T::init(std::declval<RobotContext&>()))>>
    : std::true_type {};

// SFINAE detector for register_web
template <typename T, typename Adapter, typename = void>
struct has_register_web : std::false_type {};
template <typename T, typename Adapter>
struct has_register_web<T, Adapter,
                        std::void_t<decltype(T::register_web(
                            std::declval<std::shared_ptr<Adapter>&>()))>>
    : std::true_type {};
// SFINAE detector for register_listeners
template <typename T, typename Engine, typename = void>
struct has_register_listeners : std::false_type {};
template <typename T, typename Engine>
struct has_register_listeners<T, Engine,
                              std::void_t<decltype(T::register_listeners(
                                  std::declval<std::shared_ptr<Engine>&>()))>>
    : std::true_type {};
// SFINAE detector for register_udp
template <typename T, typename Adapter, typename = void>
struct has_register_udp : std::false_type {};
template <typename T, typename Adapter>
struct has_register_udp<T, Adapter,
                        std::void_t<decltype(T::register_udp(
                            std::declval<std::shared_ptr<Adapter>&>()))>>
    : std::true_type {};
}  // namespace detail
template <typename... Features>
struct BaseAssembler {
    // Provide compile-time boolean flags
    template <typename RosAdapter>
    static constexpr bool has_ros =
        (detail::has_register_ros<Features, RosAdapter>::value || ... || false);
    template <typename RobotType>
    static constexpr bool has_init =
        (detail::has_init<Features, RobotType>::value || ... || false);
    template <typename WebAdapter>
    static constexpr bool has_web =
        (detail::has_register_web<Features, WebAdapter>::value || ... || false);
    template <typename EngineType>
    static constexpr bool has_listeners =
        (detail::has_register_listeners<Features, EngineType>::value || ... ||
         false);
    template <typename UdpAdapter>
    static constexpr bool has_udp =
        (detail::has_register_udp<Features, UdpAdapter>::value || ... || false);

   private:
    // Helper function to evaluate SFINAE conditions per feature
    template <typename Feature, typename RobotContext>
    static void call_init(RobotContext& ctx) {
        if constexpr (detail::has_init<Feature, RobotContext>::value) {
            Feature::init(ctx);
        }
    }

    template <typename Feature, typename RosAdapter>
    static void call_setup_ros(std::shared_ptr<RosAdapter>& ros) {
        if constexpr (detail::has_register_ros<Feature, RosAdapter>::value) {
            Feature::register_ros(ros);
        }
    }
    template <typename Feature, typename WebAdapter>
    static void call_setup_web(std::shared_ptr<WebAdapter>& web) {
        if constexpr (detail::has_register_web<Feature, WebAdapter>::value) {
            Feature::register_web(web);
        }
    }
    template <typename Feature, typename EngineType>
    static void call_setup_listeners(std::shared_ptr<EngineType>& engine) {
        if constexpr (detail::has_register_listeners<Feature,
                                                     EngineType>::value) {
            Feature::register_listeners(engine);
        }
    }
    template <typename Feature, typename UdpAdapter>
    static void call_setup_udp(std::shared_ptr<UdpAdapter>& udp) {
        if constexpr (detail::has_register_udp<Feature, UdpAdapter>::value) {
            Feature::register_udp(udp);
        }
    }

   public:
    template <typename RobotContext>
    static void setup_init(RobotContext& ctx) {
        (call_init<Features>(ctx), ...);
    }

    template <typename RosAdapter>
    static void setup_ros(std::shared_ptr<RosAdapter>& ros) {
        (call_setup_ros<Features>(ros), ...);
    }
    template <typename WebAdapter>
    static void setup_web(std::shared_ptr<WebAdapter>& web) {
        (call_setup_web<Features>(web), ...);
    }
    template <typename EngineType>
    static void setup_listeners(std::shared_ptr<EngineType>& engine) {
        (call_setup_listeners<Features>(engine), ...);
    }
    template <typename UdpAdapter>
    static void setup_udp(std::shared_ptr<UdpAdapter>& udp) {
        (call_setup_udp<Features>(udp), ...);
    }
};