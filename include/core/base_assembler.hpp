#pragma once
#include <utility>

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
struct AnyType {
    template <typename T>
    operator T&() const;
    template <typename T>
    operator T&&() const;
};

// 辅助模板：不管传入什么，都返回 AnyType
template <std::size_t>
using AnyType_t = AnyType;

// 泛型探测器：利用 std::index_sequence 展开特定数量的 AnyType
template <typename T, typename Tag, typename Seq, typename = void>
struct defines_tag_impl : std::false_type {};

// 魔法在这里：AnyType_t<Is>{}... 会根据序列长度自动展开成 N 个 AnyType{}
template <typename T, typename Tag, std::size_t... Is>
struct defines_tag_impl<
    T, Tag, std::index_sequence<Is...>,
    std::void_t<decltype(T::setup(std::declval<Tag>(), AnyType_t<Is>{}...))>>
    : std::true_type {};

// 递归查找：从 N 个参数一直试到 0 个参数
template <typename T, typename Tag, std::size_t N>
constexpr bool check_all_arities() {
    // 先测试 N 个参数能不能匹配
    if constexpr (defines_tag_impl<T, Tag,
                                   std::make_index_sequence<N>>::value) {
        return true;
    } else if constexpr (N > 0) {
        // 不行的话，测试 N-1 个参数
        return check_all_arities<T, Tag, N - 1>();
    } else {
        return false;
    }
}

// 最终的暴露接口：支持探测 0 到 10 个参数！
template <typename T, typename Tag>
inline constexpr bool defines_tag_v = check_all_arities<T, Tag, 10>();

template <typename T, typename Tag, typename... Args>
struct has_setup_impl {
    template <typename U>
    static auto test(int) -> decltype(U::setup(std::declval<Tag>(),
                                               std::declval<Args>()...),
                                      std::true_type{});

    template <typename>
    static std::false_type test(...);

    using type = decltype(test<T>(0));
};

template <typename T, typename Tag, typename... Args>
inline constexpr bool has_setup_v =
    has_setup_impl<T, Tag, Args...>::type::value;

template <typename... Types>
constexpr bool always_false_v = false;

}  // namespace detail

template <typename... Features>
struct BaseAssembler {
   public:
    // 核心泛型入口：支持 0 个、1 个或 N 个不同类型的参数！
    template <typename Tag, typename... Args>
    static void setup(Args&&... args) {
        (call_setup<Features, Tag>(std::forward<Args>(args)...), ...);
    }

    // 🌟 升级后的 has_feature_for：支持传入任意数量的类型进行探测 🌟
    template <typename Tag>
    static constexpr bool has_feature_for =
        (detail::defines_tag_v<Features, Tag> || ... || false);

   private:
    template <typename Feature, typename Tag, typename... Args>
    static void call_setup(Args&&... args) {
        if constexpr (detail::has_setup_v<Feature, Tag, Args...>) {
            // 情况 A：Tag 存在，且参数完美匹配。直接调用！
            Feature::setup(Tag{}, std::forward<Args>(args)...);

        } else if constexpr (detail::defines_tag_v<Feature, Tag>) {
            // 情况 B：Tag
            // 存在，但参数不匹配！这说明业务代码写错了，直接抛出鲜红的硬报错！
            static_assert(
                detail::always_false_v<Feature, Tag, Args...>,
                "[Dankong Assembler Error] -> PARAMETER SIGNATURE MISMATCH!");
        } else {
            // 情况 C：压根没定义这个 Tag。默默忽略，安全通过 (SFINAE)
        }
    }
};