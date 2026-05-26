#pragma once
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace dk {

struct ParamMeta {
    std::string name;
    std::string help;
    std::string group;
    bool hidden = false;
};

// 静态注册表，用于存储所有配置项的元数据
class ParamRegistry {
   public:
    static std::map<std::string, ParamMeta>& GetMap() {
        static std::map<std::string, ParamMeta> instance;
        return instance;
    }
};

template <typename T>
struct Param {
    T value_;
    mutable std::mutex mtx_;

    // 魔法构造函数：在对象初始化时自动注册元数据
    Param(T v, const char* key, const char* name, const char* help,
          const char* group, bool hidden = false)
        : value_(std::move(v)) {
        ParamRegistry::GetMap()[key] = {name, help, group, hidden};
    }

    // 默认构造（用于不带元数据的临时对象）
    Param(T v = T{}) : value_(std::move(v)) {}

    // 拷贝构造函数
    Param(const Param& other) { value_ = other.get(); }

    // 赋值运算符
    Param& operator=(const Param& other) {
        set(other.get());
        return *this;
    }

    Param& operator=(const T& v) {
        set(v);
        return *this;
    }

    T get() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return value_;
    }

    void set(const T& v) {
        std::lock_guard<std::mutex> lock(mtx_);
        value_ = v;
    }

    // 隐式转换为原始类型（返回拷贝以保证线程安全）
    operator T() const { return get(); }

    // 方便访问
    T operator()() const { return get(); }
    T& operator()() {
        static_assert(std::is_same_v<T, void>,
                      "Direct reference access is not allowed for thread "
                      "safety. Use get()/set().");
        return value_;
    }

    // JSON 序列化
    friend void to_json(nlohmann::json& j, const Param& p) { j = p.get(); }
    friend void from_json(const nlohmann::json& j, Param& p) {
        p.set(j.get<T>());
    }
};

// 1. 常规参数宏：提取变量名作为 key 和 name，并保留 help
#define INIT_PARAM(var_str, default_val, help) \
    {default_val, var_str, var_str, help, __group_name, false}

// 2. 隐藏参数宏：不需要 help 文本（传入空字符串即可，或根据你实际需求给某个
// hidden 标志位赋 true）
#define INIT_HIDDEN_PARAM(var_str, default_val) \
    {default_val, var_str, var_str, "", __group_name, true}

}  // namespace dk