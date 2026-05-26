#pragma once
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <dk_auto_json.hpp>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <shared_mutex>
#include <string>
#include <type_traits>  // 必须包含这个
#include <type_traits>
#include <vector>

// ==========================================
// 1. C++17 SFINAE: 检查类型 T 是否支持 == 运算符
// ==========================================
template <typename T, typename = void>
struct is_equality_comparable : std::false_type {};

template <typename T>
struct is_equality_comparable<
    T, std::void_t<decltype(std::declval<T>() == std::declval<T>())>>
    // 关键修改：不仅要 == 合法，还要保证其结果能转换为 bool
    : std::is_convertible<decltype(std::declval<T>() == std::declval<T>()),
                          bool> {};

template <typename T>
inline constexpr bool is_equality_comparable_v =
    is_equality_comparable<T>::value;

template <typename T>
class DirtyVar {
   private:
    T data_;
    mutable std::shared_mutex mtx_;
    std::atomic<bool> dirty_{true};  // 初始默认为脏

   public:
    explicit DirtyVar(T init_val = T{}) : data_(std::move(init_val)) {}

    template <typename... Args>
    void emplace(Args&&... args) {
        // 利用参数直接构造出一个临时的 T，然后转发给现在的 store 处理
        store(T(std::forward<Args>(args)...));
    }

    template <typename... Args>
    static DirtyVar create(Args&&... args) {
        // 利用完美转发先构造出 T，然后传入 DirtyVar 的构造函数
        // C++17 保证了这里不会调用任何拷贝或移动构造函数
        return DirtyVar(T{std::forward<Args>(args)...});
    }

    // 禁用拷贝和移动，防止外部注册表中保存的引用或指针失效
    DirtyVar(const DirtyVar&) = delete;
    DirtyVar& operator=(const DirtyVar&) = delete;

    template <typename U>
    void store(U&& new_data) {
        // C++17 静态检查：确保 U 可以用来构造或赋值给
        // T，提前拦截非法类型给出清晰报错
        static_assert(std::is_constructible_v<T, std::decay_t<U>> ||
                          std::is_assignable_v<T&, U>,
                      "store error: The provided type cannot be converted or "
                      "assigned to T!");

        {
            std::unique_lock<std::shared_mutex> lock(mtx_);

            if constexpr (is_equality_comparable_v<T>) {
                // 如果 T 和 U 之间支持直接比较 (比如 string == const
                // char*)，这里零开销
                // 如果不支持直接比较，这里会自动发生隐式转换后再比较
                if (data_ == new_data) return;
            }

            // 完美转发，直接触发 T 的 operator=(U&&)
            // 或隐式转换，省去外部临时对象的构造
            data_ = std::forward<U>(new_data);
        }

        dirty_.store(true, std::memory_order_release);
    }

    T load() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return data_;
    }

    template <typename Func>
    decltype(auto) write(Func&& func) {
        static_assert(std::is_invocable_v<Func, T&>,
                      "write callback error: Argument must be T&!");

        struct ScopeExit {
            std::atomic<bool>& dirty;
            ~ScopeExit() { dirty.store(true, std::memory_order_release); }
        } setter{dirty_};

        std::unique_lock<std::shared_mutex> lock(mtx_);
        return std::forward<Func>(func)(data_);
    }

    template <typename Func>
    decltype(auto) read(Func&& func) const {
        static_assert(std::is_invocable_v<Func, const T&>,
                      "read callback error: Argument must be const T&!");
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return std::forward<Func>(func)(data_);
    }

    // --- 留给外部上报机制的接口 ---

    bool is_dirty() const { return dirty_.load(std::memory_order_acquire); }

    void mark_dirty() { dirty_.store(true, std::memory_order_release); }

    // 尝试消费脏数据：如果是脏的，重置脏位并带出数据。线程安全。
    bool consume_if_dirty(T& out_data) {
        if (!dirty_.exchange(false, std::memory_order_acq_rel)) {
            return false;
        }
        std::shared_lock<std::shared_mutex> lock(mtx_);
        out_data = data_;
        return true;
    }
};
