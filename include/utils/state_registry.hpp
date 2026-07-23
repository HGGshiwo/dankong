#pragma once

#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "./dirty_var.hpp"

class RateLimiter {
   public:
    explicit RateLimiter(double hz) : last_time_(0), hz_(hz) {
        if (hz > 0) interval_ = 1.0 / hz;
    }

    bool check_and_update(double now) {
        if (hz_ <= 0) return false;
        if (now - last_time_ >= interval_) {
            last_time_ = now;
            return true;
        }
        return false;
    }

   private:
    double hz_;
    double interval_;
    double last_time_;
};

// StateRegistry 独立管理上报逻辑
class StateRegistry {
   private:
    // 内部抽象接口，对外不可见
    struct IReportable {
        virtual ~IReportable() = default;
        virtual void try_report(::nlohmann::json& j, double now) = 0;
        virtual void append_full_state(::nlohmann::json& j) = 0;
        virtual void mark_dirty() = 0;
    };

    // 绑定具体的 dirty
    template <typename T>
    class ReportableBinding : public IReportable {
       private:
        DirtyVar<T>& var_;
        std::string key_;
        RateLimiter rate_;
        std::function<void(::nlohmann::json&, const T&)> custom_serializer_;
        std::function<bool(const T&, const T&)> dirty_checker_;
        T last_reported_val_{};
        bool has_reported_ = false;

       public:
        template <typename Checker = std::nullptr_t>
        ReportableBinding(DirtyVar<T>& var, std::string key, double hz,
                          std::function<void(::nlohmann::json&, const T&)>
                              serializer = nullptr,
                          Checker dirty_checker = nullptr)
            : var_(var),
              key_(std::move(key)),
              rate_(hz),
              custom_serializer_(std::move(serializer)),
              dirty_checker_(std::move(dirty_checker)) {}

        void try_report(::nlohmann::json& j, double now) override {
            // 1. 如果变量没有被修改，直接跳过，不用检查限频
            if (!var_.is_dirty()) return;

            // 2. 检查限频
            if (!rate_.check_and_update(now)) return;

            // 3. 消费脏数据并填入 JSON
            T val;
            if (var_.consume_if_dirty(val)) {
                if (dirty_checker_ && has_reported_) {
                    if (!dirty_checker_(val, last_reported_val_)) {
                        return;
                    }
                }

                if (custom_serializer_) {
                    custom_serializer_(j, val);
                } else {
                    j[key_] = val;
                }
                last_reported_val_ = val;
                has_reported_ = true;
            }
        }

        void append_full_state(::nlohmann::json& j) override {
            T val = var_.load();
            if (custom_serializer_) {
                custom_serializer_(j, val);
            } else {
                j[key_] = val;
            }
            last_reported_val_ = val;
            has_reported_ = true;
        }

        void mark_dirty() override { var_.mark_dirty(); }
    };

    std::vector<std::unique_ptr<IReportable>> reportables_;

   public:
    // 绑定普通变量
    template <typename T, typename Checker = std::nullptr_t>
    void bind(const std::string& key, DirtyVar<T>& var, double hz,
              Checker dirty_checker = nullptr) {
        reportables_.push_back(std::make_unique<ReportableBinding<T>>(
            var, key, hz, nullptr, std::move(dirty_checker)));
    }

    // 绑定自定义序列化变量
    template <typename T, typename Callable, typename Checker = std::nullptr_t>
    void bind_custom(DirtyVar<T>& var, double hz, Callable serializer,
                     Checker dirty_checker = nullptr) {
        reportables_.push_back(std::make_unique<ReportableBinding<T>>(
            var, "", hz, std::move(serializer), std::move(dirty_checker)));
    }

    void report_all(::nlohmann::json& j, double now) {
        for (auto& reporter : reportables_) {
            reporter->try_report(j, now);
        }
    }

    ::nlohmann::json get_full_state() const {
        ::nlohmann::json j;
        for (const auto& reporter : reportables_) {
            reporter->append_full_state(j);
        }
        return j;
    }

    void mark_all_dirty() {
        for (auto& reporter : reportables_) {
            reporter->mark_dirty();
        }
    }
};