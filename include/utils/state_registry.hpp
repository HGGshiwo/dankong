#pragma once

#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "./dirty_var.hpp"

class RateLimiter {
   public:
    explicit RateLimiter(double hz)
        : last_time_(std::chrono::steady_clock::now()), hz_(hz) {
        if (hz > 0) std::chrono::duration<double>(1.0 / hz);
    }

    bool check_and_update() {
        auto now = std::chrono::steady_clock::now();
        if (hz_ <= 0) return false;
        if (now - last_time_ >= interval_) {
            last_time_ = now;
            return true;
        }
        return false;
    }

   private:
    double hz_;
    std::chrono::duration<double> interval_;
    std::chrono::steady_clock::time_point last_time_;
};

// StateRegistry 独立管理上报逻辑
class StateRegistry {
   private:
    // 内部抽象接口，对外不可见
    struct IReportable {
        virtual ~IReportable() = default;
        virtual void try_report(::nlohmann::json& j) = 0;
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

       public:
        ReportableBinding(DirtyVar<T>& var, std::string key, double hz,
                          std::function<void(::nlohmann::json&, const T&)>
                              serializer = nullptr)
            : var_(var),
              key_(std::move(key)),
              rate_(hz),
              custom_serializer_(std::move(serializer)) {}

        void try_report(::nlohmann::json& j) override {
            // 1. 如果变量没有被修改，直接跳过，不用检查限频
            if (!var_.is_dirty()) return;

            // 2. 检查限频
            if (!rate_.check_and_update()) return;

            // 3. 消费脏数据并填入 JSON
            T val;
            if (var_.consume_if_dirty(val)) {
                if (custom_serializer_) {
                    custom_serializer_(j, val);
                } else {
                    j[key_] = val;
                }
            }
        }

        void append_full_state(::nlohmann::json& j) override {
            T val = var_.load();
            if (custom_serializer_) {
                custom_serializer_(j, val);
            } else {
                j[key_] = val;
            }
        }

        void mark_dirty() override { var_.mark_dirty(); }
    };

    std::vector<std::unique_ptr<IReportable>> reportables_;

   public:
    // 绑定普通变量
    template <typename T>
    void bind(const std::string& key, DirtyVar<T>& var, double hz) {
        reportables_.push_back(
            std::make_unique<ReportableBinding<T>>(var, key, hz));
    }

    // 绑定自定义序列化变量
    template <typename T, typename Callable>
    void bind_custom(DirtyVar<T>& var, double hz, Callable serializer) {
        reportables_.push_back(std::make_unique<ReportableBinding<T>>(
            var, "", hz, std::move(serializer)));
    }

    void report_all(::nlohmann::json& j) {
        for (auto& reporter : reportables_) {
            reporter->try_report(j);
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