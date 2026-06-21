#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "dk/ITimeProvider.hpp"

class IThreadRunner {
    std::thread control_thread_;
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_terminating_{false};  // 用于通知线程彻底退出

    std::mutex mutex_;
    std::condition_variable cv_;

    bool use_thread_;
    std::atomic<int> hz_{0};  // 保存运行频率
    std::shared_ptr<dk::ITimeProvider> time_provider_;

   public:
    // 将 use_thread 提取到构造函数
    IThreadRunner(std::shared_ptr<dk::ITimeProvider> time_provider,
                  bool use_thread = true)
        : use_thread_(use_thread), time_provider_(std::move(time_provider)) {
        if (use_thread_) {
            // 在构造时即启动线程，进入休眠等待状态
            control_thread_ = std::thread(&IThreadRunner::control_loop, this);
        }
    }

    // 必须添加虚析构函数以确保安全回收线程资源
    virtual ~IThreadRunner() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_terminating_ = true;
        }
        cv_.notify_all();  // 唤醒可能正在等待的线程

        if (control_thread_.joinable()) {
            control_thread_.join();
        }
    }

    virtual void on_start() {}
    virtual void on_stop() {}
    virtual void on_step(double dt) {}
    bool is_running() { return is_running_; }
    void start(int hz) {
        if (is_running_.load()) return;

        hz_ = hz;    // 更新频率
        on_start();  // 耗时操作/回调可以放在锁外

        // 【关键修复】：修改供 condition_variable 使用的状态时，必须加锁
        if (use_thread_) {
            std::lock_guard<std::mutex> lock(mutex_);
            is_running_ = true;
            cv_.notify_one();
        } else {
            is_running_ = true;
        }
    }

    void control_loop() {
        while (!is_terminating_) {
            // 1. 休眠等待阶段
            std::unique_lock<std::mutex> lock(mutex_);
            // 线程一直 sleep，直到 is_running_ 为 true 或者准备析构
            // (is_terminating_ 为 true)
            cv_.wait(lock, [this] {
                return is_running_.load() || is_terminating_.load();
            });

            if (is_terminating_) break;  // 收到销毁信号，彻底退出线程

            lock.unlock();  // 进入循环前解锁

            // 2. 运行阶段
            const double period_s = 1.0 / static_cast<double>(hz_.load());
            while (is_running_) {
                double step_start = time_provider_->now();
                on_step(period_s);
                double step_end = time_provider_->now();
                double elapsed = step_end - step_start;
                if (elapsed < period_s) {
                    time_provider_->sleep_for(period_s - elapsed);
                }
            }
            // 当 stop() 被调用，is_running_ 变为 false，退出内层循环
            // 随后重新进入外层循环，继续在 cv_.wait 处挂起 sleep
        }
    }

    void stop() {
        if (!is_running_) return;

        is_running_ = false;  // 改变标志位，让 control_loop 退出内部循环
        on_stop();
    }
};