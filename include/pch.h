// pch.h
#pragma once

// ==========================================
// 1. 重量级第三方库（最耗时的元凶）
// ==========================================
#include <mavsdk/mavsdk.h>

#include <CLI/CLI.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#ifdef USE_ROS
#ifdef USE_ROS1
#include <ros/ros.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#endif
#endif

// ==========================================
// 2. C++ 标准库（高频使用、不变动）
// ==========================================
#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "dk/engine.hpp"
#include "dk/future.hpp"