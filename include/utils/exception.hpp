#pragma once
#include <string>

inline std::string get_error_message(std::exception_ptr eptr) {
    if (!eptr) {
        return "No exception";  // 空指针处理
    }

    try {
        // 将指针中的异常重新抛出
        std::rethrow_exception(eptr);
    } catch (const std::exception& e) {
        // 捕获所有继承自 std::exception 的标准异常
        return e.what();
    } catch (const std::string& s) {
        // 捕获抛出类型为 std::string 的异常
        return s;
    } catch (const char* c) {
        // 捕获抛出类型为 C风格字符串 的异常
        return c;
    } catch (...) {
        // 兜底：捕获未知类型（如 throw 1;）
        return "Unknown exception type";
    }
}