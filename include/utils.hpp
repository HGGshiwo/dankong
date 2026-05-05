#pragma once
#include <cstring>
#include <string>

struct FixedString64 {
    char data[64];

    // 默认构造
    FixedString64() noexcept { data[0] = 0; }

    // 从 C 字符串或 std::string 赋值（自动截断）
    FixedString64(const char* s) noexcept { assign(s); }
    FixedString64(const std::string& s) noexcept { assign(s.c_str()); }

    FixedString64& operator=(const char* s) noexcept {
        assign(s);
        return *this;
    }
    FixedString64& operator=(const std::string& s) noexcept {
        assign(s.c_str());
        return *this;
    }

    // 拷贝（平凡）
    FixedString64(const FixedString64&) = default;
    FixedString64& operator=(const FixedString64&) = default;

    // 比较
    bool operator==(const FixedString64& o) const noexcept { return strcmp(data, o.data) == 0; }
    bool operator!=(const FixedString64& o) const noexcept { return !(*this == o); }
    bool operator<(const FixedString64& o) const noexcept { return strcmp(data, o.data) < 0; }

    // 获取内容
    const char* c_str() const noexcept { return data; }
    operator std::string() const noexcept { return data; }

   private:
    void assign(const char* s) noexcept {
        if (!s) s = "";
        strncpy(data, s, 63);
        data[63] = 0;
    }
};

// 确认可以用于 std::atomic
static_assert(std::is_trivially_copyable_v<FixedString64>);
