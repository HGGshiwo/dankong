#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#elif defined(__linux__) || defined(__unix__)
#include <limits.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

inline boost::filesystem::path get_executable_path() {
#ifdef _WIN32
    std::vector<char> buffer(MAX_PATH);
    DWORD len = GetModuleFileNameA(NULL, buffer.data(),
                                   static_cast<DWORD>(buffer.size()));
    if (len == 0 || len == buffer.size()) return "";
    return boost::filesystem::path(std::string(buffer.data(), len));
#elif defined(__linux__) || defined(__unix__)
    std::vector<char> buffer(PATH_MAX);
    ssize_t len = readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
    if (len == -1) return "";
    buffer[len] = '\0';
    return boost::filesystem::path(buffer.data());
#elif defined(__APPLE__)
    std::vector<char> buffer(PATH_MAX);
    uint32_t size = buffer.size();
    if (_NSGetExecutablePath(buffer.data(), &size) != 0) return "";
    return boost::filesystem::path(buffer.data());
#else
#error "Unsupported platform"
#endif
}