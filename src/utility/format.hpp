#pragma once

// TODO: Replace w/ <format> once that becomes commonly available.

#include <cassert>
#include <cstdarg>
#include <string>

namespace nevermore {

[[gnu::format(printf, 1, 2)]]
inline std::string format_string(char const* format, ...) {
    va_list xs;
    va_start(xs, format);
    auto n = vsnprintf(nullptr, 0, format, xs);
    assert(0 <= n);
    if (n < 0) {
        va_end(xs);
        return "";
    }

    std::string buf;
    buf.resize(n);
    vsnprintf(buf.data(), buf.size() + 1, format, xs);
    va_end(xs);
    return buf;
}

}  // namespace nevermore
