#pragma once

#include <cstdint>

namespace nevermore {

// useful for templates
template <std::size_t N>
struct TemplateStringLiteral {
    char value[N]{};

    constexpr TemplateStringLiteral(const char (&str)[N]) {
        for (std::size_t i = 0; i < N; ++i)
            value[i] = str[i];
    }

    constexpr operator char const*() const {
        return value;
    }
};

}  // namespace nevermore
