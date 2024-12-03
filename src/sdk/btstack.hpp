#pragma once

#include "ble/att_server.h"
#include <bit>
#include <span>
#include <type_traits>

namespace nevermore {

static_assert(std::endian::native == std::endian::little, "Blob helpers assume machine is little endian.");

template <typename T>
uint8_t att_server_notify(hci_con_handle_t con_handle, uint16_t attribute_handle, T const& value) {
    static_assert(!std::is_pointer_v<T>);
    return ::att_server_notify(
            con_handle, attribute_handle, reinterpret_cast<uint8_t const*>(&value), sizeof(T));
};

template <typename T>
uint16_t att_read_callback_handle_blob(T const* blob, size_t len, std::span<uint8_t> buffer) {
    return ::att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(blob), len, 0, buffer.data(), buffer.size());
}

template <typename T>
uint16_t att_read_callback_handle_blob(T const& blob, std::span<uint8_t> buffer) {
    static_assert(!std::is_pointer_v<T>);
    return att_read_callback_handle_blob(reinterpret_cast<uint8_t const*>(&blob), sizeof(T), buffer);
}

}  // namespace nevermore
