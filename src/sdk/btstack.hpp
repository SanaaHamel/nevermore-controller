#pragma once

#include "ble/att_server.h"
#include <bit>
#include <type_traits>

static_assert(std::endian::native == std::endian::little, "Blob helpers assume machine is little endian.");

template <typename T>
uint8_t att_server_notify(hci_con_handle_t con_handle, uint16_t attribute_handle, T const& value) {
    static_assert(!std::is_pointer_v<T>);
    return att_server_notify(con_handle, attribute_handle, static_cast<uint8_t const*>(&value), sizeof(T));
};

template <typename T>
uint16_t att_read_callback_handle_blob(
        T const& blob, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    static_assert(!std::is_pointer_v<T>);
    return att_read_callback_handle_blob(
            reinterpret_cast<uint8_t const*>(&blob), sizeof(T), offset, buffer, buffer_size);
}
