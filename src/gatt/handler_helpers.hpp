#pragma once

#include "ble/att_server.h"
#include "bluetooth.h"
#include "btstack_config.h"
#include "btstack_defines.h"
#include "btstack_run_loop.h"
#include "characteristic_ids.hpp"  // IWYU pragma: keep [almost everyone needs this]
#include "hci.h"
#include "nevermore.h"
#include "sdk/btstack.hpp"  // IWYU pragma: keep [doesn't find overloads]
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <span>
#include <type_traits>

namespace nevermore::gatt {

constexpr uint16_t GATT_CLIENT_CFG_NOTIFY_FLAG = 0b0000'0001;

#define BT(x) ORG_BLUETOOTH_CHARACTERISTIC_##x
#define HANDLE_ATTR_(attr, kind) ATT_CHARACTERISTIC_##attr##_##kind##_HANDLE
#define HANDLE_ATTR(attr, kind) HANDLE_ATTR_(attr, kind)

#define HANDLE_READ_BLOB(attr, kind, expr) \
    case HANDLE_ATTR(attr, kind): return att_read_callback_handle_blob(expr, offset, buffer);
#define HANDLE_WRITE_EXPR(attr, kind, expr) \
    case HANDLE_ATTR(attr, kind): return expr;

#define ESM_DESCRIBE(attr, desc) HANDLE_READ_BLOB(attr, ENVIRONMENTAL_SENSING_MEASUREMENT, desc)
#define READ_CLIENT_CFG(attr, handler) \
    HANDLE_READ_BLOB(attr, CLIENT_CONFIGURATION, handler.client_configuration(conn))
#define READ_VALUE(attr, expr) HANDLE_READ_BLOB(attr, VALUE, expr)
#define SERVER_CFG_ALWAYS_BROADCAST(attr) HANDLE_READ_BLOB(attr, SERVER_CONFIGURATION, uint16_t(0x0001))
#define USER_DESCRIBE(attr, desc) HANDLE_READ_BLOB(attr, USER_DESCRIPTION, desc "")
#define WRITE_CLIENT_CFG(attr, handler) \
    HANDLE_WRITE_EXPR(attr, CLIENT_CONFIGURATION, handler.client_configuration(conn, consume))
#define WRITE_VALUE(attr, dst) \
    case HANDLE_ATTR(attr, VALUE): dst = consume.exactly<decltype(dst)>(); return 0;

struct AttrWriteException {
    int error;
};

struct WriteConsumer {
    struct NotEnoughException {};

    std::span<uint8_t const> buffer;

    template <typename A>
        requires(std::is_standard_layout_v<A> && !std::is_pointer_v<A> && !std::is_reference_v<A>)
    operator A() {
        if (!has_available(sizeof(A))) throw AttrWriteException(ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);

        // Ideally we'd like to just return a ptr within the buffer, but sadly
        // ARM has stricter alignment requirements than x86's *ANYTHING-GOES!* approach.
        A value;
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        memcpy((void*)&value, buffer.data(), sizeof(value));
        buffer = {buffer.begin() + sizeof(value), buffer.end()};
        return value;
    }

    template <typename A>
    A exactly() {
        if (remaining() != sizeof(A)) throw AttrWriteException(ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);

        return *this;
    }

    template <typename A>
    A or_default(A default_) {
        if (remaining() == 0) return default_;

        return *this;
    }

    std::span<uint8_t const> span(size_t length) {
        if (!has_available(sizeof(uint8_t) * length))
            throw AttrWriteException(ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);

        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        auto const* ptr = buffer.data();
        buffer = {buffer.begin() + int(length), buffer.end()};
        return {ptr, ptr + length};  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    [[nodiscard]] uint16_t remaining() const {
        return buffer.size();
    }

private:
    [[nodiscard]] constexpr bool has_available(size_t n) const {
        return n <= buffer.size();
    }
};

// Unless otherwise noted this can only be used by the btstack task.
// NB: This object is PINNED.
template <void (*Handler)(hci_con_handle_t)>
struct NotifyState {
    static_assert(Handler != nullptr);
    std::array<btstack_context_callback_registration_t, MAX_NR_HCI_CONNECTIONS> callbacks{};
    btstack_context_callback_registration_t notify_callback{
            .callback = [](void* ctx) { reinterpret_cast<NotifyState<Handler>*>(ctx)->notify_dispatch(); },
            .context = this,
    };

    NotifyState() {
        for (auto& cb : callbacks) {
            cb.callback = [](void* ctx) { Handler(hci_con_handle_t(uintptr_t(ctx))); };
            cb.context = reinterpret_cast<void*>(HCI_CON_HANDLE_INVALID);
        }
    }

    [[nodiscard]] bool registered(hci_con_handle_t conn) const {
        return std::ranges::any_of(callbacks, [&](auto&& cb) { return conn == uintptr_t(cb.context); });
    }

    bool register_(hci_con_handle_t conn) {
        if (registered(conn)) return false;                // no-op
        if (conn == HCI_CON_HANDLE_INVALID) return false;  // no-op

        for (auto&& cb : callbacks) {
            if (uintptr_t(cb.context) != HCI_CON_HANDLE_INVALID) continue;

            cb.context = reinterpret_cast<void*>(conn);  // NOLINT(performance-no-int-to-ptr)
            return true;
        }

        printf("ERR - BLE GATT - notify register failed for conn=%d\n", conn);
        return false;
    }

    bool unregister(hci_con_handle_t const conn) {
        if (conn == HCI_CON_HANDLE_INVALID) return false;

        // btstack only releases HCI connection info after registered event handlers
        // finish *and* no one triggered a reconnect
        auto* hci_connection = hci_connection_for_handle(conn);
        assert(hci_connection && "should still have HCI info until event handler completes");

        for (auto& cb : callbacks) {
            if (conn != uintptr_t(cb.context)) continue;

            // remove any pending notification requests
            btstack_linked_list_remove(&hci_connection->att_server.notification_requests,
                    reinterpret_cast<btstack_linked_item_t*>(&cb));
            cb.context = reinterpret_cast<void*>(HCI_CON_HANDLE_INVALID);  // unassign slot
            return true;
        }

        return false;
    }

    // can be invoked by other tasks
    void notify() {
#if NEVERMORE_PICO_W_BT
        btstack_run_loop_execute_on_main_thread(&notify_callback);
#endif
    }

    [[nodiscard]] uint16_t client_configuration(hci_con_handle_t conn) const {
        return registered(conn) ? 1 : 0;
    }

    int client_configuration(hci_con_handle_t conn, WriteConsumer& consume) {
        uint16_t state = consume;
        if (consume.remaining() != 0) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;

        if (state & GATT_CLIENT_CFG_NOTIFY_FLAG) {
            // something went wrong, might as well let the other side know about it
            if (!registered(conn) && !register_(conn)) return ATT_ERROR_WRITE_REQUEST_REJECTED;
        } else
            unregister(conn);

        return 0;
    }

private:
    void notify_dispatch() {
        for (auto&& cb : callbacks)
            if (uintptr_t(cb.context) != HCI_CON_HANDLE_INVALID)
                att_server_request_to_send_notification(&cb, hci_con_handle_t(uintptr_t(cb.context)));
    }
};

}  // namespace nevermore::gatt
