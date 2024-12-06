#include "usb_cdc_gatt.hpp"
#include "class/cdc/cdc_device.h"
#include "device/usbd.h"
#include "gatt.hpp"
#include "nevermore.h"
#include "sdk/task.hpp"
#include "tusb.h"
#include <climits>
#include <cstdio>
#include <optional>
#include <span>

using namespace std;

namespace nevermore {

namespace {

using PayloadSize = uint16_t;

constexpr int GATT_ITF = 1;  // CDC interface #, 0 is stdio
constexpr auto READ_FAILED_YIELD_PERIOD = 10ms;
constexpr auto NOT_CONNECTED_YIELD_PERIOD = 100ms;

enum class Cmd : uint8_t {
    db_read = 0xfa,
    attr_read = 0xfb,
    attr_write = 0xfc,
};

static_assert(gatt::ATTR_SIZE_MAX <= numeric_limits<PayloadSize>::max());
static_assert(sizeof(PayloadSize) == 2, "protocol req");
static_assert(sizeof(Cmd) == 1, "protocol req");

template <typename A>
    requires is_standard_layout_v<A> && is_trivial_v<A>
span<char const> raw_view(A const& a) {
    return span{reinterpret_cast<char const*>(&a), reinterpret_cast<char const*>(&a + 1)};
}

[[nodiscard]] bool recv(span<uint8_t> dst) {
    auto const end = dst.end();
    for (auto curr = dst.begin(); curr != end;) {
        if (!tud_cdc_n_connected(GATT_ITF)) {
            printf("ERR - GATT CDC - lost connection mid-recv\n");
            return false;
        }

        auto n = tud_cdc_n_read(GATT_ITF, &*curr, end - curr);
        if (n == 0) vTaskDelay(0);  // yield to other tasks
        curr += n;
    }

    return true;
}

template <typename A>
    requires is_standard_layout_v<A> && is_trivial_v<A>
optional<A> recv() {
    array<uint8_t, sizeof(A)> raw{};
    if (!recv(raw)) return {};

    return *reinterpret_cast<A*>(raw.data());
}

void send(span<char const> xs) {
    auto const end = xs.end();
    for (auto curr = xs.begin(); curr != end;) {
        if (!tud_cdc_n_connected(GATT_ITF)) {
            printf("ERR - GATT CDC - lost connection mid-send\n");
            tud_cdc_n_write_clear(GATT_ITF);  // drop anything that's still pending
            return;
        }

        curr += tud_cdc_n_write(GATT_ITF, &*curr, end - curr);
    }
}

template <typename A>
    requires is_standard_layout_v<A> && is_trivial_v<A>
void send(A const& data) {
    send(raw_view(data));
}

void db_read() {
    static_assert(profile_data[0] == 1, "DB format has changed; update consumers & assert");
    static_assert(sizeof(profile_data) <= numeric_limits<PayloadSize>::max());

    send<Cmd>(Cmd::db_read);
    send<PayloadSize>(sizeof(profile_data));
    send(profile_data);
    tud_cdc_n_write_flush(GATT_ITF);
}

// on avg it takes ~800us to recv, handle, and send
void attr_read() {
    auto const attr = recv<uint16_t>();
    if (!attr) return;  // something went wrong, ignore

    array<uint8_t, gatt::ATTR_SIZE_MAX> data{};
    auto const size = gatt::read(*attr, data);
    if (!size) {
        printf("ERR - USB CDC - read to unknown attr=%d\n", *attr);
    }

    send<Cmd>(Cmd::attr_read);
    send<uint16_t>(*attr);
    send<bool>(bool(size));
    send<PayloadSize>(size ? *size : 0);
    if (size) {
        send(span{reinterpret_cast<char const*>(data.begin()),
                reinterpret_cast<char const*>(data.begin()) + *size});
    }
    tud_cdc_n_write_flush(GATT_ITF);
}

// consume data in chunks instead of sucking it down char by char
// Sadly there's no `tud_cdc_n_read_flush(itf, n)` API.
// Simulate it by reading in chunks into a scratch-pad.
bool recv_drop(size_t const amount, span<uint8_t> const scratch_pad) {
    for (auto n = amount; 0 < n;) {
        auto const amt = min<size_t>(scratch_pad.size(), n);
        if (!recv(span{scratch_pad.begin(), scratch_pad.begin() + amt}))
            return false;  // lost connection, give up

        n -= amt;
    }

    return true;
}

void attr_write() {
    auto const attr = recv<uint16_t>();
    if (!attr) return;
    auto const size = recv<uint16_t>();
    if (!size) return;

    array<uint8_t, gatt::ATTR_SIZE_MAX> data{};

    // too big? eat data until completed, then report failure.
    if (gatt::ATTR_SIZE_MAX < *size) {
        printf("ERR - USB CDC - oversized write attr=%d size=%d\n", *attr, *size);
        if (!recv_drop(*size, data)) return;

        send<Cmd>(Cmd::attr_write);
        send<uint16_t>(*attr);
        send<bool>(false);
        tud_cdc_n_write_flush(GATT_ITF);
        return;
    }

    span data_subset{data.begin(), data.begin() + *size};
    if (!recv(data_subset)) return;  // EOF while reading payload

    auto result = gatt::write(*attr, data_subset);
    if (!result) {
        printf("ERR - USB CDC - write to unknown attr=%d size=%d\n", *attr, *size);
    }

    send<Cmd>(Cmd::attr_write);
    send<uint16_t>(*attr);
    send<bool>(result == 0);
    tud_cdc_n_write_flush(GATT_ITF);
}

}  // namespace

void usb_cdc_gatt_task() {
    for (;;) {
        // HACK: Don't read before there's a connection. It causes panics
        //       in the rp2040 platform code. Guess we just need to race between
        //       the two.
        //       To reproduce:  Disable connected check.
        //                      Power on application w/ USB unplugged.
        //                      Plug in USB -> panic.
        if (!tud_cdc_n_connected(GATT_ITF)) {
            task_delay<NOT_CONNECTED_YIELD_PERIOD>();
            continue;
        }

        auto const c = tud_cdc_n_read_char(GATT_ITF);
        if (c < 0) {  // failed or timeout, try again in a bit
            task_delay<READ_FAILED_YIELD_PERIOD>();
            continue;
        }

        switch (Cmd(c)) {
        case Cmd::db_read: db_read(); break;
        case Cmd::attr_read: attr_read(); break;
        case Cmd::attr_write: attr_write(); break;
        default: break;  // ignore, not a command escape code
        }
    }
}

}  // namespace nevermore
