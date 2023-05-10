#pragma once

#include "gap.h"
#include <cassert>
#include <chrono>

using namespace std::literals::chrono_literals;

// BlueTooth advertisement API is specified in terms of ticks, where a tick is 0.625 ms
constexpr auto BT_ADVERTISEMENT_INTERVAL_TICK = 625us;
// Cannot advertise more frequently than every 100ms.
constexpr auto BT_ADVERTISEMENT_INTERVAL_MIN = 100ms;

template <typename Dur0, typename Dur1>
void gap_advertisements_set_params(Dur0 const& advert_min, Dur1 const& advert_max) {
    assert(BT_ADVERTISEMENT_INTERVAL_MIN <= advert_min && "can't advertise faster than 100ms");

    bd_addr_t null_addr{};
    ::gap_advertisements_set_params(advert_min / BT_ADVERTISEMENT_INTERVAL_TICK,
            advert_max / BT_ADVERTISEMENT_INTERVAL_TICK, 0, 0, null_addr, 0b0111, 0x00);
}
