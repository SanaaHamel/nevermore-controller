
#include "bluetooth_data_types.h"
#include "gatt/environmental.hpp"
#include "pico/async_context.h"
#include <cstdint>
#include <cstring>

constexpr uint8_t ADVERTISEMENT_FLAGS = (1 << 1) |  // LE general discoverable
                                        (1 << 2);   // BR/EDR not supported (why? can we enable this?)
constexpr uint8_t ADVERTISEMENT_DATA_PRELUDE[] = {
        // Flags general discoverable
        2,  // len, 1 (type), 1 (flags)
        BLUETOOTH_DATA_TYPE_FLAGS,
        ADVERTISEMENT_FLAGS,
        // len: type, service ID, service data (see `EnvironmentService::ServiceData`)
        1 + 2 + sizeof(EnvironmentService::ServiceData),
        BLUETOOTH_DATA_TYPE_SERVICE_DATA_16_BIT_UUID,
        0x1a,
        0x18,
};

struct [[gnu::packed]] AdvertiseData {
    uint8_t prelude[sizeof(ADVERTISEMENT_DATA_PRELUDE)]{};
    EnvironmentService::ServiceData environment_service_data{};

    AdvertiseData() {
        std::memcpy(prelude, ADVERTISEMENT_DATA_PRELUDE, sizeof(ADVERTISEMENT_DATA_PRELUDE));
    }
};
extern AdvertiseData g_advertise_data;

// BL limitation. Extended advertising allows larger payload, but I can't get it to work.
// Apparently HCI keeps reporting extended-advert as unsupported. (Maybe related to lack of BR/EDR?)
static_assert(
        sizeof(AdvertiseData) <= 31, "DO NOT EXCEED 31 OCTETS! BTSTACK WILL SMASH THE STACK IF YOU DO.");

// Setup bluetooth and GATT services.
// Caller is responsible for subsequently calling `btstack_run_loop_execute`.
bool gatt_init(async_context_t&);
