#include "environmental.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sensors.hpp"
#include "utility/timer.hpp"
#include <cstdint>

using namespace std;

#define VOC_INDEX_UUID 216aa791_97d0_46ac_8752_60bbc00611e1
#define ENV_AGGREGATE_UUID 75134bec_dd06_49b1_bac2_c15e05fd7199

#define VOC_INDEX_01 216aa791_97d0_46ac_8752_60bbc00611e1_01
#define VOC_INDEX_02 216aa791_97d0_46ac_8752_60bbc00611e1_02
#define ENV_AGGREGATE_01 75134bec_dd06_49b1_bac2_c15e05fd7199_01

namespace nevermore::gatt::environmental {

namespace {

using ESM = BLE::EnvironmentalSensorMeasurementDesc;

// using HTU21D sensor
const ESM ESM_TEMPERATURE{
        .sampling = ESM::Sampling::Instantaneous,
        .update_interval = SENSOR_UPDATE_PERIOD / 1s,
        .application = ESM::Application::Air,
};

// using built-in rp2040 temp monitor
const ESM ESM_TEMPERATURE_MCU{
        .sampling = ESM::Sampling::Instantaneous,
        .update_interval = SENSOR_UPDATE_PERIOD / 1s,
        .application = ESM::Application::Supplementary,
};

// using HTU21D sensor
const ESM ESM_HUMIDITY{
        .sampling = ESM::Sampling::Instantaneous,
        .update_interval = SENSOR_UPDATE_PERIOD / 1s,
};

// pressure: not implemented (no sensors available)
const ESM ESM_PRESSURE{
        .sampling = ESM::Sampling::Instantaneous,
        .update_interval = SENSOR_UPDATE_PERIOD / 1s,
};

// using SGP40
const ESM ESM_VOC_INDEX{
        .sampling = ESM::Sampling::Instantaneous,
        .measure_period = 1,  // for now, we only
        .update_interval = SENSOR_UPDATE_PERIOD / 1s,
        .application = ESM::Application::Supplementary,
};

// NB: nRF Connect incorrectly reads this as a big-endian structure.
// GATT supplementary spec section 2.4 explicitly says everything is little
// endian unless otherwise noted. Section 4.1 describes `Valid Range` and has
// nothing to say regarding its endianness.
const BLE::ValidRange<nevermore::sensors::VOCIndex> VALID_RANGE_VOC_INDEX{.min = 0, .max = 500};

// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
auto g_notify_aggregate = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(
            conn, HANDLE_ATTR(ENV_AGGREGATE_01, VALUE), nevermore::sensors::g_sensors.with_fallbacks());
}>();

}  // namespace

bool init() {
    // HACK:  We'd like to notify on write changes, but the code base isn't setup
    //        for that yet. Internally poll and update based on diffs for now.
    mk_timer("gatt-env-notify", SENSOR_UPDATE_PERIOD / 2.)([](auto*) {
        auto const& current = nevermore::sensors::g_sensors.with_fallbacks();
        static nevermore::sensors::Sensors g_prev;
        if (g_prev != current) {
            g_prev = current;
            g_notify_aggregate.notify();
        }
    });

    return true;
}

void disconnected(hci_con_handle_t conn) {
    g_notify_aggregate.unregister(conn);
}

optional<uint16_t> attr_read(
        hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    auto sensors = []() { return nevermore::sensors::g_sensors.with_fallbacks(); };

    switch (att_handle) {
        // NOLINTBEGIN(bugprone-branch-clone)
        USER_DESCRIBE(BT(TEMPERATURE_01), "Intake Temperature")
        USER_DESCRIBE(BT(TEMPERATURE_02), "Exhaust Temperature")
        USER_DESCRIBE(BT(TEMPERATURE_03), "MCU Temperature")
        USER_DESCRIBE(BT(HUMIDITY_01), "Intake Humidity")
        USER_DESCRIBE(BT(HUMIDITY_02), "Exhaust Humidity")
        USER_DESCRIBE(BT(PRESSURE_01), "Intake Pressure")
        USER_DESCRIBE(BT(PRESSURE_02), "Exhaust Pressure")
        USER_DESCRIBE(VOC_INDEX_01, "Intake VOC Index")
        USER_DESCRIBE(VOC_INDEX_02, "Exhaust VOC Index")
        USER_DESCRIBE(ENV_AGGREGATE_01, "Aggregated Service Data")

        ESM_DESCRIBE(BT(TEMPERATURE_01), ESM_TEMPERATURE)
        ESM_DESCRIBE(BT(TEMPERATURE_02), ESM_TEMPERATURE)
        ESM_DESCRIBE(BT(TEMPERATURE_03), ESM_TEMPERATURE_MCU)
        ESM_DESCRIBE(BT(HUMIDITY_01), ESM_HUMIDITY)
        ESM_DESCRIBE(BT(HUMIDITY_02), ESM_HUMIDITY)
        ESM_DESCRIBE(BT(PRESSURE_01), ESM_PRESSURE)
        ESM_DESCRIBE(BT(PRESSURE_02), ESM_PRESSURE)
        ESM_DESCRIBE(VOC_INDEX_01, ESM_VOC_INDEX)
        ESM_DESCRIBE(VOC_INDEX_02, ESM_VOC_INDEX)

        HANDLE_READ_BLOB(VOC_INDEX_01, VALID_RANGE, VALID_RANGE_VOC_INDEX)
        HANDLE_READ_BLOB(VOC_INDEX_02, VALID_RANGE, VALID_RANGE_VOC_INDEX)
        // NOLINTEND(bugprone-branch-clone)

        READ_VALUE(BT(TEMPERATURE_01), sensors().temperature_intake)
        READ_VALUE(BT(TEMPERATURE_02), sensors().temperature_exhaust)
        READ_VALUE(BT(TEMPERATURE_03), sensors().temperature_mcu)
        READ_VALUE(BT(HUMIDITY_01), sensors().humidity_intake)
        READ_VALUE(BT(HUMIDITY_02), sensors().humidity_exhaust)
        READ_VALUE(BT(PRESSURE_01), sensors().pressure_intake)
        READ_VALUE(BT(PRESSURE_02), sensors().pressure_exhaust)
        READ_VALUE(VOC_INDEX_01, sensors().voc_index_intake)
        READ_VALUE(VOC_INDEX_02, sensors().voc_index_exhaust)
        READ_VALUE(ENV_AGGREGATE_01, sensors())

        READ_CLIENT_CFG(ENV_AGGREGATE_01, g_notify_aggregate)

    default: return {};
    }
}

// No attrs are writable.
optional<int> attr_write(hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t const* buffer,
        uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
        WRITE_CLIENT_CFG(ENV_AGGREGATE_01, g_notify_aggregate)

    default: return {};
    }
}

}  // namespace nevermore::gatt::environmental
