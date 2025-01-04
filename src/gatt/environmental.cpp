#include "environmental.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sensors.hpp"
#include "utility/timer.hpp"

using namespace std;

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
        .update_interval = SENSOR_UPDATE_PERIOD / 1s,
        .application = ESM::Application::Supplementary,
};

const ESM ESM_VOC_RAW{
        .sampling = ESM::Sampling::Instantaneous,
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
            conn, HANDLE_ATTR(ENV_AGGREGATE, VALUE), nevermore::sensors::g_sensors.with_fallbacks());
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
        hci_con_handle_t const conn, uint16_t const attr, uint16_t const offset, span<uint8_t> const buffer) {
    auto sensors = []() { return nevermore::sensors::g_sensors.with_fallbacks(); };

    switch (attr) {
        // NOLINTBEGIN(bugprone-branch-clone)
        USER_DESCRIBE(BT(TEMPERATURE_01), "Intake Temperature")
        USER_DESCRIBE(BT(TEMPERATURE_02), "Exhaust Temperature")
        USER_DESCRIBE(BT(TEMPERATURE_03), "MCU Temperature")
        USER_DESCRIBE(BT(HUMIDITY_01), "Intake Humidity")
        USER_DESCRIBE(BT(HUMIDITY_02), "Exhaust Humidity")
        USER_DESCRIBE(BT(PRESSURE_01), "Intake Pressure")
        USER_DESCRIBE(BT(PRESSURE_02), "Exhaust Pressure")
        USER_DESCRIBE(ENV_VOC_INDEX_INTAKE, "Intake VOC Index")
        USER_DESCRIBE(ENV_VOC_INDEX_EXHAUST, "Exhaust VOC Index")
        USER_DESCRIBE(ENV_VOC_RAW_INTAKE, "Intake VOC Raw")
        USER_DESCRIBE(ENV_VOC_RAW_EXHAUST, "Exhaust VOC Raw")
        USER_DESCRIBE(ENV_AGGREGATE, "Aggregated Service Data")

        ESM_DESCRIBE(BT(TEMPERATURE_01), ESM_TEMPERATURE)
        ESM_DESCRIBE(BT(TEMPERATURE_02), ESM_TEMPERATURE)
        ESM_DESCRIBE(BT(TEMPERATURE_03), ESM_TEMPERATURE_MCU)
        ESM_DESCRIBE(BT(HUMIDITY_01), ESM_HUMIDITY)
        ESM_DESCRIBE(BT(HUMIDITY_02), ESM_HUMIDITY)
        ESM_DESCRIBE(BT(PRESSURE_01), ESM_PRESSURE)
        ESM_DESCRIBE(BT(PRESSURE_02), ESM_PRESSURE)
        ESM_DESCRIBE(ENV_VOC_INDEX_INTAKE, ESM_VOC_INDEX)
        ESM_DESCRIBE(ENV_VOC_INDEX_EXHAUST, ESM_VOC_INDEX)
        ESM_DESCRIBE(ENV_VOC_RAW_INTAKE, ESM_VOC_RAW)
        ESM_DESCRIBE(ENV_VOC_RAW_EXHAUST, ESM_VOC_RAW)

        HANDLE_READ_BLOB(ENV_VOC_INDEX_INTAKE, VALID_RANGE, VALID_RANGE_VOC_INDEX)
        HANDLE_READ_BLOB(ENV_VOC_INDEX_EXHAUST, VALID_RANGE, VALID_RANGE_VOC_INDEX)
        // NOLINTEND(bugprone-branch-clone)

        READ_VALUE(BT(TEMPERATURE_01), sensors().temperature_intake)
        READ_VALUE(BT(TEMPERATURE_02), sensors().temperature_exhaust)
        READ_VALUE(BT(TEMPERATURE_03), sensors().temperature_mcu)
        READ_VALUE(BT(HUMIDITY_01), sensors().humidity_intake)
        READ_VALUE(BT(HUMIDITY_02), sensors().humidity_exhaust)
        READ_VALUE(BT(PRESSURE_01), sensors().pressure_intake)
        READ_VALUE(BT(PRESSURE_02), sensors().pressure_exhaust)
        READ_VALUE(ENV_VOC_INDEX_INTAKE, sensors().voc_index_intake)
        READ_VALUE(ENV_VOC_INDEX_EXHAUST, sensors().voc_index_exhaust)
        READ_VALUE(ENV_VOC_RAW_INTAKE, sensors().voc_raw_intake)
        READ_VALUE(ENV_VOC_RAW_EXHAUST, sensors().voc_raw_exhaust)
        READ_VALUE(ENV_AGGREGATE, sensors())

        READ_CLIENT_CFG(ENV_AGGREGATE, g_notify_aggregate)

    default: return {};
    }
}

// No attrs are writable.
optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
        WRITE_CLIENT_CFG(ENV_AGGREGATE, g_notify_aggregate)

    default: return {};
    }
}

}  // namespace nevermore::gatt::environmental
