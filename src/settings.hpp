#pragma once

#include "hardware/flash.h"
#include "utility/crc.hpp"
#include "utility/fan_policy.hpp"
#include "utility/fan_policy_thermal.hpp"
#include <algorithm>

namespace nevermore::settings {

constexpr auto MAX_SIZE = PICOWOTA_APP_STORE_SIZE;

// TODO: Is there a library for doing this kind of lightweight versioning?
// This structure **may not** change.
struct Header {
    // 32 bits b/c we're gonna get hit by padding anyways.
    // Version ratchets up when there's a non backwards compatible change.
    enum class Version : uint32_t { v0 };

    CRC32_t crc;  // may be dirty. populate/update before writing to storage
    Version version;
    // can be >= `sizeof Settings` if we've downgraded (i.e. extra fields at the end)
    // INVARIANT: \in [sizeof(Header), MAX_SIZE]
    uint32_t size;  // includes the CRC32 field
};

// 16 octets should be more than enough.
// Will be initialised to all-zeros by default.
using SensorCalibrationBlob = uint8_t[16];
using VOCIndex = nevermore::sensors::VOCIndex;

enum class DisplayHW : uint8_t {
    GC9A01_240_240 = 0,
};

enum class DisplayUI : uint8_t {
    CIRCLE_240_CLASSIC = 0,
};

// Layout **cannot** change. This would break back-compatibility.
// Fields **can** be appended w/o bumping the header version.
struct SettingsV0 {
    Header header{.version = Header::Version::v0, .size = sizeof(SettingsV0)};
    FanPolicyEnvironmental fan_policy_env;
    FanPolicyThermal fan_policy_thermal;
    BLE::Percentage8 fan_power_passive = 0;        // not-known -> disallowed
    BLE::Percentage8 fan_power_automatic = 100;    // not-known -> disallowed
    BLE::Percentage8 fan_power_coefficient = 100;  // not-known -> disallowed
    SensorCalibrationBlob voc_calibration[2]{};
    VOCIndex voc_gating = 340;  // not-known -> disallowed, TODO: lower defaults to 240?
    DisplayHW display_hw = DisplayHW::GC9A01_240_240;
    DisplayUI display_ui = DisplayUI::CIRCLE_240_CLASSIC;
    float display_brightness = 1.f;  // range: [0, 1]; don't edit directly, use `display::brightness`

    // replaces valid fields from RHS into self
    void merge_valid_fields(SettingsV0 const&);
};

using Settings = SettingsV0;

static_assert(sizeof(Settings) <= MAX_SIZE);

extern Settings g_active;

void init();

// HACK: by ref b/c `Settings` can get pretty big
// HACK: mutates `header::crc`
void save(Settings&);

}  // namespace nevermore::settings
