#include "settings.hpp"
#include "config.hpp"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "pico/flash.h"
#include "sdk/ble_data_types.hpp"
#include "utility/align.hpp"
#include "utility/crc.hpp"
#include "utility/timer.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>

// defined by linker
extern uint8_t PICOWOTA_APP_STORE[];
extern uint8_t PICOWOTA_APP_STORE_END[];

// UNSAFE:  Do not enable for general use. `printf` will cause interrupts,
//          which violates the flash functions' preconditions.
#define DBG_SHOW_FLASH_COPIES 0

using namespace std;

namespace nevermore::settings {

namespace {

CRC32_t crc(SettingsPersisted const& settings) {
    // NB: it is possible for `settings.header.size < sizeof(Settings)` (e.g. we've a newer version)
    auto crc = [](uint8_t const* p, uint8_t const* q, CRC32_t init) {
        if (q <= p) return init;
        return crc32(span{p, q}, init);
    };

    // compute the CRC of `settings` (skip the CRC field) + extra fields (which are in the store section)
    auto size_total = min<size_t>(settings.header.size, MAX_SIZE);
    auto size_body = min(size_total, sizeof(SettingsPersisted));
    auto const* bgn = reinterpret_cast<uint8_t const*>(&settings);
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    auto x = crc(bgn + sizeof(CRC32_t), bgn + size_body, 0xFF);  // skip the CRC field
    auto y = crc(PICOWOTA_APP_STORE + size_body, PICOWOTA_APP_STORE + size_total, x);
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return y;
}

// can't use RVO if wrapped in `optional` :(
bool UNSAFE_from_flash(SettingsPersisted& dst) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    auto const& stored = *reinterpret_cast<SettingsPersisted const*>(PICOWOTA_APP_STORE);
    if (stored.header.size < sizeof(Header) || MAX_SIZE < stored.header.size) {
        printf("corrupt settings: size=0x%08x not in range [0x%08x, 0x%08x]\n", (unsigned)stored.header.size,
                sizeof(Header), MAX_SIZE);
        return false;
    }

    if (auto x = crc(stored); x != stored.header.crc) {
        printf("corrupt settings: crc reported=0x%08x computed=0x%08x\n", (unsigned)stored.header.crc,
                (unsigned)x);
        return false;
    }

    if (stored.header.version != Header::Version::v0) {
        printf("unrecognised settings version: %u", (unsigned)stored.header.version);
        return false;
    }

    // old version, need to initialise new fields (crc will be dirty but that's fine)
    if (stored.header.size < sizeof(SettingsPersisted)) {
        // Need a temp settings to setup defaults for new fields, existing fields
        // may be malformed/invalid.
        SettingsPersisted default_init{};
        memcpy(&default_init, &stored, stored.header.size);
        default_init.header.size = sizeof(SettingsPersisted);
        dst.merge_valid_fields(default_init);
        return true;
    }

    dst.merge_valid_fields(stored);
    return true;
}

void UNSAFE_save_internal(void* param) {
    static_assert(MAX_SIZE <= FLASH_SECTOR_SIZE, "more complex impl' required");

    auto const APP_STORE_OFFSET = reinterpret_cast<unsigned>(PICOWOTA_APP_STORE) - XIP_BASE;
    auto const& settings = *reinterpret_cast<SettingsPersisted const*>(param);
    // shouldn't be trying to save the persisted version to itself
    assert(reinterpret_cast<uint8_t const*>(&settings) != PICOWOTA_APP_STORE);
    assert(sizeof(SettingsPersisted) <= settings.header.size);
    assert(settings.header.size <= MAX_SIZE);

    // need a page-sized scratch pad to copy flash-2-flash and for partial page writes.
    static uint8_t scratch_page[FLASH_PAGE_SIZE];

    auto const size_total_padded = align<uint32_t>(settings.header.size, FLASH_PAGE_SIZE);
    auto const size_main_padded = align<uint32_t>(sizeof(SettingsPersisted), FLASH_PAGE_SIZE);
    auto const size_main_paged = size_main_padded - FLASH_PAGE_SIZE;
    auto const size_main_tail = sizeof(SettingsPersisted) - size_main_paged;
    auto const size_extra_partial = FLASH_PAGE_SIZE - size_main_tail;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
#if DBG_SHOW_FLASH_COPIES
    printf("sizeof   0x%04x\n", (unsigned)sizeof(Settings));
    printf("size     0x%04x\n", (unsigned)settings.header.size);
    printf("settings [0x%p, 0x%p]\n", &settings, &settings + 1);
    printf("extra    [0x%p, 0x%p]\n", PICOWOTA_APP_STORE + sizeof(Settings),
            PICOWOTA_APP_STORE + size_total_padded);

    printf("size_total_padded   0x%04x\n", (unsigned)size_total_padded);
    printf("size_main_padded    0x%04x\n", (unsigned)size_main_padded);
    printf("size_main_paged     0x%04x\n", (unsigned)size_main_paged);
    printf("size_main_tail      0x%04x\n", (unsigned)size_main_tail);
    printf("size_extra_partial  0x%04x\n", (unsigned)size_extra_partial);

    auto const flash_range_erase = [](unsigned offset, unsigned len) {
        printf("erase [0x%06x, 0x%06x]\n", offset, offset + len);
        ::flash_range_erase(offset, len);
    };

    auto const flash_range_program = [](unsigned offset, uint8_t const* p, size_t len) {
        printf("flash [0x%06x, 0x%06x] <- [0x%p, 0x%p]\n", offset, (unsigned)(offset + len), p, p + len);
        ::flash_range_program(offset, p, len);
    };

    auto const memcpy = [&](uint8_t* const dst, uint8_t const* p, size_t len) {
        unsigned offset = APP_STORE_OFFSET + size_main_paged + (dst - scratch_page);
        printf("copy  [0x%06x, 0x%06x] <- [0x%p, 0x%p]\n", offset, (unsigned)(offset + len), p, p + len);
        ::memcpy(dst, p, len);
    };
#endif

    // TODO:  Verify, but it appears that  `flash_range_program`'s source
    //        **cannot** map to flash storage, so pull it page by page into RAM.
    auto const flash_2_flash = [&](uint8_t(&scratch_page)[FLASH_PAGE_SIZE], unsigned offset,
                                       uint8_t const* addr, unsigned const len) {
        auto const offset_end = offset + len;
#if DBG_SHOW_FLASH_COPIES
        auto const src = reinterpret_cast<unsigned>(addr) - XIP_BASE;
        printf("f2f   [0x%06x, 0x%06x] <- [0x%06x, 0x%06x]\n", offset, offset_end, src, src + len);
#endif
        for (; offset < offset_end; offset += FLASH_PAGE_SIZE, addr += FLASH_PAGE_SIZE) {
            ::memcpy(scratch_page, addr, FLASH_PAGE_SIZE);
            ::flash_range_program(offset, scratch_page, FLASH_PAGE_SIZE);
        }
    };

    // We're gonna war-crime:
    //  `.flashdata` stuff supposed to be read-only.
    //  Thankfully for our war-crime, that's **completely unenforced**.
    //  So we're gonna take advantage of that by carving out a flash-sector-sized
    //  block to use as a really slow scratch pad when saving to persistent memory.
    //
    // We **DO NOT** want to use a non-sectioned static for this, since that
    // punts us into RAM, which is very precious.
    //
    // We can safely use this static buffer b/c we've got the flash-write mutex.
    [[gnu::aligned(FLASH_SECTOR_SIZE),
            gnu::section(".flashdata")]] static uint8_t scratch_sector[FLASH_SECTOR_SIZE];
    auto const SCRATCH_SECTOR_OFFSET = reinterpret_cast<unsigned>(&scratch_sector) - XIP_BASE;

    flash_range_erase(SCRATCH_SECTOR_OFFSET, sizeof(scratch_sector));
    flash_2_flash(scratch_page, SCRATCH_SECTOR_OFFSET, PICOWOTA_APP_STORE + size_main_padded,
            size_total_padded - size_main_padded);

    // stash our partial page (if any)
    memcpy(scratch_page, reinterpret_cast<uint8_t const*>(&settings) + size_main_paged, size_main_tail);
    memcpy(scratch_page + size_main_tail, PICOWOTA_APP_STORE + sizeof(SettingsPersisted), size_extra_partial);

    flash_range_erase(APP_STORE_OFFSET, PICOWOTA_APP_STORE_SIZE);
    // copy whole-page parts
    flash_range_program(APP_STORE_OFFSET, reinterpret_cast<uint8_t const*>(&settings), size_main_paged);
    // copy the partial page tail we prepared
    flash_range_program(APP_STORE_OFFSET + size_main_paged, scratch_page, sizeof(scratch_page));
    // and copy the rest we stashed away earlier in the other flash sector
    flash_2_flash(scratch_page, APP_STORE_OFFSET + size_main_padded, scratch_sector,
            size_total_padded - size_main_padded);
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

}  // namespace

Settings g_active;

void init() {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    size_t const LINKER_SIZE = (PICOWOTA_APP_STORE_END + 0) - (PICOWOTA_APP_STORE + 0);
    assert(LINKER_SIZE == PICOWOTA_APP_STORE_SIZE);

    UNSAFE_from_flash(g_active);
    g_active.header.crc = crc(g_active);

    if constexpr (0 < SETTINGS_PERSIST_PERIOD.count()) {
        mk_timer("settings-saver", SETTINGS_PERSIST_PERIOD)([](TimerHandle_t) { save(g_active); });
    }
}

void save(SettingsPersisted& settings) {
    assert(sizeof(SettingsPersisted) <= settings.header.size && "should have a full size field");

    // FP:  false negatives due to non-canonical reps is fine, we just want to
    //      reduce the # of flashes.
    // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison)
    if (memcmp(PICOWOTA_APP_STORE, &settings, sizeof(SettingsPersisted)) == 0) return;

    printf("Persisting settings...\n");
    // update CRC (we can do this safely since everything is read-only)
    settings.header.crc = crc(settings);

    if (auto r = flash_safe_execute(UNSAFE_save_internal, &settings, UINT32_MAX); r != PICO_OK) {
        assert(false);  // something weird went wrong, break out for debugging
        printf("ERR - settings::save - flash acquire failed %d\n", r);
    }
}

// TODO: value range checks for BLE fields
void SettingsV0::merge_valid_fields(SettingsV0 const& x) {
    header = x.header;  // nothing special to do for header
    if (x.fan_policy_env.validate()) fan_policy_env = x.fan_policy_env;
    if (x.fan_policy_thermal.validate()) fan_policy_thermal = x.fan_policy_thermal;
    if (x.fan_power_passive != BLE::NOT_KNOWN) fan_power_passive = x.fan_power_passive;
    if (x.fan_power_automatic != BLE::NOT_KNOWN) fan_power_automatic = x.fan_power_automatic;
    if (x.fan_power_coefficient != BLE::NOT_KNOWN) fan_power_coefficient = x.fan_power_coefficient;
    // nothing to do for voc_calibration b/c that's sensor specific
    voc_calibration = x.voc_calibration;
    if (x.voc_gating_threshold != BLE::NOT_KNOWN && VOC_GATING_THRESHOLD_MIN <= x.voc_gating_threshold)
        voc_gating_threshold = x.voc_gating_threshold;
    if (0 <= x.display_brightness && x.display_brightness <= 1) display_brightness = x.display_brightness;

    switch (x.display_hw) {
    case DisplayHW::GC9A01_240_240: display_hw = x.display_hw; break;
    default: display_hw = DisplayHW::GC9A01_240_240; break;
    }

    display_hw = validate(x.display_hw) ? x.display_hw : DisplayHW::GC9A01_240_240;
    display_ui = validate(x.display_hw, x.display_ui) ? x.display_ui : DisplayUI::CIRCLE_240_CLASSIC;
}

}  // namespace nevermore::settings
