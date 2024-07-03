#include "settings.hpp"
#include "FreeRTOS.h"  // IWYU pragma: keep
#include "config.hpp"
#include "pico/flash.h"
#include "sdk/ble_data_types.hpp"
#include "semphr.h"  // IWYU pragma: keep [doesn't notice `SemaphoreHandle_t`]
#include "utility/align.hpp"
#include "utility/crc.hpp"
#include "utility/scope_guard.hpp"
#include "utility/timer.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>

#ifndef PICO_FLASH_SAFE_EXECUTE_SUPPORT_FREERTOS_SMP
#error `PICO_FLASH_SAFE_EXECUTE_SUPPORT_FREERTOS_SMP` must be defined
#endif

// defined by linker
extern uint8_t PICOWOTA_APP_STORE[];
extern uint8_t PICOWOTA_APP_STORE_END[];

// Used for debugging/testing the write to flash code, since it isn't trivial.
#define DBG_RAM_PROXY 0

using namespace std;

namespace nevermore::settings {

namespace {

constexpr size_t SLOT_SIZE = FLASH_SECTOR_SIZE;
static_assert(MAX_SIZE <= SLOT_SIZE, "impl' assumes MAX_SIZE is <= SLOT_SIZE");
static_assert(PICOWOTA_APP_STORE_SIZE % SLOT_SIZE == 0,
        "`PICOWOTA_APP_STORE_SIZE` must be a multiple of `SLOT_SIZE`");

constexpr size_t NUM_SLOTS = PICOWOTA_APP_STORE_SIZE / SLOT_SIZE;
static_assert(2 <= NUM_SLOTS, "at least two slots required for save cycling");

#if DBG_RAM_PROXY
uint8_t g_flash_proxy[PICOWOTA_APP_STORE_SIZE];
constexpr auto* SLOT_MEMORY = g_flash_proxy;

void flash_range_erase(unsigned offset, unsigned len) {
    printf("erase [0x%06x, 0x%06x]\n", offset, offset + len);
    assert(len % FLASH_SECTOR_SIZE == 0);
    assert(offset % FLASH_SECTOR_SIZE == 0);
    memset(g_flash_proxy + offset, 0, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
};

void flash_range_program(unsigned offset, uint8_t const* p, size_t len) {
    printf("flash [0x%06x, 0x%06x] <- [0x%p, 0x%p]\n", offset, (unsigned)(offset + len), p, p + len);
    assert(len % FLASH_PAGE_SIZE == 0);
    assert(offset % FLASH_PAGE_SIZE == 0);
    memmove(g_flash_proxy + offset, p, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
};

// doesn't mask interrupts, we need them for stdio
pico_error_codes flash_safe_execute(void (*func)(void*), void* param, uint32_t) {
    vTaskSuspendAll();
    func(param);
    xTaskResumeAll();
    return PICO_OK;
};
#else
constexpr auto* SLOT_MEMORY = PICOWOTA_APP_STORE;
#endif

CRC32_t crc(SettingsPersisted const& settings, uint8_t const slot[]) {
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
    auto y = crc(slot + size_body, slot + size_total, x);
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return y;
}

SettingsPersisted const* slot_validate(uint8_t const slot[]) {
    SettingsPersisted const& stored = *reinterpret_cast<SettingsPersisted const*>(slot);
    if (stored.header.size < sizeof(Header) || MAX_SIZE < stored.header.size) {
        printf("corrupt settings: size=0x%08x not in range [0x%08x, 0x%08x]\n", (unsigned)stored.header.size,
                sizeof(Header), MAX_SIZE);
        return {};
    }

    if (auto x = crc(stored, slot); x != stored.header.crc) {
        printf("corrupt settings: crc reported=0x%08x computed=0x%08x\n", (unsigned)stored.header.crc,
                (unsigned)x);
        return {};
    }

    if (stored.header.version != Header::Version::v0) {
        printf("unrecognised settings version: %u", (unsigned)stored.header.version);
        return {};
    }

    return &stored;
}

auto slots() {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    [[maybe_unused]] size_t const LINKER_SIZE = (PICOWOTA_APP_STORE_END + 0) - (PICOWOTA_APP_STORE + 0);
    assert(LINKER_SIZE == PICOWOTA_APP_STORE_SIZE);

    array<uint8_t const*, NUM_SLOTS> slots{};
    size_t i = 0;
    for (auto& slot : slots)
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        slot = SLOT_MEMORY + i++ * SLOT_SIZE;

    return slots;
}

SettingsPersisted const* slot_latest() {
    SettingsPersisted const* latest = nullptr;

    int i = 0;
    for (auto const* slot : slots()) {
        printf("Checking settings slot #%d\n", i++);
        auto const* settings = slot_validate(slot);
        if (!settings) continue;

        // HACK:  `slot` is valid, but not necessarily 'modern' (has all fields).
        //        However, the `save_counter` field is zero defaulted, which is
        //        perfectly okay for our uses.
        //
        // If the next slot isn't next in sequence, assume a counter wrap or slot wrap.
        // i.e. it's an older slot.
        if (latest && latest->save_counter.next() != settings->save_counter) break;

        latest = settings;
    }

    return latest;
}

uint8_t const* slot_next(uint8_t const* slot) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    assert(SLOT_MEMORY <= slot && slot < (SLOT_MEMORY + PICOWOTA_APP_STORE_SIZE));
    auto offset = slot - SLOT_MEMORY;
    assert(offset % SLOT_SIZE == 0);  // misaligned
    offset += SLOT_SIZE;
    if (PICOWOTA_APP_STORE_SIZE <= offset) offset = 0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return SLOT_MEMORY + offset;
}

void restore_from_slot(SettingsPersisted& dst, SettingsPersisted const& stored) {
    // old version, need to initialise new fields (crc will be dirty but that's fine)
    if (stored.header.size < sizeof(SettingsPersisted)) {
        // Need a temp settings to setup defaults for new fields, existing fields
        // may be malformed/invalid.
        SettingsPersisted default_init{};
        memcpy((void*)&default_init, (void*)&stored, stored.header.size);
        default_init.header.size = sizeof(SettingsPersisted);
        dst.merge_valid_fields(default_init);
        return;
    }

    dst.merge_valid_fields(stored);
}

// The save slot from which `g_active` was initialised from.
// Might not be valid.
atomic<uint8_t const*> g_active_slot = SLOT_MEMORY;

struct SaveParams {
    uint8_t const* slot_src;
    uint8_t const* slot_dst;
    SettingsPersisted* settings;  // CRC field will be mutated/updated
};

// PRECONDITION: All other cores are suspended & all interrupts masked
//               (except if `DBG_RAM_PROXY` is on).
// NB:  Cannot do stdio unless `!!DBG_RAM_PROXY`; stdio can trigger/wait for
//      interrupts, which are masked if `DBG_RAM_PROXY` isn't on.
//      If any asserts fire it'll violate the no IO rule but who cares,
//      things are already screwed.
void UNSAFE_save_internal(void* param) {
    static_assert(MAX_SIZE <= SLOT_SIZE, "more complex impl' required");
    static_assert(SLOT_SIZE == FLASH_SECTOR_SIZE, "more complex impl' required");
    static_assert(2 <= NUM_SLOTS, "impl assumes save cyclic");

    auto const& args = *reinterpret_cast<SaveParams const*>(param);
    // assume save cycling, which means we should never save to the same slot
    assert(args.slot_src != args.slot_dst);
    // shouldn't be trying to save the persisted version to itself
    assert(reinterpret_cast<uint8_t const*>(args.settings) != args.slot_dst);

    auto& settings = *args.settings;
    assert(sizeof(SettingsPersisted) <= settings.header.size);
    assert(settings.header.size <= MAX_SIZE);
    settings.save_counter = settings.save_counter.next();
    settings.header.crc = crc(settings, args.slot_src);

    // need a page-sized scratch pad to copy flash-2-flash and for partial page writes.
    static uint8_t scratch_page[FLASH_PAGE_SIZE];

#if DBG_RAM_PROXY
    auto const dst_offset = unsigned(args.slot_dst - SLOT_MEMORY);
#else
    auto const dst_offset = reinterpret_cast<unsigned>(args.slot_dst) - XIP_BASE;
#endif
    auto const size_total_padded = align<uint32_t>(settings.header.size, FLASH_PAGE_SIZE);
    auto const size_main_padded = align<uint32_t>(sizeof(SettingsPersisted), FLASH_PAGE_SIZE);
    auto const size_main_paged = size_main_padded - FLASH_PAGE_SIZE;
    auto const size_main_tail = sizeof(SettingsPersisted) - size_main_paged;
    auto const size_extra_partial = FLASH_PAGE_SIZE - size_main_tail;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
#if DBG_RAM_PROXY
    printf("sizeof   0x%04x\n", (unsigned)sizeof(Settings));
    printf("size     0x%04x\n", (unsigned)settings.header.size);
    printf("settings [0x%p, 0x%p]\n", &settings, &settings + 1);
    printf("extra    [0x%p, 0x%p]\n", args.slot_src + sizeof(SettingsPersisted),
            args.slot_src + size_total_padded);
    printf("slot src [0x%p, 0x%p]\n", args.slot_src, args.slot_src + settings.header.size);
    printf("slot dst [0x%p, 0x%p]\n", args.slot_dst, args.slot_dst + settings.header.size);

    printf("size_total_padded   0x%04x\n", (unsigned)size_total_padded);
    printf("size_main_padded    0x%04x\n", (unsigned)size_main_padded);
    printf("size_main_paged     0x%04x\n", (unsigned)size_main_paged);
    printf("size_main_tail      0x%04x\n", (unsigned)size_main_tail);
    printf("size_extra_partial  0x%04x\n", (unsigned)size_extra_partial);

    auto const memcpy = [&](uint8_t* const dst, uint8_t const* p, size_t len) {
        unsigned offset = dst_offset + size_main_paged + (dst - scratch_page);
        printf("copy  [0x%06x, 0x%06x] <- [0x%p, 0x%p]\n", offset, (unsigned)(offset + len), p, p + len);
        ::memmove(dst, p, len);
    };
#endif

    // TODO:  Verify, but it appears that  `flash_range_program`'s source
    //        **cannot** map to flash storage, so pull it page by page into RAM.
    auto const flash_2_flash = [&](uint8_t(&scratch_page)[FLASH_PAGE_SIZE], unsigned offset,
                                       uint8_t const* addr, unsigned const len) {
        auto const offset_end = offset + len;
#if DBG_RAM_PROXY
        auto const src = reinterpret_cast<unsigned>(addr) - XIP_BASE;
        printf("f2f   [0x%06x, 0x%06x] <- [0x%06x, 0x%06x]\n", offset, offset_end, src, src + len);
#endif
        for (; offset < offset_end; offset += FLASH_PAGE_SIZE, addr += FLASH_PAGE_SIZE) {
            ::memmove(scratch_page, addr, FLASH_PAGE_SIZE);
            flash_range_program(offset, scratch_page, FLASH_PAGE_SIZE);
        }
    };

    // stash our partial page (if any)
    memcpy(scratch_page, reinterpret_cast<uint8_t const*>(&settings) + size_main_paged, size_main_tail);
    memcpy(scratch_page + size_main_tail, args.slot_src + sizeof(SettingsPersisted), size_extra_partial);

    flash_range_erase(dst_offset, SLOT_SIZE);
    // copy whole-page parts
    flash_range_program(dst_offset, reinterpret_cast<uint8_t const*>(&settings), size_main_paged);
    // copy the partial page tail we prepared
    flash_range_program(dst_offset + size_main_paged, scratch_page, sizeof(scratch_page));
    // and copy the rest from the other slot
    flash_2_flash(scratch_page, dst_offset + size_main_padded, args.slot_src + size_main_padded,
            size_total_padded - size_main_padded);

    // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison) must be bit pattern identical
    assert(memcmp(args.settings, args.slot_dst, sizeof(SettingsPersisted)) == 0);
    assert(memcmp(args.slot_src + sizeof(SettingsPersisted), args.slot_dst + sizeof(SettingsPersisted),
                   args.settings->header.size - sizeof(SettingsPersisted)) == 0);
    assert(slot_validate(args.slot_dst));  // can do IO, but only if fails validate so who cares

    // HACK: Have to do this while we're serialised/locked out.
    g_active_slot = args.slot_dst;
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

constexpr bool operator&(ResetFlags lhs, ResetFlags rhs) {
    return std::to_underlying(lhs) & std::to_underlying(rhs);
}

SemaphoreHandle_t g_save_lock;

[[nodiscard]] auto save_guard() {
    xSemaphoreTake(g_save_lock, portMAX_DELAY);
    return ScopeGuard{[&] { xSemaphoreGive(g_save_lock); }};
}

}  // namespace

Settings g_active;

void Settings::reset(ResetFlags flags) {
    using enum ResetFlags;

    // HACK:  Make sure no one else is running while we do horrible things.
    //        This goes beyond the usual race-y read/writes, we don't want the
    //        periodic save timer to fire while we're mutating this.
    vTaskSuspendAll();
    SCOPE_GUARD {
        xTaskResumeAll();
    };

    if (flags & sensor_calibration) {
        sensors::calibrations_reset();
        voc_calibration = Settings{}.voc_calibration;
    }

    if (flags & policies) {
        // FIXME: This is a maintence nightmare. There must be a better way of doing things.
        auto header_ = g_active.header;
        auto voc_calibration_ = voc_calibration;
        auto display_hw_ = display_hw;
        auto save_counter_ = save_counter;
        *this = {};
        header = header_;
        voc_calibration = voc_calibration_;
        display_hw = display_hw_;
        save_counter = save_counter_;
    }

    if (flags & hardware) {
        // FIXME: This is a maintence nightmare. There must be a better way of doing things.
        display_hw = Settings{}.display_hw;
        pins = PINS_DEFAULT;
    }
}

void init() {
#if DBG_RAM_PROXY
    memcpy(g_flash_proxy, PICOWOTA_APP_STORE, sizeof(g_flash_proxy));
#endif

    g_save_lock = xSemaphoreCreateMutex();

    if (auto const* latest = slot_latest()) {
        restore_from_slot(g_active, *latest);
        g_active_slot = reinterpret_cast<uint8_t const*>(latest);
        printf("Restored settings from slot #%d (CRC: 0x%08x)\n", (g_active_slot - SLOT_MEMORY) / SLOT_SIZE,
                (unsigned)g_active.header.crc);
    }

    if constexpr (0 < SETTINGS_PERSIST_PERIOD.count()) {
        mk_timer("settings-saver", SETTINGS_PERSIST_PERIOD)([](TimerHandle_t) { save(g_active); });
    }
}

void save(SettingsPersisted& settings) {
    assert(sizeof(SettingsPersisted) <= settings.header.size && "should have a full size field");
    assert(g_active_slot);
    auto _ = save_guard();

    // FP:  False negatives due to non-canonical reps is fine, we just want to
    //      reduce the # of flashes.
    // NOLINTNEXTLINE(bugprone-suspicious-memory-comparison)
    if (memcmp(g_active_slot, &settings, sizeof(SettingsPersisted)) == 0) return;

    SaveParams args{
            .slot_src = g_active_slot,
            .slot_dst = slot_next(g_active_slot),
            .settings = &settings,
    };
    printf("Persisting settings slot #%d -> #%d (CRC: 0x%08x)\n", (args.slot_src - SLOT_MEMORY) / SLOT_SIZE,
            (args.slot_dst - SLOT_MEMORY) / SLOT_SIZE, (unsigned)settings.header.crc);

    // HACK: `UNSAFE_save_internal` will update `g_active_slot := args.slot_dst`.
    if (auto r = flash_safe_execute(UNSAFE_save_internal, &args, UINT32_MAX); r != PICO_OK) {
        printf("ERR - settings::save - flash acquire failed %d\n", r);
        assert(false);  // something weird went wrong, break out for debugging
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

    switch (x.display_hw) {
    case DisplayHW::GC9A01_240_240: display_hw = x.display_hw; break;
    default: display_hw = DisplayHW::GC9A01_240_240; break;
    }

    display_hw = validate(x.display_hw) ? x.display_hw : DisplayHW::GC9A01_240_240;
    display_ui = validate(x.display_hw, x.display_ui) ? x.display_ui : DisplayUI::CIRCLE_240_CLASSIC;

    if (0 <= x.display_brightness && x.display_brightness <= 1) display_brightness = x.display_brightness;
    save_counter = x.save_counter;

    try {
        x.pins.validate_or_throw();
        pins = x.pins;
    } catch (char const* msg) {
        printf("WARN - Settings - pins invalid, resetting to defaults. reason: %s\n", msg);
    }
}

}  // namespace nevermore::settings
