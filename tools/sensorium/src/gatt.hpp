#pragma once

namespace nevermore::sensorium::gatt {

// Setup bluetooth and GATT services.
// Caller is responsible for subsequently calling `btstack_run_loop_execute`.
bool init();

}  // namespace nevermore::sensorium::gatt
