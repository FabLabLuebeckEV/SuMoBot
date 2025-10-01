#pragma once

#include <stdint.h>

namespace comms {

// Update these MAC addresses to match the deployed hardware.
constexpr uint8_t PULT_MAC[6] = {0xac, 0x15, 0x18, 0xe9, 0x7e, 0x78};

// Optional pre-configured poller MAC. Leave as zeros to auto-discover.
constexpr uint8_t POLLER_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Optional observer/scoreboard MAC
constexpr uint8_t OBSERVER_MAC[6] = {0x08, 0x3a, 0xf2, 0x37, 0x3c, 0xfc};

}  // namespace comms
