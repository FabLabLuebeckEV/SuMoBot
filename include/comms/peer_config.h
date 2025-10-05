#pragma once

#include <stdint.h>

namespace comms {

// UDP transport port used for poller/pult communication.
constexpr uint16_t UDP_PORT = 42142;

// Helper address encoding (IPv4 + port). Defaults to broadcast discovery.
constexpr uint8_t BROADCAST_ADDRESS[6] = {
    0xff, 0xff, 0xff, 0xff,
    static_cast<uint8_t>((UDP_PORT >> 8) & 0xff),
    static_cast<uint8_t>(UDP_PORT & 0xff)};

// Default peer targets. Leave as broadcast for auto-discovery, or override with
// a fixed IPv4 (bytes 0-3) and UDP port (bytes 4-5, big endian).
constexpr uint8_t PULT_MAC[6] = {
    0xff, 0xff, 0xff, 0xff,
    static_cast<uint8_t>((UDP_PORT >> 8) & 0xff),
    static_cast<uint8_t>(UDP_PORT & 0xff)};

constexpr uint8_t POLLER_MAC[6] = {
    0xff, 0xff, 0xff, 0xff,
    static_cast<uint8_t>((UDP_PORT >> 8) & 0xff),
    static_cast<uint8_t>(UDP_PORT & 0xff)};

constexpr uint8_t OBSERVER_MAC[6] = {
    0xff, 0xff, 0xff, 0xff,
    static_cast<uint8_t>((UDP_PORT >> 8) & 0xff),
    static_cast<uint8_t>(UDP_PORT & 0xff)};

}  // namespace comms
