#include "poller_settings.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>

namespace poller {
namespace settings {
namespace {

constexpr uint32_t kMagic = 0x534D504C;  // 'SMPL'
constexpr uint8_t kVersion = 1;
constexpr size_t kStorageSize = 128;

struct StorageBlock {
  uint32_t magic = 0;
  uint8_t version = 0;
  uint8_t reserved[3] = {0};
  hardware::PollerParameters params{};
  uint32_t checksum = 0;
};

bool ensureInitialised() {
  static bool initialised = false;
  if (initialised) {
    return true;
  }
  if (!EEPROM.begin(kStorageSize)) {
    Serial.println("EEPROM init failed");
    return false;
  }
  initialised = true;
  return true;
}

uint32_t checksum(const void* data, size_t len) {
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

}  // namespace

bool loadParameters(hardware::PollerParameters* out) {
  if (!out || !ensureInitialised()) {
    return false;
  }

  StorageBlock block{};
  EEPROM.get(0, block);
  const uint32_t expected = checksum(&block.params, sizeof(block.params));
  if (block.magic != kMagic || block.version != kVersion || block.checksum != expected) {
    return false;
  }

  *out = hardware::sanitized(block.params);
  return true;
}

bool saveParameters(const hardware::PollerParameters& params) {
  if (!ensureInitialised()) {
    return false;
  }

  StorageBlock block{};
  block.magic = kMagic;
  block.version = kVersion;
  block.params = hardware::sanitized(params);
  block.checksum = checksum(&block.params, sizeof(block.params));

  EEPROM.put(0, block);
  if (!EEPROM.commit()) {
    Serial.println("EEPROM commit failed");
    return false;
  }
  return true;
}

}  // namespace settings
}  // namespace poller

