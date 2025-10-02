#pragma once

#include <stdint.h>

#include "hardware_config.h"

namespace comms {

enum class CommandType : uint8_t {
  kNoop             = 0,
  kMoveAbsolute     = 1,
  kMoveRelative     = 2,
  kMoveToLimit      = 3,
  kStopStepper      = 4,
  kStartAnimation   = 5,
  kStopAnimation    = 6,
  kCalibrate        = 7,
  kSetOverrunArmed  = 8,
  kSetParameter     = 9,
  kPing             = 10
};

enum class LimitDirection : int8_t {
  kDown = -1,
  kNone = 0,
  kUp   = 1,
};

enum class AnimationId : uint8_t {
  kNone = 0,
  kCountdown,
  kPollerOverrun,
  kArenaStop,
  kArenaControl
};

enum class PollerState : uint8_t {
  kIdle = 0,
  kMoving,
  kCalibrating,
  kFault
};

enum StatusFlag : uint16_t {
  kNone               = 0,
  kEndstopActive      = 1 << 0,
  kPollerSensorActive = 1 << 1,
  kCommandError       = 1 << 2,
  kLinkLowQuality     = 1 << 3,
  kOverrunDetected    = 1 << 4,
  kOverrunArmed       = 1 << 5,
  kCooldownActive     = 1 << 6
};

enum class PollerParameterId : uint8_t {
  kPositionHome = 0,
  kPositionUpTarget,
  kPositionDownTarget,
  kDownArmMargin,
  kStepperMaxSpeed,
  kStepperAcceleration,
  kStatusIntervalMs,
  kOverrunCooldownMs,
  kCount
};

inline StatusFlag operator|(StatusFlag a, StatusFlag b) {
  return static_cast<StatusFlag>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

inline StatusFlag& operator|=(StatusFlag& a, StatusFlag b) {
  a = a | b;
  return a;
}

#pragma pack(push, 1)
struct PollerCommand {
  uint8_t commandId = 0;          // unique id provided by pult
  CommandType type = CommandType::kNoop;
  int32_t value = 0;              // steps or other value depending on command
  LimitDirection limit = LimitDirection::kNone;
  AnimationId animation = AnimationId::kNone;
  uint8_t reserved = 0;           // parameter ID when using kSetParameter
};
static_assert(sizeof(PollerCommand) <= 16, "PollerCommand should remain compact");

struct PollerStatus {
  uint32_t uptimeMs = 0;
  PollerState state = PollerState::kIdle;
  AnimationId activeAnimation = AnimationId::kNone;
  int32_t currentPosition = 0;
  int32_t targetPosition = 0;
  uint8_t lastCommandId = 0;
  uint8_t reserved = 0;
  uint16_t statusFlags = 0;       // combination of StatusFlag bits
  int8_t lastRssi = 0;            // last received RSSI
  int8_t emaRssi = 0;             // filtered RSSI
  hardware::PollerParameters config = hardware::DEFAULT_POLLER_PARAMETERS;
};
static_assert(sizeof(PollerStatus) <= 64, "PollerStatus should remain compact");
#pragma pack(pop)

}  // namespace comms
