#pragma once

#include <stdint.h>

#include "comms/messages.h"

namespace pult {

class PultController {
 public:
  void begin();
  void loop();

  static void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi);

  bool sendMoveAbsolute(int32_t position);
  bool sendMoveRelative(int32_t delta);
  bool sendMoveToLimit(comms::LimitDirection direction);
  bool sendStop();
  bool sendStartAnimation(comms::AnimationId animation);
  bool sendStopAnimation();
  bool sendCalibrate();
  bool sendSetParameter(comms::PollerParameterId id, int32_t rawValue);
  bool sendObserverMessage(const char* text);

  uint32_t timeSinceLastPong(uint32_t now) const;
  bool hasPong() const;
  bool pingHealthy(uint32_t now, uint32_t timeoutMs = 10000) const;

  bool hasStatus() const;
  const comms::PollerStatus& status() const;
  uint32_t lastStatusTimestamp() const;
  bool pollerKnown() const { return pollerKnown_; }

 private:
  bool sendCommand(const comms::PollerCommand& command, uint8_t* outCommandId = nullptr);
  void tickPing(uint32_t now);
  bool sendPing(uint32_t now);
  void handleStatus(const comms::PollerStatus& status, const uint8_t mac[6], int8_t rssi);

  comms::PollerStatus lastStatus_{};
  bool statusReceived_ = false;
  uint32_t lastStatusMs_ = 0;
  uint8_t pollerAddress_[6] = {0};
  bool pollerKnown_ = false;
  uint8_t observerAddress_[6] = {0};
  bool observerKnown_ = false;
  uint8_t commandCounter_ = 0;
  uint32_t lastPingMs_ = 0;
  uint32_t lastPongMs_ = 0;
  uint8_t lastPingId_ = 0;

  static PultController* instance_;
};

}  // namespace pult
