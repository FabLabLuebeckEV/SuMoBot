#pragma once

#include <ESPAsyncWebServer.h>

#include "pult/match_orchestrator.h"
#include "pult/pult_controller.h"

namespace pult {

class PultWebServer {
 public:
  PultWebServer(PultController& controller, MatchOrchestrator& orchestrator);

  void begin();
  void loop();

 private:
  void handleCommandRequest(AsyncWebServerRequest* request);
  void handleStatusRequest(AsyncWebServerRequest* request);

  bool parseAnimation(const String& value, comms::AnimationId* animation) const;
  bool parseLimit(const String& value, comms::LimitDirection* direction) const;
  String stateToString(comms::PollerState state) const;
  String animationToString(comms::AnimationId animation) const;
  String phaseToString(MatchOrchestrator::Phase phase) const;

  AsyncWebServer server_;
  PultController& controller_;
  MatchOrchestrator& orchestrator_;
};

}  // namespace pult
