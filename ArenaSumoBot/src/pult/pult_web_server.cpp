#include "pult_web_server.h"

#include <Arduino.h>
#include <LittleFS.h>

namespace pult {

namespace {
const AsyncWebParameter* findParam(AsyncWebServerRequest* request, const char* name) {
  if (!request) {
    return nullptr;
  }
  const AsyncWebParameter* param = request->getParam(name, true);
  if (!param) {
    param = request->getParam(name);
  }
  return param;
}
}

PultWebServer::PultWebServer(PultController& controller, MatchOrchestrator& orchestrator)
    : server_(80), controller_(controller), orchestrator_(orchestrator) {}

void PultWebServer::begin() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
  }

  server_.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleStatusRequest(request);
  });

  server_.on("/api/command", HTTP_GET, [this](AsyncWebServerRequest* request) {
    handleCommandRequest(request);
  });
  server_.on("/api/command", HTTP_POST, [this](AsyncWebServerRequest* request) {
    handleCommandRequest(request);
  });

  server_.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server_.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "application/json", "{\"error\":\"Not found\"}");
  });

  server_.begin();
}

void PultWebServer::loop() {
  // No periodic work needed; placeholder for future needs.
}

void PultWebServer::handleCommandRequest(AsyncWebServerRequest* request) {
  const AsyncWebParameter* typeParam = findParam(request, "type");
  if (!typeParam) {
    request->send(400, "application/json", "{\"error\":\"Missing type parameter\"}");
    return;
  }

  const String type = typeParam->value();
  bool success = false;
  String error = "Unknown command";

  if (type == "startMatch") {
    orchestrator_.startMatch();
    success = true;
    error = "";
  } else if (type == "stopMatch") {
    orchestrator_.stopMatch();
    success = true;
    error = "";
  } else if (type == "raisePoller") {
    orchestrator_.raisePoller();
    success = true;
    error = "";
  } else if (type == "lowerPoller") {
    orchestrator_.lowerPoller();
    success = true;
    error = "";
  } else if (type == "triggerCountdown") {
    orchestrator_.triggerCountdown();
    success = true;
    error = "";
  } else if (type == "stopStepper") {
    orchestrator_.stopStepper();
    success = true;
    error = "";
  } else if (type == "calibrate") {
    orchestrator_.calibrate();
    success = true;
    error = "";
  } else if (type == "moveAbsolute") {
    const AsyncWebParameter* valueParam = findParam(request, "value");
    if (valueParam) {
      const int32_t target = valueParam->value().toInt();
      orchestrator_.movePollerAbsolute(target);
      success = true;
      error = success ? "" : "Command failed";
    } else {
      error = "Missing value parameter";
    }
  } else if (type == "moveRelative") {
    const AsyncWebParameter* valueParam = findParam(request, "value");
    if (valueParam) {
      const int32_t delta = valueParam->value().toInt();
      orchestrator_.movePollerRelative(delta);
      success = true;
      error = success ? "" : "Command failed";
    } else {
      error = "Missing value parameter";
    }
  } else if (type == "moveLimit") {
    const AsyncWebParameter* dirParam = findParam(request, "direction");
    if (dirParam) {
      comms::LimitDirection dir;
      if (parseLimit(dirParam->value(), &dir)) {
        orchestrator_.movePollerToLimit(dir);
        success = true;
        error = "";
      } else {
        error = "Invalid direction";
      }
    } else {
      error = "Missing direction parameter";
    }
  } else if (type == "stop") {
    orchestrator_.stopStepper();
    success = true;
    error = "";
  } else if (type == "startAnimation") {
    const AsyncWebParameter* animParam = findParam(request, "animation");
    if (animParam) {
      comms::AnimationId animation;
      if (parseAnimation(animParam->value(), &animation)) {
        success = controller_.sendStartAnimation(animation);
        error = success ? "" : "Command failed";
      } else {
        error = "Invalid animation";
      }
    } else {
      error = "Missing animation parameter";
    }
  } else if (type == "stopAnimation") {
    success = controller_.sendStopAnimation();
    error = success ? "" : "Command failed";
  } else if (type == "calibrate") {
    success = controller_.sendCalibrate();
    error = success ? "" : "Command failed";
  }

  if (success) {
    request->send(200, "application/json", "{\"success\":true}");
  } else {
    String msg = "{\"success\":false";
    msg += ",\"error\":\"";
    msg += error;
    msg += "\"}";
    request->send(400, "application/json", msg);
  }
}

void PultWebServer::handleStatusRequest(AsyncWebServerRequest* request) {
  String json;
  json.reserve(320);
  json += "{\"phase\":\"";
  json += phaseToString(orchestrator_.phase());
  json += "\"";
  json += ",\"matchRunning\":";
  json += orchestrator_.matchRunning() ? "true" : "false";
  json += ",\"countdown\":";
  json += orchestrator_.countdownActive() ? "true" : "false";
  json += ",\"remainingMs\":";
  json += orchestrator_.remainingMatchTimeMs();
  json += ",\"lastAction\":\"";
  json += orchestrator_.lastAction();
  json += "\"";
  json += ",\"ready\":";
  json += controller_.hasStatus() ? "true" : "false";

  if (controller_.hasStatus()) {
    const comms::PollerStatus& status = controller_.status();
    json += ",\"uptime\":";
    json += status.uptimeMs;
    json += ",\"state\":\"";
    json += stateToString(status.state);
    json += "\"";
    json += ",\"animation\":\"";
    json += animationToString(status.activeAnimation);
    json += "\"";
    json += ",\"position\":";
    json += status.currentPosition;
    json += ",\"target\":";
    json += status.targetPosition;
    json += ",\"flags\":";
    json += status.statusFlags;
    json += ",\"lastCommandId\":";
    json += status.lastCommandId;
    json += ",\"rssi\":";
    json += status.lastRssi;
    json += ",\"emaRssi\":";
    json += status.emaRssi;
    json += ",\"ageMs\":";
    json += millis() - controller_.lastStatusTimestamp();
  }

  json += "}";

  request->send(200, "application/json", json);
}

bool PultWebServer::parseAnimation(const String& value, comms::AnimationId* animation) const {
  if (value.equalsIgnoreCase("countdown")) {
    *animation = comms::AnimationId::kCountdown;
    return true;
  }
  if (value.equalsIgnoreCase("poller_overrun") || value.equalsIgnoreCase("poller")) {
    *animation = comms::AnimationId::kPollerOverrun;
    return true;
  }
  if (value.equalsIgnoreCase("arena_stop") || value.equalsIgnoreCase("stop")) {
    *animation = comms::AnimationId::kArenaStop;
    return true;
  }
  if (value.equalsIgnoreCase("arena_control") || value.equalsIgnoreCase("control")) {
    *animation = comms::AnimationId::kArenaControl;
    return true;
  }
  if (value.equalsIgnoreCase("none")) {
    *animation = comms::AnimationId::kNone;
    return true;
  }
  return false;
}

bool PultWebServer::parseLimit(const String& value, comms::LimitDirection* direction) const {
  if (value.equalsIgnoreCase("up")) {
    *direction = comms::LimitDirection::kUp;
    return true;
  }
  if (value.equalsIgnoreCase("down")) {
    *direction = comms::LimitDirection::kDown;
    return true;
  }
  return false;
}

String PultWebServer::stateToString(comms::PollerState state) const {
  switch (state) {
    case comms::PollerState::kIdle:
      return "idle";
    case comms::PollerState::kMoving:
      return "moving";
    case comms::PollerState::kCalibrating:
      return "calibrating";
    case comms::PollerState::kFault:
      return "fault";
  }
  return "unknown";
}

String PultWebServer::animationToString(comms::AnimationId animation) const {
  switch (animation) {
    case comms::AnimationId::kNone:
      return "none";
    case comms::AnimationId::kCountdown:
      return "countdown";
    case comms::AnimationId::kPollerOverrun:
      return "poller_overrun";
    case comms::AnimationId::kArenaStop:
      return "arena_stop";
    case comms::AnimationId::kArenaControl:
      return "arena_control";
  }
  return "unknown";
}

String PultWebServer::phaseToString(MatchOrchestrator::Phase phase) const {
  switch (phase) {
    case MatchOrchestrator::Phase::kIdle:
      return "idle";
    case MatchOrchestrator::Phase::kCountdown:
      return "countdown";
    case MatchOrchestrator::Phase::kRunning:
      return "running";
  }
  return "unknown";
}

}  // namespace pult
