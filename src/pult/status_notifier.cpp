#include "pult/status_notifier.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>

namespace {
constexpr const char* kPrefsNamespace = "status";
constexpr const char* kPrefsKeyEndpoint = "webhook";
constexpr const char* kDefaultEndpoint = "http://10.124.42.5:5000/";
}

namespace pult {

void StatusNotifier::begin() {
  if (!prefsOpen_) {
    prefsOpen_ = prefs_.begin(kPrefsNamespace, false);
  }
  if (prefsOpen_) {
    endpoint_ = prefs_.getString(kPrefsKeyEndpoint, "");
  } else {
    endpoint_.clear();
  }

  if (endpoint_.isEmpty() && kDefaultEndpoint && strlen(kDefaultEndpoint) > 0) {
    endpoint_ = kDefaultEndpoint;
    persist();
  }
}

void StatusNotifier::loop() {
  // No periodic tasks for now. Placeholder for future retry logic.
}

void StatusNotifier::setEndpoint(const String& url) {
  String trimmed = url;
  trimmed.trim();
  endpoint_ = trimmed;
  persist();
}

String StatusNotifier::endpoint() const {
  return endpoint_;
}

bool StatusNotifier::enabled() const {
  return endpoint_.length() > 0;
}

void StatusNotifier::notify(Action action) {
  if (!enabled()) {
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastPostMs_ < kMinPostIntervalMs) {
    // Throttle bursts to avoid overwhelming the endpoint.
    return;
  }

  String url = buildUrl(action);
  if (url.isEmpty()) {
    return;
  }

  HTTPClient http;
  if (!http.begin(url)) {
    Serial.printf("[Notifier] Failed to begin HTTP POST: %s\n", url.c_str());
    return;
  }

  http.addHeader("Content-Type", "application/json");
  const int code = http.POST("{}");
  if (code < 0) {
    Serial.printf("[Notifier] HTTP POST error (%d): %s\n", code, url.c_str());
  } else {
    Serial.printf("[Notifier] POST %s => %d\n", url.c_str(), code);
  }
  http.end();
  lastPostMs_ = millis();
}

const char* StatusNotifier::actionName(Action action) const {
  switch (action) {
    case Action::kMatchStart:
      return "MatchStart";
    case Action::kMatchStop:
      return "MatchStop";
    case Action::kCountdownStart:
      return "CountdownStart";
    case Action::kPollerUp:
      return "PollerUp";
    case Action::kPollerDown:
      return "PollerDown";
    case Action::kPollerOverrun:
      return "PollerOverrun";
  }
  return "Unknown";
}

String StatusNotifier::buildUrl(Action action) const {
  if (!enabled()) {
    return "";
  }

  String base = endpoint_;
  base.trim();
  if (base.length() == 0) {
    return "";
  }

  if (base.endsWith("/")) {
    base.remove(base.length() - 1);
  }

  String url = base;
  url += "/queues/queue?name=SuMo:";
  url += actionName(action);
  return url;
}

void StatusNotifier::persist() {
  if (!prefsOpen_) {
    prefsOpen_ = prefs_.begin(kPrefsNamespace, false);
  }
  if (prefsOpen_) {
    prefs_.putString(kPrefsKeyEndpoint, endpoint_);
  }
}

}  // namespace pult
