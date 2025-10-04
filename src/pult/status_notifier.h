#pragma once

#include <Arduino.h>
#include <Preferences.h>

namespace pult {

class StatusNotifier {
 public:
  enum class Action : uint8_t {
    kMatchStart,
    kMatchStop,
    kCountdownStart,
    kPollerUp,
    kPollerDown,
    kPollerOverrun
  };

  void begin();
  void loop();

  void setEndpoint(const String& url);
  String endpoint() const;
  bool enabled() const;

  void notify(Action action);

 private:
  const char* actionName(Action action) const;
  String buildUrl(Action action) const;
  void persist();

  Preferences prefs_;
  bool prefsOpen_ = false;
  String endpoint_;
  uint32_t lastPostMs_ = 0;
  static constexpr uint32_t kMinPostIntervalMs = 100;
};

}  // namespace pult
