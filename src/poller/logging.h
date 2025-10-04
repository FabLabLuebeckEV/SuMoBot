#pragma once

#include <Arduino.h>

#ifndef POLLER_LOG_ENABLED
#define POLLER_LOG_ENABLED 0
#endif

#if POLLER_LOG_ENABLED
#define POLLER_LOG_PRINTLN(msg) Serial.println(msg)
#define POLLER_LOG_PRINT(msg) Serial.print(msg)
#define POLLER_LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define POLLER_LOG_LINEBREAK() Serial.println()
#else
#define POLLER_LOG_PRINTLN(msg) do { (void)sizeof(msg); } while (0)
#define POLLER_LOG_PRINT(msg) do { (void)sizeof(msg); } while (0)
#define POLLER_LOG_PRINTF(...) do {} while (0)
#define POLLER_LOG_LINEBREAK() do {} while (0)
#endif
