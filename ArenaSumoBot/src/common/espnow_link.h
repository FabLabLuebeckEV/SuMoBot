#pragma once

#include <stddef.h>
#include <stdint.h>

#include <esp_now.h>
#include <WiFi.h>

namespace comms {

using ReceiveHandler = void (*)(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi);
using SendHandler = void (*)(const uint8_t* mac, esp_now_send_status_t status);

bool beginEspNow();
void setReceiveHandler(ReceiveHandler handler);
void setSendHandler(SendHandler handler);
bool addPeer(const uint8_t address[6]);
bool sendTo(const uint8_t address[6], const uint8_t* data, size_t len);

}  // namespace comms
