#include "espnow_link.h"

#include <string.h>

#include <esp_idf_version.h>
#include <esp_wifi.h>

namespace {
comms::ReceiveHandler gReceiveHandler = nullptr;
comms::SendHandler gSendHandler = nullptr;

void onSendInternal(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (gSendHandler) {
    gSendHandler(mac_addr, status);
  }
}

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
void onReceiveInternal(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (!gReceiveHandler || !info || !data || len <= 0) {
    return;
  }
  int8_t rssi = 0;
  if (info->rx_ctrl != nullptr) {
    rssi = info->rx_ctrl->rssi;
  }
  gReceiveHandler(info->src_addr, data, len, rssi);
}
#else
void onReceiveInternal(const uint8_t* mac_addr, const uint8_t* data, int len) {
  if (!gReceiveHandler || !mac_addr || !data || len <= 0) {
    return;
  }
  gReceiveHandler(mac_addr, data, len, -127);
}
#endif

}  // namespace

namespace comms {

bool beginEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_now_register_recv_cb(onReceiveInternal);
#else
  esp_now_register_recv_cb(onReceiveInternal);
#endif
  esp_now_register_send_cb(onSendInternal);
  return true;
}

void setReceiveHandler(ReceiveHandler handler) {
  gReceiveHandler = handler;
}

void setSendHandler(SendHandler handler) {
  gSendHandler = handler;
}

bool addPeer(const uint8_t address[6]) {
  if (!address) {
    return false;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_ERR_ESPNOW_EXIST) {
    return true;
  }
  return result == ESP_OK;
}

bool sendTo(const uint8_t address[6], const uint8_t* data, size_t len) {
  if (!address || !data || len == 0 || len > ESP_NOW_MAX_DATA_LEN) {
    return false;
  }
  return esp_now_send(address, const_cast<uint8_t*>(data), len) == ESP_OK;
}

}  // namespace comms
