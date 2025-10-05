#include "espnow_link.h"

#include <string.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#include "comms/peer_config.h"

namespace {
constexpr size_t kMaxPacketSize = 250;

comms::ReceiveHandler gReceiveHandler = nullptr;
comms::SendHandler gSendHandler = nullptr;
WiFiUDP gUdp;
bool gUdpActive = false;

IPAddress ipFromAddress(const uint8_t* address) {
  if (!address) {
    return IPAddress(0, 0, 0, 0);
  }
  return IPAddress(address[0], address[1], address[2], address[3]);
}

uint16_t portFromAddress(const uint8_t* address) {
  if (!address) {
    return comms::UDP_PORT;
  }
  return static_cast<uint16_t>(static_cast<uint16_t>(address[4]) << 8 |
                               static_cast<uint16_t>(address[5]));
}

uint16_t sanitisePort(uint16_t port) {
  return port == 0 ? comms::UDP_PORT : port;
}

IPAddress sanitiseIp(const IPAddress& ip) {
  if (ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0) {
    return IPAddress(255, 255, 255, 255);
  }
  return ip;
}

void packAddress(const IPAddress& ip, uint16_t port, uint8_t out[6]) {
  out[0] = ip[0];
  out[1] = ip[1];
  out[2] = ip[2];
  out[3] = ip[3];
  out[4] = static_cast<uint8_t>((port >> 8) & 0xff);
  out[5] = static_cast<uint8_t>(port & 0xff);
}

}  // namespace

namespace comms {

bool beginEspNow() {
  if (gUdpActive) {
    return true;
  }
  if (gUdp.begin(UDP_PORT) == 0) {
    return false;
  }
  gUdpActive = true;
  return true;
}

void setReceiveHandler(ReceiveHandler handler) {
  gReceiveHandler = handler;
}

void setSendHandler(SendHandler handler) {
  gSendHandler = handler;
}

bool addPeer(const uint8_t* /*address*/) {
  // UDP transport does not require explicit peer registration.
  return true;
}

bool sendTo(const uint8_t address[6], const uint8_t* data, size_t len) {
  if (!data || len == 0) {
    return false;
  }
  if (!gUdpActive && !beginEspNow()) {
    return false;
  }

  IPAddress target = sanitiseIp(ipFromAddress(address));
  const uint16_t port = sanitisePort(portFromAddress(address));

  if (gUdp.beginPacket(target, port) == 0) {
    if (gSendHandler) {
      gSendHandler(address, false);
    }
    return false;
  }

  const size_t written = gUdp.write(data, len);
  const bool success = (written == len) && (gUdp.endPacket() == 1);
  if (gSendHandler) {
    gSendHandler(address, success);
  }
  return success;
}

void pump() {
  if (!gUdpActive) {
    return;
  }

  int packetSize = 0;
  while ((packetSize = gUdp.parsePacket()) > 0) {
    if (packetSize <= 0 || packetSize > static_cast<int>(kMaxPacketSize)) {
      while (gUdp.available() > 0) {
        gUdp.read();
      }
      continue;
    }

    uint8_t buffer[kMaxPacketSize];
    const int len = gUdp.read(buffer, packetSize);
    if (len <= 0) {
      continue;
    }

    uint8_t address[6];
    packAddress(gUdp.remoteIP(), static_cast<uint16_t>(gUdp.remotePort()), address);

    if (gReceiveHandler) {
      gReceiveHandler(address, buffer, len, 0);
    }
  }
}

}  // namespace comms

