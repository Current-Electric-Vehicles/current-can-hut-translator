
#include <CAN.h>
#include "can_codec.h"

typedef struct {
  long id;
  bool extended;
  uint8_t len;
  union {
    struct {
      union { uint8_t d1; };
      union { uint8_t d2; };
      union { uint8_t d3; };
      union { uint8_t d4; };
      union { uint8_t d5; };
      union { uint8_t d6; };
      union { uint8_t d7; };
      union { uint8_t d8; };
    };
    struct {
      union { uint64_t u64; };
    };
    uint8_t data[8];
  };
} CANMessage;

void send(CANMessage& msg) {
  if (msg.extended) {
    CAN.beginExtendedPacket(msg.id, msg.len);
  } else {
    CAN.beginPacket(msg.id, msg.len);
  }
  for (auto i = 0; i < msg.len; i++) {
    CAN.write(msg.data[i]);
  }
  CAN.endPacket();
}

void setup() {

  delay(2000);

  Serial.begin(115200);
  while (!Serial);

  if (!CAN.begin(500E3)) {
    Serial.println("Unable to start CAN");
    while (1);
  }

  Serial.println("CAN translator started");
}

void loop() {

  auto packetSize = CAN.parsePacket();
  if (packetSize <= 0) {
    return;
  } else if (CAN.packetRtr()) {
    return;
  }

  CANMessage msg;
  msg.id = CAN.packetId();
  msg.extended = CAN.packetExtended();
  msg.len = CAN.packetDlc();
  for (int i = 0; i < 8; i++) {
    msg.data[i] = (i < msg.len && CAN.available())
        ? CAN.read()
        : 0;
  }

  switch (msg.id) {
    case 0x0A2:
      auto signal = decodeSignal(&msg.data[0], 0, 16, false, true, 0.1, 0);
      if (signal <= 0) {
        break;
      }

      msg.id = 0x126;
      msg.u64 = 0;
      encodeSignal(&msg.data[0], signal, 32, 16, false, true, 1, 0);

      send(msg);
      break;
  }
}
