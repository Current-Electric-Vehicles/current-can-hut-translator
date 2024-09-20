
#include <CAN.h>

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

  Serial.begin(115200);
  while (!Serial);

  if (!CAN.begin(500E3)) {
    Serial.println("Unable to start CAN");
    while (1);
  }
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
    msg.data[0] = (i < msg.len && CAN.available())
        ? CAN.read()
        : 0;
  }

  switch (msg.id) {
    case 0x0A2:
      uint8_t d1 = msg.d1;
      uint8_t d2 = msg.d2;

      msg.id = 0x126;
      msg.u64 = 0;
      msg.d4 = d2;
      msg.d5 = d1;

      send(msg);
      break;
  }
}
