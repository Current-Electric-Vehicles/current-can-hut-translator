
#include <CAN.h>
#include <array>

#include "can_codec.h"

template <typename T>
T mapValue(T x, T in_min, T in_max, T out_min, T out_max) {
  if (in_max - in_min == T(0)) {
    return out_min;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

  // delay(2000);

  // Serial.begin(115200);

  if (!CAN.begin(500E3)) {
    Serial.println("Unable to start CAN");
    while (1);
  }

  // Serial.println("CAN translator started");
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
    case 0x0A2: {
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

    case 0x100: {
      auto signal = decodeSignal(&msg.data[0], 0, 16, false, false, 0.01, 0);
      if (signal <= 0) {
        break;
      }

      static std::array<std::pair<double, double>, 23> batteryChargeTable = {
        std::make_pair(100, 4.202),
        std::make_pair(97, 4.133),
        std::make_pair(95, 4.102),
        std::make_pair(90, 4.043),
        std::make_pair(85, 3.983),
        std::make_pair(80, 3.920),
        std::make_pair(75, 3.859),
        std::make_pair(70, 3.801),
        std::make_pair(65, 3.744),
        std::make_pair(60, 3.689),
        std::make_pair(55, 3.629),
        std::make_pair(50, 3.572),
        std::make_pair(45, 3.542),
        std::make_pair(40, 3.521),
        std::make_pair(35, 3.504),
        std::make_pair(30, 3.488),
        std::make_pair(25, 3.458),
        std::make_pair(20, 3.426),
        std::make_pair(15, 3.387),
        std::make_pair(10, 3.342),
        std::make_pair(5, 3.200),
        std::make_pair(2, 2.600),
        std::make_pair(0, 2.500)
      };

      float batteryCharge = 0;
      for (int i = 0; i < batteryChargeTable.size(); i++) {
        auto [pct, voltage] = batteryChargeTable[i];
        if (signal == voltage) {
          batteryCharge = pct;
          break;
        } else if (signal > voltage && i == 0) {
          batteryCharge = pct;
          break;
        } else if (i == 0) {
          continue;
        } else if (signal < voltage) {
          continue;
        } else {
          auto [lastPct, lastVoltage] = batteryChargeTable[i];
          batteryCharge = mapValue<double>(signal, voltage, lastVoltage, pct, lastPct);
          break;
        }
      }

      Serial.print("id: "); Serial.print(msg.id, HEX);
      Serial.print("  - signal: "); Serial.println(signal);
      Serial.print("  - batteryCharge: "); Serial.println(batteryCharge);
      
      msg.id = 0x355;
      msg.u64 = 0;
      encodeSignal(&msg.data[0], batteryCharge, 0, 2, false, false, 0.01, 0);

      send(msg);
      break;
    }

  }
}
