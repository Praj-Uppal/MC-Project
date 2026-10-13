
#include "kobuki.h"

void Kobuki_Drive(uint16_t speed) {
  uint8_t speed_lsb = 0;
  uint8_t speed_msb = 0;
  // Construct payload
  uint8_t payload[] = {0x01, 0x04, 0x32, 0xC8, 0x00, 0x00};
  // saturation to avoid going over speed limit
  if (speed > SPEED_LIMIT) {
    speed = SPEED_LIMIT;
  } else if (speed < -SPEED_LIMIT) {
    speed = (~SPEED_LIMIT) + 1;
  }
  // Split LSB & MSB for payload
  // There is no truncation warning due to stdint
  payload[2] = (uint8_t)speed;
  payload[3] = (uint8_t)(speed >> 8);
  Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Rotate(uint16_t speed) {
  uint8_t speed_lsb = 0;
  uint8_t speed_msb = 0;
  // Construct payload
  uint8_t payload[] = {0x01, 0x04, 0x32, 0xC8, 0x01, 0x00};
  // saturation to avoid going over speed limit
  if (speed > SPEED_LIMIT) {
    speed = SPEED_LIMIT;
  } else if (speed < -SPEED_LIMIT) {
    speed = (~SPEED_LIMIT) + 1;
  }
  // split lsb & msb for payload
  payload[2] = (uint8_t)speed;
  payload[3] = (uint8_t)(speed >> 8);
  Kobuki_Tx(payload, sizeof(payload));
}
