
#include <stdint.h>
#define STM32G474xx
#include "../ADC/adc.h" // New Header
#include "../Kobuki_Drivers/kobuki.h"
#include "stm32g4xx.h"
#include "uart.h"
#include "ultrasonic.h" // New Header

#define BUTTON_0_START 0x01U  // Confirm the values from my_kobuki
#define BUTTON_1_STOP 0x02U   // Confirm the values from my_kobuki
#define BUTTON_2_ROTATE 0x04U // Confirm the values from my_kobuki

#define DRIVE_SPEED_MM_S ((int16_t)100)
#define ROTATE_SPEED_MM_S ((int16_t)100)
// static void Send_Kobuki_Move(int16_t speed, int16_t radius) {
//   uint8_t payload[6];
//   payload[0] = 0x01U; /* Base Control sub - packet ID . */
//   payload[1] = 0x04U; /* Speed and radius occupy four bytes . */
//   payload[2] = (uint8_t)(speed & 0xFF);
//   payload[3] = (uint8_t)((uint16_t)speed >> 8);
//   payload[4] = (uint8_t)(radius & 0xFF);
//   payload[5] = (uint8_t)((uint16_t)radius >> 8);
//   Kobuki_Tx(payload, sizeof(payload));
// }
int main(void) {
  Kobuki_Typedef my_kobuki = {0};
  US_Typedef ultrasonic_object = {0};
  uint8_t is_running = 0U;
  uint8_t is_rotating = 0U;
  UART_Init();
  ADC_Init();
  Ultrasonic_Init();
  // Send_Kobuki_Move(0, 0);
  Kobuki_Drive(0); // Command zero velocity . IDLE STATE
  Kobuki_Rotate(0);
  while (1) {
    Kobuki_Read(&my_kobuki);
    Ultrasonic_Detect(&ultrasonic_object);
    /* Button 0 selects forward motion when the path is clear . */
    if (((my_kobuki.button & 1U) != 0U) && (ultrasonic_object.detected == 0U)) {
      is_running = 1U;
      is_rotating = 0U;
    }
    /* Button 2 selects rotation when the path is clear . */
    else if (((my_kobuki.button & 4U) != 0U) &&
             (ultrasonic_object.detected == 0U)) {
      is_rotating = 1U;
      is_running = 0U;
    }
    /* Button 1 , bumper or ultrasonic detection stops the robot .
     */
    if (((my_kobuki.button & 2U) != 0U) || (my_kobuki.bumper != 0U) ||
        (ultrasonic_object.detected != 0U)) {
      is_running = 0U;
      is_rotating = 0U;
    }
    if (is_running != 0U)
      Kobuki_Drive(100);
    else if (is_rotating != 0U)
      Kobuki_Rotate(100);
    else {
      Kobuki_Drive(0); // Command zero velocity . IDLE STATE
      Kobuki_Rotate(0);
    }
  }
}