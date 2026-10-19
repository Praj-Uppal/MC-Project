
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

// Logic
#define NULL 0
#define TRUE 1

// States
static enum { IDLE, SEARCH, OBSTACLE, AVOID, OBJECT, PATHFIND } state;

int main(void) {
  // Both of these only for testing purposes . Not required
  volatile uint16_t ultrasonic_read;
  volatile uint16_t adc_read;
  Kobuki_Typedef kobuki;
  US_Typedef object;
  Kobuki_LED_Enum led_1, led_2;
  UART_Init();
  ADC_Init();
  Ultrasonic_Init();
  state = IDLE;
  led_1 = OFF;
  led_2 = OFF;
  while (1) {
    Kobuki_Read(&kobuki);
    Ultrasonic_Detect(&object);
    // Both of these only for testing purposes . Not required 
    ultrasonic_read = Ultrasonic_Read();
    adc_read = ADC_Read(ADC_CHANNEL_1);
    Kobuki_Update_LEDs(led_1, led_2);
    // A TEST FSM
    switch (state) {
    case IDLE:
      if (kobuki.button == BUTTON_0_START) {
        state = SEARCH;
        led_1 = RED;
      //  Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 450);
        Kobuki_Drive_Distance_Setpoint( kobuki.distance ,100) ;
      } else if (kobuki.button == BUTTON_1_STOP) {
        state = SEARCH;
        led_1 = RED;
      //Kobuki_Rotate_Angle_Setpoint(kobuki.angle,90);
      Kobuki_Drive_Distance_Setpoint( kobuki.distance , 300) ;
      } else if (kobuki.button == BUTTON_2_ROTATE) {
        state = SEARCH;
        led_1 = RED;
       // Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 90);
        Kobuki_Drive_Distance_Setpoint(kobuki.distance,500);
      } else {
        Kobuki_Drive(STOP);
      }
      break;
    case SEARCH:
      if ((kobuki.wheeldrop != NULL) |
          ((kobuki.bumper + kobuki.cliff) != NULL) |
          (kobuki.rotation_complete == TRUE) |
          (kobuki.distance_complete == TRUE)) {
        state = IDLE;
        led_1 = YELLOW;
      } else {
        //Kobuki_Rotate_Angle(kobuki.angle);
        Kobuki_Drive_Distance ( kobuki.distance );
      }
      break;
    case OBSTACLE:
      break;
    case AVOID:
      break;
    case OBJECT:
      break;
    case PATHFIND:
      break;
    }
  }
}
