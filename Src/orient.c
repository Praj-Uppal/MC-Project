
#include <stdbool.h>
#include <stdint.h>

#include "ADC/adc.h" // New Header
#include "G474RE_Drivers/uart.h"
#include "Kobuki_Drivers/kobuki.h"
#include "Ultrasonic/ultrasonic.h" // New Header
#define DEG_PER_TICK (1.0 / 100.0)

#define DRIVE_SPEED_MM_S ((int16_t)100)
#define ROTATE_SPEED_MM_S ((int16_t)100)
#define ULTRASONIC_CALIBRATION 10

typedef enum { LAYOUT_A, LAYOUT_B } LayoutState; // 0 = A, 1= B
volatile LayoutState layout;

// Kobuki is global as used in many functions.
volatile Kobuki_Typedef kobuki;

uint16_t bollard_distance = 0; // Variable to store the distance to the bollard
uint8_t layout_detected = 0; // Flag to indicate if a bollard is detected

void Layout_Detect(void) {
  // Read the distance from the ultrasonic sensor
  bollard_distance = Ultrasonic_Read() + ULTRASONIC_CALIBRATION;

  // Check if the distance is within the range for layout A or B
  if (325 <= bollard_distance && bollard_distance <= 375) {
    layout = LAYOUT_A;
    layout_detected = 1; // Set flag to indicate layout A detected
  } else if (175 <= bollard_distance && bollard_distance <= 225) {
    layout = LAYOUT_B;
    layout_detected = 1; // Set flag to indicate layout B detected
  } else {
    layout_detected = 0; // No valid layout detected
  }
}
