
#include <stdbool.h>
#include <stdint.h>

#include "ADC/adc.h"  // New Header
#include "G474RE_Drivers/uart.h"
#include "Kobuki_Drivers/kobuki.h"
#include "Ultrasonic/ultrasonic.h"  // New Header
#define DEG_PER_TICK (1.0 / 100.0)

#define BUTTON_0_START 0x01U   // Confirm the values from my_kobuki
#define BUTTON_1_STOP 0x02U    // Confirm the values from my_kobuki
#define BUTTON_2_ROTATE 0x04U  // Confirm the values from my_kobuki

#define DRIVE_SPEED_MM_S ((int16_t)100)
#define ROTATE_SPEED_MM_S ((int16_t)100)
#define ULTRASONIC_CALIBRATION 10

// 0 = A, 1= B

typedef enum { LAYOUT_A, LAYOUT_B } LayoutState;
volatile LayoutState layout;
// States
typedef enum { IDLE, SEARCH, ORIENT, PATHFIND, STORAGE, DEPOSIT } State;

// Kobuki is global as used in many functions.
volatile Kobuki_Typedef kobuki;
