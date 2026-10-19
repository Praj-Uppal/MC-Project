
#include <stdint.h>
#include <stdbool.h>
#include "ADC/adc.h" // New Header
#include "Kobuki_Drivers/kobuki.h"
#include "G474RE_Drivers/uart.h"
#include "Ultrasonic/ultrasonic.h" // New Header

#define BUTTON_0_START 0x01U  // Confirm the values from my_kobuki
#define BUTTON_1_STOP 0x02U   // Confirm the values from my_kobuki
#define BUTTON_2_ROTATE 0x04U // Confirm the values from my_kobuki

#define DRIVE_SPEED_MM_S ((int16_t)100)
#define ROTATE_SPEED_MM_S ((int16_t)100)
#define ULTRASONIC_CALIBRATION

// Logic
#define NULL 0
#define TRUE 1

<<<<<<< HEAD:main.c
// 0 = A, 1= B
volatile bool state = 0
=======
// States
static enum { IDLE, SEARCH, OBSTACLE, AVOID, DEPOSIT, PATHFIND } state;
>>>>>>> 12f19aea81f903173c051d2c4f312d8339c60a69:src/main.c

    // States
    static enum
{
    IDLE,
    SEARCH,
    ORIENT,
    PATHFIND,
    STORAGE,
    DEPOSIT
} state;

int main(void)
{
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
    while (1)
    {
        Kobuki_Read(&kobuki);
        Ultrasonic_Detect(&object);
        // Both of these only for testing purposes . Not required
        ultrasonic_read = Ultrasonic_Read();
        adc_read = ADC_Read(ADC_CHANNEL_1);
        Kobuki_Update_LEDs(led_1, led_2);
        // A TEST FSM

        if (kobuki.button == BUTTON_1_STOP || kobuki.wheeldrop)
        {
            state = IDLE;
        }
        switch (state)
        {
        case IDLE:
            Kobuki_Drive(STOP);
            Kobuki_Rotate(STOP);
            if (kobuki.button == BUTTON_0_START)
            {
                state = SEARCH;
                led_1 = RED;
            }
            break;
        case SEARCH:
            // Align and approach
            // Aligns with the bollard

            break;
        case ORIENT:
            // Measure Distance to the bollard
            // Based of distance detect layout
            //  if (325 + calibration <distance < 350 + calibration) layout A. else if (...) layout B
            //
            // Rotate 90 CCW or CW
            // Based on layout A or B
            // Orient to correct path. and enter pathfind
            break;
        case PATHFIND:
            // Hardcode path for the current layout
            // Drive 10 m.
            // We need to indicate this is finished.
            // If (distance_complete) -> step 2 update a counter
            // angle rotate -> if (angle_complete)- > step 3
            // Drive again
            // Reached section 3
            break;
        case DEPOSIT:
            break;
        case STORAGE:
            break;
        }
    }
<<<<<<< HEAD:main.c
=======
  }

  //this is a test
  //test again
>>>>>>> 12f19aea81f903173c051d2c4f312d8339c60a69:src/main.c
}
