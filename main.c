
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

// Logic
#define NULL 0
#define TRUE 1

// 0 = A, 1= B

typedef enum { LAYOUT_A, LAYOUT_B } LayoutState;
volatile LayoutState layout;
// States
typedef enum { IDLE, SEARCH, ORIENT, PATHFIND, STORAGE, DEPOSIT } State;

// Kobuki is global as used in many functions.
volatile Kobuki_Typedef kobuki;

typedef enum {
  STATE_INIT,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_ALIGNING,
  STATE_DONE
} AAstate;

void Kobuki_Rotate_AbsAngleTickBased(int16_t current_angle_tick,
                                     int16_t target_angle_tick) {
  // Calculate the relative difference in ticks. This would give us the amount
  // of ticks to rotate by to get to target angle.
  int32_t relative_ticks =
      (int32_t)(current_angle_tick) - (int32_t)(target_angle_tick);

  // Convert it to a relative angle reading. This is to use the wrapper for
  // setpoint.
  int16_t relative_angle_deg = (int16_t)((double)relative_ticks * DEG_PER_TICK);
  // Use the setpoint wrapper to set the rotaiton.
  Kobuki_Rotate_Angle_Setpoint(current_angle_tick, relative_angle_deg);
}

bool align(int16_t distanceThresh) {
  // variable to track alignment state.
  volatile static AAstate state = STATE_INIT;
  // Variables for thetaleft, thetaright, and average theta(theta)
  volatile static int16_t thetaLeft, thetaRight, theta;

  // If in the middle of a rotation, return 0 to process rotation.
  if (!kobuki.rotation_complete) {
    return 0;
  }

  // ultrasonic reading
  volatile int16_t d = Ultrasonic_Read() + ULTRASONIC_CALIBRATION;
  switch (state) {
    // initial state
    case STATE_INIT:
      // If an object detected initially, rotate to get it out of LOS
      // Once rotated out of los, enter search state
      // This is to ensure an accurate theta alignment value.
      if (d <= distanceThresh) {
        // Rotate by 45 degrees. Large rotation to ensure get out of LOS.
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 45);
      } else {
        state = STATE_LEFT;  // Find theta left.
      }
      break;
      // State for getting theta left
    case STATE_LEFT:
      // if detected an edge, record and enter theta right detection state
      if (d <= distanceThresh) {
        thetaLeft = kobuki.angle;
        state = STATE_RIGHT;
      } else {  // Else continue sweep in 5 degree increments.
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 5);
      }
      break;
    case STATE_RIGHT:
      // If object is detected, continue sweeping in 5 degree increments
      if (d <= distanceThresh) {
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 5);
      } else {  // Once object out of LOS, record the angle, and set to
                // alignment state.
        thetaRight = kobuki.angle;
        state = STATE_ALIGNING;
      }
      break;
    case STATE_ALIGNING:
      // Determine the theta average to go to.
      // The angles currently are measured in the rotary encoder ticks.
      // so from [-18000,18000] for [0,360]. Should not affect maths, and
      // conversion adds more work.
      theta = ((int32_t)thetaLeft + (int32_t)thetaRight) / 2;
      // Helper function to set a rotation to a specified absolute angle based.
      // Process based on the rotary encoder ticks
      Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, theta);
      // once set, enter state done.
      state = STATE_DONE;
      break;
    case STATE_DONE:
      // The next time the align function enters here,
      // The alignment rotation should be complete. Return 1.
      return 1;
      break;
    default:
      break;
  }
  return 0;  // Return 0 to indicate in process.
}

int main(void) {
  // Both of these only for testing purposes . Not required
  volatile uint16_t ultrasonic_read;
  volatile uint16_t adc_read;
  US_Typedef object;
  Kobuki_LED_Enum led_1, led_2;
  UART_Init();
  ADC_Init();
  Ultrasonic_Init();
  State current_state = IDLE;
  led_1 = OFF;
  led_2 = OFF;
  while (1) {
    // Update Kobuki Values
    Kobuki_Read(&kobuki);
    Kobuki_Update_LEDs(led_1, led_2);

    // Immediately set to state idle if button 1 or wheel drop.
    if (kobuki.button == BUTTON_1_STOP || kobuki.wheeldrop) {
      current_state = IDLE;
    }

    // FSM
    switch (current_state) {
      case IDLE:  // IDLE case
        // Stop and drive or rotation commands
        Kobuki_Drive(STOP);
        Kobuki_Rotate(STOP);
        // Await search button press.
        if (kobuki.button == BUTTON_0_START) {
          current_state = SEARCH;  // Switch to search mode.
          led_1 = RED;             //
        }
        break;
      case SEARCH:                 // Aligns with blocking bollard
        bool ret = align(500);     // Align FSM subroutine
        if (ret == 1) {            // If alignment finished
          current_state = ORIENT;  // Set to ORIENT state
          // Check current ultrasonic reading
          ultrasonic_read = Ultrasonic_Read() + ULTRASONIC_CALIBRATION;
          // based on distance select layout A or layout B.
          if (325 <= ultrasonic_read && ultrasonic_read <= 375) {
            layout = LAYOUT_A;
          } else if (175 <= ultrasonic_read && ultrasonic_read <= 225) {
            layout = LAYOUT_B;
          }
        } else {  // if alignment incomplete
          // Rotate angle, and repeat.
          Kobuki_Rotate_Angle(kobuki.angle);
        }

        break;
      case ORIENT:
        // Measure Distance to the bollard
        // Based of distance detect layout
        //  if (325 + calibration <distance < 375 + calibration) layout A. else
        //  if (...) layout B
        // based on which layout
        // Rotate 90 CCW or CW
        // Based on layout A or B
        // Orient to correct path. orientation is complete
        // and then enter pathfind
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
      default:
        current_state = IDLE;
        break;
    }
  }
}
