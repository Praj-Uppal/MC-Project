
/*******Headers**********/

#include "ADC/adc.h"
#include "G474RE_Drivers/uart.h"
#include "Kobuki_Drivers/kobuki.h"
#include "Ultrasonic/ultrasonic.h"
#include <stdbool.h>
#include <stdint.h>

/*******Definitions and ENUMS**********/

#define DEG_PER_TICK (1.0 / 100.0) // Conversion factor from ticks to degrees

#define BUTTON_0_START 0x01U  // Button 0 is pressed
#define BUTTON_1_STOP 0x02U   // Button 1 is pressed
#define BUTTON_2_ROTATE 0x04U // Button 2 is pressed

#define DRIVE_SPEED_MM_S ((int16_t)100) // Drive speed in millimeters per second
#define ROTATE_SPEED_MM_S                                                      \
  ((int16_t)100)                  // Rotate speed in millimeters per second
#define ULTRASONIC_CALIBRATION 55 // Calibration value for the ultrasonic sensor
#define FRONT_LENGTH 245          // Length of the front of the robot
#define TOTAL_CALIBRATION                                                      \
  (ULTRASONIC_CALIBRATION - FRONT_LENGTH) // Total calibration value
bool buttonFlag = 0,
     storageFlag = 0; // Flag to indicate if button 2 has been pressed
typedef enum {
  LAYOUT_A,
  LAYOUT_B
} LayoutState; // Enum to represent the layout state (A or B)

volatile LayoutState layout; // Variable to store the current layout state

volatile Kobuki_Typedef kobuki; // Kobuki is global as used in many functions.
typedef enum {
  IDLE,
  SEARCH,
  ORIENT,
  PATHFIND,
  STORAGE,
  DEPOSIT
} State; // Enum to represent the different states of the robot

typedef enum {
  STATE_INIT,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_ALIGNING,
  STATE_DONE
} AAstate; // Enum to represent the alignment states

/*******Functions**********/

volatile float debug_left_deg;
volatile float debug_right_deg;
volatile float debug_theta_deg;
volatile int16_t debug_d;
volatile int32_t debug_span;
volatile AAstate debug_state;
volatile int32_t debug_distance;

// Function to wrap an angle in ticks to the range [-18000, 18000].
int32_t wrap(int32_t angle) {
  while (angle > 18000)
    angle -= 36000;

  while (angle < -18000)
    angle += 36000;

  return angle;
}

void Kobuki_Rotate_AbsAngleTickBased(int32_t current_angle_tick,
                                     int32_t target_angle_tick) {
  // Calculate the relative difference in ticks. This would give us the amount
  // of ticks to rotate by to get to target angle.
  int32_t relative_ticks = wrap(target_angle_tick - current_angle_tick);

  // Convert it to a relative angle reading. This is to use the wrapper for
  // setpoint.
  int16_t relative_angle_deg = (int16_t)((double)relative_ticks * DEG_PER_TICK);
  // Use the setpoint wrapper to set the rotaiton.
  Kobuki_Rotate_Angle_Setpoint(current_angle_tick, relative_angle_deg);
}

// PRAJ -> Alignment function to align with the bollard. Returns 1 if alignment
// is complete, 0 if still in process.
bool align(int16_t distanceThresh) {

  // variable to track alignment state.
  static volatile AAstate state = STATE_INIT;
  // Variables for thetaleft, thetaright, and average theta(theta)
  static volatile int32_t thetaLeft, thetaRight, theta;

  // If in the middle of a rotation, return 0 to process rotation.
  if (!kobuki.rotation_complete) {
    return 0;
  }

  // ultrasonic reading
  volatile int16_t d = Ultrasonic_Read();
  debug_d = d;
  d += TOTAL_CALIBRATION; // Add calibration to the ultrasonic reading
  debug_distance = d;
  debug_state = state;
  debug_left_deg = thetaLeft * DEG_PER_TICK;
  debug_right_deg = thetaRight * DEG_PER_TICK;

  switch (state) {
  // initial state
  case STATE_INIT:
    // If an object detected initially, rotate to get it out of LOS
    // Once rotated out of los, enter search state
    // This is to ensure an accurate theta alignment value.
    if (d <= distanceThresh) {
      // Rotate 45 degrees to get the object out of LOS
      Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 45);
    }
    state = STATE_LEFT;
    break;
    break;
    // State for getting theta left
  case STATE_LEFT:
    // if detected an edge, record and enter theta right detection state
    if (d <= distanceThresh) {
      thetaLeft = kobuki.angle;
      state = STATE_RIGHT;
    } else { // Else continue sweep in 5 degree increments.
      Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 1);
    }
    break;
  case STATE_RIGHT:
    // If object is detected, continue sweeping in 5 degree increments
    if (d <= distanceThresh) {
      Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 1);
    } else { // Once object out of LOS, record the angle, and set to
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
    theta = thetaLeft + wrap(thetaRight - thetaLeft) / 2;

    theta = wrap(theta - 600); // Add 6 degree offset to ensure alignment is not
                               // too close to the bollard.
    debug_theta_deg = theta * DEG_PER_TICK;
    debug_span = wrap(thetaRight - thetaLeft);
    // Helper function to set a rotation to a specified absolute angle based.
    // Process based on the rotary encoder ticks
    Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, theta);
    // once set, enter state done.
    state = STATE_DONE;
    break;
  case STATE_DONE:
    // The next time the align function enters here,
    // The alignment rotation should be complete. Return 1.
    // Return to init state in case need to use again
    state = STATE_INIT;
    return 1;
    break;
  default:
    break;
  }
  return 0; // Return 0 to indicate in process.
}

// Vatsal - Orient Code
uint16_t bollard_distance = 0; // Variable to store the distance to the bollard
uint8_t layout_detected = 0;   // Flag to indicate if a bollard is detected

int Layout_Detect(void) {
  // Read the distance from the ultrasonic sensor
  if (!kobuki.rotation_complete) {
    return 0;
  } // Only read distance if rotation is complete

  bollard_distance = Ultrasonic_Read() +
                     TOTAL_CALIBRATION; // Read distance and apply calibration
  static volatile AAstate state = STATE_INIT;

  switch (state) {
  case STATE_INIT:
    // Check if the distance is within the range for layout A or B
      if (325 <= bollard_distance && bollard_distance <= 375) {
        layout = LAYOUT_A;
        state = STATE_DONE; // Move to done state
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle,
                                     90); // Rotate 90 degrees CW
      } else if (175 <= bollard_distance && bollard_distance <= 225) {
        layout = LAYOUT_B;
        layout_detected = 1; // Set flag to indicate layout B detected
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle,
                                     270); // Rotate 90 degrees CCW
        state = STATE_DONE;                // Move to done state
      }
    return 0;
    break;
  case STATE_DONE:
    // The next time the Layout_Detect function enters here,
    // The layout detection should be complete. Return 1.
    // Return to init state in case need to use again
    state = STATE_INIT;
    return 1;
    break;
  default:
    return 0;
  }
}

bool orient(int16_t angle) {
  static volatile AAstate current_state = STATE_INIT;
  if (!kobuki.rotation_complete || !kobuki.distance_complete) {
    return 0;
  }

  switch (current_state) {
  case STATE_INIT:
    Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 180);
    current_state = STATE_DONE;
    break;
  case STATE_DONE:
    current_state = STATE_INIT;
    return 1;
    break;
  default:
    break;
  }
  return 0;
}

// Function to stop the Kobuki robot by stopping both driving and rotation.
void Kobuki_stop() {
  Kobuki_Drive(STOP);
  Kobuki_Rotate(STOP);
  kobuki.rotation_complete = 1;
  kobuki.distance_complete = 1;
}

typedef enum {
  STAGE_FORWARD1,
  STAGE_ROTATE1,
  STAGE_FORWARD2,
  STAGE_DONE
} pfstate;

bool layoutAFSM(void) {
  static volatile pfstate current_state = STAGE_FORWARD1;
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (current_state) {
  case STAGE_FORWARD1:
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1195);
    current_state = STAGE_ROTATE1;
    return 0;
    break;
  case STAGE_ROTATE1:
    Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 270);
    current_state = STAGE_FORWARD2;
    return 0;
    break;
  case STAGE_FORWARD2:
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1395);
    current_state = STAGE_DONE;
    return 0;
    break;
  case STAGE_DONE:
    current_state = STAGE_FORWARD1;
    return 1;
    break;
  default:
    return 0;
    break;
  }
}
bool layoutBFSM(void) {
  static volatile pfstate current_state = STAGE_FORWARD1;
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (current_state) {
  case STAGE_FORWARD1:
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1395);
    current_state = STAGE_ROTATE1;
    return 0;
    break;
  case STAGE_ROTATE1:
    Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 270);
    current_state = STAGE_FORWARD2;
    return 0;
    break;
  case STAGE_FORWARD2:
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1195);
    current_state = STAGE_DONE;
    return 0;
    break;
  case STAGE_DONE:
    current_state = STAGE_FORWARD1;
    return 1;
    break;
  default:
    return 0;
    break;
  }
}

typedef enum {
  STORAGE_ALIGN,
  STORAGE_ORIENT,
  STORAGE_FORWARD,
  STORAGE_DONE
} storageState;
bool storageFSM() {
  static volatile storageState current_state = STORAGE_ALIGN;
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (current_state) {
  case STORAGE_ALIGN:
    bool volatile ret = align(100); // Align FSM subroutine
    if (ret == 1) {                 // If alignment finished
      current_state = STORAGE_ORIENT;
    }
    break;
  case STORAGE_ORIENT:
    Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 180);
    current_state = STORAGE_FORWARD;
    break;
  case STORAGE_FORWARD:
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 918);
    current_state = STORAGE_DONE;
    break;
  case STORAGE_DONE:
    current_state = STORAGE_ALIGN;
    return 1;
    break;
  default:
    break;
  }
  return 0;
}

typedef enum { DEPOSIT_ALIGN, DEPOSIT_FORWARD, DEPOSIT_DONE } depositState;

bool depositFSM() {

  static volatile depositState current_state = DEPOSIT_ALIGN;
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (current_state) {
  case DEPOSIT_ALIGN:
    bool volatile ret = align(100); // Align FSM subroutine
    if (ret == 1) {                 // If alignment finished
      current_state = DEPOSIT_FORWARD;
    }
    break;
  case DEPOSIT_FORWARD: {
    int16_t travelD =
        Ultrasonic_Read() + TOTAL_CALIBRATION - 100; // 10cm safegaurd
    if (travelD > 0) {
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, travelD);
    }
    current_state = DEPOSIT_DONE;
    break;
  }
  case DEPOSIT_DONE:
    current_state = DEPOSIT_ALIGN;
    return 1;
    break;
  default:
    break;
  }
  return 0;
}

// Main function to initialize the robot and run the main loop.
int main(void) {

  // Both of these only for testing purposes . Not required
  volatile int16_t ultrasonic_read;
  volatile int16_t adc_read;
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
    // ultrasonic_read = Ultrasonic_Read() + ULTRASONIC_CALIBRATION;

    // Immediately set to state idle if button 1 or wheel drop. Add wheel drop
    if (kobuki.button & BUTTON_1_STOP || kobuki.wheeldrop != 0) {
      current_state = IDLE;
    }

    if (kobuki.button & BUTTON_2_ROTATE) {
      current_state = ORIENT;
    }

    // FSM
    switch (current_state) {

    case IDLE: // IDLE case
      // Stop and drive or rotation commands
      {
        Kobuki_stop();
        // Await search button press.
        if (kobuki.button & BUTTON_0_START) {
          current_state = SEARCH; // Switch to search mode.
          led_1 = RED;            // Turn on red LED to indicate search mode.
        }
        break;
      }
    case SEARCH: {                    // Aligns with blocking bollard
      bool volatile ret = align(300); // Align FSM subroutine
      if (ret == 1) {                 // If alignment finished
        current_state = IDLE; // Set to IDLE state for testing purposes. In
                              // final code, should be set to ORIENT state.
        led_1 = OFF; // Turn off red LED to indicate search mode is complete.
      } else {       // if alignment incomplete
        // Rotate angle, and repeat.
        Kobuki_Rotate_Angle(kobuki.angle);
      }

      break;
    }
    case ORIENT: {
      // Measure Distance to the bollard
      // Based of distance detect layout
      //  if (325 + calibration <distance < 375 + calibration) layout A. else
      //  if (...) layout B
      // based on which layout
      // Rotate 90 CCW or CW
      // Based on layout A or B
      // Orient to correct path. orientation is complete
      // and then enter pathfind

      // Vatsal -> Layout Detecttion Function. This function will read the
      // distance from the ultrasonic sensor and determine the layout based on
      // the distance. It will set the layout variable to LAYOUT_A or LAYOUT_B
      // and set the layout_detected flag to 1 if a valid layout is detected.

      int volatile ret2 = Layout_Detect();
      if (ret2 == 0) {
        Kobuki_Rotate_Angle(kobuki.angle); // Rotate to get a better reading
      } else if (ret2 == 1) {
        current_state =
            IDLE; // Move to ____ state if layout detection is complete
      }
    } break;

    case PATHFIND: {
      // Hardcode path for the current layout
      // Drive Xm.
      // We need to indicate this is finished.
      // If (distance_complete) -> step 2 update a counter
      // angle rotate -> if (angle_complete)- > step 3
      // Drive again
      // Reached section 3
      bool pfret = 0;
      if (layout == LAYOUT_A) {
        pfret = layoutAFSM();
      } else if (layout == LAYOUT_B) {
        pfret = layoutBFSM();
      }

      if (pfret == 1) {
        current_state = DEPOSIT;
      } else {
        if (!kobuki.distance_complete) {
          Kobuki_Drive_Distance(kobuki.distance);
        }
        if (!kobuki.rotation_complete) {
          Kobuki_Rotate_Angle(kobuki.angle);
        }
      }
    } break;

    case STORAGE: {
      volatile bool sRet = storageFSM();
      if (sRet) {
        current_state = DEPOSIT;
      } else {
        if (!kobuki.distance_complete) {
          Kobuki_Drive_Distance(kobuki.distance);
        } else if (!kobuki.rotation_complete) {
          Kobuki_Rotate_Angle(kobuki.angle);
        }
      }
    } break;
    case DEPOSIT: {
      volatile bool depositRet = depositFSM();
      if (depositRet) {
        current_state = IDLE; // Finished
      } else {
        if (!kobuki.distance_complete) {
          Kobuki_Drive_Distance(kobuki.distance);
        } else if (!kobuki.rotation_complete) {
          Kobuki_Rotate_Angle(kobuki.angle);
        }
      }
    } break;
    default: {
      current_state = IDLE;
      break;
    }
    }
  }
}
