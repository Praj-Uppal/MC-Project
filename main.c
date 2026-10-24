/*******Headers**********/

#include <stdbool.h>
#include <stdint.h>

#include "ADC/adc.h"
#include "G474RE_Drivers/uart.h"
#include "Kobuki_Drivers/kobuki.h"
#include "Ultrasonic/ultrasonic.h"

/*******Definitions and ENUMS**********/

#define DEG_PER_TICK (1.0 / 100.0)  // Conversion factor from ticks to degrees

#define BUTTON_0_START 0x01U   // Button 0 is pressed
#define BUTTON_1_STOP 0x02U    // Button 1 is pressed
#define BUTTON_2_ROTATE 0x04U  // Button 2 is pressed

#define DRIVE_SPEED_MM_S \
  ((int16_t)100)  // Drive speed in millimeters per second
#define ROTATE_SPEED_MM_S \
  ((int16_t)100)  // Rotate speed in millimeters per second
#define ULTRASONIC_CALIBRATION \
  55                      // Calibration value for the ultrasonic sensor
#define FRONT_LENGTH 245  // Length of the front of the robot
#define TOTAL_CALIBRATION \
  (ULTRASONIC_CALIBRATION - FRONT_LENGTH)  // Total calibration value

#define ALIGN_SWEEP_SPEED \
  ((int16_t)60)  // mm/s wheel speed during the alignment sweep (+ = CCW)

//*******Enums**********/
typedef enum {
  LAYOUT_A,
  LAYOUT_B
} LayoutState;  // Enum to represent the layout state (A or B)

typedef enum {
  IDLE,
  SEARCH,
  ORIENT,
  PATHFIND,
  STORAGE,
  DEPOSIT
} State;  // Enum to represent the different states of the robot

typedef enum {
  ALIGN_INIT,
  ALIGN_LEFT,
  ALIGN_RIGHT,
  ALIGN_ALIGNING,
  ALIGN_DONE
} ALIGNstate;  // Enum to represent the alignment states

typedef enum {
  ORIENT_INIT,
  ORIENT_DONE
} ORIENTstate;  // Enum to represent the ORIENT states

typedef enum {
  STAGE_ROTATE0,
  STAGE_FORWARD1,
  STAGE_ROTATE1,
  STAGE_FORWARD2,
  STAGE_DONE
} PFstate;  // Enum to represent the pathfinding states

typedef enum {
  STORAGE_ORIENT,
  STORAGE_FORWARD,
  STORAGE_DONE
} STORAGEstate;  // Enum to represent the storage states

typedef enum {
  DEPOSIT_ALIGN,
  DEPOSIT_FORWARD,
  DEPOSIT_DONE
} DEPOSITstate;  // Enum to represent the deposit states

///*******Global Variables**********/

// for debugging purposes
volatile float debug_left_deg;
volatile float debug_right_deg;
volatile float debug_theta_deg;
volatile int16_t debug_d;
volatile int32_t debug_span;
volatile ALIGNstate debug_state;
volatile int32_t debug_distance;

volatile LayoutState layout;  // Variable to store the current layout state

volatile Kobuki_Typedef kobuki;  // Kobuki is global as used in many functions.

// FSM state variables (file scope so reset_all_fsms() can reach them)
static volatile ALIGNstate align_state = ALIGN_INIT;
static volatile ORIENTstate orient_state =
    ORIENT_INIT;  // used by Layout_Detect
static volatile PFstate pathfind_state =
    STAGE_ROTATE0;  // shared by layout A and B
static volatile STORAGEstate storage_state = STORAGE_ORIENT;
static volatile DEPOSITstate deposit_state = DEPOSIT_ALIGN;

// Results from the last align sweep. Distance to closest object in aligmin_d
// Absolute angle/heading to detected object in align_theta.
static int16_t align_min_d;
static int32_t align_theta;

// Absolute angle/Heading of the blocking bollard at the begining.
// Use this as a reference for every subseuquent calculation/decision making.
// This will be set after search.
static int32_t headingRef;

//*******Functions**********/

// Function to wrap an angle in ticks to the range [-18000, 18000].
int32_t wrap(int32_t angle) {
  while (angle > 18000) angle -= 36000;

  while (angle < -18000) angle += 36000;

  return angle;
}

// Function to rotate the robot to an absolute angle in ticks.
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
  // Variables for thetaleft, thetaright, and average theta(theta)
  static volatile int32_t thetaLeft, thetaRight, theta;

  // If in the middle of a rotation, return 0 to process rotation.
  if (!kobuki.rotation_complete) {
    return 0;
  }

  // ultrasonic reading
  volatile int16_t d = Ultrasonic_Read();
  // debug variables
  debug_d = d;
  d += TOTAL_CALIBRATION;  // Add calibration to the ultrasonic reading
  debug_distance = d;
  debug_state = align_state;
  debug_left_deg = thetaLeft * DEG_PER_TICK;
  debug_right_deg = thetaRight * DEG_PER_TICK;

  switch (align_state) {
    // initial state
    case ALIGN_INIT:
      // If an object detected initially, rotate to get it out of LOS
      // Once rotated out of los, enter search state
      // This is to ensure an accurate theta alignment value.
      if (d <= distanceThresh) {
        // Rotate 45 degrees to get the object out of LOS
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 45);
      }
      align_state = ALIGN_LEFT;
      break;
      // vatsal -> the ALIGN_LEFT and ALIGN_RIGHT was chnaged by me, apparently
      // this will perfrom better than the previous implementation. The previous
      // implementation was commented out below.
    case ALIGN_LEFT:
      if (d <= distanceThresh) {
        thetaLeft = kobuki.angle;
        align_min_d = d;  // Take current distance as minimum distance for now
        align_state = ALIGN_RIGHT;
      } else
        Kobuki_Rotate(SEARCH_SPEED);  // keep turning
      break;
    case ALIGN_RIGHT:
      if (d <= distanceThresh) {
        if (d < align_min_d)
          align_min_d = d;  // Update minimum alignment distance, if shorter
                            // distance detected
        Kobuki_Rotate(SEARCH_SPEED);
      } else {
        thetaRight = kobuki.angle;
        Kobuki_Rotate(STOP);
        align_state = ALIGN_ALIGNING;
      }
      break;

      // State for getting theta left
      // case ALIGN_LEFT:
      //   // if detected an edge, record and enter theta right detection
      //   state if (d <= distanceThresh) {
      //     thetaLeft = kobuki.angle;
      //     align_state = ALIGN_RIGHT;
      //   } else { // Else continue sweep in 5 degree increments.
      //     Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 1);
      //   }
      //   break;
      // case ALIGN_RIGHT:
      //   // If object is detected, continue sweeping in 5 degree
      //   increments if (d <= distanceThresh) {
      //     Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 1);
      //   } else { // Once object out of LOS, record the angle, and set to
      //     // alignment state.
      //     thetaRight = kobuki.angle;
      //     align_state = ALIGN_ALIGNING;
      //   }
    case ALIGN_ALIGNING:
      // Determine the theta average to go to.
      // The angles currently are measured in the rotary encoder ticks.
      // so from [-18000,18000] for [0,360]. Should not affect maths, and
      // conversion adds more work.
      theta = thetaLeft + wrap(thetaRight - thetaLeft) / 2;

      theta = wrap(theta - 600);  // Add 6 degree offset to ensure alignment
                                  // is not too close to the bollard.

      align_theta = theta;  // Set the
      debug_theta_deg = theta * DEG_PER_TICK;
      debug_span = wrap(thetaRight - thetaLeft);
      // Helper function to set a rotation to a specified absolute angle
      // based. Process based on the rotary encoder ticks
      Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, theta);
      // once set, enter state done.
      align_state = ALIGN_DONE;
      break;
    case ALIGN_DONE:
      // The next time the align function enters here,
      // The alignment rotation should be complete. Return 1.
      // Return to init state in case need to use again
      align_state = ALIGN_INIT;
      return 1;
      break;
    default:
      break;
  }
  return 0;  // Return 0 to indicate in process.
}

// Vatsal - Orient Code

uint16_t bollard_distance = 0;  // Variable to store the distance to the bollard
uint8_t layout_detected = 0;    // Flag to indicate if a bollard is detected

int Layout_Detect(void) {
  // Read the distance from the ultrasonic sensor
  if (!kobuki.rotation_complete) {
    return 0;
  }  // Only read distance if rotation is complete

  bollard_distance = Ultrasonic_Read() +
                     TOTAL_CALIBRATION;  // Read distance and apply calibration

  switch (orient_state) {
    case ORIENT_INIT:
      // Check if the distance is within the range for layout A or B
      if (325 <= bollard_distance && bollard_distance <= 375) {
        layout = LAYOUT_A;
        orient_state = ORIENT_DONE;  // Move to done state
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle,
                                     90);  // Rotate 90 degrees CCW
      } else if (175 <= bollard_distance && bollard_distance <= 225) {
        layout = LAYOUT_B;
        layout_detected = 1;  // Set flag to indicate layout B detected
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle,
                                     270);  // Rotate 90 degrees CW
        orient_state = ORIENT_DONE;         // Move to done state
      } else {
        // If the distance is not within the range for either layout,
        // rotate 3 degrees to get a better reading
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle,
                                     3);  // Rotate 3 degrees CCW
      }
      return 0;
      break;
    case ORIENT_DONE:
      // The next time the Layout_Detect function enters here,
      // The layout detection should be complete. Return 1.
      // Return to init state in case need to use again
      orient_state = ORIENT_INIT;
      return 1;
      break;
    default:
      return 0;
  }
}

// PRAJ - Pathfinding FSM for Layout A
bool layoutAFSM(void) {
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (pathfind_state) {
    case STAGE_ROTATE0:
      // Facing A1 (up the page). Turn right, towards section 4.
      // heading - 9000 is a clockwise rotation of 90 degrees, making heading
      // towards right.
      Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, wrap(headingRef - 9000));
      pathfind_state = STAGE_FORWARD1;
      return 0;
      break;
    case STAGE_FORWARD1:
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1195);
      pathfind_state = STAGE_ROTATE1;
      return 0;
      break;
    case STAGE_ROTATE1:
      // Turn back up the page, towards section 3.
      // make heading same as headingRef
      Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, headingRef);
      pathfind_state = STAGE_FORWARD2;
      return 0;
      break;
    case STAGE_FORWARD2:
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1395);
      pathfind_state = STAGE_DONE;
      return 0;
      break;
    case STAGE_DONE:
      // Reset to initial state
      pathfind_state = STAGE_ROTATE0;
      return 1;
      break;
    default:
      return 0;
      break;
  }
}

// PRAJ - Layout B FSM
bool layoutBFSM(void) {
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (pathfind_state) {
    case STAGE_ROTATE0:
      // Facing B1 (to the right). Turn up the page, towards section 2.
      // Add 9000 to current heading to perform a 90 degrees ACW rotation.
      Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, wrap(headingRef + 9000));
      pathfind_state = STAGE_FORWARD1;
      return 0;
      break;
    case STAGE_FORWARD1:
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1395);
      pathfind_state = STAGE_ROTATE1;
      return 0;
      break;
    case STAGE_ROTATE1:
      // Turn back to the right, towards section 3.
      // Make heading back to heading reference.
      Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, headingRef);
      pathfind_state = STAGE_FORWARD2;
      return 0;
      break;
    case STAGE_FORWARD2:
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1195);
      pathfind_state = STAGE_DONE;
      return 0;
      break;
    case STAGE_DONE:
      // Reset to initial state.
      pathfind_state = STAGE_ROTATE0;
      return 1;
      break;
    default:
      return 0;
      break;
  }
}

// PRAJ - Storage FSM

bool storageFSM(void) {
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (storage_state) {
    case STORAGE_ORIENT:
      // Both paths end at the same spot in section 3, on the line through
      // the middle of the storage entrance. Face down-left along it ()
      // (Layout A headingRef is up the page, layout B, the reference is to the
      // right.). For layout A rotate 135 degrees ACW, for layout B rotate 135
      // degrees CW.
      if (layout == LAYOUT_A) {
        Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, wrap(headingRef + 13500));
      } else {
        Kobuki_Rotate_AbsAngleTickBased(kobuki.angle, wrap(headingRef - 13500));
      }
      storage_state = STORAGE_FORWARD;
      break;
    case STORAGE_FORWARD:
      // Drive through the entrance to the point between D1 and D2.
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, 945);
      storage_state = STORAGE_DONE;
      break;
    case STORAGE_DONE:
      // Return to orient state for reset
      storage_state = STORAGE_ORIENT;
      return 1;
      break;
    default:
      break;
  }
  return 0;
}

// PRAJ - Deposit FSM

bool depositFSM(void) {
  if (!kobuki.distance_complete || !kobuki.rotation_complete) {
    return 0;
  }

  switch (deposit_state) {
    case DEPOSIT_ALIGN:
      // The marker is approximately 330 mm away from the storage stop
      // point
      // the A2/B2 bollards outside the unit are above 500 mm so they should be
      // ignored.
      bool volatile ret = align(260);  // Align FSM subroutine
      if (ret == 1) {                  // If alignment finished
        deposit_state = DEPOSIT_FORWARD;
      }
      break;
    case DEPOSIT_FORWARD: {
      // Use the closest reading from the sweep.
      // Negative = reverse from current point.
      int16_t travelD = align_min_d - 100;  // 10cm safegaurd
      if (travelD != 0) {
        Kobuki_Drive_Distance_Setpoint(kobuki.distance, travelD);
      }
      deposit_state = DEPOSIT_DONE;
      break;
    }
    case DEPOSIT_DONE:
      deposit_state = DEPOSIT_ALIGN;
      return 1;
      break;
    default:
      break;
  }
  return 0;
}

void reset_all_fsms(void) {
  // Reset all FSM states to their initial values
  align_state = ALIGN_INIT;
  orient_state = ORIENT_INIT;
  pathfind_state = STAGE_ROTATE0;
  storage_state = STORAGE_ORIENT;
  deposit_state = DEPOSIT_ALIGN;
}

//********MAIN*******************/
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
      reset_all_fsms();  // Reset all FSMs to their initial states
      current_state = IDLE;
    }

    if (kobuki.button & BUTTON_2_ROTATE) {
      reset_all_fsms();  // Reset all FSMs to their initial states
      current_state = ORIENT;
    }

    // FSM
    switch (current_state) {
      case IDLE:  // IDLE case
        // Stop and drive or rotation commands
        {
          Kobuki_Stop();
          // Await search button press.
          reset_all_fsms();  // Reset all FSMs to their initial states
          if (kobuki.button & BUTTON_0_START) {
            current_state = SEARCH;  // Switch to search mode.
            led_1 = RED;             // Turn on red LED to indicate search mode.
          }
          break;
        }
      case SEARCH: {  // Aligns with blocking bollard
        // A1 is approximately 360 mm (calibrated) from the start, B1
        // approximately 210 mm.
        bool volatile ret = align(400);  // Align FSM subroutine
        if (ret == 1) {                  // If alignment finished
          headingRef = align_theta;
          current_state = ORIENT;
        } else {  // if alignment incomplete
          // Rotate angle, and repeat.
          if (!kobuki.rotation_complete) {
            Kobuki_Rotate_Angle(kobuki.angle);
          }
        }

        break;
      }
      case ORIENT: {
        // Vatsal -> Layout Detecttion Function. This function will read the
        // distance from the ultrasonic sensor and determine the layout based
        // on the distance. It will set the layout variable to LAYOUT_A or
        // LAYOUT_B and set the layout_detected flag to 1 if a valid layout is
        // detected.

        // Use  the closest distance reading of the sweep
        // 285 used as it is halfway between the A1 bollard at 360 mm
        // and the B1 bollard at 210 mm. Below or above decides layout.
        if (align_min_d > 285) {
          layout = LAYOUT_A;
        } else {
          layout = LAYOUT_B;
        }
        current_state = PATHFIND;
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
          current_state = STORAGE;
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
          current_state = IDLE;  // Finished
          led_1 = OFF;
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
