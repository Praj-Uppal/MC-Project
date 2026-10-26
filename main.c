/*******Headers**********/

#include <stdbool.h>
#include <stdint.h>

#include "ADC/adc.h"
#include "G474RE_Drivers/uart.h"
#include "Kobuki_Drivers/kobuki.h"
#include "Ultrasonic/ultrasonic.h"
/*******Definitions**********/

#define DEG_PER_TICK (1.0 / 100.0)   // Gyro ticks are 0.01 degrees
#define DEG(x) ((int32_t)(x) * 100)  // convert degrees to gyro ticks

#define BUTTON_0_START 0x01U  // Button 0 is pressed
#define BUTTON_1_STOP 0x02U   // Button 1 is pressed

// Calibration value for the ultrasonic sensor
#define ULTRASONIC_CALIBRATION 55
#define FRONT_LENGTH 245  // Length of the front of the robot
#define TOTAL_CALIBRATION   \
  (ULTRASONIC_CALIBRATION - \
   FRONT_LENGTH)  // Total calibration value including front

// align() detection thresholds (calibrated mm)
#define SEARCH_THRESH \
  500  // bollard A1 is 360 mm from start, B1  is 210 mm. Pick a detection range
       // that picks up both while not picking up deposit or other bollards
#define LAYOUT_SPLIT 285  // halfway between A1 and B1
#define DEPOSIT_THRESH \
  260  // markers in the storage are close around 140 mm; A2/B2 outside the unit
       // at around 370 mm. Choose 260 mm.
#define ALIGN_OFFSET DEG(6)  // sensor latency makes the sweep centre late
#define DEPOSIT_GAP 100      // stop this far from the deposit marker

// The ultrasonic sensor minimum range of 300mm. Anything close gets clamped.
// This is important in deposit state, where if we are too close (15mm) to
// bollard it reads as 300. Need to back up first, then remeasure.
#define US_MIN_RANGE (300 + TOTAL_CALIBRATION)  // calibrated mm
#define US_CLOSE_MARGIN 15                      // mm
#define DEPOSIT_BACKUP 100                      // mm

// Cliff and bumper bits, read as 0bLCR (same layout for kobuki.cliff and
// kobuki.bumper).
#define SENSOR_RIGHT 0x01U
#define SENSOR_CENTRE 0x02U
#define SENSOR_LEFT 0x04U

// Kobuki drives drives towards a platform edge, until the cliff sensor is
// triggered. Then reverse a bit, sepcified by edge backoff. From diagram, cliff
// sensor 165 mm ahead of centre, so we stop around 205 mm from the edge. This
// lines up decently with corner.

#define EDGE_SPEED ((int16_t)100)  // speed while looking for the edge
#define EDGE_BACKOFF 40            // backoff distance

// Course correction: one of the side cliff sensors right or left triggering on
// its own means the robot has drifted towards the edge off the course. Back off
// a little and turn this far away from that edge, then continue. A side bumper
// on its own is handled the same way: the legs pass close to the storage
// unit's walls, so drifting towards one scrapes it with that side's bumper.
#define SIDE_BACKOFF 30  // mm
#define CORRECTION_DEG 5
// Also the same for both layouts.
#define STORAGE_LEG 945  // to the point equidistant from D1 and D2

//*******Enums**********/
typedef enum {
  LAYOUT_A,
  LAYOUT_B
} LayoutState;  // Enum to represent the layout state (A or B)

typedef enum {
  IDLE,
  SEARCH,
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
volatile int16_t debug_d;
volatile ALIGNstate debug_state;
volatile float debug_theta_deg;
volatile int32_t debug_span;

volatile LayoutState layout;  // Variable to store the current layout state

volatile Kobuki_Typedef kobuki;  // Kobuki is global as used in many functions.

// FSM state variables (file scope so reset_all_fsms() can reach them)
static ALIGNstate align_state = ALIGN_INIT;
static PFstate pathfind_state = STAGE_ROTATE0;
static STORAGEstate storage_state = STORAGE_ORIENT;
static DEPOSITstate deposit_state = DEPOSIT_ALIGN;

// Results from the last align sweep. Distance to closest object in
// align_min_d, absolute heading (ticks) to the object in align_theta.
static int16_t align_min_d;
static int32_t align_theta;

// Absolute heading (ticks) of the blocking bollard, set after SEARCH.
// Every later turn is relative to this, so turn errors don't add up.
static int32_t headingRef;

// Pending course-correction turn (degrees) from Drive_To_Edge().
static int16_t edge_correction_deg;

//*******Functions**********/

// Function to wrap an angle in ticks to the range [-18000, 18000].
int32_t wrap(int32_t angle) {
  while (angle > 18000) angle -= 36000;
  while (angle < -18000) angle += 36000;
  return angle;
}

// Start a rotation to an absolute heading in ticks. All turns go through
// here; Motion_Update() then drives it to completion.
void Rotate_To(int32_t target_angle_tick) {
  int32_t relative_ticks = wrap(target_angle_tick - kobuki.angle);
  Kobuki_Rotate_Angle_Setpoint(kobuki.angle,
                               (int16_t)(relative_ticks * DEG_PER_TICK));
}

// Start a rotation relative to headingRef (degrees, + = CCW).
void Rotate_To_Ref(int16_t offset_deg) {
  Rotate_To(wrap(headingRef + DEG(offset_deg)));
}

// Keep the current drive or rotation setpoint moving. Returns 1 while busy.
bool Motion_Update(void) {
  if (!kobuki.distance_complete) {
    Kobuki_Drive_Distance(kobuki.distance);
    return 1;
  }
  if (!kobuki.rotation_complete) {
    Kobuki_Rotate_Angle(kobuki.angle);
    return 1;
  }
  return 0;
}

// Sweep CCW until an object within distanceThresh (calibrated mm) enters and
// leaves the beam, then turn to its centre. Records align_min_d and
// align_theta. Returns 1 once aligned, 0 while in progress.
bool align(int16_t distanceThresh) {
  static int32_t thetaLeft, thetaRight;

  int16_t d = Ultrasonic_Read() + TOTAL_CALIBRATION;
  debug_d = d;
  debug_state = align_state;

  switch (align_state) {
    case ALIGN_INIT:
      // If an object is already in view, turn it out of the beam first so
      // the sweep catches its leading edge.
      if (d <= distanceThresh) {
        Rotate_To(wrap(kobuki.angle + DEG(45)));
      }
      align_state = ALIGN_LEFT;
      break;
    case ALIGN_LEFT:
      // Keep turning until the object enters the beam.
      if (d <= distanceThresh) {
        thetaLeft = kobuki.angle;
        align_min_d = d;
        align_state = ALIGN_RIGHT;
      } else {
        Kobuki_Rotate(SEARCH_SPEED);
      }
      break;
    case ALIGN_RIGHT:
      // Keep turning until it leaves the beam, tracking the closest reading.
      if (d <= distanceThresh) {
        if (d < align_min_d) {
          align_min_d = d;
        }
        Kobuki_Rotate(SEARCH_SPEED);
      } else {
        thetaRight = kobuki.angle;
        Kobuki_Rotate(STOP);
        align_state = ALIGN_ALIGNING;
      }
      break;
    case ALIGN_ALIGNING:
      // Turn to the middle of the two edges.
      align_theta =
          wrap(thetaLeft + wrap(thetaRight - thetaLeft) / 2 - ALIGN_OFFSET);
      debug_theta_deg = align_theta * DEG_PER_TICK;
      debug_span = wrap(thetaRight - thetaLeft);
      Rotate_To(align_theta);
      align_state = ALIGN_DONE;
      break;
    case ALIGN_DONE:
      align_state = ALIGN_INIT;
      return 1;
    default:
      break;
  }
  return 0;
}

// Drive forward until the centre cliff sensor sees the platform edge ahead,
// then start reversing EDGE_BACKOFF. If only a side cliff sensor or a side
// bumper fires, steer away from that side and keep going. Call every loop;
// returns 1 once the edge ahead is found.
bool Drive_To_Edge(void) {
  // Turn left over from a side correction, once the back-off has finished.
  if (edge_correction_deg != 0) {
    Rotate_To(wrap(kobuki.angle + DEG(edge_correction_deg)));
    edge_correction_deg = 0;
    return 0;
  }

  if (kobuki.cliff == 0 && kobuki.bumper == 0) {
    Kobuki_Drive(EDGE_SPEED);
    return 0;
  }

  Kobuki_Drive(STOP);
  if (kobuki.cliff == SENSOR_LEFT || kobuki.bumper == SENSOR_LEFT) {
    // Edge or wall found on the left.
    // back off, then turn right to correct (CW).
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, -SIDE_BACKOFF);
    edge_correction_deg = -CORRECTION_DEG;
    return 0;
  }
  if (kobuki.cliff == SENSOR_RIGHT || kobuki.bumper == SENSOR_RIGHT) {
    // Edge or wall on the right.
    // back off, then turn left  to correct (CCW).
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, -SIDE_BACKOFF);
    edge_correction_deg = CORRECTION_DEG;
    return 0;
  }

  // Centre cliff sensor (or more than one), Edge ahead of us meaning we have
  // finished driving to edge.
  // A centre bump means something is directly ahead, so stop here as well
  // rather than push into it. This shouldnt happen though.
  Kobuki_Drive_Distance_Setpoint(kobuki.distance, -EDGE_BACKOFF);
  return 1;
}

// PRAJ - Pathfinding FSM, from the start to section 3.
// Layout A: A1 is up the page, so go right (section 4) then up.
// Layout B: B1 is to the right, so go up (section 2) then right.
// Each leg runs to the platform edge, so it doesn't rely on the encoders.
bool pathfindFSM(void) {
  int16_t turn;
  if (layout == LAYOUT_A) {
    turn = -90;
  } else {
    turn = 90;
  }

  switch (pathfind_state) {
    case STAGE_ROTATE0:
      Rotate_To_Ref(turn);
      pathfind_state = STAGE_FORWARD1;
      break;
    case STAGE_FORWARD1:
      if (Drive_To_Edge()) {
        pathfind_state = STAGE_ROTATE1;
      }
      break;
    case STAGE_ROTATE1:
      Rotate_To_Ref(0);  // back to the bollard's heading
      pathfind_state = STAGE_FORWARD2;
      break;
    case STAGE_FORWARD2:
      if (Drive_To_Edge()) {
        pathfind_state = STAGE_DONE;
      }
      break;
    case STAGE_DONE:
      pathfind_state = STAGE_ROTATE0;
      return 1;
    default:
      break;
  }
  return 0;
}

// PRAJ - Storage FSM, from section 3 into the storage unit.
bool storageFSM(void) {
  switch (storage_state) {
    case STORAGE_ORIENT:
      // Face down-left along the line through the entrance.
      if (layout == LAYOUT_A) {
        Rotate_To_Ref(135);
      } else {
        Rotate_To_Ref(-135);
      }
      storage_state = STORAGE_FORWARD;
      break;
    case STORAGE_FORWARD:
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, STORAGE_LEG);
      storage_state = STORAGE_DONE;
      break;
    case STORAGE_DONE:
      storage_state = STORAGE_ORIENT;
      return 1;
    default:
      break;
  }
  return 0;
}

// PRAJ - Deposit FSM, find the marker and stop DEPOSIT_GAP in front of it.
bool depositFSM(void) {
  switch (deposit_state) {
    case DEPOSIT_ALIGN:
      if (align(DEPOSIT_THRESH)) {
        if (align_min_d <= US_MIN_RANGE + US_CLOSE_MARGIN) {
          // Too close to measure: reverse away from the marker (we're facing
          // it) and sweep again from further back.
          Kobuki_Drive_Distance_Setpoint(kobuki.distance, -DEPOSIT_BACKUP);
        } else {
          deposit_state = DEPOSIT_FORWARD;
        }
      }
      break;
    case DEPOSIT_FORWARD:
      // Use the closest reading from the sweep: a reading taken right after
      // the turn can still be stale (sensor latency). Negative = reverse.
      Kobuki_Drive_Distance_Setpoint(kobuki.distance,
                                     align_min_d - DEPOSIT_GAP);
      deposit_state = DEPOSIT_DONE;
      break;
    case DEPOSIT_DONE:
      deposit_state = DEPOSIT_ALIGN;
      return 1;
    default:
      break;
  }
  return 0;
}

void reset_all_fsms(void) {
  // Reset all FSM states to their initial values
  align_state = ALIGN_INIT;
  pathfind_state = STAGE_ROTATE0;
  storage_state = STORAGE_ORIENT;
  deposit_state = DEPOSIT_ALIGN;
  edge_correction_deg = 0;
}

// Main function to initialize the robot and run the main loop.
int main(void) {
  Kobuki_LED_Enum led_1 = OFF, led_2 = OFF;  // Start LEDS as as off
  State current_state = IDLE;                // start at IDLE

  // Initialise UART, ADC, and ultrasonic sensor.
  UART_Init();
  ADC_Init();
  Ultrasonic_Init();

  while (1) {
    // Update Kobuki Values
    Kobuki_Read(&kobuki);
    Kobuki_Update_LEDs(led_1, led_2);

    // Immediately stop on button 1 or wheel drop.
    if (kobuki.button & BUTTON_1_STOP || kobuki.wheeldrop != 0) {
      current_state = IDLE;
    }

    // Let any drive or rotation in progress finish before the next step.
    // Skipped in IDLE so a stop takes effect immediately.
    if (current_state != IDLE && Motion_Update()) {
      continue;
    }

    switch (current_state) {
      case IDLE:
        Kobuki_Stop();
        reset_all_fsms();
        led_1 = OFF;
        if (kobuki.button & BUTTON_0_START) {
          current_state = SEARCH;
          led_1 = RED;  // red while running
        }
        break;
      case SEARCH:  // Align with the blocking bollard, its distance gives
                    // the layout.
        if (align(SEARCH_THRESH)) {
          headingRef = align_theta;
          if (align_min_d > LAYOUT_SPLIT) {
            layout = LAYOUT_A;
          } else {
            layout = LAYOUT_B;
          }
          current_state = PATHFIND;
        }
        break;
      case PATHFIND:
        if (pathfindFSM()) {
          current_state = STORAGE;
        }
        break;
      case STORAGE:
        if (storageFSM()) {
          current_state = DEPOSIT;
        }
        break;
      case DEPOSIT:
        if (depositFSM()) {
          current_state = IDLE;  // Finished
        }
        break;
      default:
        current_state = IDLE;
        break;
    }
  }
}
