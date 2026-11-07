/*******Headers**********/

#include <stdbool.h>
#include <stdint.h>

#include "ADC/adc.h"
#include "G474RE_Drivers/uart.h"
#include "Kobuki_Drivers/kobuki.h"
#include "Ultrasonic/ultrasonic.h"
// Definitions of variables

#define DEG_PER_TICK \
  (1.0 / 100.0)  // 0.01 degrees per gyroscope tick. (degrees goes from -180 to
                 // +180, and gyroscope produces -18000 to 18000)
#define DEG_TO_TICK(x) \
  ((int32_t)(x) * 100)  // Define macro to convert degrees to gyro ticks

// Button presses recorded in a three bits 0bxxx, each bit for one button.
#define BUT0 0x01U  // Button 0 is pressed (bit 0)
#define BUT1 0x02U  // Button 1 is pressed (bit 1)
#define BUT2 0x04U  // Button 2 is pressed (bit 2)

// Calibration value for the ultrasonic sensor. Calibrated at start of practical
// through testing.
#define ULTRASONIC_CALIBRATION 35
#define FRONT_LENGTH \
  245  // Length of the front of the robot. Obtained from the diagram online and
       // physical measurements.
// Total calibration for the sensor to get distance from front of robot.
// Define as the ultrasonic calbiration offset - the front length.
// For our final run we found, a total calbiration offset of -210 to work the
// best. This varies on different sensors/robots.
#define TOTAL_CALIBRATION   \
  (ULTRASONIC_CALIBRATION - \
   FRONT_LENGTH)  // Total calibration value including front

// align() detection thresholds (calibrated in mm)
#define SEARCH_THRESH \
  500  // bollard A1 is around 375 mm from start, B1  is 225 mm. We chose a
       // detection range that picks up both while not picking up deposit or
       // other bollards, as they are over 500 away. Going lower than this
       // results in some cases where the starting bollards not detected (random
       // variance in begining position).

#define LAYOUT_MIDPOINT \
  285  // Try to set between halfway of layout A detected bollard (A1) and
       // layout B first detected bollard (B1). From our testing, 285 worked.
       // If distance above, we are in layout A else if below, layout B.

#define DEPOSIT_THRESH 260  // markers in the storage are around 140 mm.
//  A2/B2 outside the unit at around 400 ish mm. Pick a search threshold which
//  doesnt detect those.
#define ALIGN_OFFSET \
  DEG_TO_TICK(9)  // sensor latency or command delay makes the sweep centre
                  // rotate more
// than intended. We had to subtract an offset angle to make it align
// correctly. Determined to be 9 degrees from align testing.
// Putting in DEG_TO_TICK converts this to ticks.
#define DEPOSIT_GAP \
  90  // intended gap from bollards. Set to 90 mm, as the robot was already
      // stopping further away than intended.

// The ultrasonic sensor has minimum reading of ~300mm (around 295 for out final
// robot) with anything closer getting clamped to this value. This is important
// in deposit state, where if we are too close (15mm) to bollard it reads as
// 300. Need to back up first, then remeasure.
#define ULTRASONIC_MIN \
  (300 + TOTAL_CALIBRATION)  // calibrated range. (From front of robot, this is
                             // the minimum range)
#define DETECT_MARGIN 15     // Close margin. Set to 15mm from experimentation.
#define DEPOSIT_REVERSE 100  // distance to backup from bollard in this case.

// Cliff and bumper bits, read as 0bLCR (same layout for kobuki.cliff and
// kobuki.bumper). Identical to the button, but directions (left and right) may
// be mixed as the results from Practical 4-6 measurements were lost. We simply
// calibrated our functions to fit this (flip signs of rotations).
#define SENSOR_RIGHT 0x01U   // bit 1
#define SENSOR_CENTRE 0x02U  // bit 2
#define SENSOR_LEFT 0x04U    // bit 3

// Kobuki drives drives towards a edge, until the cliff sensor is
// triggered. Then reverse a bit, by the distance in edge backoff. From diagram
// of kobuki online, cliff sensor 165 mm ahead of centre of the robot, so we
// back up to around 205 mm from edge to align with centre of corner. (corners
// are 400 by 400, we want to be in the centreish so we want to be around ~200
// away from edge)

#define SPEED \
  ((int16_t)100)  // speed while looking for the edge. Can be sped up, we kept
                  // it relatively slow to minimise errors from slip, etc.
#define FRONT_REVERSE 40  // backoff distance.

// Course correction: one of the side cliff sensors right or left triggering on
// its own means the robot has drifted towards the edge off the course. Back off
// a little and turn a certain angle away from that edge towards centre, then
// continue. A side bumper on its own is handled exact same way where the
// depending on the triggered bumper, back off, and turn a correction angle
// towards centre. Depedning on if left bumper or right bumper, turn CW or ACW.
// Centre handled differently.
#define SIDE_REVERSE 30  // mm
#define CORRECTION_DEG 5
// Also the same for both layouts.

#define STORAGE_ENTRY \
  920  // to the point equidistant from D1 and D2 at the centre of storage.
       // Using pythagoras, this was 918.4mm and so ~920mm, from section 3
       // corner.

// ENUMS
typedef enum {
  LAYOUT_A,
  LAYOUT_B
} LayoutState;  // Enum to represent the layout state (A or B)

typedef enum {
  IDLE,
  SEARCH,
  PATHFIND,
  STORAGE,
  DEPOSIT,
} State;  // Enum to represent the different states of the robot

typedef enum {
  ALIGN_INIT,
  ALIGN_LEFT,
  ALIGN_RIGHT,
  ALIGN_ALIGNING,
  ALIGN_DONE
} ALIGNstate;  // Enum to represent the alignment states

typedef enum {
  STAGE_ROTATE1,
  STAGE_FORWARD1,
  STAGE_ROTATE2,
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
  DEPOSIT_BACK,
  DEPOSIT_DONE
} DEPOSITstate;  // Enum to represent the deposit states

// Global variables

// debugging variables
volatile int16_t debug_d;
volatile int16_t debug_align_min_d;
volatile int16_t debug_deposit_d;

volatile State debug_state;
volatile ALIGNstate debug_align_state;
volatile PFstate debug_pathfind_state;
volatile STORAGEstate debug_storage_state;
volatile DEPOSITstate debug_deposit_state;

volatile int32_t debug_thetaRight_deg;
volatile int32_t debug_thetaLeft_deg;
volatile float debug_thetaTarget_deg;
volatile int32_t debug_span;

volatile uint16_t debug_cliff;
volatile uint16_t debug_bumper;
volatile uint16_t debug_wheeldrop;
volatile int16_t
    debug_layout;  // Variable to store the current layout state for debugging

// Useful variables
volatile LayoutState layout;  // Variable for current layout state

volatile Kobuki_Typedef
    kobuki;  // Kobuki is global scope as used in many functions.

// FSM state variables (global scope so reset_all_fsms() can reach them as well)
volatile static ALIGNstate align_state = ALIGN_INIT;
volatile static PFstate pathfind_state = STAGE_ROTATE1;
volatile static STORAGEstate storage_state = STORAGE_ORIENT;
volatile static DEPOSITstate deposit_state = DEPOSIT_ALIGN;

// Results from the last align sweep. Distance to closest object in
// align_min_d, absolute heading in gyroscope ticks [-18000,18000] to object in
// align_theta.
volatile static int16_t align_min_d;
volatile static int32_t align_theta;

// Absolute heading (also ticks) of the blocking bollard, set after initial
// search align. Every later turn is relative to this, so we can use absolute
// headings, and errors from turning do not snowball.
volatile static int32_t headingRef;

// course-correction turn (degrees) from Drive_To_Edge().
volatile static int16_t edge_correction;

// Functions

// Praj
//  Function to wrap a given angle in ticks to the range [-18000, 18000].
//  Equivalent to how any angle can be wrapped to [-180, 180].
int32_t wrap(int32_t angle) {
  // If angle above 18000, subtract 36000 until in range (full rotation = 36000)
  while (angle > 18000) angle -= 36000;
  // If angle below 18000, add 36000 until in range (full rotation = 36000)
  while (angle < -18000) angle += 36000;
  return angle;
}

// Start a rotation to an absolute heading based on gyroscope in ticks.
// Calculated the difference in ticks between desired heading, and current
// heading. Then commands a rotation based off this difference. All turns are
// set through this and MotionUpdate() then drives it to completion.
// Praj
void setRotation(int32_t targetAngle) {
  int32_t relativeDifference = wrap(targetAngle - kobuki.angle);
  Kobuki_Rotate_Angle_Setpoint(kobuki.angle,
                               (int16_t)(relativeDifference * DEG_PER_TICK));
}

// Helper to rotate with respect to the reference heading based on degrees.
// Vatsal
void setRotation_Ref(int16_t offset) {
  setRotation(wrap(headingRef + DEG_TO_TICK(offset)));
}

// Keep the current drive or rotation moving. returns 1 while motion pending,
// and returns 0 when all motion has been completed.
// Josh
bool MotionUpdate(void) {
  // Distance takes priority over angle. This is simply to ensure that both dont
  // occur simulatneously.
  if (!kobuki.distance_complete)  // If distance not done, move distance.
  {
    Kobuki_Drive_Distance(kobuki.distance);
    return 1;
  }
  // If distance done, but pending rotation, update rotation.
  if (!kobuki.rotation_complete) {
    Kobuki_Rotate_Angle(kobuki.angle);
    return 1;
  }
  // If no motion pending.
  return 0;
}

// Praj
// Sweep ACW until an object within distanceThresh (calibrated mm) enters and
// leaves the beam, then turn to its centre. Essentially identical to the flow
// chart provided in the hint/help file for this project. Records the minimum
// distance to object in align_min_d and, its angle in align_theta. returns 1
// once aligned, 0 while in progress.

// Does so in 4 stages. Initialisation stage moves object out of LOS if starting
// with object in sight.
//  This is to prevent the incorrect angle being computed as theta left and
//  theta right need to be the left and right side of objects, not somewhere in
//  the middle.
// After moving out of LOS, start scan for theta right, angle of right edge.
// After getting theta right, continue sweeping to get theta left.
// As scanning, keep track of minimum detected distance to object. Theoretically
// this may not work with different shaped bollards (like triangle) but fine for
// cylinder. After finding both edges, compute the average, and store in
// align_theta, and align using it. Once done, return 1.

// Note in practice, we were rotating ACW so actually theta right is detected
// first and then theta left unlike the flow chart provided.

bool align(int16_t distanceThresh) {
  // Initialise left, and right theta. Theta_avg stored in align_theta since
  // thats the angle to object.
  static int32_t thetaLeft, thetaRight;

  // Ultrasonic reading with calibration. Gives distance to object detected.
  int16_t d = Ultrasonic_Read() + TOTAL_CALIBRATION;
  // Debugging vars
  //  debug_d = d;
  //  debug_align_state = align_state;

  // Align Sub FSM.
  switch (align_state) {
      // Initial stage
    case ALIGN_INIT:
      // If object detected.
      if (d <= distanceThresh) {
        // Rotate out of the way. It will keep rotating 45 degrees until its not
        // in LOS.
        setRotation(wrap(kobuki.angle + DEG_TO_TICK(45)));
      } else {  // If out of LOS, enter align_right to detect right theta.
        align_state = ALIGN_RIGHT;
      }
      break;
    case ALIGN_RIGHT:
      // Keep turning until the object enters the beam.
      if (d <= distanceThresh) {
        // Once object detected, store current angle in theta_rigth
        thetaRight = kobuki.angle;
        // Debug
        // Update minimum distance, and enter align_left state.
        align_min_d = d;
        align_state = ALIGN_LEFT;
      } else {
        // If not detected continue rotating. Rotating using kobuki_rotate
        // directly as we havent defined our reference heading yet. Also could
        // use set_rotation if we wanted to, but it is the same thing, and we
        // are not looking for precise rotation in sweep.
        Kobuki_Rotate(SEARCH_SPEED);
      }
      break;
    case ALIGN_LEFT:
      // Keep turning until it leaves the beam, tracking the closest reading.
      if (d <= distanceThresh)  // If detected
      {
        // Update minimum distance if lower.
        if (d < align_min_d) {
          align_min_d = d;
        }
        // Continue sweeping.
        Kobuki_Rotate(SEARCH_SPEED);
      } else {  // Once object out of LOS
        // Store theta left.
        thetaLeft = kobuki.angle;
        // Stop rotation.
        Kobuki_Rotate(STOP);
        // Enter alignment state.
        align_state = ALIGN_ALIGNING;
      }
      break;
    case ALIGN_ALIGNING:
      // Turn to the middle of the two detected angles minus the align offset.
      // Using wrap to wrap angles into the tick range. Computed in ticks.
      align_theta =
          wrap(thetaRight + wrap(thetaLeft - thetaRight) / 2 - ALIGN_OFFSET);

      // Now we can use setRotation to rotate to an absolute angle and align
      // with object heading.
      setRotation(align_theta);
      // Enter align done state.
      align_state = ALIGN_DONE;
      break;
    case ALIGN_DONE:
      // Once rotation has finished, and we are aligned, simply comes in; resets
      // the FSM state to init, and returns 1 for success.
      align_state = ALIGN_INIT;
      return 1;
    default:
      break;
  }
  return 0;
}

// Drive forward until the centre cliff sensor sees the platform edge ahead,
// then start reversing the backoff amount. If only a side cliff sensor or a
// side bumper fires, steer away from that side and keep going (sidebackoff
// followed by edge correction). returns 1 once the front cliff detected and
// backing off.

// Vatsal
bool Drive_To_Edge(void) {
  // If edge correction from last loop.
  if (edge_correction != 0) {
    // Rotate edge correction amount from current heading.
    setRotation(wrap(kobuki.angle + DEG_TO_TICK(edge_correction)));
    // Reset edge correction.
    edge_correction = 0;
    // return.
    return 0;
  }

  // If no bumper or cliff sensors triggered, continue going forward
  if (kobuki.cliff == 0 && kobuki.bumper == 0) {
    // Drive forward
    Kobuki_Drive(SPEED);
    return 0;
  }

  // If something detected on cliff or bumper sensors, stop.
  Kobuki_Drive(STOP);
  // Both bumpers and cliff sensors follow same convention (1 bit for each
  // sensor, 0bxxx) If left sensor triggered, need to rotate CW.
  if (kobuki.cliff == SENSOR_LEFT || kobuki.bumper == SENSOR_LEFT) {
    // back off by side_backoff amount.
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, -SIDE_REVERSE);
    // Seperate into two steps to ensure that rotation and distance motion not
    // occur simultaneosuly. Edge correction occurs in next loop after backed
    // off is finished. Theoretically with how motionupdate works, we could
    // probably set the rotation here, as distance takes precedence over
    // rotations anyways. But better to seperate like this just in case.
    // Negative correction as negative corresponds to CW
    edge_correction = -CORRECTION_DEG;
    return 0;
  }
  // If right sensor triggered, need to rotate ACW
  if (kobuki.cliff == SENSOR_RIGHT || kobuki.bumper == SENSOR_RIGHT) {
    // Back off.
    Kobuki_Drive_Distance_Setpoint(kobuki.distance, -SIDE_REVERSE);
    // Set rotation. Positive edge correction as adding is ACW.
    edge_correction = CORRECTION_DEG;
    return 0;
  }

  // Centre cliff sensor (or centre bumper/ also could be multiple
  // cliff/bumpers). Edge is ahead of us meaning we have finished driving to
  // edge. A centre bump means something is directly ahead, so stop here as well
  // rather than push into it. This shouldnt happen during the course though.
  Kobuki_Drive_Distance_Setpoint(kobuki.distance, -FRONT_REVERSE);
  // Return 1 to indicate complete.
  return 1;
}

// PRAJ - Pathfinding FSM, from the start to section 3.
// Layout A: A1 is up the page, so go right (section 4) then up.
// Layout B: B1 is to the right, so go up (section 2) then right.
// Each drive goes to the edge, so we dont rely on hardcoding distances.
bool pathfindFSM(void) {
  // Variable for turn degree. After aligning with bollard, in layout A, we
  // rotate 90degrees CW. In layout B, we rotate 90 degrees ACW.
  static volatile int16_t turn;
  if (layout == LAYOUT_A) {
    turn = -90;
  } else {
    turn = 90;
  }

  // FSM
  switch (pathfind_state) {
    case STAGE_ROTATE1:  // Initial rotation; Occurs when facing Bollard 1
      // Rotate based on the turn degrees decided above with respect to
      // reference.
      setRotation_Ref(turn);
      // Proceed to next stage.
      pathfind_state = STAGE_FORWARD1;
      break;
    case STAGE_FORWARD1:
      // Once rotation complete, drive to the edge.
      if (Drive_To_Edge()) {
        // Once driven to edge and in corner, enter second rotation stage.
        pathfind_state = STAGE_ROTATE2;
      }
      break;
    case STAGE_ROTATE2:
      // align back to bollards heading. This works for both layouts.
      setRotation_Ref(0);
      // Enter next stage.
      pathfind_state = STAGE_FORWARD2;
      break;
    case STAGE_FORWARD2:
      // Second drive to edge, this aligns in the corner of section 3.
      if (Drive_To_Edge()) {
        // Once driven to edge, and in corner, enter done stage.
        pathfind_state = STAGE_DONE;
      }
      break;
    case STAGE_DONE:
      // Reset the pathfind_state variable to initial, and return 1 for
      // succeess. This occurs after backoff has finished since motion update
      // does not allow us to enter in middle of update.
      pathfind_state = STAGE_ROTATE1;
      return 1;
    default:
      break;
  }
  return 0;  // Return 0 in all cases besides done to indicate in progress.
}

// Josh - Storage FSM, from section 3 into the storage unit.
bool storageFSM(void) {
  switch (storage_state) {
    case STORAGE_ORIENT:
      // Face down-left towards the centre of the storage area.
      if (layout == LAYOUT_A) {
        // In layout A, need to rotate 135 degrees ACW (Currently facing
        // reference, so need to rotate 135 degrres ACW to face centre)
        setRotation_Ref(135);
      } else {  // In layout B, need to rotate 135 degrees CW from reference.
        setRotation_Ref(-135);
      }
      // Set to next stage.
      storage_state = STORAGE_FORWARD;
      break;
    case STORAGE_FORWARD:
      // Once rotated, command kobuki to drive in.
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, STORAGE_ENTRY);
      // Set to complete state.
      storage_state = STORAGE_DONE;
      break;
    case STORAGE_DONE:
      // Once drove in, return 1 to indicate completion of state.
      storage_state = STORAGE_ORIENT;
      return 1;
    default:
      break;
  }
  // Return 0 to indicate in progress.
  return 0;
}

// Vatsal - Deposit FSM, find the marker and stop DEPOSIT_GAP in front of it.
bool depositFSM(void) {
  // Read US reading.
  static volatile int16_t deposit_d = 0;
  deposit_d = Ultrasonic_Read() + TOTAL_CALIBRATION;
  switch (deposit_state) {
      // Align with bollard
    case DEPOSIT_ALIGN:
      if (align(DEPOSIT_THRESH))  // If align finished
      {
        // If align distance below the close margin (i.e too close to bollard)
        if (align_min_d <= ULTRASONIC_MIN + DETECT_MARGIN) {
          // Enter back case
          deposit_state = DEPOSIT_BACK;
        } else {  // If align distance above close margin, enter drive forward
                  // state.
          deposit_d = align_min_d;
          deposit_state = DEPOSIT_FORWARD;
        }
      }
      break;
    case DEPOSIT_BACK:
      // debug_deposit_d = deposit_d;
      // If deposit is too close.
      // The robot still remains aligned.
      if (deposit_d <= ULTRASONIC_MIN + DETECT_MARGIN) {
        // Too close to measure: reverse away from the marker (we're facing
        // it)
        Kobuki_Drive_Distance_Setpoint(kobuki.distance, -DEPOSIT_REVERSE);
      } else {  // If sufficiently far, enter forward state.
        deposit_state = DEPOSIT_FORWARD;
      }
      break;
    case DEPOSIT_FORWARD:
      // Still aligned in this state. Use the current distance reading, and
      // subtract the desired gap To align close with the bollard.
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, deposit_d - DEPOSIT_GAP);

      // Enter done state.
      deposit_state = DEPOSIT_DONE;
      break;
    case DEPOSIT_DONE:
      // once all motion finished, reset FSM, and return 1 to indicate success.
      deposit_state = DEPOSIT_ALIGN;
      return 1;
    default:
      break;
  }
  // Return for in progress.
  return 0;
}

// Helper function to reset all FSMs state to initial states, and also added
// edge correction to reset.
// Vatsal
void reset_all_fsms(void) {
  align_state = ALIGN_INIT;
  pathfind_state = STAGE_ROTATE1;
  storage_state = STORAGE_ORIENT;
  deposit_state = DEPOSIT_ALIGN;
  edge_correction = 0;
}

// All of us
//  Main function to initialize the robot and run the main loop.
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

    // Debug
    /* debug_cliff = kobuki.cliff;
     debug_bumper = kobuki.bumper;
     debug_wheeldrop = kobuki.wheeldrop;
     debug_state = current_state; // Store the current state for debugging
     debug_align_state = align_state;
     debug_pathfind_state = pathfind_state;
     debug_storage_state = storage_state;
     debug_deposit_state = deposit_state;
    */
    // Immediately stop on button 1 or wheel drop.
    if ((kobuki.button & BUT1) || (kobuki.wheeldrop != 0)) {
      // Reset current_state
      current_state = IDLE;
      reset_all_fsms();  // Reset all FSMs
      led_2 = RED;       // red while stopped manually
    }

    // For debugging
    // if ((kobuki.button & BUT2))
    // {
    //  current_state = DEPOSIT;
    //}

    // Let any drive or rotation in progress finish before the next step.
    // Skipped in IDLE so a stop takes effect immediately in this loop.
    if (current_state != IDLE && MotionUpdate()) {
      continue;  // Proceed to next loop. Continue motion if needed,etc.
    }

    // debug_d = Ultrasonic_Read() + TOTAL_CALIBRATION;
    // main FSM
    switch (current_state) {
      case IDLE:  // IDLE case
        // Stop kobuki immediately
        Kobuki_Stop();
        // Reset FSMs if needed.
        reset_all_fsms();
        led_1 = OFF;  // Deactivate LED
        if (kobuki.button & BUT0) {
          current_state = SEARCH;
          led_1 = RED;  // red while running
          led_2 = OFF;  // Turn off led2
        }

        break;
      case SEARCH:  // Align with the blocking bollard, its distance gives
                    // the layout.
        // If align has finished
        if (align(SEARCH_THRESH)) {
          // Set the heading reference to bollard angle.
          headingRef = align_theta;
          // If alignment distnace above the midpoint of the layout (285), then
          // layout A. Otherwise layout B
          if (align_min_d > LAYOUT_MIDPOINT) {
            layout = LAYOUT_A;
          } else {
            layout = LAYOUT_B;
          }
          // debug_layout = layout; // Store the current layout state for
          // debugging Enter pathfind.
          current_state = PATHFIND;
        }
        break;
      case PATHFIND:
        // Pathfind FSM.
        // If pathfinding has finished, we are in sector 3 corner. Enter storage
        // state.
        if (pathfindFSM()) {
          current_state = STORAGE;
        }
        break;
      case STORAGE:
        // If storage has finished, we are at the centre of the storage box.
        if (storageFSM()) {
          // update to deposit state.
          current_state = DEPOSIT;
        }
        break;
      case DEPOSIT:
        // Deposit FSM. Aligns with deposit bollard, and distances itself around
        // 100-90mm away from it.
        if (depositFSM()) {      // Once deposit finishes, reset to IDLE.
          current_state = IDLE;  // Finished
        }
        break;
      default:
        // In case of a undefined state, enter IDLE.
        current_state = IDLE;
        break;
    }
  }
}