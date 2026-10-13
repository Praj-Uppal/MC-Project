#include <stdint.h>
#include <stdio.h>
#define STM32G474xx
#include "kobuki.h"
#include "stm32g4xx.h"
#include "uart.h"

// int main(void) {
//   // Create a structure to store kobuki values :
//   Kobuki_Typedef my_kobuki = {0};
//   // Initialize system UART
//   UART_Init();
//   while (1) {
//     // Read the full payload arriving at the RX pin and parse it
//     Kobuki_Read(&my_kobuki);
//     // Pause so you can inspect ’ my_kobuki ’ in your debugger
//     for (volatile int i = 0; i < 500000; i++)
//       ;
//   }
// }

int main(void) {
  UART_Init(); // Initiate UART
  Kobuki_Typedef my_kobuki = {
      0}; // Initiate a variable with Kobuki Typedef that contains sensors
          // reading 0 = Stopped ( Safe state ) , 1 = Running
  uint8_t is_running = 0;
  uint8_t is_rotating = 0;
  while (1) {
    // 1. Read the latest sensor data
    Kobuki_Read(&my_kobuki);
    // 2. Evaluate Start / Stop triggers
    if (my_kobuki.button == 1U) { // DRIVE STATE
      is_running = 1;  // Start command received ( Button 0) .DRIVE STATE
      is_rotating = 0; // Switch off ROTATE STATE
    } else if (my_kobuki.button == 4U) { // ROTATE STATE
      is_rotating = 1; // Start command received ( Button 2) .ROTATESTATE
      is_running = 0;  // Switch off DRIVE STATE
    }
    // Stop triggers take priority over start triggers . IDLE STATE
    if (my_kobuki.button == 2U || my_kobuki.bumper == (2U)) {
      is_running = 0; // Stop command ( Button 1) or collision detected.IDLE STATE
      is_rotating = 0;
    }
    // 3. Command the motors based on the current state
    if (is_running == 1) {
      Kobuki_Drive(100); // Drive forward at 100 mm / s . DRIVE STATE
    } else if (is_rotating == 1) {
      Kobuki_Rotate(100); // Rotate at 100 mm / s . ROTATE STATE
    } else {
      Kobuki_Drive(1); // Command zero velocity . IDLE STATE
      Kobuki_Rotate(1);
    }
    // 4. Control loop delay
    for (volatile int i = 0; i < 50000; i++)
      ; // Shorter delay for quick recognition of button inputs
  }
}
