/*
  task1_preprogrammed.ino

  This program controls the Braccio robotic arm to perform
  a simple pre-programmed pick-and-place movement sequence.

  Created for Task 1 demonstration.

  This example is based on the Braccio movement library.
*/

#include <Braccio.h>
#include <Servo.h>


Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;


void setup() {  
  // Initialize the Braccio robotic arm
  // Initial safety positions:
  // Base (M1): 90 degrees
  // Shoulder (M2): 45 degrees
  // Elbow (M3): 180 degrees
  // Wrist vertical (M4): 180 degrees
  // Wrist rotation (M5): 90 degrees
  // Gripper (M6): 10 degrees
  Braccio.begin();
}

void loop() {

  /*
    Servo configuration details:
    
    Step Delay : Delay between servo movements (10 to 30 ms)

    M1 = Base rotation        (0 to 180 degrees)
    M2 = Shoulder movement    (15 to 165 degrees)
    M3 = Elbow movement       (0 to 180 degrees)
    M4 = Wrist vertical       (0 to 180 degrees)
    M5 = Wrist rotation       (0 to 180 degrees)
    M6 = Gripper              (10 to 73 degrees)

    Gripper values:
    10 = Open
    73 = Closed
  */

  // Initial position
                      //(step delay  M1 , M2 , M3 , M4 , M5 , M6);
  Braccio.ServoMovement(20,           0,  45, 180, 180,  90,  10);
  
  // Wait for 1 second
  delay(1000);

  // Close the gripper
  Braccio.ServoMovement(20,           0,  45, 180, 180,  90,  73);

  delay(1000);

  // Move wrist vertically upward
  Braccio.ServoMovement(20,           0,  45, 180, 90,  90,  73);

  delay(1000);

  // Rotate the base to another side
  Braccio.ServoMovement(20,           90,  45, 180, 90,  90,  73);

  delay(1000);

  // Lower the wrist vertically
  Braccio.ServoMovement(20,           90,  45, 180, 180,  90,  73);

  delay(1000);

  // Open the gripper to release the object
  Braccio.ServoMovement(20,           90,  45, 180, 180,  90,  10);

  delay(1000);

  // End of pre-programmed sequence

}