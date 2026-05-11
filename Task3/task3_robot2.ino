/*
  task3_robot2.ino

  Part 3 - Second Robot

  This program controls the second Braccio robotic arm
  to perform a predefined movement sequence for Task 3.

  Created for robotic arm coordination and demonstration.

  Based on the Braccio movement library example.
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

  // Initial delay before starting the movement sequence
  delay(12000);

  // Move robotic arm to starting pickup position
  Braccio.ServoMovement(20,           0,  30, 180, 90,  90,  10);
  
  // Wait for 1 second
  delay(1000);

  // Close gripper to hold the object
  Braccio.ServoMovement(20,           0,  30, 180, 90,  90,  120);

  delay(1000);

  // Rotate base to transfer position
  Braccio.ServoMovement(20,           60,  30, 180, 90,  90,  120);

  delay(1000);

  // Lower wrist vertically
  Braccio.ServoMovement(20,           60,  30, 180, 10,  90,  120);

  delay(1000);

  // Open gripper to release the object
  Braccio.ServoMovement(20,           60,  30, 180, 10,  90,  10);
}