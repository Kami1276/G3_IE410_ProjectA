#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

void setup() {

  Braccio.begin();

  // Initial neutral position
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 73);
  delay(2000);

  // Open gripper and orient for top approach
  Braccio.ServoMovement(20, 90, 90, 90, 90, 160, 73);
  delay(2000);

  // Move above handover point
  Braccio.ServoMovement(20, 110, 65, 40, 90, 160, 73);
  delay(2000);

  // Descend toward object
  Braccio.ServoMovement(20, 110, 85, 55, 90, 160, 73);
  delay(2000);

  // Grab object
  Braccio.ServoMovement(20, 110, 85, 55, 90, 160, 20);
  delay(2000);

  // Wait for Arm 1 release
  delay(4000);

  // Lift object
  Braccio.ServoMovement(20, 110, 60, 35, 90, 160, 20);
  delay(2000);

  // Rotate to right side
  Braccio.ServoMovement(20, 40, 60, 35, 90, 160, 20);
  delay(2000);

  // Lower object for placement
  Braccio.ServoMovement(20, 40, 85, 55, 90, 160, 20);
  delay(2000);

  // Release object
  Braccio.ServoMovement(20, 40, 85, 55, 90, 160, 73);
  delay(2000);

  // Lift back up
  Braccio.ServoMovement(20, 40, 60, 35, 90, 160, 73);
  delay(2000);

  // Return to home position
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 73);
  delay(2000);
}

void loop() {

}