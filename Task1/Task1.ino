#include <Braccio.h>
#include <Servo.h>

// Declare servos
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

void setup() {
  Braccio.begin();
}

void loop() {

  // 1. Start position
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 10);
  delay(1000);

  // 2. Move above Point A
  Braccio.ServoMovement(20, 80, 100, 120, 90, 90, 10);
  delay(1000);

  // 3. Lower to object
  Braccio.ServoMovement(20, 80, 120, 140, 90, 90, 10);
  delay(800);

  // 4. Close gripper (grab)
  Braccio.ServoMovement(20, 80, 120, 140, 90, 90, 60);
  delay(800);

  // 5. Lift object
  Braccio.ServoMovement(20, 80, 100, 120, 90, 90, 60);
  delay(800);

  // 6. Move to Point B
  Braccio.ServoMovement(20, 120, 100, 120, 90, 90, 60);
  delay(1000);

  // 7. Lower to place
  Braccio.ServoMovement(20, 120, 120, 140, 90, 90, 60);
  delay(800);

  // 8. Open gripper (release)
  Braccio.ServoMovement(20, 120, 120, 140, 90, 90, 10);
  delay(800);

  // 9. Return to home
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 10);
  delay(3000);
}