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
  Serial.begin(9600);
  Serial.println("Send '1' to start motion...");
}

void loop() {

  // WAIT for command
  while (1) {
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == '1') {
        Braccio.ServoMovement(20, 20, 40, 15, 90, 90, 10);
        delay(500);
        
        Braccio.ServoMovement(20, 20, 40, 15, 180, 90, 10);
        delay(500);

        Braccio.ServoMovement(20, 20, 40, 15, 180, 90, 150);
        delay(500);
      }
      else if(input=='2'){
        Braccio.ServoMovement(20, 90, 40, 15, 180, 90, 150);
        delay(500);

        Braccio.ServoMovement(20, 90, 40, 65, 145, 90, 150);
        delay(500);

        Braccio.ServoMovement(20, 90, 40, 65, 145, 90, 10);
        delay(500);
      }
    }
  }

  // ===== YOUR MOTION SEQUENCE =====
    // Reset so it waits again
  
  
}
