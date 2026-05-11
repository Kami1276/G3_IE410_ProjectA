#include <Servo.h>
#include "BraccioRobot.h"
#include <math.h>

// Updated Link parameters (mm) [cite: 1, 2]
const double BASE_HEIGHT = 71.5;  
const double BICEP_LEN = 125.0; 
const double FOREARM_LEN = 125.0; 
const double END_EFFECTOR_LEN = 192.0; [cite: 3]

// Desired end-effector pitch (radians): vertical down = -90 [cite: 4]
const double PITCH_RAD = -PI / 2;
Position targetPose;

void setup() {
    Serial.begin(115200);
    BraccioRobot.init();
    Serial.println("Braccio IK System Initialized"); [cite: 5]
}

/**
 * Read pick and place coordinates: G:x1,y1,z1,x2,y2,z2 [cite: 6]
 */
bool fetchCoordinates(double &xPick, double &yPick, double &zPick,
                      double &xDrop, double &yDrop, double &zDrop) {
    if (!Serial.available()) return false;
    
    String incomingLine = Serial.readStringUntil('\n'); [cite: 6]
    if (incomingLine.length() < 2 || incomingLine.charAt(0) != 'G') return false;

    incomingLine = incomingLine.substring(2);
    double parsedVals[6];
    int counter = 0; [cite: 7]
    char buffer[64];
    
    incomingLine.toCharArray(buffer, sizeof(buffer));
    char *token = strtok(buffer, ",");
    
    while (token && counter < 6) { [cite: 8]
        parsedVals[counter++] = atof(token);
        token = strtok(NULL, ","); [cite: 9]
    }

    if (counter != 6) return false;

    xPick = parsedVals[0]; yPick = parsedVals[1]; zPick = parsedVals[2]; [cite: 10]
    xDrop = parsedVals[3]; yDrop = parsedVals[4]; zDrop = parsedVals[5];
    return true;
}

/**
 * Compute IK (radians) then convert to degrees [cite: 11]
 */
void calculateKinematics(double x, double y, double z,
                         double &theta1, double &theta2, double &theta3, double &theta4) {
    
    theta1 = atan2(y, x);
    double radius = sqrt(x * x + y * y); [cite: 12]
    double z_offset = z - BASE_HEIGHT;
    
    double dist = sqrt(radius * radius + z_offset * z_offset); [cite: 13]
    double dist_prime = dist - END_EFFECTOR_LEN;
    
    // Law of Cosines for elbow (theta3) [cite: 14]
    double cosAlpha = (BICEP_LEN * BICEP_LEN + FOREARM_LEN * FOREARM_LEN - dist_prime * dist_prime) / (2 * BICEP_LEN * FOREARM_LEN);
    cosAlpha = constrain(cosAlpha, -1, 1); [cite: 15]
    double angle_alpha = acos(cosAlpha);
    theta3 = M_PI - angle_alpha;
    
    // Law of Cosines for shoulder (theta2) [cite: 16]
    double angle_beta = atan2(z_offset, radius);
    double cosGamma = (BICEP_LEN * BICEP_LEN + dist_prime * dist_prime - FOREARM_LEN * FOREARM_LEN) / (2 * BICEP_LEN * dist_prime); [cite: 17]
    cosGamma = constrain(cosGamma, -1, 1); [cite: 18]
    double angle_gamma = acos(cosGamma);
    theta2 = angle_beta + angle_gamma;
    
    // Solve for wrist pitch [cite: 19]
    theta4 = PITCH_RAD - (theta2 + theta3);
    
    // Convert to degrees [cite: 20]
    theta1 *= 180.0 / PI;
    theta2 *= 180.0 / PI;
    theta3 *= 180.0 / PI; [cite: 21]
    theta4 *= 180.0 / PI;
}

void executeMove(double x, double y, double z, bool closeClaw) {
    double t1, t2, t3, t4;
    calculateKinematics(x, y, z, t1, t2, t3, t4); [cite: 23]

    // Map IK angles to Braccio servo ranges
    int b_servo = round(180 - t1);
    int s_servo = round(t2);
    int e_servo = round(t3);
    int w_servo = round(180 - t4);
    int wr_servo = 90;
    int g_servo = closeClaw ? 73 : 10;  // 10 for safer motor range [cite: 24]

    targetPose.set(b_servo, s_servo, e_servo, w_servo, wr_servo, g_servo);
    BraccioRobot.moveToPosition(targetPose, 100); [cite: 25]
    delay(300);
}

void executeTask(double pX, double pY, double pZ,
                 double dX, double dY, double dZ) {
    
    executeMove(pX, pY, pZ + 100, false); // Approach above pick [cite: 26]
    executeMove(pX, pY, pZ, false);       // Lower to pick [cite: 27]
    executeMove(pX, pY, pZ, true);        // Close gripper [cite: 28]
    executeMove(pX, pY, pZ + 100, true);  // Lift [cite: 29]

    executeMove(dX, dY, dZ + 100, true);  // Approach above place [cite: 30]
    executeMove(dX, dY, dZ, true);        // Lower to place [cite: 31]
    executeMove(dX, dY, dZ, false);       // Open gripper [cite: 32]
    executeMove(dX, dY, dZ + 100, false); // Clear [cite: 33]

    // Return to home position
    targetPose.set(90, 90, 90, 90, 90, 73);
    BraccioRobot.moveToPosition(targetPose, 150); [cite: 34]
    delay(300);
}

void loop() {
    double px, py, pz, dx, dy, dz;
    if (fetchCoordinates(px, py, pz, dx, dy, dz)) { [cite: 35]
        executeTask(px, py, pz, dx, dy, dz);
        Serial.println("Operation Complete"); [cite: 36]
    }
}