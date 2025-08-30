#include <Stepper.h>
#include <math.h>

// Motor steps per revolution (for 28BYJ-48 it's 2048 steps per full rotation)
const int stepsPerRevolution = 2048;

// Create stepper objects for shoulder and elbow
Stepper shoulder(stepsPerRevolution, 8, 10, 9, 11); // IN1, IN3, IN2, IN4
Stepper elbow(stepsPerRevolution, 4, 6, 5, 7);      // IN1, IN3, IN2, IN4

// Link lengths
const float L1 = 10.0;
const float L2 = 10.0;

// Convert angle (degrees) to steps
int angleToSteps(float angle) {
  return (int)(angle * stepsPerRevolution / 360.0);
}

// Store current angle for each motor
float theta1_curr = 0;
float theta2_curr = 0;

// Move to angle
void moveToAngles(float theta1_target, float theta2_target) {
  int delta1 = angleToSteps(theta1_target - theta1_curr);
  int delta2 = angleToSteps(theta2_target - theta2_curr);
  
  shoulder.step(delta1);
  elbow.step(delta2);

  theta1_curr = theta1_target;
  theta2_curr = theta2_target;
}

// Inverse Kinematics
bool computeIK(float x, float y, float &theta1, float &theta2) {
  float r2 = x * x + y * y;
  float c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (c2 < -1 || c2 > 1) return false; // Point not reachable

  float s2 = sqrt(1 - c2 * c2);
  theta2 = atan2(s2, c2);

  float k1 = L1 + L2 * c2;
  float k2 = L2 * s2;
  theta1 = atan2(y, x) - atan2(k2, k1);

  // Convert radians to degrees
  theta1 *= 180.0 / PI;
  theta2 *= 180.0 / PI;

  return true;
}

void setup() {
  shoulder.setSpeed(10); // steps per second
  elbow.setSpeed(10);
  Serial.begin(9600);
}

void loop() {
  // Coordinates to draw "A" shape
  float path[][2] = {
    {10, 0}, {5, 10}, {0, 0}, {2.5, 5}, {7.5, 5}
  };

  int n = sizeof(path) / sizeof(path[0]);

  for (int i = 0; i < n; i++) {
    float x = path[i][0];
    float y = path[i][1];

    float t1, t2;
    if (computeIK(x, y, t1, t2)) {
      moveToAngles(t1, t2);
      delay(500);
    } else {
      Serial.println("Point not reachable");
    }
  }

  // Hold position
  while (1);
}

