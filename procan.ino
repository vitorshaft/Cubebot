#include <Servo.h>

const int numServos = 6; // Number of servos
Servo servos[numServos]; // Array of servos
int servoPins[numServos] = {9, 10, 11, 12, 13, 14}; // Array of servo pins
int previousPos[numServos]; // Array to store previous servo positions

void setup() {
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
    previousPos[i] = 90; // Initialize previous positions to 90 degrees
  }
}

void controlServos(double accelerationVector[3], double timeStep) {
  // Acceleration vector is assumed to be in the format [ax, ay, az]
  double xAcceleration = accelerationVector[0];
  double yAcceleration = accelerationVector[1];
  double zAcceleration = accelerationVector[2];

  // Use the accelerations to control the servos
  for (int i = 0; i < 3; i++) {
    int currentPos = previousPos[i] + (accelerationVector[i] * timeStep);
    currentPos = constrain(currentPos, 0, 180); // Constrain position to 0-180 degrees
    servos[i].write(currentPos);
    previousPos[i] = currentPos;
  }
}

void loop() {
  // Call the controlServos function with the acceleration vector and time step
  double accelerationVector[3] = {0.5, -1.3, 0.2};
  double timeStep = 0.1; // Time step in seconds
  controlServos(accelerationVector, timeStep);
  delay(10);
}
