#include <Servo.h>

// Pin Definitions
const int LDR_PINS[] = {A0, A1, A2, A3}; // Top-left, Top-right, Bottom-left, Bottom-right
const int MOTOR_PINS[] = {7, 6, 5, 4}; // IN1, IN2, IN3, IN4
const int MOTOR_ENABLE_PINS[] = {9, 10}; // ENA, ENB
const int ULTRASONIC_PINS[] = {11, 8}; // Trigger, Echo
const int SERVO_PINS[] = {12, 13}; // Horizontal, Vertical

// Constants
const int LIGHT_THRESHOLD = 10; // Light intensity threshold for movement
const int DISTANCE_THRESHOLD = 50; // Distance threshold for obstacle avoidance in centimeters
const int SERVO_NEUTRAL_POS = 90;
const int SERVO_MAX_POS = 180;
const int MOTOR_SPEED = 255; // Half speed

// Class Declarations
class Rover {
  private:
    Servo horizServo;
    Servo vertServo;
    int ldrPins[4];
    int ultrasonicPins[2];
    int servoPins[2];
    int motorPins[4];
    int motorEnablePins[2];

  public:
    Rover(int ldrPins[], int ultrasonicPins[], int servoPins[], int motorPins[], int motorEnablePins[]);
    void setup();
    void loop();
    void moveForward();
    void moveBackward();
    void turnRight();
    void turnLeft();
    void stopMotors();
    long getDistance();
    void adjustServos(int leftAvg, int rightAvg, int topAvg, int bottomAvg);
    void moveTowardsSun(int leftAvg, int rightAvg, int topAvg, int bottomAvg);
    void scanForObstacles();
    void handleErrors(const char* errorMsg);
};

// Rover Constructor
Rover::Rover(int ldrPins[], int ultrasonicPins[], int servoPins[], int motorPins[], int motorEnablePins[]) {
  for (int i = 0; i < 4; ++i) {
    this->ldrPins[i] = ldrPins[i];
    this->motorPins[i] = motorPins[i];
    if (i < 2) {
      this->ultrasonicPins[i] = ultrasonicPins[i];
      this->motorEnablePins[i] = motorEnablePins[i];
    }
    if (i < 2) {
      this->servoPins[i] = servoPins[i];
    }
  }
}

// Rover Setup
void Rover::setup() {
  for (int i = 0; i < 4; ++i) {
    pinMode(motorPins[i], OUTPUT);
    pinMode(ldrPins[i], INPUT);
  }
  for (int i = 0; i < 2; ++i) {
    pinMode(motorEnablePins[i], OUTPUT);
  }
  pinMode(ultrasonicPins[0], OUTPUT);
  pinMode(ultrasonicPins[1], INPUT);
  
  horizServo.attach(servoPins[0]);
  vertServo.attach(servoPins[1]);
  horizServo.write(SERVO_NEUTRAL_POS);
  vertServo.write(SERVO_NEUTRAL_POS);

  Serial.begin(9600);
}

// Rover Loop
void Rover::loop() {
  // Read LDR values
  int ldrValues[4];
  for (int i = 0; i < 4; ++i) {
    ldrValues[i] = analogRead(ldrPins[i]);
  }

  // Calculate average LDR values for top and bottom
  int topAverage = (ldrValues[0] + ldrValues[1]) / 2;
  int bottomAverage = (ldrValues[2] + ldrValues[3]) / 2;

  // Calculate average LDR values for left and right
  int leftAverage = (ldrValues[0] + ldrValues[2]) / 2;
  int rightAverage = (ldrValues[1] + ldrValues[3]) / 2;

  // Adjust servo positions based on LDR values
  adjustServos(leftAverage, rightAverage, topAverage, bottomAverage);

  // Obstacle avoidance
  scanForObstacles();

  // Move towards the sun
  if (horizServo.read() != SERVO_MAX_POS) {
    moveTowardsSun(leftAverage, rightAverage, topAverage, bottomAverage);
  }

  delay(100); // Short delay for sensor stability
}

// Move the rover forward
void Rover::moveForward() {
  digitalWrite(motorPins[0], HIGH);
  digitalWrite(motorPins[1], LOW);
  digitalWrite(motorPins[2], HIGH);
  digitalWrite(motorPins[3], LOW);
  analogWrite(motorEnablePins[0], MOTOR_SPEED); // Half speed
  analogWrite(motorEnablePins[1], MOTOR_SPEED); // Half speed
}

// Move the rover backward
void Rover::moveBackward() {
  digitalWrite(motorPins[0], LOW);
  digitalWrite(motorPins[1], HIGH);
  digitalWrite(motorPins[2], LOW);
  digitalWrite(motorPins[3], HIGH);
  analogWrite(motorEnablePins[0], MOTOR_SPEED); // Half speed
  analogWrite(motorEnablePins[1], MOTOR_SPEED); // Half speed
}

// Turn the rover right
void Rover::turnRight() {
  digitalWrite(motorPins[0], HIGH);
  digitalWrite(motorPins[1], LOW);
  digitalWrite(motorPins[2], LOW);
  digitalWrite(motorPins[3], HIGH);
  analogWrite(motorEnablePins[0], MOTOR_SPEED); // Half speed
  analogWrite(motorEnablePins[1], MOTOR_SPEED); // Half speed
}

// Turn the rover left
void Rover::turnLeft() {
  digitalWrite(motorPins[0], LOW);
  digitalWrite(motorPins[1], HIGH);
  digitalWrite(motorPins[2], HIGH);
  digitalWrite(motorPins[3], LOW);
  analogWrite(motorEnablePins[0], MOTOR_SPEED); // Half speed
  analogWrite(motorEnablePins[1], MOTOR_SPEED); // Half speed
}

// Stop all motors
void Rover::stopMotors() {
  for (int i = 0; i < 4; ++i) {
    digitalWrite(motorPins[i], LOW);
  }
  analogWrite(motorEnablePins[0], 0);
  analogWrite(motorEnablePins[1], 0);
}

// Get distance from the ultrasonic sensor
long Rover::getDistance() {
  digitalWrite(ultrasonicPins[0], LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicPins[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicPins[0], LOW);
  long duration = pulseIn(ultrasonicPins[1], HIGH);
  long distance = (duration / 2) / 29.1; // Convert to centimeters
  return distance;
}

// Adjust the servo positions based on LDR values
void Rover::adjustServos(int leftAvg, int rightAvg, int topAvg, int bottomAvg) {
  int horizPos = horizServo.read();
  int vertPos = vertServo.read();

  // Adjust horizontal servo
  if (abs(leftAvg - rightAvg) > LIGHT_THRESHOLD) {
    if (leftAvg > rightAvg) {
      horizPos = constrain(horizPos - 1, 0, SERVO_MAX_POS); // Turn left
    } else {
      horizPos = constrain(horizPos + 1, 0, SERVO_MAX_POS); // Turn right
    }
  }

  // Adjust vertical servo
  if (abs(topAvg - bottomAvg) > LIGHT_THRESHOLD) {
    if (topAvg > bottomAvg) {
      vertPos = constrain(vertPos - 1, 0, SERVO_MAX_POS); // Tilt up
    } else {
      vertPos = constrain(vertPos + 1, 0, SERVO_MAX_POS); // Tilt down
    }
  }

  // Write new positions to servos
  horizServo.write(horizPos);
  vertServo.write(vertPos);
}

// Move the rover towards the sun's position
void Rover::moveTowardsSun(int leftAvg, int rightAvg, int topAvg, int bottomAvg) {
  if (abs(leftAvg - rightAvg) > LIGHT_THRESHOLD || abs(topAvg - bottomAvg) > LIGHT_THRESHOLD) {
    // If there's a significant difference in light intensity, adjust direction
    if (leftAvg < rightAvg) {
      turnRight();
    } else {
      turnLeft();
    }
    delay(1000); // Give time for the turn
  } else {
    moveForward(); // Move forward towards the sun
  }
}

// Scan for obstacles using a static ultrasonic sensor
void Rover::scanForObstacles() {
  long distance = getDistance();
  if (distance < DISTANCE_THRESHOLD) {
    // Obstacle detected, perform avoidance maneuvers
    stopMotors();
    delay(1000);
    moveBackward();
    delay(1000);
    stopMotors();
    delay(1000);
    turnRight();
    delay(1000);
  }
}

// Handle errors and display message on serial monitor
void Rover::handleErrors(const char* errorMsg) {
  Serial.println(errorMsg);
}

Rover rover(LDR_PINS, ULTRASONIC_PINS, SERVO_PINS, MOTOR_PINS, MOTOR_ENABLE_PINS);

void setup() {
  rover.setup();
}

void loop() {
  rover.loop();
}
