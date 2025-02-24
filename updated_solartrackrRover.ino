#include <Servo.h>

// Pin Definitions
const int LDR_PINS[] = {A0, A1, A2, A3}; // Light sensors: Top-left, Top-right, Bottom-left, Bottom-right
const int MOTOR_PINS[] = {7, 6, 5, 4};   // Motor control: IN1, IN2, IN3, IN4
const int MOTOR_ENABLE_PINS[] = {9, 10}; // Motor speed control: ENA, ENB
const int ULTRASONIC_TRIGGER = 11;       // Ultrasonic sensor trigger pin
const int ULTRASONIC_ECHO = 8;           // Ultrasonic sensor echo pin
const int SERVO_HORIZONTAL_PIN = 12;     // Servo controlling left-right movement
const int SERVO_VERTICAL_PIN = 13;       // Servo controlling up-down movement

// Constants
const int LIGHT_THRESHOLD = 10;    // Difference in light intensity to trigger movement
const int DISTANCE_THRESHOLD = 50; // Distance (cm) to detect obstacles
const int MOTOR_SPEED = 255;       // Speed for motors (0-255)
const int SERVO_NEUTRAL_POS = 90;  // Default position for servos
const int SERVO_MAX_POS = 180;     // Maximum angle for servos

// Servo objects
Servo horizServo;
Servo vertServo;

// Function Declarations
void setupPins();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
long getDistance();
void adjustServos(int leftAvg, int rightAvg, int topAvg, int bottomAvg);
void moveTowardsSun(int leftAvg, int rightAvg, int topAvg, int bottomAvg);
void scanForObstacles();

void setup() {
    Serial.begin(9600);
    setupPins();

    // Attach servos and set them to the neutral position
    horizServo.attach(SERVO_HORIZONTAL_PIN);
    vertServo.attach(SERVO_VERTICAL_PIN);
    horizServo.write(SERVO_NEUTRAL_POS);
    vertServo.write(SERVO_NEUTRAL_POS);
}

void loop() {
    // Read LDR sensor values
    int ldrValues[4];
    for (int i = 0; i < 4; i++) {
        ldrValues[i] = analogRead(LDR_PINS[i]);
    }

    // Compute average light intensity for left, right, top, and bottom
    int leftAvg = (ldrValues[0] + ldrValues[2]) / 2;
    int rightAvg = (ldrValues[1] + ldrValues[3]) / 2;
    int topAvg = (ldrValues[0] + ldrValues[1]) / 2;
    int bottomAvg = (ldrValues[2] + ldrValues[3]) / 2;

    // Adjust the servo positions to track the sun
    adjustServos(leftAvg, rightAvg, topAvg, bottomAvg);

    // Check for obstacles and avoid them
    scanForObstacles();

    // Move towards the brightest light source (sun)
    moveTowardsSun(leftAvg, rightAvg, topAvg, bottomAvg);

    delay(100); // Small delay for sensor stability
}

// Initialize pin modes
void setupPins() {
    for (int i = 0; i < 4; i++) {
        pinMode(MOTOR_PINS[i], OUTPUT);
        pinMode(LDR_PINS[i], INPUT);
    }
    for (int i = 0; i < 2; i++) {
        pinMode(MOTOR_ENABLE_PINS[i], OUTPUT);
    }
    pinMode(ULTRASONIC_TRIGGER, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
}

// Move the rover forward
void moveForward() {
    digitalWrite(MOTOR_PINS[0], HIGH);
    digitalWrite(MOTOR_PINS[1], LOW);
    digitalWrite(MOTOR_PINS[2], HIGH);
    digitalWrite(MOTOR_PINS[3], LOW);
    analogWrite(MOTOR_ENABLE_PINS[0], MOTOR_SPEED);
    analogWrite(MOTOR_ENABLE_PINS[1], MOTOR_SPEED);
}

// Move the rover backward
void moveBackward() {
    digitalWrite(MOTOR_PINS[0], LOW);
    digitalWrite(MOTOR_PINS[1], HIGH);
    digitalWrite(MOTOR_PINS[2], LOW);
    digitalWrite(MOTOR_PINS[3], HIGH);
    analogWrite(MOTOR_ENABLE_PINS[0], MOTOR_SPEED);
    analogWrite(MOTOR_ENABLE_PINS[1], MOTOR_SPEED);
}

// Turn left
void turnLeft() {
    digitalWrite(MOTOR_PINS[0], LOW);
    digitalWrite(MOTOR_PINS[1], HIGH);
    digitalWrite(MOTOR_PINS[2], HIGH);
    digitalWrite(MOTOR_PINS[3], LOW);
    analogWrite(MOTOR_ENABLE_PINS[0], MOTOR_SPEED);
    analogWrite(MOTOR_ENABLE_PINS[1], MOTOR_SPEED);
}

// Turn right
void turnRight() {
    digitalWrite(MOTOR_PINS[0], HIGH);
    digitalWrite(MOTOR_PINS[1], LOW);
    digitalWrite(MOTOR_PINS[2], LOW);
    digitalWrite(MOTOR_PINS[3], HIGH);
    analogWrite(MOTOR_ENABLE_PINS[0], MOTOR_SPEED);
    analogWrite(MOTOR_ENABLE_PINS[1], MOTOR_SPEED);
}

// Stop all motors
void stopMotors() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(MOTOR_PINS[i], LOW);
    }
    analogWrite(MOTOR_ENABLE_PINS[0], 0);
    analogWrite(MOTOR_ENABLE_PINS[1], 0);
}

// Get distance using the ultrasonic sensor
long getDistance() {
    digitalWrite(ULTRASONIC_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIGGER, LOW);
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH);
    return (duration / 2) / 29.1; // Convert time to distance (cm)
}

// Adjust servo positions based on light intensity
void adjustServos(int leftAvg, int rightAvg, int topAvg, int bottomAvg) {
    int horizPos = horizServo.read();
    int vertPos = vertServo.read();

    // Adjust horizontal movement
    if (abs(leftAvg - rightAvg) > LIGHT_THRESHOLD) {
        horizPos = (leftAvg > rightAvg) ? constrain(horizPos - 1, 0, SERVO_MAX_POS) 
                                         : constrain(horizPos + 1, 0, SERVO_MAX_POS);
    }

    // Adjust vertical movement
    if (abs(topAvg - bottomAvg) > LIGHT_THRESHOLD) {
        vertPos = (topAvg > bottomAvg) ? constrain(vertPos - 1, 0, SERVO_MAX_POS)
                                       : constrain(vertPos + 1, 0, SERVO_MAX_POS);
    }

    // Update servo positions
    horizServo.write(horizPos);
    vertServo.write(vertPos);
}

// Move towards the sun by adjusting movement
void moveTowardsSun(int leftAvg, int rightAvg, int topAvg, int bottomAvg) {
    if (abs(leftAvg - rightAvg) > LIGHT_THRESHOLD || abs(topAvg - bottomAvg) > LIGHT_THRESHOLD) {
        if (leftAvg < rightAvg) {
            turnRight();
        } else {
            turnLeft();
        }
        delay(1000); // Pause for turn adjustment
    } else {
        moveForward();
    }
}

// Scan for obstacles and avoid them
void scanForObstacles() {
    long distance = getDistance();
    if (distance < DISTANCE_THRESHOLD) {
        // Obstacle detected, perform avoidance maneuver
        stopMotors();
        delay(500);
        moveBackward();
        delay(1000);
        stopMotors();
        delay(500);
        turnRight();
        delay(1000);
    }
}
