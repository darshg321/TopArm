#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Servo pulse range
#define SERVOMIN  150
#define SERVOMAX  600
// Joystick pins
#define JOY_X A0
#define JOY_Y A1
#define JOY_PUSH 2
// Settings
#define DEADZONE 50
#define DEFAULT_ANGLE 90
#define SERVO_SPEED 2  // degrees per loop iteration
#define GRIPPER_OPEN_ANGLE 180  // Angle when button is pressed
#define GRIPPER_NEUTRAL_ANGLE 90  // Neutral position when button is released
// Servo channels
#define GRIPPER 4
#define BASE 5
#define ARM_UPDOWN 6
#define ARM_FWDBK 7
// Current servo positions
int currentAngles[4] = {90, 90, 90, 90};

void setup() {
  Serial.begin(9600);
  pinMode(JOY_PUSH, INPUT_PULLUP);
 
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
 
  // Initialize all servos to default position
  for (int i = 0; i < 4; i++) {
    setServo(i + 4, DEFAULT_ANGLE);
  }
 
  Serial.println("Robot Arm Ready!");
  Serial.println("X=Base Rotate (Left/Right), Y=Arm Up/Down");
  Serial.println("Button: Press to open gripper, release to return to neutral");
}

void loop() {
  // Read joystick
  int xValue = analogRead(JOY_X);
  int yValue = analogRead(JOY_Y);
  bool buttonPressed = !digitalRead(JOY_PUSH);
 
  // Handle gripper control with button
  if (buttonPressed) {
    // Button is pressed - open gripper
    currentAngles[GRIPPER - 4] = GRIPPER_OPEN_ANGLE;
    setServo(GRIPPER, GRIPPER_OPEN_ANGLE);
    Serial.print("BUTTON PRESSED - Gripper angle: ");
    Serial.println(GRIPPER_OPEN_ANGLE);
  } else {
    // Button is released - return to neutral
    currentAngles[GRIPPER - 4] = GRIPPER_NEUTRAL_ANGLE;
    setServo(GRIPPER, GRIPPER_NEUTRAL_ANGLE);
  }
 
  // Apply deadzone
  int xMove = 0, yMove = 0;
  if (abs(xValue - 512) > DEADZONE) {
    xMove = map(xValue, 0, 1023, -SERVO_SPEED, SERVO_SPEED);
  }
  if (abs(yValue - 512) > DEADZONE) {
    yMove = map(yValue, 0, 1023, -SERVO_SPEED, SERVO_SPEED);
  }
 
  // X controls Base (Left/Right)
  if (xMove != 0) {
    updateServo(BASE, xMove);
    Serial.print("X-AXIS - Base angle: ");
    Serial.println(currentAngles[BASE - 4]);
  }
  
  // Y controls Arm Up/Down
  if (yMove != 0) {
    updateServo(ARM_UPDOWN, yMove);
    Serial.print("Y-AXIS - Arm Up/Down angle: ");
    Serial.println(currentAngles[ARM_UPDOWN - 4]);
  }
 
  delay(20);
}

void updateServo(int channel, int delta) {
  if (delta == 0) return;
 
  int index = channel - 4;
  currentAngles[index] = constrain(currentAngles[index] + delta, 0, 180);
  setServo(channel, currentAngles[index]);
}

void setServo(int channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}