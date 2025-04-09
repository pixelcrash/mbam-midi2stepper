#include <MIDI.h>
#include <AccelStepper.h>

// Define stepper motor pins
// Motor 1
#define M1_STEP_PIN 2
#define M1_DIR_PIN 3
#define M1_ENABLE_PIN 4

// Motor 2
#define M2_STEP_PIN 5
#define M2_DIR_PIN 6
#define M2_ENABLE_PIN 7

// Define MIDI channels
#define MOTOR1_CHANNEL 10
#define MOTOR2_CHANNEL 20

// Create MIDI instance
MIDI_CREATE_DEFAULT_INSTANCE();

// Create stepper instances
// Parameters: 1 = driver type, 2 = step pin, 3 = direction pin
AccelStepper stepper1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// Maximum speed for motors (steps per second)
const float MAX_SPEED = 1000;
// Maximum acceleration for motors (steps per second per second)
const float MAX_ACCEL = 500;

// Variables to store current MIDI control values
byte motor1ControlValue = 63; // Default to stop (63)
byte motor2ControlValue = 63; // Default to stop (63)

void setup() {
  // Configure stepper motors
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(MAX_ACCEL);
  stepper1.setEnablePin(M1_ENABLE_PIN);
  stepper1.setPinsInverted(false, false, true); // Direction invert, step invert, enable invert
  stepper1.enableOutputs();
  
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(MAX_ACCEL);
  stepper2.setEnablePin(M2_ENABLE_PIN);
  stepper2.setPinsInverted(false, false, true); // Direction invert, step invert, enable invert
  stepper2.enableOutputs();
  
  // Initialize MIDI
  MIDI.begin(MIDI_CHANNEL_OMNI); // Listen to all MIDI channels
  
  // Set up MIDI callback
  MIDI.setHandleControlChange(handleControlChange);
  
  // For debugging
  Serial.begin(115200);
  Serial.println("MIDI Stepper Motor Controller initialized");
}

void loop() {
  // Read incoming MIDI messages
  MIDI.read();
  
  // Update motor speeds based on current control values
  updateMotorSpeed(stepper1, motor1ControlValue);
  updateMotorSpeed(stepper2, motor2ControlValue);
  
  // Run the steppers
  stepper1.run();
  stepper2.run();
}

// MIDI Control Change callback
void handleControlChange(byte channel, byte number, byte value) {
  // For simplicity, we're assuming the control number doesn't matter
  // In a more advanced setup, you could use specific control numbers for different functions
  
  // Check which channel the message is from and update the appropriate motor
  if (channel == MOTOR1_CHANNEL) {
    motor1ControlValue = value;
    Serial.print("Motor 1 control value: ");
    Serial.println(value);
  } 
  else if (channel == MOTOR2_CHANNEL) {
    motor2ControlValue = value;
    Serial.print("Motor 2 control value: ");
    Serial.println(value);
  }
}

// Update motor speed based on MIDI control value (0-127)
void updateMotorSpeed(AccelStepper &stepper, byte controlValue) {
  // Stop conditions: 0, 63, 127
  if (controlValue == 0 || controlValue == 63 || controlValue == 127) {
    stepper.stop();
    stepper.setSpeed(0);
    return;
  }
  
  // Calculate direction and speed
  bool clockwise;
  float speedPercent;
  
  if (controlValue < 63) {
    // Clockwise (values 1-62)
    clockwise = true;
    // Map 1-62 to 1-100%
    speedPercent = map(controlValue, 1, 62, 1, 100) / 100.0;
  } else {
    // Counter-clockwise (values 64-126)
    clockwise = false;
    // Map 64-126 to 1-100%
    speedPercent = map(controlValue, 64, 126, 1, 100) / 100.0;
  }
  
  // Calculate actual speed
  float speed = MAX_SPEED * speedPercent;
  
  // Set direction by making speed positive or negative
  stepper.setSpeed(clockwise ? speed : -speed);
  
  // For continuous rotation, set a "target" far in the appropriate direction
  if (stepper.distanceToGo() == 0) {
    long newPosition = clockwise ? 1000000 : -1000000;
    stepper.moveTo(newPosition);
  }
}
