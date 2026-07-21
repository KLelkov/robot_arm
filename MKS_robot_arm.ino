#include <Arduino.h>
#include "FastAccelStepper.h"
//#include <TMCStepper.h>

const int NUM_MOTORS = 4;

// --- Pins ---
#define enablePinStepper 8  // Common for all motors
//                                       X   Y   Z   A
const uint8_t pin_DIR[NUM_MOTORS] =   { 15,  6,  4, 19 };
const uint8_t pin_STEP[NUM_MOTORS] =  { 16,  7,  5, 20 };
const uint8_t pin_LIMIT[NUM_MOTORS] = { 39, 40, 41, 37 };

FastAccelStepper* steppers[NUM_MOTORS];
const int homingDirs[NUM_MOTORS] = {1, -1, -1, -1}; // Direction for each axis
//bool homed[NUM_MOTORS] = {false, false, false, false};

const int BACKOFF_STEPS = 1000;  // Steps to back away from switch after hitting it
const long MAX_TRAVEL[NUM_MOTORS] = {0, 0, 135000, 0};  // Maximum allowed steps from the switch

// --- FastAccelStepper Setup ---
FastAccelStepperEngine engine = FastAccelStepperEngine();

const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPS = 8;
const int STEPS_PER_ROTATION = MOTOR_STEPS_PER_REV * MICROSTEPS;

const int HOMING_SPEED = 2 * STEPS_PER_ROTATION;  // Slow, safe speed for finding the switch

// Homing States for the State Machine
enum HomingState {
  SEEKING_SWITCH,
  BACKING_OFF,
  HOMING_COMPLETE
};

//void runSimultaneousHoming();

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Robot arm code initialized.");

  // --- Configure FastAccelStepper ---
  engine.init();
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    pinMode(pin_LIMIT[i], INPUT_PULLUP);
    steppers[i] = engine.stepperConnectToPin(pin_STEP[i]);
    if (steppers[i])
    {
      steppers[i]->setDelayToEnable(50); // 50ms delay after enabling
      steppers[i]->setDirectionPin(pin_DIR[i]);
      steppers[i]->setEnablePin(enablePinStepper, true); // Active LOW
      steppers[i]->enableOutputs();
      uint32_t speed_hz = 16 * STEPS_PER_ROTATION;
      steppers[i]->setSpeedInHz(speed_hz);       
      steppers[i]->setAcceleration(speed_hz * 4);
      Serial.print("TMC2209 and stepper ");
      Serial.print(i);
      Serial.println(" are initialized!");
    }
    else
    {
      Serial.print("TMC2209 for stepper ");
      Serial.print(i);
      Serial.println(" configuration error!");
    }
  }
  
  runHomingSequence_Z();
  Serial.println("Z axis is done!");

  runHomingSequence_X();
  Serial.println("X axis is done!");

  runHomingSequence_Y();
  Serial.println("Y axis is done!");

  runHomingSequence_A();
  Serial.println("A axis is done!");
}

unsigned long lastPrintTime = 0; // Timer variable

void loop() {

}

// --- HOMING FUNCTION ---
void runHomingSequence_Z() {
  Serial.println("Homing started...");

  // 1. Set homing speed
  steppers[2]->setSpeedInHz(HOMING_SPEED);

  // 2. Start moving continuously in the homing direction
  if (homingDirs[2] == 1) {
    steppers[2]->runForward();
  } else {
    steppers[2]->runBackward();
  }

  // 3. Wait until the limit pin goes HIGH (switch is hit)
  // (Since FastAccelStepper runs in the background, this loop is empty!)
  while (digitalRead(pin_LIMIT[2]) == LOW) {
    delay(1); // Yield to ESP32 OS to prevent watchdog triggers
  }

  // 4. Switch hit! Stop immediately (no deceleration ramp)
  steppers[2]->forceStopAndNewPosition(steppers[2]->getCurrentPosition());
  Serial.println("Switch hit. Backing off...");
  delay(200);

  // 5. Move slightly away from the switch to release it
  // (We use relative move: negative homing direction * backoff steps)
  steppers[2]->move(-BACKOFF_STEPS * homingDirs[2]); 

  // Wait until backoff movement is finished (blocking)
  while (steppers[2]->isRunning()) {
    delay(1);
  }

  // 6. Set this exact position as the absolute ZERO point
  steppers[2]->setCurrentPosition(0);
  Serial.println("System Homed. Position set to 0.");
  delay(500);
}

void runHomingSequence_X() {
  Serial.println("Homing started...");

  // 1. Set homing speed
  steppers[0]->setSpeedInHz(HOMING_SPEED);

  // 2. Start moving continuously in the homing direction
  if (homingDirs[0] == 1) {
    steppers[0]->runForward();
  } else {
    steppers[0]->runBackward();
  }

  // 3. Wait until the limit pin goes HIGH (switch is hit)
  // (Since FastAccelStepper runs in the background, this loop is empty!)
  while (digitalRead(pin_LIMIT[0]) == LOW) {
    delay(1); // Yield to ESP32 OS to prevent watchdog triggers
  }

  // 4. Switch hit! Stop immediately (no deceleration ramp)
  steppers[0]->forceStopAndNewPosition(steppers[0]->getCurrentPosition());
  Serial.println("Switch hit. Backing off...");
  delay(200);

  // 5. Move slightly away from the switch to release it
  // (We use relative move: negative homing direction * backoff steps)
  steppers[0]->move(-BACKOFF_STEPS * homingDirs[0]); 

  // Wait until backoff movement is finished (blocking)
  while (steppers[0]->isRunning()) {
    delay(1);
  }

  // 6. Set this exact position as the absolute ZERO point
  steppers[0]->setCurrentPosition(0);
  Serial.println("System Homed. Position set to 0.");
  delay(500);
}

void runHomingSequence_Y() {
  Serial.println("Homing started...");

  // 1. Set homing speed
  steppers[1]->setSpeedInHz(HOMING_SPEED);

  // 2. Start moving continuously in the homing direction
  if (homingDirs[1] == 1) {
    steppers[1]->runForward();
  } else {
    steppers[1]->runBackward();
  }

  // 3. Wait until the limit pin goes HIGH (switch is hit)
  // (Since FastAccelStepper runs in the background, this loop is empty!)
  while (digitalRead(pin_LIMIT[1]) == LOW) {
    delay(1); // Yield to ESP32 OS to prevent watchdog triggers
  }

  // 4. Switch hit! Stop immediately (no deceleration ramp)
  steppers[1]->forceStopAndNewPosition(steppers[1]->getCurrentPosition());
  Serial.println("Switch hit. Backing off...");
  delay(200);

  // 5. Move slightly away from the switch to release it
  // (We use relative move: negative homing direction * backoff steps)
  steppers[1]->move(-BACKOFF_STEPS * homingDirs[1]); 

  // Wait until backoff movement is finished (blocking)
  while (steppers[1]->isRunning()) {
    delay(1);
  }

  // 6. Set this exact position as the absolute ZERO point
  steppers[1]->setCurrentPosition(0);
  Serial.println("System Homed. Position set to 0.");
  delay(500);
}

void runHomingSequence_A() {
  Serial.println("Homing started...");

  // 1. Set homing speed
  steppers[3]->setSpeedInHz(HOMING_SPEED);

  // 2. Start moving continuously in the homing direction
  if (homingDirs[3] == 1) {
    steppers[3]->runForward();
  } else {
    steppers[3]->runBackward();
  }

  // 3. Wait until the limit pin goes HIGH (switch is hit)
  // (Since FastAccelStepper runs in the background, this loop is empty!)
  while (digitalRead(pin_LIMIT[3]) == LOW) {
    delay(1); // Yield to ESP32 OS to prevent watchdog triggers
  }

  // 4. Switch hit! Stop immediately (no deceleration ramp)
  steppers[3]->forceStopAndNewPosition(steppers[3]->getCurrentPosition());
  Serial.println("Switch hit. Backing off...");
  delay(200);

  // 5. Move slightly away from the switch to release it
  // (We use relative move: negative homing direction * backoff steps)
  steppers[3]->move(-BACKOFF_STEPS * homingDirs[3]); 

  // Wait until backoff movement is finished (blocking)
  while (steppers[3]->isRunning()) {
    delay(1);
  }

  // 6. Set this exact position as the absolute ZERO point
  steppers[3]->setCurrentPosition(0);
  Serial.println("System Homed. Position set to 0.");
  delay(500);
}
