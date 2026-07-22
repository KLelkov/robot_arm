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

// --- Hardware settings ---
const float L1 = 235.0;  // length of Inner Arm (Y-axis link) in mm
const float L2 = 140.0;  // length of Outer Arm (X-axis link) in mm

// Transmission ratios
const float GEAR_RATIO_A = 1.0;
const float GEAR_RATIO_Z = 1.0; 
const float GEAR_RATIO_Y = 1.0; 
const float GEAR_RATIO_X = 4.5;

FastAccelStepper* steppers[NUM_MOTORS];
const int homingDirs[NUM_MOTORS] = {1, -1, -1, -1};  // direction for each axis
//bool homed[NUM_MOTORS] = {false, false, false, false};

const int BACKOFF_STEPS = 1000;  // steps to back away from switch after hitting it
const long MAX_TRAVEL[NUM_MOTORS] = {5400, 0, 135000, 0};  // maximum allowed steps from the switch (from the backoff distance)

// --- FastAccelStepper Setup ---
FastAccelStepperEngine engine = FastAccelStepperEngine();

const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPS = 8;
const int STEPS_PER_ROTATION = MOTOR_STEPS_PER_REV * MICROSTEPS;

const int HOMING_SPEED = 1 * STEPS_PER_ROTATION;  // slow, safe speed for finding the switch

// Transition coefficients
const float STEPS_PER_DEG_A = (STEPS_PER_ROTATION * GEAR_RATIO_A) / 360.0;
const float STEPS_PER_MM_Z = STEPS_PER_ROTATION * GEAR_RATIO_Z;
const float STEPS_PER_DEG_Y = (STEPS_PER_ROTATION * GEAR_RATIO_Y) / 360.0;
const float STEPS_PER_DEG_X = (STEPS_PER_ROTATION * GEAR_RATIO_X) / 360.0;

// Software limits
const float MIN_HEIGHT = -100.0;  // mm
const float MAX_HEIGHT = 0.0;  // mm
const float MIN_REACH = abs(L1 - L2) - 5.0;  // mm
const float MAX_REACH = abs(L1 + L2) - 5.0;  // mm
const float MAX_PLANAR = 300.0;  // deg
const float MIN_PLANAR = 0.0;  // deg

// Homing States for the State Machine
enum HomingState {
  SEEKING_SWITCH,
  BACKING_OFF,
  HOMING_COMPLETE
};

void runSimultaneousHoming();

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
      Serial.print("TMC2209 and stepper "); Serial.print(i); Serial.println(" are initialized!");
    }
    else
    {
      Serial.print("TMC2209 for stepper "); Serial.print(i); Serial.println(" configuration error!");
    }
  }
  
  // Run simultaneous homing for all axes
  runSimultaneousHoming();

  delay(4500);
  steppers[0]->move(-180 * STEPS_PER_DEG_X * homingDirs[0]);
  Serial.print("Stepper X outer limit: "); Serial.println(steppers[0]->getCurrentPosition());
}


void loop() {

}


// --- SIMULTANEOUS HOMING FUNCTION ---
void runSimultaneousHoming() {
  Serial.println("Simultaneous homing sequence started...");

  HomingState axisState[NUM_MOTORS];

  // Start all motors moving in their homing directions
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (steppers[i]) {
      steppers[i]->setSpeedInHz(HOMING_SPEED);
      
      if (homingDirs[i] == 1) {
        steppers[i]->runForward();
      } else {
        steppers[i]->runBackward();
      }
      axisState[i] = SEEKING_SWITCH;
    } else {
      axisState[i] = HOMING_COMPLETE;  // skip if the motor is not connected
    }
  }

  // Monitor all switches (motors) at once
  bool allHomed = false;
  while (!allHomed) {
    allHomed = true;  // assume true, will be set to false if any motor is still homing

    for (int i = 0; i < NUM_MOTORS; i++) {
      if (axisState[i] == SEEKING_SWITCH) {
        allHomed = false;  // if one is still moving, duh

        // Check if the limit switch is triggered (HIGH means triggered)
        if (digitalRead(pin_LIMIT[i]) == HIGH) { 
          steppers[i]->forceStopAndNewPosition(steppers[i]->getCurrentPosition());
          Serial.print("Axis "); Serial.print(i); Serial.println(" hit switch. Backing off...");
          
          // Start the backoff movement (opposite of homing direction)
          steppers[i]->move(-BACKOFF_STEPS * homingDirs[i]);
          axisState[i] = BACKING_OFF;
        }
      } 
      else if (axisState[i] == BACKING_OFF) {
        allHomed = false;

        // Check if backoff movement is finished
        if (!steppers[i]->isRunning()) {
          steppers[i]->setCurrentPosition(0);  // set absolute zero
          axisState[i] = HOMING_COMPLETE;
          Serial.print("Axis "); Serial.print(i); Serial.println(" successfully homed to 0.");
        }
      }
    }
    delay(1);  // yield to ESP32 OS to prevent Watchdog Timer (WDT) resets (triggers)
  }

  Serial.println("All axes successfully homed!");
}
