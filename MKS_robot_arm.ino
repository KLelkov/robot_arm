#include <Arduino.h>
#include "FastAccelStepper.h"
#include "RobotNet.h"

const char* WIFI_SSID = "CleverRat";
const char* WIFI_PASS = "rattastic";

RobotNet robotNet;

const int NUM_MOTORS = 4;
#define enablePinStepper 8
const uint8_t pin_DIR[NUM_MOTORS] =   { 15,  6,  4, 19 };
const uint8_t pin_STEP[NUM_MOTORS] =  { 16,  7,  5, 20 };
const uint8_t pin_LIMIT[NUM_MOTORS] = { 39, 40, 41, 37 };

const float L1 = 226.0;
const float L2 = 135.0;
const float TIP_RADIUS = 38.8;

const float GEAR_RATIO_A = 19.055; // True
const float GEAR_RATIO_Z = 0.124; // True
const float GEAR_RATIO_Y = 16.071; // True
const float GEAR_RATIO_X = 4.61;

FastAccelStepper* steppers[NUM_MOTORS];
const int homingDirs[NUM_MOTORS] = {1, -1, -1, -1};
const int BACKOFF_STEPS = 1000;
const int HOMING_SPEED = 2200;

FastAccelStepperEngine engine = FastAccelStepperEngine();
const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPS = 8;
const int STEPS_PER_ROTATION = MOTOR_STEPS_PER_REV * MICROSTEPS;

const float STEPS_PER_DEG_A = (STEPS_PER_ROTATION * GEAR_RATIO_A) / 360.0;
const float STEPS_PER_MM_Z = STEPS_PER_ROTATION * GEAR_RATIO_Z;
const float STEPS_PER_DEG_Y = (STEPS_PER_ROTATION * GEAR_RATIO_Y) / 360.0;
const float STEPS_PER_DEG_X = (STEPS_PER_ROTATION * GEAR_RATIO_X) / 360.0;

const float MIN_HEIGHT = 0.0;
const float MAX_HEIGHT = 170.0;
const float MIN_REACH = abs(L1 - L2) + 5.0;
const float MAX_REACH = abs(L1 + L2);
const float MAX_PLANAR = 136;
const float MIN_PLANAR = -164;


const float base_z = 170;
const float base_a_deg = 163;
const float base_x_deg = 100;
const float base_y_deg = 136;

enum HomingState { SEEKING_SWITCH, BACKING_OFF, HOMING_COMPLETE };
const int Z_AXIS_INDEX = 2;
const unsigned long OTHER_AXES_DELAY_MS = 5000;

// --- ASYNCHRONOUS COMMAND BUFFER ---
struct MotionCommand {
  bool newCommand = false;
  long targetSteps[NUM_MOTORS];
};
volatile MotionCommand pendingMove;

// Declarations
void runSimultaneousHoming();
bool handleCylindricalRequest(float target_z, float target_r, float target_theta_deg, int elbow_mode);
bool handleMotorRequest(float target_x_deg, float target_y_deg, float target_z, float target_a_deg);
bool calculateSteps(float target_x_deg, float target_y_deg, float target_z, float target_a_deg, long outSteps[4]);
void getRobotStatus(long &z, long &a, long &y, long &x, bool &isBusy);
bool handleStepsRequest(long target_x, long target_y, long target_z, long target_a);
void getCurrentSteps(long &x, long &y, long &z, long &a);
void getCurrentAngles(float &x, float &y, float &z, float &a);

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Robot Arm Initialization Started ---");

  engine.init();
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(pin_LIMIT[i], INPUT_PULLUP);
    steppers[i] = engine.stepperConnectToPin(pin_STEP[i]);
    if (steppers[i]) {
      steppers[i]->setDelayToEnable(50);
      steppers[i]->setDirectionPin(pin_DIR[i]);
      steppers[i]->setEnablePin(enablePinStepper, true);
      steppers[i]->enableOutputs();
      uint32_t speed_hz = 16 * STEPS_PER_ROTATION;
      steppers[i]->setSpeedInHz(speed_hz);       
      steppers[i]->setAcceleration(speed_hz * 2);
    }
  }

  runSimultaneousHoming();
  delay(500);

  // Link callbacks
  robotNet.registerCallbacks(handleCylindricalRequest, handleMotorRequest, handleStepsRequest, getRobotStatus, getCurrentSteps, getCurrentAngles);
  robotNet.begin(WIFI_SSID, WIFI_PASS);
}

void loop()
{
  // Check if a new command arrived from the web server
  if (pendingMove.newCommand) {
    // Send steps to steppers
    steppers[2]->moveTo(pendingMove.targetSteps[2]); // Z
    steppers[3]->moveTo(pendingMove.targetSteps[3]); // A
    steppers[1]->moveTo(pendingMove.targetSteps[1]); // Y
    steppers[0]->moveTo(pendingMove.targetSteps[0]); // X

    pendingMove.newCommand = false; // Command consumed
  }

  delay(5); // Non-blocking loop yield
}

// Check if arm is moving
bool isArmMoving() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (steppers[i]->isRunning()) return true;
  }
  return pendingMove.newCommand;
}

// Feeds current positions back to HTTP
void getRobotStatus(long &z, long &a, long &y, long &x, bool &isBusy) {
  z = steppers[2]->getCurrentPosition();
  a = steppers[3]->getCurrentPosition();
  y = steppers[1]->getCurrentPosition();
  x = steppers[0]->getCurrentPosition();
  isBusy = isArmMoving();
}

// Kinematics Validation & Conversion (Non-Blocking)
bool calculateSteps(float target_x_deg, float target_y_deg, float target_z, float target_a_deg, long outSteps[4]) {
  float height = -target_z + base_z;
  float planar_angle_deg = target_a_deg + base_a_deg;
  float elbow_angle_deg = target_y_deg + base_y_deg;
  float tip_angle_deg = -(target_x_deg + base_x_deg);

  if (height < 5 || height > 170) return false;
  if (planar_angle_deg < 3 || planar_angle_deg > 303) return false;
  if (elbow_angle_deg < 6 || elbow_angle_deg > 276) return false;
  if (tip_angle_deg > 0 || tip_angle_deg < -270) return false;

  outSteps[2] = height * STEPS_PER_MM_Z;           // Z
  outSteps[3] = planar_angle_deg * STEPS_PER_DEG_A;// A
  outSteps[1] = elbow_angle_deg * STEPS_PER_DEG_Y; // Y
  outSteps[0] = tip_angle_deg * STEPS_PER_DEG_X;   // X

  return true;
}

// Network Callback: Cylindrical
bool handleCylindricalRequest(float target_z, float target_r, float target_theta_deg, int elbow_mode) {
  if (isArmMoving()) return false; // Reject if already executing a move

  if (target_z < MIN_HEIGHT || target_z > MAX_HEIGHT) return false;
  if (target_r < MIN_REACH || target_r > MAX_REACH) return false;
  if (target_theta_deg < MIN_PLANAR || target_theta_deg > MAX_PLANAR) return false;

  float cos_psiX = (-sq(target_r) + sq(L1) + sq(L2)) / (2.0 * L1 * L2);
  float psiX = acos(cos_psiX);
  float thetaX_deg = (PI - psiX) * 180 / PI;

  float cos_thetaY = (-sq(L2) + sq(target_r) + sq(L1)) / (2.0 * L1 * target_r);
  float thetaY_deg = acos(cos_thetaY) * 180.0 / PI;
  thetaY_deg = -thetaY_deg;

  if (elbow_mode == 1) {
    thetaY_deg = -thetaY_deg;
    thetaX_deg = -thetaX_deg;
  }
  float thetaA_deg = target_theta_deg + thetaY_deg; 

  long steps[4];
  if (!calculateSteps(0, thetaX_deg, target_z, thetaA_deg, steps)) {
    return false;
  }

  // Queue to loop()
  for (int i = 0; i < NUM_MOTORS; i++) pendingMove.targetSteps[i] = steps[i];
  pendingMove.newCommand = true;
  return true;
}

// Network Callback: Direct Motors
bool handleMotorRequest(float target_x_deg, float target_y_deg, float target_z, float target_a_deg) {
  if (isArmMoving()) return false;

  // X-Axis check
  if (target_x_deg > 170 || target_x_deg < -100) {
    Serial.println("Angle error: X target out of bounds!");
    return false;
  }
  // Y-Axis check
  if (target_y_deg < 130 || target_y_deg > 140) {
    Serial.println("Angle error: Y target out of bounds!");
    return false;
  }
  // Z-Axis check
  if (target_z < 5 || target_z > 170) {
    Serial.println("Angle error: Z target out of bounds!");
    return false;
  }
  // A-Axis check
  if (target_a_deg < 160 || target_a_deg > 140) {
    Serial.println("Angle error: A target out of bounds!");
    return false;
  }

  long steps[4];
  if (!calculateSteps(target_x_deg, target_y_deg, target_z, target_a_deg, steps)) {
    return false;
  }

  for (int i = 0; i < NUM_MOTORS; i++) pendingMove.targetSteps[i] = steps[i];
  pendingMove.newCommand = true;
  return true;
}

bool handleStepsRequest(long target_x, long target_y, long target_z, long target_a) {
  if (isArmMoving()) return false;
  
  // X-Axis check
  if (target_x > 0 || target_x < 5000) {
    Serial.println("Step error: X target out of bounds!");
    return false;
  }
  // Y-Axis check
  if (target_y < 0 || target_y > 20000) {
    Serial.println("Step error: Y target out of bounds!");
    return false;
  }
  // Z-Axis check
  if (target_z < 0 || target_z > 33000) {
    Serial.println("Step error: Z target out of bounds!");
    return false;
  }
  // A-Axis check
  if (target_a < 0 || target_a > 26000) {
    Serial.println("Step error: A target out of bounds!");
    return false;
  }

  // Queue raw steps into the pending move struct
  pendingMove.targetSteps[0] = target_x; // X (Tip)
  pendingMove.targetSteps[1] = target_y; // Y (Elbow)
  pendingMove.targetSteps[2] = target_z; // Z (Height)
  pendingMove.targetSteps[3] = target_a; // A (Base)
  
  pendingMove.newCommand = true;
  return true;
}

// --- HOMING LOGIC ---
void startMotorHoming(int i, HomingState axisState[]) {
  if (steppers[i]) {
    steppers[i]->setSpeedInHz(HOMING_SPEED);
    if (homingDirs[i] == 1) steppers[i]->runForward();
    else steppers[i]->runBackward();
    axisState[i] = SEEKING_SWITCH;
  } else {
    axisState[i] = HOMING_COMPLETE;
  }
}

void runSimultaneousHoming() {
  Serial.println("Staggered homing sequence started...");
  HomingState axisState[NUM_MOTORS];
  bool axisStarted[NUM_MOTORS] = {false};

  startMotorHoming(Z_AXIS_INDEX, axisState);
  axisStarted[Z_AXIS_INDEX] = true;

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (i != Z_AXIS_INDEX) axisState[i] = HOMING_COMPLETE;
  }

  unsigned long startTime = millis();
  bool otherAxesStarted = false;
  bool allHomed = false;

  while (!allHomed) {
    if (!otherAxesStarted && 
       ((millis() - startTime >= OTHER_AXES_DELAY_MS) || (axisState[Z_AXIS_INDEX] == HOMING_COMPLETE))) {
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (i != Z_AXIS_INDEX) {
          startMotorHoming(i, axisState);
          axisStarted[i] = true;
        }
      }
      otherAxesStarted = true;
    }

    allHomed = true;
    if (!otherAxesStarted) allHomed = false;

    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!axisStarted[i]) continue;

      if (axisState[i] == SEEKING_SWITCH) {
        allHomed = false;
        if (digitalRead(pin_LIMIT[i]) == HIGH) { 
          steppers[i]->forceStopAndNewPosition(steppers[i]->getCurrentPosition());
          steppers[i]->move(-BACKOFF_STEPS * homingDirs[i]);
          axisState[i] = BACKING_OFF;
        }
      } else if (axisState[i] == BACKING_OFF) {
        allHomed = false;
        if (!steppers[i]->isRunning()) {
          steppers[i]->setCurrentPosition(0);
          axisState[i] = HOMING_COMPLETE;
        }
      }
    }
    delay(1);
  }
  Serial.println("All axes homed successfully!");
}

void getCurrentSteps(long &x, long &y, long &z, long &a) {
  x = steppers[0] ? steppers[0]->getCurrentPosition() : 0;
  y = steppers[1] ? steppers[1]->getCurrentPosition() : 0;
  z = steppers[2] ? steppers[2]->getCurrentPosition() : 0;
  a = steppers[3] ? steppers[3]->getCurrentPosition() : 0;
}

void getCurrentAngles(float &x, float &y, float &z, float &a) {
  x = steppers[0] ? -(steppers[0]->getCurrentPosition() / STEPS_PER_DEG_X + base_x_deg): 0;
  y = steppers[1] ? (steppers[1]->getCurrentPosition() / STEPS_PER_DEG_Y - base_y_deg): 0;
  z = steppers[2] ? (base_z - steppers[2]->getCurrentPosition() / STEPS_PER_MM_Z): 0;
  a = steppers[3] ? (steppers[3]->getCurrentPosition() / STEPS_PER_DEG_A - base_a_deg): 0;
}
