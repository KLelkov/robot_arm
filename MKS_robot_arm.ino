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

const float MIN_HEIGHT = 5.0;
const float MAX_HEIGHT = 170.0;
const float MIN_REACH = 150;  // limited by Y angle range and construction
const float MAX_REACH = abs(L1 + L2);
const float MAX_PLANAR = 165;
const float MIN_PLANAR = -180;


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
bool handleSphericalRequest(float target_r, float target_theta_deg, float target_fi_deg, int elbow_mode);
bool handleCartesianRequest(float target_x, float target_y, float target_z, int elbow_mode);
bool handleMotorRequest(float target_x_deg, float target_y_deg, float target_z, float target_a_deg);
bool calculateSteps(float target_x_deg, float target_y_deg, float target_z, float target_a_deg, long outSteps[4]);
void getRobotStatus(long &z, long &a, long &y, long &x, bool &isBusy);
bool handleStepsRequest(long target_x, long target_y, long target_z, long target_a);
void getCurrentSteps(long &x, long &y, long &z, long &a);
void getCurrentAngles(float &x, float &y, float &z, float &a);
void getCurrentCylinder(float &z, float &r, float &theta_deg);
void getCurrentSphere(float &r_out, float &theta_deg_out, float &fi_deg_out);
void getCurrentCartesian(float &x_out, float &y_out, float &z_out);

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
  robotNet.registerCallbacks(handleCylindricalRequest, handleMotorRequest, handleStepsRequest, getRobotStatus, getCurrentSteps, getCurrentAngles, getCurrentCylinder, handleSphericalRequest, getCurrentSphere, handleCartesianRequest, getCurrentCartesian);
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

  if (target_z < MIN_HEIGHT || target_z > MAX_HEIGHT) {
    Serial.print("Height is out of bounds! "); Serial.println(target_z);
    return false;
  }
  if (target_r < MIN_REACH || target_r > MAX_REACH) {
    Serial.print("Reach distance is out of bounds! "); Serial.println(target_r);
    return false;
  }
  if (target_theta_deg < MIN_PLANAR || target_theta_deg > MAX_PLANAR) {
    Serial.print("Planar angle is out of bounds! "); Serial.println(target_theta_deg);
    return false;
  }

  float cos_psiX = (-sq(target_r) + sq(L1) + sq(L2)) / (2.0 * L1 * L2);
  float psiX = acos(cos_psiX);
  float thetaX_deg = (PI - psiX) * 180 / PI;

  float cos_thetaY = (-sq(L2) + sq(target_r) + sq(L1)) / (2.0 * L1 * target_r);
  float thetaY_deg = acos(cos_thetaY) * 180.0 / PI;
  thetaY_deg = -thetaY_deg;

  float thetaA_deg = target_theta_deg + thetaY_deg; 
  
  long steps_right[4], steps_left[4];
  bool valid_right = calculateSteps(0, thetaX_deg, target_z, thetaA_deg, steps_right);
  bool valid_left = calculateSteps(0, -thetaX_deg, target_z, target_theta_deg - thetaY_deg, steps_left);

  if (elbow_mode == 1) { // left
    // positive value corresponds to the elbow pointing positive half of the field
    if (!valid_left) return false;  // respect the user's choice
    for (int i = 0; i < NUM_MOTORS; i++) pendingMove.targetSteps[i] = steps_left[i];
    pendingMove.newCommand = true;
    return true;
  }
  else if (elbow_mode == -1) { // right
    // negative value corresponds to the elbow pointing negative half of the field
    if (!valid_right) return false;  // respect the user's choice
    for (int i = 0; i < NUM_MOTORS; i++) pendingMove.targetSteps[i] = steps_right[i];
    pendingMove.newCommand = true;
    return true;
  }

  // Automatic selector for user position
  if (!valid_left && !valid_right) {
    Serial.println("No valid position could be found!");
    return false;
  }

  bool use_config_right = false;
  if (valid_right && !valid_left) {  // only right config is valid
    use_config_right = true;
  }
  else if (!valid_right && valid_left){  // only left config is valid
    use_config_right = false;
  }
  else {  // both configurations are valid - compute the effciency
    long current_steps_x = steppers[0]->getCurrentPosition();
    long current_steps_y = steppers[1]->getCurrentPosition();
    long current_steps_z = steppers[2]->getCurrentPosition();
    long current_steps_a = steppers[3]->getCurrentPosition();

    long cost_left = abs(steps_left[0] - current_steps_x) + abs(steps_left[1] - current_steps_y) +
                     abs(steps_left[2] - current_steps_z) + abs(steps_left[3] - current_steps_a);

    long cost_right = abs(steps_right[0] - current_steps_x) + abs(steps_right[1] - current_steps_y) +
                     abs(steps_right[2] - current_steps_z) + abs(steps_right[3] - current_steps_a);

    use_config_right = (cost_right <= cost_left);
  }

  long* chosen_steps = use_config_right ? steps_right : steps_left;
  // Queue to loop()
  for (int i = 0; i < NUM_MOTORS; i++) pendingMove.targetSteps[i] = chosen_steps[i];
  pendingMove.newCommand = true;
  return true;
}

bool handleSphericalRequest(float target_r, float target_theta_deg, float target_fi_deg, int elbow_mode) {
  if (isArmMoving()) return false; // Reject if already executing a move
  
  if ((target_fi_deg < 42) || (target_fi_deg > 89)) {
    Serial.print("Polar angle is out of bounds! "); Serial.println(target_fi_deg);
    return false;
  }
  if ((target_theta_deg < MIN_PLANAR) || (target_theta_deg > MAX_PLANAR)) {
    Serial.print("Azimuth angle is out of bounds! "); Serial.println(target_theta_deg);
    return false;
  }
  if ((target_r < 227) || (target_r > 399)) {
    Serial.print("Radius is out of bounds! "); Serial.println(target_r);
    return false;
  }
  float R = abs(target_r * sin(target_fi_deg * PI / 180.0)); // planar radius - always positive
  float azimuth = target_theta_deg;
  float height = sqrt(sq(target_r) - sq(R));

  return handleCylindricalRequest(height, R, azimuth, elbow_mode);
}

bool handleCartesianRequest(float target_x, float target_y, float target_z, int elbow_mode) {
  if (isArmMoving()) return false; // Reject if already executing a move

  if (target_x < -345 || target_x > MAX_REACH) {
    Serial.print("X is out of bounds! "); Serial.println(target_x);
    return false;
  }
  if (target_y < -MAX_REACH || target_y > MAX_REACH) {
    Serial.print("Y is out of bounds! "); Serial.println(target_y);
    return false;
  }
  if (target_z < MIN_HEIGHT || target_z > MAX_HEIGHT) {
    Serial.print("Z is out of bounds! "); Serial.println(target_z);
    return false;
  }
  float azimuth = atan2(target_y, target_x);
  float radius = sqrt(sq(target_x) + sq(target_y));
//  if (abs(target_x) > 5) {
//    azimuth = asin(target_y / target_x);
//    radius = abs(target_x / cos(azimuth));
//  }
//  else if (abs(target_y) > 5) {
//    azimuth = acos(target_x / target_y);
//    radius = abs(target_y / sin(azimuth));
//  }
//  else return false;
  //Serial.print("Azimuth: "); Serial.println(azimuth);
  
  azimuth = azimuth * 180.0 / PI;
  return handleCylindricalRequest(target_z, radius, azimuth, elbow_mode);
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
  if (target_y_deg < -130 || target_y_deg > 140) {
    Serial.println("Angle error: Y target out of bounds!");
    return false;
  }
  // Z-Axis check
  if (target_z < MIN_HEIGHT || target_z > MAX_HEIGHT) {
    Serial.println("Height error: Z target out of bounds!");
    return false;
  }
  // A-Axis check
  if (target_a_deg < -160 || target_a_deg > 140) {
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

void getCurrentCylinder(float &z_out, float &r_out, float &theta_deg_out) {
  float cur_x, cur_y, cur_z, cur_a;
  getCurrentAngles(cur_x, cur_y, cur_z, cur_a);

  z_out = cur_z;  // 'cur_z' from getCurrentAngles is already height (mm)
  float R2 = sq(L1) + sq(L2) - cos(PI - cur_y * PI / 180) * 2 * L1 * L2;
  float R = sqrt(R2);
  float theta_y = - acos((-sq(L2) + R2 + sq(L1)) / (2 * L1 * R));
  if (cur_y < 0) theta_y = -theta_y;  // adjust for left-sided elbow angle
  theta_deg_out = cur_a - theta_y * 180 / PI;
  r_out = R;
}

void getCurrentSphere(float &r_out, float &theta_deg_out, float &fi_deg_out) {
  float cur_height, cur_radius, cur_azimuth;
  getCurrentCylinder(cur_height, cur_radius, cur_azimuth);

  r_out = sqrt(sq(cur_height) + sq(cur_radius));
  theta_deg_out = cur_azimuth;
  fi_deg_out = asin(cur_radius / r_out) * 180.0 / PI;
}

void getCurrentCartesian(float &x_out, float &y_out, float &z_out) {
  float cur_height, cur_radius, cur_azimuth;
  getCurrentCylinder(cur_height, cur_radius, cur_azimuth);

  x_out = cur_radius * cos(cur_azimuth * PI / 180.0);
  y_out = cur_radius * sin(cur_azimuth * PI / 180.0);
  z_out = cur_height;
}
