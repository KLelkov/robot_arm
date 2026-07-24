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
const float GEAR_RATIO_A = 19.1;
const float GEAR_RATIO_Z = 0.121; 
const float GEAR_RATIO_Y = 16.0; 
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

const int HOMING_SPEED = 2200;  // slow, safe speed for finding the switch

// Transition coefficients
const float STEPS_PER_DEG_A = (STEPS_PER_ROTATION * GEAR_RATIO_A) / 360.0;
const float STEPS_PER_MM_Z = STEPS_PER_ROTATION * GEAR_RATIO_Z;
const float STEPS_PER_DEG_Y = (STEPS_PER_ROTATION * GEAR_RATIO_Y) / 360.0;
const float STEPS_PER_DEG_X = (STEPS_PER_ROTATION * GEAR_RATIO_X) / 360.0;

// Software limits
const float MIN_HEIGHT = -100.0;  // mm
const float MAX_HEIGHT = 0.0;  // mm
const float MIN_REACH = abs(L1 - L2) + 5.0;  // mm
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
bool moveToCylindrical(float target_z, float target_r, float target_theta_deg, int elbow_mode);
bool send2motors(float target_x, float target_y, float target_z, float target_a);

void setup()
{
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
      steppers[i]->setAcceleration(speed_hz * 2);
      Serial.print("TMC2209 and stepper "); Serial.print(i); Serial.println(" are initialized!");
    }
    else
    {
      Serial.print("TMC2209 for stepper "); Serial.print(i); Serial.println(" configuration error!");
    }
  }
  
  // Run simultaneous homing for all axes
  runSimultaneousHoming();

  delay(1500);
  //steppers[0]->move(-100 * STEPS_PER_DEG_X * homingDirs[0]);
  //Serial.print("Stepper X outer limit: "); Serial.println(steppers[0]->getCurrentPosition());
  //delay(2500);
  //steppers[3]->setSpeedInHz(2 * STEPS_PER_ROTATION);
  //steppers[2]->move(10 * STEPS_PER_MM_Z);

  if (send2motors(0, 0, 0, 0))
  {
    Serial.println("Target position reached");
  }
}


void loop()
{

}


// --- SIMULTANEOUS HOMING FUNCTION ---
void runSimultaneousHoming()
{
  Serial.print("Homing speed set to: "); Serial.println(HOMING_SPEED);
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

// Moves the arm to a target in cylindrical coordinates:
// z: Height in mm
// r: Extension radius in mm
// theta: Planar angle in degrees
bool moveToCylindrical(float target_z, float target_r, float target_theta_deg, int elbow_mode = 0)
{
  // Safety Check: Is the target within physical limits?
  if (target_z < MIN_HEIGHT || target_z > MAX_HEIGHT) {
    Serial.println("Error: Target Z is out of bounds!");
    return false;
  }
  if (target_r < MIN_REACH || target_r > MAX_REACH) {
    Serial.println("Error: Target Radius is unreachable!");
    return false;
  }
  // TODO: This limitation can be extended by operating the L1 and L2 joints
  if (target_theta_deg < MIN_PLANAR || target_theta_deg > MAX_PLANAR) {
    Serial.println("Error: Target Planar angle is unreachable!");
    return false;
  }

  // Inverse Kinematics (law of cosines)
  // The angle for L2 (elbow)
  float cos_psiX = (sq(target_r) - sq(L1) - sq(L2)) / (2.0 * L1 * L2);
  float psiX = acos(cos_psiX);
  float thetaX_deg = (PI - psiX) * 180 / PI;

  // The angle of L1 (Y-axis / Shoulder) relative to the radial line (goal)
  float cos_thetaY = (sq(L2) - sq(target_r) - sq(L1)) / (2.0 * L1 * target_r);
  float thetaY_deg = acos(cos_thetaY) * 180.0 / PI;

  // TODO: consider posible shoulder-elbow positons. For the most points both positions would work,
  // but for some points (near the operating edge) - only one position might be valid.
  // Regardless the case, thetaX and thetaY should always have the opposite signs:
  thetaY_deg = - thetaY_deg;  // by the default the elbow is fasing RIGHT

  // Apply Angular Compensation for the SCARA Offset
  // To keep the tip at 'target_theta', the base (A) must rotate LESS (or more)
  // because the shoulder (Y) swings outward by 'thetaY_deg'.
  if (elbow_mode == 0)
  {
    // TODO: Kinematic position automatic solver
  }
  else if (elbow_mode == 1)  // force elbow to face LEFT
  {
    thetaY_deg = - thetaY_deg;
    thetaX_deg = - thetaX_deg;
  }
  float thetaA_deg = target_theta_deg + thetaY_deg; 

  // Execute the Movement
  Serial.print("Moving to -> Z: "); Serial.print(target_z);
  Serial.print("mm, R: "); Serial.print(target_r);
  Serial.print("mm, Angle: "); Serial.print(target_theta_deg); Serial.println("°");

  if (send2motors(thetaX_deg, thetaY_deg, target_z, thetaA_deg))
  {
    Serial.println("Target position reached.");
  }
  else
  {
    Serial.println("Invalid position calculation.");
  }

  
  return true;
}

bool send2motors(float target_x_deg, float target_y_deg, float target_z, float target_a_deg)
{
  float base_z = 170;  // mm, base Z position
  float base_a_deg = 164;  // deg, base A position
  float base_x_deg = 100;  // deg, base X position
  float base_y_deg = 136;  // deg, base Y position
  float height = -target_z + base_z;  // target_z is calculated up from the bottom, while the real zero - up-most position
  float planar_angle_deg = target_a_deg + base_a_deg;  // the real zero is the right-most position
  float elbow_angle_deg = target_y_deg + base_y_deg;  // the real zero is the right-most position
  float tip_angle_deg = -(target_x_deg + base_x_deg);  // the real zero is right-most position

  if (height < 0 || height > 170)
  {
    Serial.print("Invalid Z axis cmd (mm): "); Serial.println(height);
    return false;
  }
  if (planar_angle_deg < 0 || planar_angle_deg > 300)
  {
    Serial.print("Invalid A axis cmd (deg): "); Serial.println(planar_angle_deg);
    return false;
  }
  if (elbow_angle_deg < 0 || elbow_angle_deg > 275)
  {
    Serial.print("Invalid Y axis cmd (deg): "); Serial.println(elbow_angle_deg);
    return false;
  }
  if (tip_angle_deg > 0 || tip_angle_deg < -270)
  {
    Serial.print("Invalid X axis cmd (deg): "); Serial.println(tip_angle_deg);
    return false;
  }

  // Convert Physical Units to Motor Steps
  long steps_Z = height * STEPS_PER_MM_Z;
  long steps_A = planar_angle_deg * STEPS_PER_DEG_A;
  long steps_Y = elbow_angle_deg * STEPS_PER_DEG_Y;
  long steps_X = tip_angle_deg * STEPS_PER_DEG_X;

  // Execute the Movement
  Serial.print("Motor Targets -> Z (Height): "); Serial.print(height);
  Serial.print("mm, A (Base): "); Serial.print(planar_angle_deg);
  Serial.print("°, Y (Elbow): "); Serial.print(elbow_angle_deg);
  Serial.print("°, X (Tip): "); Serial.print(tip_angle_deg); Serial.println("°");

  // Send targets to FastAccelStepper (non-blocking)
  steppers[2]->moveTo(steps_Z); // Z-axis
  steppers[3]->moveTo(steps_A); // A-axis (Base/Shouler)
  steppers[1]->moveTo(steps_Y); // Y-axis (Elbow)
  steppers[0]->moveTo(steps_X); // X-axis (Tip)

  // Wait for movement to complete (blocking)
  while (steppers[0]->isRunning() || steppers[1]->isRunning() || 
         steppers[2]->isRunning() || steppers[3]->isRunning()) {
    delay(1); // Yield to ESP32
  }
  
  return true;
}
