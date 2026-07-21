#include <Arduino.h>
#include "FastAccelStepper.h"
#include <TMCStepper.h>

// --- Pins ---
//#define dirPinStepper 11
#define enablePinStepper 8
//#define stepPinStepper 9
#define TX_PIN 13      // Connect to TMC UART via 1k resistor
#define RX_PIN 14      // Connect directly to TMC UART

#define DRIVER_ADDRESS_X 0b00
#define DRIVER_ADDRESS_Y 0b01
#define DRIVER_ADDRESS_Z 0b10
#define DRIVER_ADDRESS_A 0b11

const int NUM_MOTORS = 4;
FastAccelStepper* steppers[NUM_MOTORS];
const int homingDirs[NUM_MOTORS] = {1, -1, -1, -1}; // Direction for each axis
bool homed[NUM_MOTORS] = {false, false, false, false};

//                                       X  Y   Z  A
const uint8_t pin_DIR[NUM_MOTORS] =   { 15, 6, 4, 19 };
const uint8_t pin_STEP[NUM_MOTORS] =  { 16, 7, 5, 20 };
const uint8_t pin_LIMIT[NUM_MOTORS] = { 39, 40, 41, 37 };


// --- TMC2209 Settings ---
#define R_SENSE 0.11f        // Standard for most TMC2209 drivers

const int BACKOFF_STEPS = 1000;  // Steps to back away from switch after hitting it
const long MAX_TRAVEL[NUM_MOTORS] = {0, 0, 135000, 0};  // Maximum allowed steps from the switch

// Create TMC2209 Driver Instance using Hardware Serial 1
//TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper drivers[] = {
  TMC2209Stepper(&Serial1, R_SENSE, DRIVER_ADDRESS_X),
  TMC2209Stepper(&Serial1, R_SENSE, DRIVER_ADDRESS_Y),
  TMC2209Stepper(&Serial1, R_SENSE, DRIVER_ADDRESS_Z),
  TMC2209Stepper(&Serial1, R_SENSE, DRIVER_ADDRESS_A)
};


// --- FastAccelStepper Setup ---
FastAccelStepperEngine engine = FastAccelStepperEngine();

//FastAccelStepper *steppers[NUM_MOTORS]; 

const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPS = 4; // Set to 1, 2, 4, 8, 16, 32, 64, 128, or 256!
const int STEPS_PER_ROTATION = MOTOR_STEPS_PER_REV * MICROSTEPS;

const int HOMING_SPEED = 2 * STEPS_PER_ROTATION;  // Slow, safe speed for finding the switch

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Robot arm code");
  //pinMode(2, OUTPUT);
  //digitalWrite(2, HIGH);
  
  // Initialize Hardware Serial 1 for TMC2209 UART
  // ESP32-S3 allows mapping Serial1 to any pins. We use GPIO 18 for RX and TX.
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  //Serial1.begin(115200, SERIAL_8N1, 21, 22);
  //pinMode(RX_PIN, OUTPUT);
  //pinMode(TX_PIN, OUTPUT);
  pinMode(pin_LIMIT[2], INPUT_PULLUP);
  //digitalWrite(TX_PIN, HIGH);
  //digitalWrite(RX_PIN, HIGH);

  // --- Configure TMC2209 over UART ---
  drivers[2].begin();
  drivers[2].toff(4);                 // Enable driver
  drivers[2].rms_current(1400);        // Set motor current to 800mA (Adjust for your NEMA 17)
  drivers[2].microsteps(4);  // <-- SET MICROSTEPS HERE IN SOFTWARE!
  drivers[2].en_spreadCycle(false);   // false = StealthChop (silent), true = SpreadCycle (torque)
  drivers[2].pwm_autoscale(true);
  drivers[2].shaft(false); // Or true, but keep it consistent

  drivers[1].begin();
  drivers[1].toff(4);                 // Enable driver
  drivers[1].rms_current(1600);        // Set motor current to 800mA (Adjust for your NEMA 17)
  drivers[1].microsteps(4);  // <-- SET MICROSTEPS HERE IN SOFTWARE!
  drivers[1].en_spreadCycle(false);   // false = StealthChop (silent), true = SpreadCycle (torque)
  drivers[1].pwm_autoscale(true);
  drivers[1].shaft(false); // Or true, but keep it consistent

  drivers[0].begin();
  drivers[0].toff(4);                 // Enable driver
  drivers[0].rms_current(1600);        // Set motor current to 800mA (Adjust for your NEMA 17)
  drivers[0].microsteps(4);  // <-- SET MICROSTEPS HERE IN SOFTWARE!
  drivers[0].en_spreadCycle(false);   // false = StealthChop (silent), true = SpreadCycle (torque)
  drivers[0].pwm_autoscale(true);
  drivers[0].shaft(false); // Or true, but keep it consistent

  drivers[3].begin();
  drivers[3].toff(4);                 // Enable driver
  drivers[3].rms_current(1600);        // Set motor current to 800mA (Adjust for your NEMA 17)
  drivers[3].microsteps(4);  // <-- SET MICROSTEPS HERE IN SOFTWARE!
  drivers[3].en_spreadCycle(false);   // false = StealthChop (silent), true = SpreadCycle (torque)
  drivers[3].pwm_autoscale(true);
  drivers[3].shaft(false); // Or true, but keep it consistent

  Serial.print("Driver 0 current: ");
  Serial.println(drivers[0].rms_current());
  Serial.print("Driver 0 microsteps: ");
  Serial.println(drivers[0].microsteps());

  Serial.print("Driver 2 current: ");
  Serial.println(drivers[2].rms_current());
  Serial.print("Driver 2 microsteps: ");
  Serial.println(drivers[2].microsteps());

  for(int i = 0; i < NUM_MOTORS; i++) {
    uint8_t result = drivers[i].test_connection();
    Serial.print("Driver ");
    Serial.print(i);
    if (result == 0) {
      Serial.println(" -> UART Connection OK!");
    } else {
      Serial.print(" -> UART Connection FAILED! Error code: ");
      Serial.println(result);
    }
  }
  

  // --- Configure FastAccelStepper ---
  engine.init();
  steppers[2] = engine.stepperConnectToPin(pin_STEP[2]);
  steppers[0] = engine.stepperConnectToPin(pin_STEP[0]);
  steppers[1] = engine.stepperConnectToPin(pin_STEP[1]);
  steppers[3] = engine.stepperConnectToPin(pin_STEP[3]);

  if (steppers[2]) {
    steppers[2]->setDelayToEnable(50); // 50ms delay after enabling
    //steppers[2]->setDelayToDirection(50); // 50ms delay after changing direction
    steppers[2]->setDirectionPin(pin_DIR[2]);
    steppers[2]->setEnablePin(enablePinStepper, true); // Active LOW
    steppers[2]->enableOutputs();

    // Speed: 5 rotations per second
    uint32_t speed_hz = 16 * STEPS_PER_ROTATION;
    //uint32_t speed_hz = 6000;
    steppers[2]->setSpeedInHz(speed_hz);       
    steppers[2]->setAcceleration(speed_hz * 4); 
    
    Serial.println("TMC2209 and Stepper Z Initialized!");
    Serial.print("Microsteps set to: ");
    Serial.println(drivers[2].microsteps());
  }
  else
  {
    Serial.println("error Z");
  }

  if (steppers[0]) {
    steppers[0]->setDelayToEnable(50); // 50ms delay after enabling
    //steppers[2]->setDelayToDirection(50); // 50ms delay after changing direction
    steppers[0]->setDirectionPin(pin_DIR[0]);
    steppers[0]->setEnablePin(enablePinStepper, true); // Active LOW
    steppers[0]->enableOutputs();

    // Speed: 5 rotations per second
    uint32_t speed_hz = 4 * STEPS_PER_ROTATION;
    //uint32_t speed_hz = 6000;
    steppers[0]->setSpeedInHz(speed_hz);       
    steppers[0]->setAcceleration(speed_hz * 4); 
    
    Serial.println("TMC2209 and Stepper X Initialized!");
    Serial.print("Microsteps set to: ");
    Serial.println(drivers[0].microsteps());
  }
  else
  {
    Serial.println("error X");
  }

  if (steppers[1]) {
    steppers[1]->setDelayToEnable(50); // 50ms delay after enabling
    //steppers[2]->setDelayToDirection(50); // 50ms delay after changing direction
    steppers[1]->setDirectionPin(pin_DIR[1]);
    steppers[1]->setEnablePin(enablePinStepper, true); // Active LOW
    steppers[1]->enableOutputs();

    // Speed: 5 rotations per second
    uint32_t speed_hz = 16 * STEPS_PER_ROTATION;
    //uint32_t speed_hz = 6000;
    steppers[1]->setSpeedInHz(speed_hz);       
    steppers[1]->setAcceleration(speed_hz * 4); 
    
    Serial.println("TMC2209 and Stepper Y Initialized!");
    Serial.print("Microsteps set to: ");
    Serial.println(drivers[1].microsteps());
  }
  else
  {
    Serial.println("error Y");
  }

  if (steppers[3]) {
    steppers[3]->setDelayToEnable(50); // 50ms delay after enabling
    //steppers[2]->setDelayToDirection(50); // 50ms delay after changing direction
    steppers[3]->setDirectionPin(pin_DIR[3]);
    steppers[3]->setEnablePin(enablePinStepper, true); // Active LOW
    steppers[3]->enableOutputs();

    // Speed: 5 rotations per second
    uint32_t speed_hz = 16 * STEPS_PER_ROTATION;
    //uint32_t speed_hz = 6000;
    steppers[3]->setSpeedInHz(speed_hz);       
    steppers[3]->setAcceleration(speed_hz * 4); 
    
    Serial.println("TMC2209 and Stepper A Initialized!");
    Serial.print("Microsteps set to: ");
    Serial.println(drivers[3].microsteps());
  }
  else
  {
    Serial.println("error A");
  }

  runHomingSequence_Z();
  //steppers[2]->setSpeedInHz(2 * STEPS_PER_ROTATION);
  //steppers[2]->moveTo(20000);

  Serial.println("Z axis is done!");

  runHomingSequence_X();

  Serial.println("X axis is done!");

  runHomingSequence_Y();

  Serial.println("Y axis is done!");

  runHomingSequence_A();

  //steppers[0]->setSpeedInHz(2 * STEPS_PER_ROTATION);
  //steppers[0]->moveTo(-3100);
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
