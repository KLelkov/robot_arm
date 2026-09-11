#ifndef ROBOT_NET_H
#define ROBOT_NET_H

#include <Arduino.h>

typedef bool (*CylindricalMoveCallback)(float z, float r, float theta, int elbow);
typedef bool (*MotorMoveCallback)(float x, float y, float z, float a);
typedef bool (*StepsMoveCallback)(long x, long y, long z, long a);
typedef void (*StatusCallback)(long &z, long &a, long &y, long &x, bool &isBusy);
typedef void (*GetStepsCallback)(long &x, long &y, long &z, long &a);
typedef void (*GetAnglesCallback)(float &x, float &y, float &z, float &a);

class RobotNet {
public:
  RobotNet();
  
  void registerCallbacks(CylindricalMoveCallback cylCb, 
                         MotorMoveCallback motorCb,
                         StepsMoveCallback stepsCb,
                         StatusCallback statusCb,
                         GetStepsCallback getStepsCb,
                         GetAnglesCallback getAnglesCb);

  void begin(const char* ssid, const char* password);

private:
  void setupRoutes();
  
  CylindricalMoveCallback onCylindricalMove = nullptr;
  MotorMoveCallback onMotorMove = nullptr;
  StepsMoveCallback onStepsMove = nullptr;
  StatusCallback onGetStatus = nullptr;
  GetStepsCallback onGetSteps = nullptr; 
  GetAnglesCallback onGetAngles = nullptr; 
};

#endif
