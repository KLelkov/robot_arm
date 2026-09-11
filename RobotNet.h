#ifndef ROBOT_NET_H
#define ROBOT_NET_H

#include <Arduino.h>

typedef bool (*CylindricalMoveCallback)(float z, float r, float theta, int elbow);
typedef bool (*MotorMoveCallback)(float x, float y, float z, float a);
typedef bool (*StepsMoveCallback)(long x, long y, long z, long a);
typedef void (*StatusCallback)(long &z, long &a, long &y, long &x, bool &isBusy);

class RobotNet {
public:
  RobotNet();
  
  void registerCallbacks(CylindricalMoveCallback cylCb, 
                         MotorMoveCallback motorCb,
                         StepsMoveCallback stepsCb,
                         StatusCallback statusCb);

  void begin(const char* ssid, const char* password);

private:
  void setupRoutes();
  
  CylindricalMoveCallback onCylindricalMove = nullptr;
  MotorMoveCallback onMotorMove = nullptr;
  StepsMoveCallback onStepsMove = nullptr;
  StatusCallback onGetStatus = nullptr;
};

#endif
