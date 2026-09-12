#include "RobotNet.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

static AsyncWebServer server(80);

RobotNet::RobotNet() {}

void RobotNet::registerCallbacks(CylindricalMoveCallback cylCb, 
                                 MotorMoveCallback motorCb,
                                 StepsMoveCallback stepsCb,
                                 StatusCallback statusCb,
                                 GetStepsCallback getStepsCb,
                                 GetAnglesCallback getAnglesCb,
                                 GetCylinderCallback getCylinderCb,
                                 SphericalMoveCallback sphCb) {
  onCylindricalMove = cylCb;
  onMotorMove = motorCb;
  onStepsMove = stepsCb;
  onGetStatus = statusCb;
  onGetSteps = getStepsCb;
  onGetAngles = getAnglesCb;
  onGetCylinder = getCylinderCb;
  onSphericalMove = sphCb;
}

void RobotNet::begin(const char* ssid, const char* password) {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected successfully!");
  Serial.print("ESP32-S3 IP Address: ");
  Serial.println(WiFi.localIP());

  setupRoutes();
  server.begin();
  Serial.println("Async HTTP Server started.");
}

void RobotNet::setupRoutes() {
  // --- Route: Cylindrical Move ---
  server.on("/move/cylindrical", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, (char*)data);

      if (err) {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
        return;
      }

      if (!doc.containsKey("z") || !doc.containsKey("r") || !doc.containsKey("theta")) {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters (z, r, theta)\"}");
        return;
      }

      float z = doc["z"];
      float r = doc["r"];
      float theta = doc["theta"];
      int elbow = doc.containsKey("elbow") ? doc["elbow"].as<int>() : 0;

      if (onCylindricalMove && onCylindricalMove(z, r, theta, elbow)) {
        request->send(200, "application/json", "{\"status\":\"accepted\"}");
      } else {
        request->send(422, "application/json", "{\"status\":\"error\", \"message\":\"Robot busy or limit exceeded\"}");
      }
    }
  );

  // --- Route: Spherical Move ---
  server.on("/move/spherical", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, (char*)data);

      if (err) {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
        return;
      }

      if (!doc.containsKey("radius") || !doc.containsKey("azimuth") || !doc.containsKey("polar")) {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters (radius, azimuth, polar)\"}");
        return;
      }

      float r = doc["radius"];
      float theta = doc["azimuth"];
      float fi = doc["polar"];
      int elbow = doc.containsKey("elbow") ? doc["elbow"].as<int>() : 0;

      if (onSphericalMove && onSphericalMove(r, theta, fi, elbow)) {
        request->send(200, "application/json", "{\"status\":\"accepted\"}");
      } else {
        request->send(422, "application/json", "{\"status\":\"error\", \"message\":\"Robot busy or limit exceeded\"}");
      }
    }
  );

  // --- Route: Angles/MM Move ---
  server.on("/move/motors", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, (char*)data);

      if (err) {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
        return;
      }

      float x = doc["x"] | 0.0;
      float y = doc["y"] | 0.0;
      float z = doc["z"] | 0.0;
      float a = doc["a"] | 0.0;

      if (onMotorMove && onMotorMove(x, y, z, a)) {
        request->send(200, "application/json", "{\"status\":\"accepted\"}");
      } else {
        request->send(422, "application/json", "{\"status\":\"error\", \"message\":\"Robot busy or limit exceeded\"}");
      }
    }
  );

  // --- NEW Route: Direct Step Counts ---
  // Payload example: {"x": -1000, "y": 3000, "z": 12000, "a": 5000}
  server.on("/move/steps", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, (char*)data);

      if (err) {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
        return;
      }

      if (!doc.containsKey("x") || !doc.containsKey("y") || !doc.containsKey("z") || !doc.containsKey("a")) {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters (x, y, z, a)\"}");
        return;
      }

      long x = doc["x"];
      long y = doc["y"];
      long z = doc["z"];
      long a = doc["a"];

      if (onStepsMove && onStepsMove(x, y, z, a)) {
        request->send(200, "application/json", "{\"status\":\"accepted\"}");
      } else {
        request->send(422, "application/json", "{\"status\":\"error\", \"message\":\"Robot busy or step limits exceeded\"}");
      }
    }
  );

  server.on("/position/steps", HTTP_GET, [this](AsyncWebServerRequest *request) {
    long x = 0, y = 0, z = 0, a = 0;
    if (onGetSteps) {
      onGetSteps(x, y, z, a);
    }

    StaticJsonDocument<256> res;
    res["x"] = x;
    res["y"] = y;
    res["z"] = z;
    res["a"] = a;

    String response;
    serializeJson(res, response);
    request->send(200, "application/json", response);
  });

  server.on("/position/angles", HTTP_GET, [this](AsyncWebServerRequest *request) {
    float x = 0, y = 0, z = 0, a = 0;
    if (onGetAngles) {
      onGetAngles(x, y, z, a);
    }

    StaticJsonDocument<256> res;
    res["x"] = x;
    res["y"] = y;
    res["z"] = z;
    res["a"] = a;

    String response;
    serializeJson(res, response);
    request->send(200, "application/json", response);
  });

  server.on("/position/cylinder", HTTP_GET, [this](AsyncWebServerRequest *request) {
    float height = 0, reach = 0, theta = 0;
    if (onGetCylinder) {
      onGetCylinder(height, reach, theta);
    }

    StaticJsonDocument<256> res;
    res["height"] = height;
    res["reach"] = reach;
    res["theta"] = theta;

    String response;
    serializeJson(res, response);
    request->send(200, "application/json", response);
  });

  // --- Route: Status Request ---
  server.on("/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
    long z = 0, a = 0, y = 0, x = 0;
    bool isBusy = false;

    if (onGetStatus) {
      onGetStatus(z, a, y, x, isBusy);
    }

    StaticJsonDocument<256> res;
    res["status"] = isBusy ? "moving" : "idle";
    res["motor_z"] = z;
    res["motor_a"] = a;
    res["motor_y"] = y;
    res["motor_x"] = x;

    String response;
    serializeJson(res, response);
    request->send(200, "application/json", response);
  });
}
