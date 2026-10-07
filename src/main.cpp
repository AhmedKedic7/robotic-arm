#include <Arduino.h>

// =============================================
//  6 DOF Robotic Arm — ESP32 Web Controller
//  v5 — WebSocket control, soft limits, e-stop,
//       staggered start, pose persistence
// =============================================
//
//  Hardware, calibration and tuning:  include/config.h
//  Wi-Fi credentials:                 include/secrets.h
//
//  Open http://robotarm.local (or http://192.168.4.1)
//  after joining the access point.

#include "arm.h"
#include "web.h"

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Robotic Arm v5 ===");

  arm.begin();
  webBegin();

  Serial.println("Ready");
}

void loop() {
  arm.update();
  webUpdate();
}
