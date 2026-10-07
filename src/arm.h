#pragma once
#include <Arduino.h>
#include <mutex>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include "config.h"

enum class ArmFault : uint8_t {
  None,
  PcaMissing,   // PCA9685 did not answer on I2C
  I2cError,     // writes started failing while running
};

enum class CmdResult : uint8_t {
  Ok,
  BadJoint,
  BadValue,
  EStopped,
  Fault,
};

const char* toString(ArmFault f);
const char* toString(CmdResult r);

struct ArmState {
  float    cur[NUM_JOINTS];
  float    tgt[NUM_JOINTS];
  bool     enabled[NUM_JOINTS];
  float    speed;      // deg/s
  bool     estopped;
  bool     moving;
  ArmFault fault;
};

// Owns the PCA9685 and all joint state.
//
// Thread safety: the web server runs on the AsyncTCP task (core 0) while
// update() runs from loop() (core 1). Every public method takes the mutex,
// so commands may be issued from either side. I2C traffic only ever
// happens from update()/begin() on the loop task.
class Arm {
public:
  void begin();
  void update();  // call from loop() as often as possible

  CmdResult setTarget(int joint, float deg);
  CmdResult moveToPose(const float deg[NUM_JOINTS]);  // all joints arrive together
  CmdResult setSpeed(float degPerSec);

  void hold();     // decelerate to a smooth stop where we are
  void eStop();    // freeze immediately and latch; commands rejected until resume()
  void resume();

  ArmState snapshot();

private:
  struct Joint {
    float    cur = 0;      // commanded position the servo is at now
    float    tgt = 0;
    float    vel = 0;      // deg/s, signed
    float    scale = 1;    // speed/accel multiplier for synchronised moves
    bool     enabled = false;
    uint16_t lastCounts = 0;  // 0 = unknown, force a write
  };

  bool initPca();
  bool adoptRunningOutputs();  // recover pose after an ESP32-only reset
  bool readReg(uint8_t reg, uint8_t& out);
  uint8_t  expectedPrescale() const;
  uint16_t degToCounts(int j, float deg) const;
  float    countsToDeg(int j, uint16_t counts) const;

  void stepMotion(float dt);
  void writeOutputs();
  void enableNextJoint();
  void maybeSavePose();
  void loadPose();
  void freezeLocked();

  std::mutex mtx_;
  Adafruit_PWMServoDriver pca_{PCA_ADDR};
  Preferences prefs_;

  Joint    joints_[NUM_JOINTS];
  float    saved_[NUM_JOINTS] = {};
  float    speed_ = SPEED_DEFAULT_DEG_S;
  bool     estopped_ = false;
  ArmFault fault_ = ArmFault::None;
  bool     pcaReady_ = false;

  uint32_t lastStepMs_ = 0;
  uint32_t lastPcaTryMs_ = 0;
  uint32_t lastEnableMs_ = 0;
  uint32_t idleSinceMs_ = 0;
  uint8_t  i2cErrors_ = 0;
};

extern Arm arm;
