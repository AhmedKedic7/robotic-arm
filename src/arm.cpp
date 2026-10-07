#include "arm.h"
#include <Wire.h>
#include <math.h>

Arm arm;

namespace {

// Register addresses (PCA9685_MODE1 etc.) come from Adafruit_PWMServoDriver.h
constexpr uint8_t LED_FULL_BIT = 0x10;  // bit 4 of LEDn_ON_H / LEDn_OFF_H

constexpr uint16_t PWM_FULL_OFF   = 4096;  // setPWM(ch, 0, 4096) = output fully off
constexpr uint8_t  MAX_I2C_ERRORS = 10;    // consecutive failed writes before faulting
constexpr float    MAX_DT_S       = 0.1f;  // cap a step if loop() was stalled
constexpr float    MIN_SYNC_SCALE = 0.05f;
constexpr float    SNAP_DEG       = 0.05f;

const char* PREFS_NS  = "arm";
const char* PREFS_KEY = "pose";

float clampToLimits(int j, float deg) {
  return constrain(deg, JOINTS[j].minDeg, JOINTS[j].maxDeg);
}

}  // namespace

const char* toString(ArmFault f) {
  switch (f) {
    case ArmFault::None:       return "";
    case ArmFault::PcaMissing: return "PCA9685 not found on I2C";
    case ArmFault::I2cError:   return "I2C write errors - check wiring";
  }
  return "unknown";
}

const char* toString(CmdResult r) {
  switch (r) {
    case CmdResult::Ok:       return "ok";
    case CmdResult::BadJoint: return "bad joint";
    case CmdResult::BadValue: return "bad value";
    case CmdResult::EStopped: return "e-stop active";
    case CmdResult::Fault:    return "hardware fault";
  }
  return "unknown";
}

// ---------------------------------------------------
//  Setup
// ---------------------------------------------------
void Arm::begin() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  prefs_.begin(PREFS_NS, false);
  loadPose();

  std::lock_guard<std::mutex> lock(mtx_);
  lastPcaTryMs_ = millis();
  initPca();
  lastStepMs_ = millis();
  idleSinceMs_ = millis();
}

void Arm::loadPose() {
  bool fromFlash = prefs_.getBytesLength(PREFS_KEY) == sizeof(saved_) &&
                   prefs_.getBytes(PREFS_KEY, saved_, sizeof(saved_)) == sizeof(saved_);

  for (int j = 0; j < NUM_JOINTS; j++) {
    if (!fromFlash || !isfinite(saved_[j]) || saved_[j] < 0 || saved_[j] > 180) {
      saved_[j] = JOINTS[j].homeDeg;
    }
    joints_[j].cur = saved_[j];
    joints_[j].tgt = clampToLimits(j, saved_[j]);
  }
  Serial.printf("Start pose loaded from %s\n", fromFlash ? "flash" : "config (home)");
}

// Must be called with mtx_ held.
bool Arm::initPca() {
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() != 0) {
    if (fault_ != ArmFault::PcaMissing) Serial.println("ERROR: PCA9685 not responding");
    fault_ = ArmFault::PcaMissing;
    pcaReady_ = false;
    return false;
  }

  // If only the ESP32 rebooted (brownout, crash, upload) the PCA9685 is
  // still driving the servos. Read back those pulses before begin() resets
  // the chip so we know exactly where every joint is.
  bool adopted = adoptRunningOutputs();

  if (!pca_.begin()) {
    fault_ = ArmFault::PcaMissing;
    pcaReady_ = false;
    return false;
  }
  pca_.setOscillatorFrequency(PCA_OSC_FREQ);
  pca_.setPWMFreq(PWM_FREQ_HZ);

  for (int j = 0; j < NUM_JOINTS; j++) {
    Joint& jt = joints_[j];
    jt.lastCounts = 0;
    if (!jt.enabled) pca_.setPWM(JOINTS[j].channel, 0, PWM_FULL_OFF);
  }

  pcaReady_ = true;
  fault_ = ArmFault::None;
  i2cErrors_ = 0;
  lastEnableMs_ = millis();
  writeOutputs();  // re-assert adopted joints within a frame or two

  Serial.printf("PCA9685 ready (%s)\n",
                adopted ? "resumed running outputs" : "servos off, staggered start");
  return true;
}

bool Arm::readReg(uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(PCA_ADDR, (uint8_t)1) != 1) return false;
  out = Wire.read();
  return true;
}

bool Arm::adoptRunningOutputs() {
  for (auto& jt : joints_) jt.enabled = false;

  uint8_t mode1, prescale;
  if (!readReg(PCA9685_MODE1, mode1) || !readReg(PCA9685_PRESCALE, prescale)) return false;
  // Fresh power-up: chip is asleep with default prescale -> nothing to adopt.
  if ((mode1 & MODE1_SLEEP) || prescale != expectedPrescale()) return false;

  bool any = false;
  for (int j = 0; j < NUM_JOINTS; j++) {
    uint8_t base = PCA9685_LED0_ON_L + 4 * JOINTS[j].channel;
    uint8_t onL, onH, offL, offH;
    bool ok = readReg(base, onL) && readReg(base + 1, onH) &&
              readReg(base + 2, offL) && readReg(base + 3, offH);
    uint16_t on  = onL | ((onH & 0x0F) << 8);
    uint16_t off = offL | ((offH & 0x0F) << 8);

    Joint& jt = joints_[j];
    jt.vel = 0;
    jt.enabled = ok && on == 0 && !(onH & LED_FULL_BIT) && !(offH & LED_FULL_BIT) && off > 0;
    if (!jt.enabled) continue;

    float deg = countsToDeg(j, off);
    if (deg < -5 || deg > 185) {  // not a pulse we would have produced
      jt.enabled = false;
      continue;
    }
    jt.cur = constrain(deg, 0.0f, 180.0f);
    jt.tgt = clampToLimits(j, jt.cur);
    any = true;
    Serial.printf("  %s: resumed at %.1f deg\n", JOINTS[j].name, jt.cur);
  }
  return any;
}

// Same formula Adafruit_PWMServoDriver::setPWMFreq() uses.
uint8_t Arm::expectedPrescale() const {
  float val = (PCA_OSC_FREQ / (PWM_FREQ_HZ * 4096.0f)) + 0.5f - 1;
  return (uint8_t)constrain(val, 3.0f, 255.0f);
}

// Computed locally instead of pca_.writeMicroseconds(), which does an I2C
// read of the prescaler on every single call.
uint16_t Arm::degToCounts(int j, float deg) const {
  const JointConfig& c = JOINTS[j];
  deg = constrain(deg, 0.0f, 180.0f);
  float us = c.minUs + (deg / 180.0f) * (c.maxUs - c.minUs);
  float countsPerUs = PCA_OSC_FREQ / ((expectedPrescale() + 1) * 1e6f);
  return (uint16_t)constrain(lroundf(us * countsPerUs), 1L, 4095L);
}

float Arm::countsToDeg(int j, uint16_t counts) const {
  const JointConfig& c = JOINTS[j];
  float usPerCount = ((expectedPrescale() + 1) * 1e6f) / PCA_OSC_FREQ;
  float us = counts * usPerCount;
  return (us - c.minUs) * 180.0f / (c.maxUs - c.minUs);
}

// ---------------------------------------------------
//  Main loop
// ---------------------------------------------------
void Arm::update() {
  uint32_t now = millis();

  if (!pcaReady_) {
    if (now - lastPcaTryMs_ >= PCA_RETRY_MS) {
      std::lock_guard<std::mutex> lock(mtx_);
      lastPcaTryMs_ = now;
      initPca();
      lastStepMs_ = millis();
    }
    return;
  }

  if (now - lastStepMs_ < MOTION_PERIOD_MS) return;
  float dt = min((now - lastStepMs_) / 1000.0f, MAX_DT_S);
  lastStepMs_ = now;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    enableNextJoint();
    stepMotion(dt);
    writeOutputs();
  }
  maybeSavePose();
}

void Arm::enableNextJoint() {
  if (estopped_ || millis() - lastEnableMs_ < STARTUP_STAGGER_MS) return;

  for (uint8_t j : STARTUP_ORDER) {
    if (joints_[j].enabled) continue;
    joints_[j].enabled = true;
    joints_[j].lastCounts = 0;
    lastEnableMs_ = millis();
    Serial.printf("Enabled %s at %.1f deg\n", JOINTS[j].name, joints_[j].cur);
    return;  // one joint per stagger interval
  }
}

// Trapezoidal velocity profile: accelerate up to the speed limit, then
// brake so we arrive at the target with zero velocity. Handles target
// changes mid-move (including reversals) without jerks.
void Arm::stepMotion(float dt) {
  bool anyMoving = false;

  for (auto& jt : joints_) {
    if (!jt.enabled) continue;

    float rem = jt.tgt - jt.cur;
    float accel = ACCEL_DEG_S2 * jt.scale;
    float vmax = speed_ * jt.scale;

    if (fabsf(rem) < SNAP_DEG && fabsf(jt.vel) <= accel * dt) {
      jt.cur = jt.tgt;
      jt.vel = 0;
      continue;
    }
    anyMoving = true;

    float vdes = copysignf(min(vmax, sqrtf(2 * accel * fabsf(rem))), rem);
    float dv = constrain(vdes - jt.vel, -accel * dt, accel * dt);
    jt.vel += dv;

    float step = jt.vel * dt;
    if (rem != 0 && step / rem >= 1.0f) {  // would reach or pass the target
      jt.cur = jt.tgt;
      jt.vel = 0;
    } else {
      jt.cur += step;
    }
  }

  if (anyMoving) idleSinceMs_ = millis();
}

void Arm::writeOutputs() {
  bool failed = false;

  for (int j = 0; j < NUM_JOINTS; j++) {
    Joint& jt = joints_[j];
    if (!jt.enabled) continue;

    uint16_t counts = degToCounts(j, jt.cur);
    if (counts == jt.lastCounts) continue;

    if (pca_.setPWM(JOINTS[j].channel, 0, counts) == 0) {
      jt.lastCounts = counts;
    } else {
      jt.lastCounts = 0;  // retry next tick
      failed = true;
    }
  }

  if (!failed) {
    i2cErrors_ = 0;
    return;
  }
  if (++i2cErrors_ < MAX_I2C_ERRORS) return;

  // The servos keep the last pulse the chip received; stop planning motion
  // until the bus is back. initPca() will read back the real positions.
  Serial.println("ERROR: repeated I2C failures, holding and re-initialising PCA9685");
  freezeLocked();
  fault_ = ArmFault::I2cError;
  pcaReady_ = false;
  lastPcaTryMs_ = millis();
}

void Arm::maybeSavePose() {
  float pose[NUM_JOINTS];
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (estopped_ || fault_ != ArmFault::None) return;
    if (millis() - idleSinceMs_ < SAVE_AFTER_IDLE_MS) return;

    bool changed = false;
    for (int j = 0; j < NUM_JOINTS; j++) {
      if (!joints_[j].enabled) return;  // still starting up
      pose[j] = joints_[j].cur;
      if (fabsf(pose[j] - saved_[j]) > SAVE_MIN_CHANGE_DEG) changed = true;
    }
    if (!changed) return;
  }

  // Flash write takes a few ms — done outside the lock.
  if (prefs_.putBytes(PREFS_KEY, pose, sizeof(pose)) == sizeof(pose)) {
    memcpy(saved_, pose, sizeof(pose));
    Serial.println("Pose saved");
  } else {
    Serial.println("WARN: failed to save pose");
  }
}

void Arm::freezeLocked() {
  for (auto& jt : joints_) {
    jt.tgt = jt.cur;
    jt.vel = 0;
  }
}

// ---------------------------------------------------
//  Commands (any task)
// ---------------------------------------------------
CmdResult Arm::setTarget(int joint, float deg) {
  if (joint < 0 || joint >= NUM_JOINTS) return CmdResult::BadJoint;
  if (!isfinite(deg)) return CmdResult::BadValue;

  std::lock_guard<std::mutex> lock(mtx_);
  if (estopped_) return CmdResult::EStopped;
  if (fault_ != ArmFault::None) return CmdResult::Fault;

  joints_[joint].tgt = clampToLimits(joint, deg);
  joints_[joint].scale = 1;
  return CmdResult::Ok;
}

CmdResult Arm::moveToPose(const float deg[NUM_JOINTS]) {
  for (int j = 0; j < NUM_JOINTS; j++) {
    if (!isfinite(deg[j])) return CmdResult::BadValue;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  if (estopped_) return CmdResult::EStopped;
  if (fault_ != ArmFault::None) return CmdResult::Fault;

  float dist[NUM_JOINTS];
  float maxDist = 0;
  for (int j = 0; j < NUM_JOINTS; j++) {
    joints_[j].tgt = clampToLimits(j, deg[j]);
    dist[j] = fabsf(joints_[j].tgt - joints_[j].cur);
    maxDist = max(maxDist, dist[j]);
  }
  // Scale each joint's speed and acceleration by its share of the longest
  // move so every joint starts and finishes at the same time.
  for (int j = 0; j < NUM_JOINTS; j++) {
    joints_[j].scale = maxDist > 0 ? max(dist[j] / maxDist, MIN_SYNC_SCALE) : 1;
  }
  return CmdResult::Ok;
}

CmdResult Arm::setSpeed(float degPerSec) {
  if (!isfinite(degPerSec)) return CmdResult::BadValue;
  std::lock_guard<std::mutex> lock(mtx_);
  speed_ = constrain(degPerSec, SPEED_MIN_DEG_S, SPEED_MAX_DEG_S);
  return CmdResult::Ok;
}

void Arm::hold() {
  std::lock_guard<std::mutex> lock(mtx_);
  for (int j = 0; j < NUM_JOINTS; j++) {
    Joint& jt = joints_[j];
    float accel = ACCEL_DEG_S2 * jt.scale;
    float stopDist = jt.vel * jt.vel / (2 * accel);
    jt.tgt = clampToLimits(j, jt.cur + copysignf(stopDist, jt.vel));
  }
}

void Arm::eStop() {
  std::lock_guard<std::mutex> lock(mtx_);
  freezeLocked();
  estopped_ = true;
  Serial.println("E-STOP");
}

void Arm::resume() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!estopped_) return;
  estopped_ = false;
  lastEnableMs_ = millis();
  Serial.println("Resumed");
}

ArmState Arm::snapshot() {
  std::lock_guard<std::mutex> lock(mtx_);
  ArmState s;
  s.moving = false;
  for (int j = 0; j < NUM_JOINTS; j++) {
    s.cur[j] = joints_[j].cur;
    s.tgt[j] = joints_[j].tgt;
    s.enabled[j] = joints_[j].enabled;
    if (joints_[j].cur != joints_[j].tgt || joints_[j].vel != 0) s.moving = true;
  }
  s.speed = speed_;
  s.estopped = estopped_;
  s.fault = fault_;
  return s;
}
