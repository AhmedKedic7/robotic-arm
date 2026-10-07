#pragma once
#include <Arduino.h>

// =============================================
//  6 DOF Robotic Arm — hardware & tuning config
// =============================================
//
//  WIRING:
//  PCA9685 SDA  -> ESP32 GPIO 21
//  PCA9685 SCL  -> ESP32 GPIO 22
//  PCA9685 VCC  -> ESP32 3.3V
//  PCA9685 V+   -> PSU 5-6V (size it for stall current: ~2.5 A per
//                  MG996R-class servo, so 10 A+ for six of them)
//  PCA9685 GND  -> PSU GND + ESP32 GND (common ground is mandatory)
//  Bulk cap     -> 1000 uF+ electrolytic across V+ / GND at the PCA9685
//  Servos       -> PCA9685 channels listed in JOINTS below
//
// =============================================

// ---- I2C / PCA9685 ---------------------------
constexpr uint8_t  I2C_SDA_PIN  = 21;
constexpr uint8_t  I2C_SCL_PIN  = 22;
constexpr uint8_t  PCA_ADDR     = 0x40;
constexpr float    PWM_FREQ_HZ  = 50.0f;

// The PCA9685 internal oscillator is nominally 25 MHz but real chips run
// anywhere from ~23 to ~27 MHz. Every pulse width scales with this error.
// Calibrate: set a channel to 1500 us, measure the real pulse width with a
// scope / logic analyser, then OSC = 27000000 * 1500 / measured_us.
constexpr uint32_t PCA_OSC_FREQ = 27000000;

// ---- Joints ----------------------------------
//
//  minUs / maxUs : pulse width at servo angle 0 and 180 deg.
//  minDeg/maxDeg : SOFT LIMITS — the arm will never be commanded outside
//                  these. Tighten them so joints can't hit the frame, each
//                  other, or (for the gripper) stall against a closed jaw.
//  homeDeg       : pose used when nothing has been saved yet.
//
//  The defaults reproduce the old firmware's 150..600 tick range.
//  CALIBRATE EVERY JOINT before trusting the limits.
struct JointConfig {
  const char* name;
  uint8_t     channel;
  float       minDeg;
  float       maxDeg;
  float       homeDeg;
  uint16_t    minUs;
  uint16_t    maxUs;
};

constexpr JointConfig JOINTS[] = {
  // name           ch  minDeg maxDeg home  minUs maxUs
  { "Base",         0,  0,     180,   90,   730,  2930 },
  { "Shoulder",     1,  0,     180,   90,   730,  2930 },
  { "Elbow",        2,  0,     180,   90,   730,  2930 },
  { "Wrist Pitch",  3,  0,     180,   90,   730,  2930 },
  { "Wrist Roll",   4,  0,     180,   90,   730,  2930 },
  { "Gripper",      5,  0,     180,   90,   730,  2930 },
};
constexpr int NUM_JOINTS = sizeof(JOINTS) / sizeof(JOINTS[0]);

// Order in which servos are powered at boot, one at a time, to limit
// inrush current. Light / distal joints first, base last.
constexpr uint8_t STARTUP_ORDER[NUM_JOINTS] = { 5, 4, 3, 2, 1, 0 };
constexpr uint32_t STARTUP_STAGGER_MS = 400;

// ---- Named poses (sent from the UI by name) --
struct PoseConfig {
  const char* name;
  float       deg[NUM_JOINTS];
};

constexpr PoseConfig POSES[] = {
  { "centre", { 90, 90,  90, 90, 90, 90 } },
  { "park",   { 90, 45, 135, 90, 90,  0 } },
};
constexpr int NUM_POSES = sizeof(POSES) / sizeof(POSES[0]);

// ---- Motion ----------------------------------
constexpr uint32_t MOTION_PERIOD_MS   = 20;      // control loop rate (matches 50 Hz PWM)
constexpr float    ACCEL_DEG_S2       = 400.0f;  // ramp up / down rate
constexpr float    SPEED_MIN_DEG_S    = 5.0f;
constexpr float    SPEED_MAX_DEG_S    = 300.0f;
constexpr float    SPEED_DEFAULT_DEG_S = 50.0f;

// ---- Persistence -----------------------------
// The last resting pose is saved to flash so the next boot starts from
// where the arm actually is (as long as nobody moved it by hand).
constexpr uint32_t SAVE_AFTER_IDLE_MS = 2000;
constexpr float    SAVE_MIN_CHANGE_DEG = 0.5f;

// ---- Safety ----------------------------------
// If the client that last commanded motion goes silent this long
// (closed tab, Wi-Fi dropped), the arm holds where it is.
constexpr uint32_t DEADMAN_TIMEOUT_MS = 1500;
constexpr uint32_t PCA_RETRY_MS       = 2000;

// ---- Network ---------------------------------
constexpr uint8_t  MAX_WS_CLIENTS     = 4;
constexpr uint32_t BROADCAST_MS       = 100;
constexpr const char* MDNS_HOSTNAME   = "robotarm";  // http://robotarm.local
