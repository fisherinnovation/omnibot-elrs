/*
  Omnibot ELRS
  ============
  ExpressLRS CRSF radio control for the Crunchlabs Omnibot, replacing the
  stock binary RF board with a Radiomaster XR1 Nano ELRS receiver.

  The stock controller uses simple buttons, so motion was purely on/off.
  This firmware reads analog joystick values over CRSF and feeds them
  directly into the existing omni-directional kinematic model, giving
  smooth proportional control over all three degrees of freedom at once:
    - Right stick: translate (forward/back + strafe) in any direction
    - Left stick X: rotate in place
    - Left stick Y: forklift speed (throttle axis — non-centering ratchet
      holds the last commanded lift speed when you let go)

  Because the omni wheels decouple translation from rotation, you can drive
  in any direction while spinning, or strafe sideways while staying facing
  forward — all from a single joystick position.

  Kinematic model
  ───────────────
  Wheel speeds are derived from the commanded velocity vector {vx, vy, ω}
  using the algebraic inverse kinematics for a three-wheeled omnidirectional
  robot. The relationship is linear, so proportional stick inputs produce
  proportional wheel outputs with no special-casing needed for diagonals.

  Reference: Siradjuddin, Indrazno. "Kinematics and control a three wheeled
  omnidirectional mobile robot." Int. J. Electr. Electron. Eng 6.12 (2019).
  https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf

  Hardware
  ────────
  Microcontroller : Crunchlabs Arduino Nano (ATmega328P)
  Receiver        : RadioMaster XR1 Nano Multi-Frequency ELRS

  Wiring
  ──────
  XR1 Nano TX  ──→  Arduino pin 0 (RX)
  XR1 Nano 5V  ──→  Arduino 5V
  XR1 Nano GND ──→  Arduino GND
  Arduino pin 1 (TX) — leave unconnected

  No signal inverter needed. CRSF is standard (non-inverted) UART logic.
  Disconnect the XR1 TX wire from pin 0 before uploading firmware, then
  reconnect it for normal operation.

  One-time XR1 WebUI setup
  ────────────────────────
  The XR1 defaults to 420,000 baud, which the ATmega328P cannot reach.
  Change it once via the receiver's built-in WiFi interface:
    1. Power the XR1 and connect your phone/laptop to its WiFi hotspot
    2. Open the WebUI (check the XR1 docs for the IP address)
    3. Set UART baud rate to 115200
    4. Leave the protocol as CRSF
    5. Save and let the receiver restart

  Safety
  ──────
  The robot will not move until all four axes sit within the dead zone
  continuously for one second after power-on, binding, or signal recovery.
  This prevents the robot from lurching if the transmitter is switched on
  with sticks already displaced, or if control is regained mid-air after a
  dropout. Signal loss stops all motors within 300 ms.

  Serial / debugging
  ──────────────────
  Hardware Serial (pins 0/1) is dedicated to CRSF during normal use, so the
  Arduino serial monitor is not available while the receiver is connected.
  To use the serial monitor: disconnect the XR1 TX wire from pin 0, upload
  or open the monitor as usual, then reconnect the wire when done.

  All tuning parameters are in config.h.
*/

#include "config.h"

// ── Pin assignments ───────────────────────────────────────────────────────────
// Drive motors use DRV8833 or similar DIR+PWM interface.
// LIFT shares the same interface for the forklift motor.
#define LED      13

#define M1_DIR   4
#define M1_PWM   5
#define M2_DIR   7
#define M2_PWM   6
#define M3_DIR   8
#define M3_PWM   9
#define LIFT_DIR 2
#define LIFT_PWM 3

// ── CRSF protocol constants ───────────────────────────────────────────────────
// CRSF (Crossfire Serial Format) is the native ExpressLRS protocol: standard
// UART at a fixed baud rate, framed as [sync][length][type][payload][CRC8].
// RC channel frames carry sixteen 11-bit channel values packed into 22 bytes.
const unsigned long CRSF_BAUD        = 115200; // configured in XR1 WebUI
const byte   CRSF_SYNC_BYTE          = 0xC8;   // all receiver→FC frames start here
const byte   CRSF_FRAMETYPE_RC       = 0x16;   // RC channels packed frame type
const byte   CRSF_FRAME_SIZE_MAX     = 64;     // max total frame bytes
const byte   CRSF_CRC_POLY           = 0xD5;   // DVB-S2 CRC8 polynomial

// Raw channel value range as measured on a Radiomaster transmitter.
// The CRSF spec center is 992, but real transmitters sit slightly off;
// splitting the mapping at the measured midpoint avoids center-stick drift.
const unsigned int CRSF_CH_MIN       = 174;
const unsigned int CRSF_CH_MID       = 922;
const unsigned int CRSF_CH_MAX       = 1811;

const unsigned long CRSF_TIMEOUT_MS  = 300;   // failsafe: stop if silent this long
const unsigned long CRSF_ARM_MS      = 1000;  // sticks must be neutral this long to arm

// ── Drive state ───────────────────────────────────────────────────────────────
// Velocity vector fed into the kinematic model each loop.
// vSpeed[0] = vx  (strafe):      −1.0 = full left,  +1.0 = full right
// vSpeed[1] = vy  (drive):       −1.0 = full back,  +1.0 = full forward
// vSpeed[2] = ω   (rotate):      −1.0 = full CCW,   +1.0 = full CW
float vSpeed[3] = {0, 0, 0};

// Fork motor command.  −1.0 = full down, 0 = stop, +1.0 = full up.
float moveFork = 0.0;

// ── Robot geometry ────────────────────────────────────────────────────────────
// Wheel mounting angles define where each omniwheel points relative to the
// robot centre (degrees, measured counter-clockwise from the robot's forward
// axis). The three wheels are evenly spaced 120° apart.
const double angleWheel1 = 90.0;
const double angleWheel2 = 210.0;
const double angleWheel3 = 330.0;

// localAngle rotates the effective "forward" direction so the kinematic
// equations treat the correct heading as forward. 60° aligns wheel 1
// (mounted at 90°) with the Omnibot's physical front.
const double localAngle  = 60.0;

const double botRadius   = 100.0;  // mm, wheel centre to robot centre
const double wheelRadius = 35.0;   // mm, omniwheel rolling radius

// Calculated wheel PWM outputs, kept global to avoid re-allocating each loop.
double wheelSpeeds[3];

// tune=1 gives a linear speed response. Values >1 make the response
// exponential (slow near centre, fast at the extremes). Start at 1.
const double tune = 1.0;

// ── CRSF parser state ─────────────────────────────────────────────────────────
byte crsfFrame[CRSF_FRAME_SIZE_MAX];
byte crsfFramePos    = 0;  // write cursor into crsfFrame[]
byte crsfExpectedLen = 0;  // payload+type+CRC byte count from the length field

// ── Arm / failsafe state ──────────────────────────────────────────────────────
unsigned long lastCrsfFrameMs  = 0;     // timestamp of the last valid RC frame
unsigned long neutralStartedMs = 0;     // when sticks first entered the dead zone
bool          crsfArmed        = false; // true once the neutral-hold check passes



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup() {
  Serial.begin(CRSF_BAUD);

  pinMode(LED, OUTPUT);

  pinMode(M1_DIR, OUTPUT);   pinMode(M1_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);   pinMode(M2_PWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);   pinMode(M3_PWM, OUTPUT);
  pinMode(LIFT_DIR, OUTPUT); pinMode(LIFT_PWM, OUTPUT);
}



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void loop() {
  getDirection();     // consume any waiting CRSF bytes; update vSpeed on complete frames
  checkCrsfTimeout(); // zero vSpeed if frames stop arriving
  moveBot();          // translate vSpeed into motor PWM
  updateLed();
}

void updateLed() {
  unsigned long now = millis();
  if (lastCrsfFrameMs == 0) {
    // No signal — fast blink (200 ms)
    digitalWrite(LED, (now / 200) % 2);
  } else if (!crsfArmed) {
    // Signal but not armed — slow blink (800 ms)
    digitalWrite(LED, (now / 800) % 2);
  } else {
    // Armed — solid on
    digitalWrite(LED, HIGH);
  }
}



// >>>>>>>>>>>>>>>>>>>>> CRSF RECEPTION & PARSING <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Read every byte that has arrived since the last loop and hand each one to
// the frame state machine. Returning quickly keeps motor PWM updates smooth.
void getDirection() {
  while (Serial.available() > 0) {
    parseCrsfByte(Serial.read());
  }
}

// Failsafe: if valid RC frames stop arriving the transmitter is off, out of
// range, or the link was lost. Disarm and zero all outputs so the robot stops.
// The arm check must pass again before motion is allowed after recovery.
void checkCrsfTimeout() {
  if (lastCrsfFrameMs == 0) return;
  if (millis() - lastCrsfFrameMs > CRSF_TIMEOUT_MS) {
    lastCrsfFrameMs  = 0;
    crsfArmed        = false;
    neutralStartedMs = 0;
    vSpeed[0] = 0;  vSpeed[1] = 0;  vSpeed[2] = 0;
    moveFork  = 0;
  }
}

// Non-blocking CRSF frame parser. Called once per received byte.
// State is tracked via crsfFramePos so the main loop is never blocked
// waiting for a complete frame to arrive.
//
// Frame layout:
//   byte 0       : sync (0xC8)
//   byte 1       : length (number of bytes that follow, including type and CRC)
//   byte 2       : frame type
//   bytes 3..N-1 : payload
//   byte N       : CRC8 over bytes 2..N-1
void parseCrsfByte(byte value) {
  if (crsfFramePos == 0) {
    if (value != CRSF_SYNC_BYTE) return;  // wait for frame start
    crsfFrame[crsfFramePos++] = value;
    return;
  }
  if (crsfFramePos == 1) {
    if (value < 2 || value > CRSF_FRAME_SIZE_MAX - 2) {
      crsfFramePos = 0;  // implausible length; resync
      return;
    }
    crsfExpectedLen = value;
    crsfFrame[crsfFramePos++] = value;
    return;
  }
  crsfFrame[crsfFramePos++] = value;
  if (crsfFramePos >= crsfExpectedLen + 2) {
    handleCrsfFrame();
    crsfFramePos    = 0;
    crsfExpectedLen = 0;
  }
}

// Called once a complete frame has been buffered. Validates the CRC, unpacks
// channel data, runs the arm-state check, then updates vSpeed and moveFork.
void handleCrsfFrame() {
  if (crsfFrame[2] != CRSF_FRAMETYPE_RC) return;  // ignore non-RC frames (telemetry, etc.)
  if (!crsfFrameValid()) return;

  unsigned int channels[16];
  unpackCrsfChannels(crsfFrame + 3, channels);

  float s = crsfToFloat(channels[CH_STRAFE], INV_STRAFE, CRSF_DEADBAND);
  float d = crsfToFloat(channels[CH_DRIVE],  INV_DRIVE,  CRSF_DEADBAND);
  float r = crsfToFloat(channels[CH_ROTATE], INV_ROTATE, CRSF_DEADBAND);
  float f = crsfToFloat(channels[CH_FORK],   INV_FORK,   FORK_DEADBAND);

  lastCrsfFrameMs = millis();

  if (!updateArmState(s, d, r, f)) {
    vSpeed[0] = 0;  vSpeed[1] = 0;  vSpeed[2] = 0;
    moveFork  = 0;
    return;
  }

  vSpeed[0] = s;
  vSpeed[1] = d;
  vSpeed[2] = r;
  moveFork  = f;
}

// Verify the frame CRC. Covers the frame type and payload bytes (everything
// between the length field and the final CRC byte).
bool crsfFrameValid() {
  byte crc = 0;
  for (byte i = 2; i < crsfFramePos - 1; i++) {
    crc = crc8DvbS2(crc, crsfFrame[i]);
  }
  return crc == crsfFrame[crsfFramePos - 1];
}

// DVB-S2 CRC8 used by the CRSF protocol.
byte crc8DvbS2(byte crc, byte value) {
  crc ^= value;
  for (byte bit = 0; bit < 8; bit++) {
    crc = (crc & 0x80) ? (crc << 1) ^ CRSF_CRC_POLY : (crc << 1);
  }
  return crc;
}

// Unpack sixteen 11-bit channel values from the 22-byte RC_CHANNELS_PACKED
// payload. Values are stored little-endian, back-to-back, with no padding,
// so each channel straddles byte boundaries and must be extracted with a
// rolling bit buffer.
void unpackCrsfChannels(byte *payload, unsigned int *channels) {
  unsigned long buf  = 0;
  byte          bits = 0;
  byte          pi   = 0;
  for (byte ch = 0; ch < 16; ch++) {
    while (bits < 11) {
      buf  |= ((unsigned long)payload[pi++]) << bits;
      bits += 8;
    }
    channels[ch]  = buf & 0x07FF;
    buf  >>= 11;
    bits  -= 11;
  }
}

// Map a raw CRSF channel value to the −1.0 .. +1.0 range used by the
// kinematic model, with dead zone applied.
//
// The mapping is split at CRSF_CH_MID rather than the arithmetic centre so
// that small factory calibration offsets on either side don't cause the
// output to drift from zero when the stick is released.
float crsfToFloat(unsigned int raw, bool invert, int deadband) {
  long val = constrain((long)raw, CRSF_CH_MIN, CRSF_CH_MAX);
  float out;
  if (val < CRSF_CH_MID) {
    out = (float)map(val, CRSF_CH_MIN, CRSF_CH_MID, -1000, 0) / 1000.0f;
  } else {
    out = (float)map(val, CRSF_CH_MID, CRSF_CH_MAX, 0, 1000) / 1000.0f;
  }
  if (abs(out) * 1000.0f < (float)deadband) return 0.0f;
  return invert ? -out : out;
}

// Arm-state machine. The robot will not move until all axes report zero
// (within the dead zone) for CRSF_ARM_MS milliseconds continuously.
// Any non-zero input resets the timer, and arming resets on signal loss,
// so the neutral-hold requirement must be met again after every dropout.
bool updateArmState(float s, float d, float r, float f) {
  unsigned long now     = millis();
  bool          neutral = (s == 0.0f && d == 0.0f && r == 0.0f && f == 0.0f);

  if (!neutral) {
    neutralStartedMs = 0;
    return crsfArmed;
  }
  if (crsfArmed) return true;
  if (neutralStartedMs == 0) { neutralStartedMs = now; return false; }
  if (now - neutralStartedMs >= CRSF_ARM_MS) { crsfArmed = true; return true; }
  return false;
}



// >>>>>>>>>>>>>>>>>>>>>>>> ROBOT ACTION FUNCTIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void moveBot() {
  float ws[3];
  getWheelSpeeds(ws);
  driveWheels(ws, maxSpeed);
  driveLift();
}

// Convert kinematic wheel speed outputs to motor PWM signals.
//
// Speeds from getWheelSpeeds() are normalised ratios: the fastest wheel
// always reaches maxSpeed while slower wheels are scaled proportionally.
// This keeps the commanded direction accurate at any overall speed without
// saturating any one motor.
void driveWheels(float *speeds, float topAllowed) {
  float topSpeed = max(max(abs(speeds[0]), abs(speeds[1])), abs(speeds[2]));

  for (int i = 0; i < 3; i++) {
    double norm = (topSpeed > 0) ? speeds[i] / topSpeed : 0.0;
    wheelSpeeds[i] = (norm < 0)
      ? -pow(abs(norm), tune) * topAllowed
      :  pow(norm,      tune) * topAllowed;
  }

  if (flipM1) wheelSpeeds[0] = -wheelSpeeds[0];
  if (flipM2) wheelSpeeds[1] = -wheelSpeeds[1];
  if (flipM3) wheelSpeeds[2] = -wheelSpeeds[2];

  digitalWrite(M1_DIR, wheelSpeeds[0] < 0 ? LOW : HIGH);
  analogWrite(M1_PWM,  int(abs(wheelSpeeds[0])));
  digitalWrite(M2_DIR, wheelSpeeds[1] < 0 ? LOW : HIGH);
  analogWrite(M2_PWM,  int(abs(wheelSpeeds[1])));
  digitalWrite(M3_DIR, wheelSpeeds[2] < 0 ? LOW : HIGH);
  analogWrite(M3_PWM,  int(abs(wheelSpeeds[2])));
}

// Drive the forklift motor at a speed proportional to stick displacement.
// moveFork = 0 stops the motor; ±1.0 runs it at full duty cycle.
void driveLift() {
  int  power = int(abs(moveFork) * 255);
  bool goUp  = (moveFork > 0);
  if (flipM4) goUp = !goUp;
  digitalWrite(LIFT_DIR, goUp ? HIGH : LOW);
  analogWrite(LIFT_PWM, power);
}



// >>>>>>>>>>>>>>>>>>>>>>>>>> INVERSE KINEMATICS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Compute individual wheel speeds from the commanded body velocity vector
// {vx, vy, ω} using the standard three-wheel omnidirectional kinematic model.
//
// For each wheel i at mounting angle θᵢ, with the robot rotated by localAngle α:
//
//   wheelᵢ = [ −sin(α+θᵢ)·cos(α)·vx + cos(α+θᵢ)·cos(α)·vy + R·ω ] / r
//
// where R = distance from wheel centre to robot centre, r = wheel radius.
// The cos(α) terms collapse to a constant at fixed α and can be absorbed into
// scaling, but are left explicit here to match the reference derivation.
void getWheelSpeeds(float *wheelList) {
  double a = localAngle;

  wheelList[0] = (-sin((a + angleWheel1) * 0.0174533) * cos(a * 0.0174533) * vSpeed[0]
                + cos((a + angleWheel1) * 0.0174533) * cos(a * 0.0174533) * vSpeed[1]
                + botRadius * vSpeed[2]) / wheelRadius;

  wheelList[1] = (-sin((a + angleWheel2) * 0.0174533) * cos(a * 0.0174533) * vSpeed[0]
                + cos((a + angleWheel2) * 0.0174533) * cos(a * 0.0174533) * vSpeed[1]
                + botRadius * vSpeed[2]) / wheelRadius;

  wheelList[2] = (-sin((a + angleWheel3) * 0.0174533) * cos(a * 0.0174533) * vSpeed[0]
                + cos((a + angleWheel3) * 0.0174533) * cos(a * 0.0174533) * vSpeed[1]
                + botRadius * vSpeed[2]) / wheelRadius;
}
