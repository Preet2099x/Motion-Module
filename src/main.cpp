/*
 * Motion Module - new robot: two Leadshine ELD2-CAN servo drives, Profile Velocity (PV) over CANopen.
 * Teensy 4.0, FlexCAN_T4. One drive per CAN bus (see DRIVE_CFG). Single file.
 *
 * Same motion command numbers as the old PWM module, now turned into wheel RPM:
 *   0              stop
 *   1 / 2          straight forward / backward
 *   11-20          turn right in place      21-30    turn left in place
 *   111-120        forward arc right        121-130  forward arc left
 *   211-220        backward arc             221-230  backward arc (mirror of 211-220)
 *   172/174/272/274  d-pad aliases for 115/125/215/225
 *   65 / 66        toggle straight boost / turn boost
 * (101/102 ... 401/402 camera lock codes and 67 are accepted and ignored.)
 *
 * Commands arrive on USB serial (one per line) or I2C slave address 0x72, exactly like the old module.
 * Like the old module, a command has to keep being re-sent: no command for COMMAND_TIMEOUT_MS = stop.
 *
 * "L" and "R" below are the old module's L/R motor channels, so every PWM table is unchanged.
 * Drive 0 of DRIVE_CFG is channel L, drive 1 is channel R. If the robot turns the wrong way for 11-20 /
 * 21-30, swap the two entries of DRIVE_CFG; if a wheel spins the wrong way, flip its `sign`.
 *
 * Wiring (one transceiver per bus, e.g. SN65HVD230 at 3.3V with Rs pin to GND):
 *   CAN1: Teensy pin 22 (CTX1) -> transceiver TXD, pin 23 (CRX1) -> transceiver RXD
 *   CAN2: Teensy pin 1  (CTX2) -> transceiver TXD, pin 0  (CRX2) -> transceiver RXD
 *   Transceiver CANH/CANL/GND -> drive CN5 pin 1 (CANH) / pin 3 (CANL) / pin 5 (GND)
 *   Each bus needs 120R at both ends (drive DIP SW6 = ON + 120R at the transceiver)
 *
 * Simultaneous start: each drive's RPDO1 carries its target velocity (60FFh), transmission type 1
 * (synchronous). Both speeds are sent first, then a SYNC on each bus back to back, so both wheels change
 * speed together. If a drive refuses synchronous RPDOs it falls back to asynchronous RPDOs.
 *
 * Serial monitor extras (115200, end with Enter): r<L>,<R> custom wheel rpm (streamed like a code),
 * x hard stop (fast decel), e enable servos, d disable servos, f fault reset, ? status, c scan buses,
 * m identify.
 */

#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>

// ---------------- User configuration ----------------
struct DriveConfig { uint8_t port; uint8_t id; int8_t sign; };   // port: 1 = CAN1 (pins 22/23), 2 = CAN2 (pins 1/0)
static const DriveConfig DRIVE_CFG[]   = {{1, 16, +1},      // channel L: drive on CAN1, node ID 16
                                          {2, 16, -1}};     // channel R: drive on CAN2, node ID 16 (mirrored mount)
static uint32_t       canBaud          = 1000000;   // must match the drives (scan can update it)
static const int32_t  MAX_RPM          = 255;       // motor rpm that old "PWM 255" corresponds to (255 = rpm equals the old PWM number) - TUNE
static const int32_t  BOOST_MAX_RPM    = 0;         // same, but while straight boost (65) is on: straight = this rpm, arcs scale with it.
                                                    // 0 = full speed: the motor's own max speed (2711h) read from the drive.
                                                    // Any value is capped to that max at startup.
static const int32_t  BOOST_FALLBACK_RPM = 1500;    // used for BOOST_MAX_RPM = 0 if the drive's max speed can't be read
static const uint32_t ACCEL_RPM_PER_S  = 1000;      // profile acceleration
static const uint32_t DECEL_RPM_PER_S  = 1000;      // profile deceleration
static const uint32_t HARD_STOP_DECEL_RPM_PER_S = 60000;  // deceleration used by the "x" hard stop. 60000 rpm/s = the drive's
                                                          // own factory 6084h (10,000,000 counts/s^2 at 10000 ppr): so steep the
                                                          // drive just brakes at its torque limit, i.e. as fast as it physically can.
                                                          // Too high can trip a DC-bus overvoltage fault from regen - lower it if so.
static const uint32_t FALLBACK_PPR     = 10000;     // command units per rev if it can't be read
static const uint32_t COMMAND_TIMEOUT_MS = 1000;    // no command for this long -> stop
static const uint32_t REFRESH_MS       = 250;       // re-send the current target this often while moving
static const uint8_t  I2C_ADDRESS      = 0x72;
// ----------------------------------------------------

static const uint8_t NUM_DRIVES = sizeof(DRIVE_CFG) / sizeof(DRIVE_CFG[0]);
static_assert(NUM_DRIVES == 2, "motion tables are written for exactly two drives (L, R)");

struct Drive {
  uint8_t  id;
  uint8_t  port;       // 1 = CAN1, 2 = CAN2, 0 = not found
  uint32_t countsPerRev;
  int32_t  targetRpm;
  bool     online;
  bool     bootupSeen;
  uint8_t  rpdoType;   // 1 = synchronous, 255 = asynchronous, 0 = RPDO not configured (SDO fallback)
};
static Drive drives[NUM_DRIVES];

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // pin 22 = CTX1, pin 23 = CRX1
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;  // pin 1  = CTX2, pin 0  = CRX2

// Both controllers are different template types, so all bus access goes through these
static bool canWrite(uint8_t port, const CAN_message_t &m) { return port == 2 ? can2.write(m) : can1.write(m); }
static bool canRead(uint8_t port, CAN_message_t &m)        { return port == 2 ? can2.read(m)  : can1.read(m); }
static void canError(uint8_t port, CAN_error_t &e)         { if (port == 2) can2.error(e, false); else can1.error(e, false); }
static void canSetBaud(uint32_t baud)                      { can1.setBaudRate(baud); can2.setBaudRate(baud); }
static const char *canPins(uint8_t port)                   { return port == 2 ? "pin 1 TX / pin 0 RX" : "pin 22 TX / pin 23 RX"; }

// CANopen object indexes (DS301 / DSP402)
static const uint16_t OD_RPDO1_COMM    = 0x1400;  // sub 1: COB-ID, sub 2: transmission type
static const uint16_t OD_RPDO1_MAP     = 0x1600;
static const uint16_t OD_CONTROLWORD   = 0x6040;
static const uint16_t OD_STATUSWORD    = 0x6041;
static const uint16_t OD_ERROR_CODE    = 0x603F;
static const uint16_t OD_MODE_OF_OP    = 0x6060;
static const uint16_t OD_MODE_DISPLAY  = 0x6061;
static const uint16_t OD_PROFILE_ACCEL = 0x6083;
static const uint16_t OD_PROFILE_DECEL = 0x6084;
static const uint16_t OD_FEED_CONSTANT = 0x6092;  // sub 1: command units per motor rev
static const uint16_t OD_TARGET_VEL    = 0x60FF;
static const uint16_t OD_PR0_08        = 0x2008;  // Pr0.08 pulses per rev (overrides 6092h if != 0)
static const uint16_t OD_VELOCITY_ACTUAL = 0x606C;  // command units/s
static const uint16_t OD_MOTOR_RATED_RPM = 0x2710;  // Pr7.10, read-only, set by the motor
static const uint16_t OD_MOTOR_MAX_RPM   = 0x2711;  // Pr7.11, read-only; drive faults (Er1A0) at 1.2x this

static int32_t motorMaxRpm = 0;  // smallest motor max speed read from the drives, 0 = unknown (no cap)

static const uint8_t MODE_PROFILE_VELOCITY = 3;

static const uint8_t NMT_START  = 0x01;
static const uint8_t NMT_PRE_OP = 0x80;
static const uint8_t NMT_RESET  = 0x81;

// ---------------- CAN / CANopen helpers ----------------

static Drive *findDrive(uint8_t port, uint8_t id) {
  for (Drive &d : drives)
    if (d.port == port && d.id == id) return &d;
  return nullptr;
}

static void handleAsyncFrame(uint8_t port, const CAN_message_t &m) {
  if (m.id > 0x700 && m.id <= 0x77F && m.len >= 1 && m.buf[0] == 0x00) {
    if (Drive *d = findDrive(port, m.id - 0x700)) d->bootupSeen = true;
  } else if (m.id > 0x080 && m.id <= 0x0FF && m.len >= 3) {
    uint16_t code = m.buf[0] | (m.buf[1] << 8);
    Serial.printf("EMCY CAN%u node %lu: error code 0x%04X, error register 0x%02X\n",
                  port, (unsigned long)(m.id - 0x080), code, m.buf[2]);
  }
}

static void pollCan() {
  CAN_message_t m;
  while (canRead(1, m)) handleAsyncFrame(1, m);
  while (canRead(2, m)) handleAsyncFrame(2, m);
}

static void sendNmt(uint8_t port, uint8_t command, uint8_t node) {
  CAN_message_t m;
  m.id = 0x000;
  m.len = 2;
  m.buf[0] = command;
  m.buf[1] = node;
  canWrite(port, m);
}

static void sendSync(uint8_t port) {
  CAN_message_t m;
  m.id = 0x080;
  m.len = 0;
  canWrite(port, m);
}

// Time for a few CAN frames to leave the controller at the current baud rate
static void waitFramesOnBus(uint8_t frames) {
  delayMicroseconds(max<uint32_t>(200, (uint32_t)frames * 135u * 1000000u / canBaud * 2u));
}

// Sends an SDO request and waits for the matching reply. Returns false on timeout/abort.
static bool sdoTransfer(uint8_t port, uint8_t node, const uint8_t req[8], uint8_t reply[8],
                        uint32_t timeoutMs = 100) {
  CAN_message_t tx;
  tx.id = 0x600u + node;
  tx.len = 8;
  memcpy(tx.buf, req, 8);
  canWrite(port, tx);

  uint32_t start = millis();
  CAN_message_t rx;
  while (millis() - start < timeoutMs) {
    if (!canRead(port, rx)) continue;
    if (rx.id != 0x580u + node || rx.len < 8 ||
        rx.buf[1] != req[1] || rx.buf[2] != req[2] || rx.buf[3] != req[3]) {
      handleAsyncFrame(port, rx);
      continue;
    }
    memcpy(reply, rx.buf, 8);
    if (rx.buf[0] == 0x80) {
      uint32_t abortCode = rx.buf[4] | (rx.buf[5] << 8) | (rx.buf[6] << 16) | ((uint32_t)rx.buf[7] << 24);
      Serial.printf("CAN%u node %u: SDO abort 0x%04X:%02X code 0x%08lX\n",
                    port, node, req[1] | (req[2] << 8), req[3], (unsigned long)abortCode);
      return false;
    }
    return true;
  }
  Serial.printf("CAN%u node %u: SDO timeout 0x%04X:%02X (no reply)\n", port, node, req[1] | (req[2] << 8), req[3]);
  return false;
}

// size = 1, 2 or 4 bytes
static bool sdoWrite(const Drive &d, uint16_t index, uint8_t sub, uint32_t value, uint8_t size) {
  static const uint8_t cmdForSize[5] = {0, 0x2F, 0x2B, 0x27, 0x23};
  uint8_t req[8] = {cmdForSize[size], (uint8_t)index, (uint8_t)(index >> 8), sub,
                    (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
  uint8_t reply[8];
  return sdoTransfer(d.port, d.id, req, reply) && reply[0] == 0x60;
}

static bool decodeSdoRead(const uint8_t reply[8], int32_t &value) {
  uint32_t raw = reply[4] | (reply[5] << 8) | (reply[6] << 16) | ((uint32_t)reply[7] << 24);
  switch (reply[0]) {
    case 0x4F: value = (int8_t)raw;  return true;
    case 0x4B: value = (int16_t)raw; return true;
    case 0x47: value = (int32_t)(raw << 8) >> 8; return true;
    case 0x43: value = (int32_t)raw; return true;
    default:   return false;
  }
}

// Reads an expedited value, sign-extended according to the size the drive reports.
static bool sdoRead(const Drive &d, uint16_t index, uint8_t sub, int32_t &value) {
  uint8_t req[8] = {0x40, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0};
  uint8_t reply[8];
  return sdoTransfer(d.port, d.id, req, reply) && decodeSdoRead(reply, value);
}


// ---------------- DS402 state machine ----------------

enum Ds402State { ST_UNKNOWN, ST_NOT_READY, ST_SWITCH_ON_DISABLED, ST_READY, ST_SWITCHED_ON,
                  ST_OP_ENABLED, ST_QUICK_STOP, ST_FAULT_REACTION, ST_FAULT };

static Ds402State decodeState(uint16_t sw) {
  if ((sw & 0x4F) == 0x00) return ST_NOT_READY;
  if ((sw & 0x4F) == 0x40) return ST_SWITCH_ON_DISABLED;
  if ((sw & 0x6F) == 0x21) return ST_READY;
  if ((sw & 0x6F) == 0x23) return ST_SWITCHED_ON;
  if ((sw & 0x6F) == 0x27) return ST_OP_ENABLED;
  if ((sw & 0x6F) == 0x07) return ST_QUICK_STOP;
  if ((sw & 0x4F) == 0x0F) return ST_FAULT_REACTION;
  if ((sw & 0x4F) == 0x08) return ST_FAULT;
  return ST_UNKNOWN;
}

static const char *stateName(Ds402State s) {
  switch (s) {
    case ST_NOT_READY:          return "Not ready to switch on";
    case ST_SWITCH_ON_DISABLED: return "Switch on disabled";
    case ST_READY:              return "Ready to switch on";
    case ST_SWITCHED_ON:        return "Switched on";
    case ST_OP_ENABLED:         return "Operation enabled";
    case ST_QUICK_STOP:         return "Quick stop active";
    case ST_FAULT_REACTION:     return "Fault reaction active";
    case ST_FAULT:              return "Fault";
    default:                    return "Unknown";
  }
}

static bool readState(const Drive &d, Ds402State &state, uint16_t *rawOut = nullptr) {
  int32_t sw;
  if (!sdoRead(d, OD_STATUSWORD, 0, sw)) return false;
  if (rawOut) *rawOut = (uint16_t)sw;
  state = decodeState((uint16_t)sw);
  return true;
}

// Waits until every online drive reaches the wanted state
static bool waitAllForState(Ds402State wanted, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    bool all = true;
    for (Drive &d : drives) {
      Ds402State s;
      if (d.online && !(readState(d, s) && s == wanted)) all = false;
    }
    if (all) return true;
    delay(5);
  }
  for (Drive &d : drives) {
    Ds402State s = ST_UNKNOWN;
    if (d.online && readState(d, s) && s != wanted)
      Serial.printf("CAN%u node %u: timeout waiting for '%s' (is '%s')\n", d.port, d.id, stateName(wanted), stateName(s));
  }
  return false;
}

static void writeControlwordAll(uint16_t cw) {
  for (Drive &d : drives)
    if (d.online) sdoWrite(d, OD_CONTROLWORD, 0, cw, 2);
}

static void printDriveError(const Drive &d) {
  int32_t err;
  if (sdoRead(d, OD_ERROR_CODE, 0, err))
    Serial.printf("CAN%u node %u: drive error code (603Fh) 0x%04X - see manual chapter 7\n", d.port, d.id, (uint16_t)err);
}

static int32_t rpmToCounts(const Drive &d, int32_t rpm) {
  return (int32_t)((int64_t)rpm * d.countsPerRev / 60);
}

static bool anyOnline() {
  for (Drive &d : drives)
    if (d.online) return true;
  return false;
}

// ---------------- Simultaneous speed command ----------------

static void sendRpdo1(const Drive &d, int32_t counts) {
  CAN_message_t m;
  m.id = 0x200u + d.id;
  m.len = 4;
  m.buf[0] = (uint8_t)counts;
  m.buf[1] = (uint8_t)(counts >> 8);
  m.buf[2] = (uint8_t)(counts >> 16);
  m.buf[3] = (uint8_t)(counts >> 24);
  canWrite(d.port, m);
}

// Applies rpm[i] to drives[i] at the same instant.
static void applySpeeds(const int32_t rpm[NUM_DRIVES]) {
  bool syncOnPort[3] = {false, false, false};
  uint8_t framesOnPort[3] = {0, 0, 0};
  for (uint8_t i = 0; i < NUM_DRIVES; i++) {
    Drive &d = drives[i];
    if (!d.online) continue;
    int32_t counts = rpmToCounts(d, rpm[i]);
    if (d.rpdoType != 0) {
      sendRpdo1(d, counts);
      framesOnPort[d.port]++;
      if (d.rpdoType == 1) syncOnPort[d.port] = true;
    } else {
      sdoWrite(d, OD_TARGET_VEL, 0, (uint32_t)counts, 4);
    }
    d.targetRpm = rpm[i];
  }
  if (syncOnPort[1] || syncOnPort[2]) {
    // SYNC (ID 0x080) wins bus arbitration over RPDOs (0x200+id), so let the RPDOs leave the
    // controllers first; then SYNC on both buses back to back makes every drive apply together.
    waitFramesOnBus(max(framesOnPort[1], framesOnPort[2]));
    if (syncOnPort[1]) sendSync(1);
    if (syncOnPort[2]) sendSync(2);
  }
}

static void zeroAllTargets() {
  int32_t zero[NUM_DRIVES] = {0};
  applySpeeds(zero);
}

// ---------------- Enable / disable (all drives step through the state machine together) ----------------

static void faultResetAll() {
  for (Drive &d : drives) {
    Ds402State s;
    if (!d.online || !readState(d, s)) continue;
    if (s != ST_FAULT && s != ST_FAULT_REACTION) continue;
    printDriveError(d);
    Serial.printf("CAN%u node %u: clearing fault\n", d.port, d.id);
    sdoWrite(d, OD_CONTROLWORD, 0, 0x0000, 2);
    sdoWrite(d, OD_CONTROLWORD, 0, 0x0080, 2);  // rising edge of bit 7 = fault reset
  }
  delay(50);
  writeControlwordAll(0x0000);
}

static bool allEnabled() {
  for (Drive &d : drives) {
    Ds402State s;
    if (d.online && !(readState(d, s) && s == ST_OP_ENABLED)) return false;
  }
  return true;
}

static bool servoEnableAll() {
  if (!anyOnline()) return false;
  if (allEnabled()) return true;
  faultResetAll();

  zeroAllTargets();  // never enable with a stale speed command
  for (Drive &d : drives)
    if (d.online) sdoWrite(d, OD_MODE_OF_OP, 0, MODE_PROFILE_VELOCITY, 1);

  writeControlwordAll(0x0006);                       // Shutdown -> Ready to switch on
  if (!waitAllForState(ST_READY, 1000)) return false;
  writeControlwordAll(0x0007);                       // Switch on (main circuit relay)
  if (!waitAllForState(ST_SWITCHED_ON, 1000)) return false;
  writeControlwordAll(0x000F);                       // Enable operation (motors energised)
  if (!waitAllForState(ST_OP_ENABLED, 1000)) return false;

  Serial.println("All servos enabled (Profile Velocity mode)");
  return true;
}

static void servoDisableAll() {
  zeroAllTargets();
  writeControlwordAll(0x0006);
  Serial.println("All servos disabled");
}

// Enables the servos first if a non-zero speed is requested. Returns false if they would not enable.
static bool setSpeeds(const int32_t rpm[NUM_DRIVES], bool verbose) {
  bool anyMotion = false;
  for (uint8_t i = 0; i < NUM_DRIVES; i++) anyMotion |= rpm[i] != 0;
  if (anyMotion && !allEnabled() && !servoEnableAll()) return false;
  applySpeeds(rpm);
  if (verbose)
    for (Drive &d : drives)
      if (d.online)
        Serial.printf("CAN%u node %u: target %ld rpm\n", d.port, d.id, (long)d.targetRpm);
  return true;
}

static void printStatus() {
  for (Drive &d : drives) {
    if (!d.online) {
      Serial.printf("CAN%u node %u: offline\n", d.port, d.id);
      continue;
    }
    uint16_t sw = 0;
    Ds402State s;
    int32_t mode = 0, speed = 0;
    if (!readState(d, s, &sw)) continue;
    sdoRead(d, OD_MODE_DISPLAY, 0, mode);
    bool haveSpeed = sdoRead(d, OD_VELOCITY_ACTUAL, 0, speed);  // command units/s
    Serial.printf("CAN%u node %u: %s (0x%04X)  mode %ld  target %ld rpm  actual ",
                  d.port, d.id, stateName(s), sw, (long)mode, (long)d.targetRpm);
    if (haveSpeed) Serial.printf("%.1f rpm\n", speed * 60.0f / d.countsPerRev);
    else Serial.println("n/a");
    if (s == ST_FAULT) printDriveError(d);
  }
}

// ---------------- Setup ----------------

static void readCountsPerRev(Drive &d) {
  int32_t v;
  if (sdoRead(d, OD_PR0_08, 0, v) && v > 0) {
    d.countsPerRev = (uint32_t)v;
    Serial.printf("CAN%u node %u: Pr0.08 = %lu counts/rev\n", d.port, d.id, (unsigned long)d.countsPerRev);
  } else if (sdoRead(d, OD_FEED_CONSTANT, 1, v) && v > 0) {
    d.countsPerRev = (uint32_t)v;
    Serial.printf("CAN%u node %u: 6092h-01 = %lu counts/rev\n", d.port, d.id, (unsigned long)d.countsPerRev);
  } else {
    d.countsPerRev = FALLBACK_PPR;
    Serial.printf("CAN%u node %u: could not read counts/rev, assuming %lu\n", d.port, d.id, (unsigned long)d.countsPerRev);
  }
}

// RPDO1 = 60FFh target velocity (int32). Must be done in NMT pre-operational.
static void configureRpdo1(Drive &d) {
  const uint32_t cobId = 0x200u + d.id;
  d.rpdoType = 0;
  if (!sdoWrite(d, OD_RPDO1_COMM, 1, 0x80000000u | cobId, 4)) return;  // disable PDO while editing
  sdoWrite(d, OD_RPDO1_MAP, 0, 0, 1);
  if (!sdoWrite(d, OD_RPDO1_MAP, 1, ((uint32_t)OD_TARGET_VEL << 16) | 0x20, 4)) return;
  if (!sdoWrite(d, OD_RPDO1_MAP, 0, 1, 1)) return;

  if (sdoWrite(d, OD_RPDO1_COMM, 2, 1, 1)) {
    d.rpdoType = 1;
  } else if (sdoWrite(d, OD_RPDO1_COMM, 2, 255, 1)) {
    d.rpdoType = 255;
    Serial.printf("CAN%u node %u: synchronous RPDO refused, using asynchronous RPDO\n", d.port, d.id);
  } else {
    return;
  }
  if (!sdoWrite(d, OD_RPDO1_COMM, 1, cobId, 4)) {
    d.rpdoType = 0;
    return;
  }
  Serial.printf("CAN%u node %u: RPDO1 0x%03lX -> 60FFh (%s)\n", d.port, d.id, (unsigned long)cobId,
                d.rpdoType == 1 ? "synchronous" : "asynchronous");
}

// Switches the profile ramps between the normal and the hard-stop value. The hard stop sets acceleration
// too: depending on the drive, slowing down while reversing can count as "acceleration".
static bool hardDecel = false;
static void setProfileDecel(bool hard) {
  uint32_t accel = hard ? HARD_STOP_DECEL_RPM_PER_S : ACCEL_RPM_PER_S;
  uint32_t decel = hard ? HARD_STOP_DECEL_RPM_PER_S : DECEL_RPM_PER_S;
  for (Drive &d : drives) {
    if (!d.online) continue;
    sdoWrite(d, OD_PROFILE_ACCEL, 0, (uint32_t)((uint64_t)accel * d.countsPerRev / 60), 4);
    sdoWrite(d, OD_PROFILE_DECEL, 0, (uint32_t)((uint64_t)decel * d.countsPerRev / 60), 4);
  }
  hardDecel = hard;
}

static void initDrives() {
  hardDecel = false;
  motorMaxRpm = 0;
  // Reset every drive on its own bus and wait for the boot-up messages
  for (Drive &d : drives) {
    d.bootupSeen = false;
    d.online = false;
    sendNmt(d.port, NMT_RESET, d.id);
  }
  uint32_t start = millis();
  bool allBooted = false;
  while (!allBooted && millis() - start < 3000) {
    pollCan();
    allBooted = true;
    for (Drive &d : drives) allBooted &= d.bootupSeen;
  }
  delay(100);

  for (Drive &d : drives) {
    sendNmt(d.port, NMT_PRE_OP, d.id);
    int32_t dummy;
    d.online = sdoRead(d, OD_STATUSWORD, 0, dummy);
    if (!d.online) {
      Serial.printf("CAN%u node %u: not answering (%s) - check node ID, baud rate, transceiver, wiring\n",
                    d.port, d.id, canPins(d.port));
      continue;
    }
    Serial.printf("CAN%u node %u: online (%s)%s\n", d.port, d.id, canPins(d.port),
                  d.bootupSeen ? "" : ", no boot-up message seen");

    readCountsPerRev(d);
    int32_t ratedRpm = 0, maxRpm = 0;
    sdoRead(d, OD_MOTOR_RATED_RPM, 0, ratedRpm);
    sdoRead(d, OD_MOTOR_MAX_RPM, 0, maxRpm);
    Serial.printf("CAN%u node %u: motor rated %ld rpm, max %ld rpm\n", d.port, d.id, (long)ratedRpm, (long)maxRpm);
    if (maxRpm >= 100 && maxRpm <= 20000)  // ignore values that don't look like rpm
      motorMaxRpm = motorMaxRpm ? min(motorMaxRpm, maxRpm) : maxRpm;
    uint32_t accel = (uint32_t)((uint64_t)ACCEL_RPM_PER_S * d.countsPerRev / 60);
    uint32_t decel = (uint32_t)((uint64_t)DECEL_RPM_PER_S * d.countsPerRev / 60);
    sdoWrite(d, OD_PROFILE_ACCEL, 0, accel, 4);
    sdoWrite(d, OD_PROFILE_DECEL, 0, decel, 4);
    sdoWrite(d, OD_MODE_OF_OP, 0, MODE_PROFILE_VELOCITY, 1);
    configureRpdo1(d);
    if (d.rpdoType == 0) Serial.printf("CAN%u node %u: RPDO setup failed, using SDO (not simultaneous)\n", d.port, d.id);
    sendNmt(d.port, NMT_START, d.id);  // PDOs only work in NMT operational
  }
}

static void printBusErrors(uint8_t port) {
  CAN_error_t err;
  canError(port, err);
  Serial.printf("  CAN%u controller: %s, TX errors %u, RX errors %u%s%s%s%s%s\n",
                port, err.state, err.TX_ERR_COUNTER, err.RX_ERR_COUNTER,
                err.ACK_ERR ? ", ACK error (nobody acknowledged our frames)" : "",
                err.BIT0_ERR ? ", BIT0 error (sent dominant, read recessive: transceiver not driving the bus)" : "",
                err.BIT1_ERR ? ", BIT1 error (sent recessive, read dominant: RX pin stuck low / bus stuck)" : "",
                err.CRC_ERR || err.FRM_ERR || err.STF_ERR ? ", CRC/frame/stuff error (baud mismatch?)" : "",
                err.TX_WRN ? ", TX warning" : "");
}

// Tries every baud rate the ELD2-CAN supports on both buses, resets all nodes and lists
// who boots up where. Adopts the first baud rate where something answered.
static bool scanBus() {
  static const uint32_t bauds[] = {1000000, 800000, 500000, 250000, 125000};
  bool found = false;
  uint32_t foundBaud = canBaud;
  Serial.println("Scanning CAN1 and CAN2...");
  for (uint32_t baud : bauds) {
    canSetBaud(baud);
    delay(20);
    pollCan();
    sendNmt(1, NMT_RESET, 0x00);  // reset all nodes -> each sends a boot-up frame 0x700+id
    sendNmt(2, NMT_RESET, 0x00);

    uint32_t frames[3] = {0, 0, 0};
    uint32_t start = millis();
    CAN_message_t m;
    while (millis() - start < 2500) {
      for (uint8_t port = 1; port <= 2; port++) {
        if (!canRead(port, m)) continue;
        frames[port]++;
        if (m.id > 0x700 && m.id <= 0x77F) {
          Serial.printf("  %lu bit/s: node %lu answered on CAN%u\n",
                        (unsigned long)baud, (unsigned long)(m.id - 0x700), port);
          if (!found) {
            found = true;
            foundBaud = baud;
          }
        }
      }
    }
    Serial.printf("  %lu bit/s: %lu frames on CAN1, %lu frames on CAN2\n",
                  (unsigned long)baud, (unsigned long)frames[1], (unsigned long)frames[2]);
    printBusErrors(1);
    printBusErrors(2);
  }
  canBaud = foundBaud;
  canSetBaud(canBaud);
  if (found)
    Serial.printf("Using %lu bit/s. Check the buses and node IDs above match DRIVE_CFG.\n", (unsigned long)canBaud);
  else
    Serial.println("No drive found on either bus at any baud rate - check transceivers and wiring.");
  return found;
}

// ---------------- Motion commands -> wheel speeds ----------------

static bool turnBoost = false;      // 66 toggles
static bool straightBoost = false;  // 65 toggles

// PWM-equivalent figures, signed: + = forward, - = backward. 255 = MAX_RPM.
// Each table is {boost, normal}, picked by the straight/turn boost flags.
struct Wheels { int pwmL; int pwmR; int32_t maxRpm; };  // maxRpm = rpm that pwm 255 maps to

// rpm that PWM 255 maps to while straight boost is on
static int32_t boostMaxRpm() {
  if (BOOST_MAX_RPM > 0) return BOOST_MAX_RPM;
  return motorMaxRpm > 0 ? motorMaxRpm : BOOST_FALLBACK_RPM;
}

static int pick(int boost, int normal, bool useBoost) { return useBoost ? boost : normal; }

// Which of the 3 speed tiers a code falls in; `lo` is the first code of its 10-code range.
// lo..lo+2 -> 0 (slow), lo+3..lo+5 -> 1 (mid), the rest -> 2 (fast)
static int tierOf(int cmd, int lo) {
  int off = cmd - lo;
  return off <= 2 ? 0 : off <= 5 ? 1 : 2;
}

// {L boost, L normal, R boost, R normal}
static const int ARC_FWD_A[3][4] = {{120, 90, 170, 125}, {150, 115, 200, 150}, {80, 60, 240, 180}};  // 111-120
static const int ARC_FWD_B[3][4] = {{170, 125, 120, 90}, {200, 150, 150, 115}, {240, 180, 80, 60}};  // 121-130
static const int ARC_BWD_A[3][4] = {{120, 90, 170, 125}, {120, 90, 200, 150}, {80, 60, 240, 180}};   // 211-220
static const int ARC_BWD_B[3][4] = {{170, 125, 120, 90}, {200, 150, 120, 90}, {240, 180, 80, 60}};   // 221-230
static const int TURN_PWM[3][2]  = {{80, 60}, {120, 80}, {180, 120}};                                // {boost, normal}

static Wheels arc(const int t[3][4], int cmd, int lo, int dir) {
  const int *row = t[tierOf(cmd, lo)];
  return {dir * pick(row[0], row[1], straightBoost), dir * pick(row[2], row[3], straightBoost),
          straightBoost ? boostMaxRpm() : MAX_RPM};
}

static Wheels turnInPlace(int cmd, int lo, int dir) {  // dir +1: L forward / R backward
  int p = pick(TURN_PWM[tierOf(cmd, lo)][0], TURN_PWM[tierOf(cmd, lo)][1], turnBoost);
  return {dir * p, -dir * p, MAX_RPM};
}

// Fills `w` for a motion code. Returns false for codes that are not (or no longer) a motion.
static bool motionToWheels(int cmd, Wheels &w) {
  // D-pad aliases, same as the old module
  if (cmd == 172) cmd = 115;
  if (cmd == 174) cmd = 125;
  if (cmd == 272) cmd = 215;
  if (cmd == 274) cmd = 225;

  if (cmd == 1 || cmd == 2) {
    int p = straightBoost ? 255 : 200;
    w = {cmd == 1 ? p : -p, cmd == 1 ? p : -p, straightBoost ? boostMaxRpm() : MAX_RPM};
    return true;
  }
  if (cmd >= 11  && cmd <= 20)  { w = turnInPlace(cmd, 11, -1); return true; }  // right: L back, R fwd
  if (cmd >= 21  && cmd <= 30)  { w = turnInPlace(cmd, 21, +1); return true; }  // left:  L fwd,  R back
  if (cmd >= 111 && cmd <= 120) { w = arc(ARC_FWD_A, cmd, 111, +1); return true; }
  if (cmd >= 121 && cmd <= 130) { w = arc(ARC_FWD_B, cmd, 121, +1); return true; }
  if (cmd >= 211 && cmd <= 220) { w = arc(ARC_BWD_A, cmd, 211, -1); return true; }
  if (cmd >= 221 && cmd <= 230) { w = arc(ARC_BWD_B, cmd, 221, -1); return true; }
  return false;  // 0 and anything unknown = stop
}

static int32_t pwmToRpm(int pwm, int8_t sign, int32_t maxRpm) {
  return (int32_t)sign * pwm * maxRpm / 255;
}

static int  motionCmd = 0;          // current command, 0 = stop
static uint32_t lastCmdMs = 0;

// Custom wheel speeds from "r<L>,<R>" (signed rpm per channel, + = forward). Like a motion code it
// has to keep being re-sent, or it times out.
static bool customMode = false;
static int32_t customL = 0, customR = 0;

// 101/102, 201/202, 301/302, 401/402 camera locks and 67 are not used by this module.
static bool isIgnoredCode(int v) {
  return v == 67 || ((v % 100 == 1 || v % 100 == 2) && v >= 101 && v <= 402);
}

static void setCommand(int v) {
  static uint32_t boostTimer = 0;
  if (isIgnoredCode(v)) return;
  if (v == 65 || v == 66) {  // toggles; debounced because the sender keeps repeating the code
    if (millis() - boostTimer >= 500) {
      boostTimer = millis();
      if (v == 65) straightBoost = !straightBoost;
      else         turnBoost = !turnBoost;
      Serial.printf("Boost: straight %s (%ld rpm), turn %s\n", straightBoost ? "ON" : "OFF",
                    (long)(straightBoost ? boostMaxRpm() : MAX_RPM), turnBoost ? "ON" : "OFF");
    }
    lastCmdMs = millis();  // keep the current motion alive, like the old module
    return;
  }
  if (v != 0 && hardDecel) setProfileDecel(false);  // back to the gentle ramp once moving again
  customMode = false;
  motionCmd = v;
  lastCmdMs = millis();
}

// Turns the current command into drive targets and sends them when they change (or periodically).
static void updateMotion() {
  static int32_t last[NUM_DRIVES] = {0};
  static uint32_t lastSend = 0;
  static uint32_t retryAt = 0;
  static bool firstRun = true;

  if ((int32_t)(millis() - retryAt) < 0) return;  // servos failed to enable recently
  if ((motionCmd != 0 || customMode) && millis() - lastCmdMs > COMMAND_TIMEOUT_MS) {
    motionCmd = 0;
    customMode = false;
  }

  int32_t want[NUM_DRIVES];
  if (customMode) {
    want[0] = DRIVE_CFG[0].sign * customL;
    want[1] = DRIVE_CFG[1].sign * customR;
  } else {
    Wheels w = {0, 0, MAX_RPM};
    if (!motionToWheels(motionCmd, w)) w = {0, 0, MAX_RPM};
    want[0] = pwmToRpm(w.pwmL, DRIVE_CFG[0].sign, w.maxRpm);
    want[1] = pwmToRpm(w.pwmR, DRIVE_CFG[1].sign, w.maxRpm);
  }
  if (motorMaxRpm > 0)
    for (uint8_t i = 0; i < NUM_DRIVES; i++) want[i] = constrain(want[i], -motorMaxRpm, motorMaxRpm);

  bool changed = firstRun;
  for (uint8_t i = 0; i < NUM_DRIVES; i++) changed |= want[i] != last[i];
  bool moving = want[0] != 0 || want[1] != 0;
  if (!changed && !(moving && millis() - lastSend >= REFRESH_MS)) return;

  firstRun = false;
  lastSend = millis();
  if (setSpeeds(want, changed)) {
    for (uint8_t i = 0; i < NUM_DRIVES; i++) last[i] = want[i];
  } else {
    Serial.println("Servos would not enable - holding stop, retrying");
    retryAt = millis() + 1000;  // retry in 1 s instead of every loop
  }
}

// ---------------- Command input (serial + I2C) ----------------

static void handleLine(const char *line) {
  char c = line[0];
  if ((c >= '0' && c <= '9') || c == '-') { setCommand(atoi(line)); return; }
  switch (c) {
    case 'm': {
      uint32_t n = HW_OCOTP_MAC0 & 0xFFFFFF;
      Serial.printf("%lu | Motion Module\n", (unsigned long)(n < 10000000 ? n * 10 : n));
      break;
    }
    case 'r': case 'R': {  // r<rpmL>,<rpmR>: custom wheel speeds, e.g. "r300,300" straight, "r-300,-300" back
      long l, r;
      if (sscanf(line + 1, "%ld,%ld", &l, &r) != 2) { Serial.println("Usage: r<rpmL>,<rpmR>"); break; }
      if (hardDecel && (l != 0 || r != 0)) setProfileDecel(false);
      customL = l;
      customR = r;
      customMode = true;
      motionCmd = 0;
      lastCmdMs = millis();
      break;
    }
    case 'x': case 'X':  // hard stop: steep deceleration first, then zero speed right away (not on the next loop)
      motionCmd = 0;
      customMode = false;
      setProfileDecel(true);
      zeroAllTargets();
      break;
    case 'e': case 'E': servoEnableAll(); break;
    case 'd': case 'D': motionCmd = 0; servoDisableAll(); break;
    case 'f': case 'F': faultResetAll(); break;
    case '?':           printStatus(); break;
    case 'c': case 'C': motionCmd = 0; zeroAllTargets(); if (scanBus()) initDrives(); break;
    default: Serial.printf("Unknown command '%s'. Motion codes are numeric; others: r<L>,<R> x m e d f ? c\n", line); break;
  }
}

static void handleSerial() {
  static char line[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    if (len == 0) continue;
    line[len] = '\0';
    len = 0;
    handleLine(line);
  }
}

// I2C receive runs in interrupt context: just stash the number, setCommand() runs from loop().
static volatile int  i2cValue = 0;
static volatile bool i2cNew = false;

static void receiveEvent(int bytesReceived) {
  char buf[16];
  uint8_t n = 0;
  while (Wire.available()) {
    char c = Wire.read();
    if (n < sizeof(buf) - 1) buf[n++] = c;
  }
  buf[n] = '\0';
  if ((buf[0] >= '0' && buf[0] <= '9') || buf[0] == '-') {
    i2cValue = atoi(buf);
    i2cNew = true;
  }
}

static void handleI2c() {
  if (!i2cNew) return;
  int v = i2cValue;
  i2cNew = false;
  setCommand(v);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.printf("Motion Module (CAN servo), %u drives on CAN1/CAN2 at %lu bit/s, MAX_RPM %ld\n",
                NUM_DRIVES, (unsigned long)canBaud, (long)MAX_RPM);

  for (uint8_t i = 0; i < NUM_DRIVES; i++)
    drives[i] = {DRIVE_CFG[i].id, DRIVE_CFG[i].port, FALLBACK_PPR, 0, false, false, 0};

  can1.begin();
  can2.begin();
  canSetBaud(canBaud);

  initDrives();
  if (!anyOnline() && scanBus()) initDrives();
  if (anyOnline()) servoEnableAll();

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);

  Serial.println("Ready. Send a motion code (0, 1, 2, 11-30, 111-130, 211-230) or: m e d f ? c");
}

void loop() {
  handleSerial();
  handleI2c();
  pollCan();
  updateMotion();
}
