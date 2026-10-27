/*
 * Motion Module - ROBOT build (I2C commands only, no CLI).
 * New robot: two Leadshine ELD2-CAN servo drives in Profile Velocity mode over CANopen. Teensy 4.0, FlexCAN_T4.
 *
 *   Upload:  pio run -e robot -t upload
 *   (src/main.cpp + tools/driveCLI.py is the bench build for driving from a terminal.)
 *
 * Commands: an ASCII number over I2C, slave address 0x72 (pins 18 SDA / 19 SCL), same as the old module.
 * The sender has to keep re-sending the current command, exactly like before.
 *
 *   0                    stop (hard stop)
 *   1 / 2                straight forward / backward            NORMAL_RPM, or BOOST_RPM with boost
 *   3 / 4                slow straight forward / backward
 *   11-20 / 21-30        turn in place (11-20 = right), 3 tiers: x1-x3 slow, x4-x6 mid, x7-x0 fast
 *   31-40 / 41-50        follow-me turns (same directions as 21-30 / 11-20), no boost
 *   111-120 / 121-130    forward arcs,  211-220 / 221-230 backward arcs, 3 tiers each
 *   172 / 174 / 272 / 274  d-pad aliases for 115 / 125 / 215 / 225
 *   65                   toggle boost: straight 1/2/3/4 and all arcs
 *   66                   toggle turbo turn: in-place turns 11-30
 *   67, 101/102, 201/202, 301/302, 401/402   accepted and ignored (camera codes)
 *   anything else        stop (hard stop)
 *
 * Speeds (motor rpm):
 *   straight 1/2         500 normal, 1000 boost                  (NORMAL_RPM, BOOST_RPM)
 *   in-place turns       150 / 200 / 300 normal, 311 / 466 / 700 turbo   (TURBO_TURN_RPM = fast tier)
 *   Everything is the old module's PWM table rescaled, so every code keeps its old left/right proportions:
 *   normal: PWM 200 -> NORMAL_RPM, boost: PWM 255 -> BOOST_RPM, turbo turn: PWM 180 -> TURBO_TURN_RPM.
 *
 * Arcs are different from the PWM robot: the drives hold each wheel at its exact speed, so the inner/outer
 * ratio in the table IS the curve now (with PWM, a low inner PWM under load sagged and the arc came out
 * tighter). Each code is the old ratio exactly, e.g. 117 = inner 150 / outer 450 rpm, a firm 1:3 arc.
 * To keep the curve right while speeds change, both wheels' ramps are scaled so they reach their new speeds
 * at the same moment (a wheel that changes less ramps slower); otherwise going straight -> arc or arc -> arc
 * would briefly swerve because the smaller change finishes first.
 *
 * Stopping: 0, any non-motion code, or no command for COMMAND_TIMEOUT_MS -> HARD stop. Both drives get a
 * very steep deceleration (HARD_STOP_RPM_PER_S, the drive's own factory 6084h value), so they brake at their
 * torque limit, i.e. as fast as the motors physically can, then hold position at zero speed (servo stays on).
 * The next motion command ramps normally again.
 *
 * Safety (all automatic, no CLI needed):
 *   - Drive watchdog: the Teensy sends a CANopen heartbeat every 100 ms and each drive is set up as its
 *     heartbeat consumer (1016h). If the Teensy hangs or the CAN cable comes off, the drives fault
 *     (Er903 heartbeat timeout) and stop on their own after DRIVE_WATCHDOG_MS.
 *   - A faulted or disabled drive is fault-reset and re-enabled on the next motion command.
 *   - A drive that reboots (boot-up frame) is reconfigured; a drive missing at startup is retried every 2 s.
 *   - Speeds are capped to the motor's max speed read from the drive (2711h).
 *
 * USB serial (115200) only prints status / faults for debugging; it does not take commands.
 *
 * Wiring (one transceiver per bus, e.g. SN65HVD230 at 3.3V with Rs pin to GND):
 *   CAN1: Teensy pin 22 (CTX1) -> transceiver TXD, pin 23 (CRX1) -> transceiver RXD
 *   CAN2: Teensy pin 1  (CTX2) -> transceiver TXD, pin 0  (CRX2) -> transceiver RXD
 *   Transceiver CANH/CANL/GND -> drive CN5 pin 1 (CANH) / pin 3 (CANL) / pin 5 (GND)
 *   Each bus needs 120R at both ends (drive DIP SW6 = ON + 120R at the transceiver)
 *
 * "L" and "R" are the old module's motor channels: drive 0 of DRIVE_CFG is channel L, drive 1 is channel R.
 * If 11-20 turns the wrong way, swap the two DRIVE_CFG entries; if one wheel runs backwards, flip its sign.
 */

#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>

// ---------------- User configuration ----------------
struct DriveConfig { uint8_t port; uint8_t id; int8_t sign; };   // port: 1 = CAN1 (pins 22/23), 2 = CAN2 (pins 1/0)
static const DriveConfig DRIVE_CFG[]   = {{1, 16, +1},      // channel L: drive on CAN1, node ID 16
                                          {2, 16, -1}};     // channel R: drive on CAN2, node ID 16 (mirrored mount)
static uint32_t       canBaud          = 1000000;   // must match the drives (scanned at boot if nothing answers)
static const uint8_t  I2C_ADDRESS      = 0x72;

static const int32_t  NORMAL_RPM       = 500;       // straight 1/2; also scales normal turns and arcs
static const int32_t  BOOST_RPM        = 1000;      // straight 1/2 with boost (65); also scales boosted arcs
static const int32_t  TURBO_TURN_RPM   = 700;       // fastest in-place turn with turbo turn (66)

static const uint32_t RAMP_RPM_PER_S   = 1000;      // speed-change rate of the wheel that changes most
static const uint32_t MIN_RAMP_RPM_PER_S = 50;      // floor for the slower, synchronised wheel
static const uint32_t HARD_STOP_RPM_PER_S = 60000;  // hard-stop deceleration. 60000 rpm/s = the drive's own factory
                                                    // 6084h (10,000,000 counts/s^2 at 10000 ppr): steep enough that the
                                                    // drive just brakes at its torque limit. Lower it if a drive trips
                                                    // on DC-bus overvoltage (regen) when stopping from full speed.
static const uint32_t COMMAND_TIMEOUT_MS = 1000;    // no command for this long -> hard stop (old module: 1000 ms)
static const uint32_t REFRESH_MS       = 250;       // re-send the current targets this often while moving

static const bool     DRIVE_WATCHDOG   = true;      // drives stop themselves if the Teensy goes silent (1016h)
static const uint8_t  MASTER_NODE_ID   = 0x7F;      // heartbeat COB-ID 0x77F; must not clash with a drive ID
static const uint16_t HEARTBEAT_MS     = 100;
static const uint16_t DRIVE_WATCHDOG_MS = 500;      // drive faults (Er903) after this long without a heartbeat

static const uint32_t FALLBACK_PPR     = 10000;     // command units per rev if it can't be read
static const uint32_t OFFLINE_RETRY_MS = 2000;
// ----------------------------------------------------

static const uint8_t NUM_DRIVES = sizeof(DRIVE_CFG) / sizeof(DRIVE_CFG[0]);
static_assert(NUM_DRIVES == 2, "motion tables are written for exactly two drives (L, R)");

struct Drive {
  uint8_t  id;
  uint8_t  port;
  uint32_t countsPerRev;
  int32_t  targetRpm;
  int32_t  maxRpm;        // motor max speed read from 2711h, 0 = unknown
  uint32_t accelCounts;   // last 6083h / 6084h written (0 = unknown, write next time)
  uint32_t decelCounts;
  bool     online;
  bool     configured;    // setupDrive() finished; a boot-up frame after this means the drive rebooted
  bool     needsSetup;
  bool     bootupSeen;
  bool     reportedOffline;
  uint8_t  rpdoType;      // 1 = synchronous, 255 = asynchronous, 0 = RPDO not configured (SDO fallback)
};
static Drive drives[NUM_DRIVES];

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // pin 22 = CTX1, pin 23 = CRX1
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;  // pin 1  = CTX2, pin 0  = CRX2

static bool canWrite(uint8_t port, const CAN_message_t &m) { return port == 2 ? can2.write(m) : can1.write(m); }
static bool canRead(uint8_t port, CAN_message_t &m)        { return port == 2 ? can2.read(m)  : can1.read(m); }
static void canSetBaud(uint32_t baud)                      { can1.setBaudRate(baud); can2.setBaudRate(baud); }
static const char *canPins(uint8_t port)                   { return port == 2 ? "pin 1 TX / pin 0 RX" : "pin 22 TX / pin 23 RX"; }

// CANopen object indexes (DS301 / DSP402)
static const uint16_t OD_HEARTBEAT_CONSUMER = 0x1016;  // sub 1: (producer node ID << 16) | timeout ms
static const uint16_t OD_RPDO1_COMM    = 0x1400;  // sub 1: COB-ID, sub 2: transmission type
static const uint16_t OD_RPDO1_MAP     = 0x1600;
static const uint16_t OD_CONTROLWORD   = 0x6040;
static const uint16_t OD_STATUSWORD    = 0x6041;
static const uint16_t OD_ERROR_CODE    = 0x603F;
static const uint16_t OD_MODE_OF_OP    = 0x6060;
static const uint16_t OD_PROFILE_ACCEL = 0x6083;
static const uint16_t OD_PROFILE_DECEL = 0x6084;
static const uint16_t OD_FEED_CONSTANT = 0x6092;  // sub 1: command units per motor rev
static const uint16_t OD_TARGET_VEL    = 0x60FF;
static const uint16_t OD_PR0_08        = 0x2008;  // Pr0.08 pulses per rev (overrides 6092h if != 0)
static const uint16_t OD_MOTOR_RATED_RPM = 0x2710;  // Pr7.10, read-only
static const uint16_t OD_MOTOR_MAX_RPM   = 0x2711;  // Pr7.11, read-only; drive faults (Er1A0) at 1.2x this

static const uint8_t MODE_PROFILE_VELOCITY = 3;

static const uint8_t NMT_START  = 0x01;
static const uint8_t NMT_PRE_OP = 0x80;
static const uint8_t NMT_RESET  = 0x81;

static int32_t motorMaxRpm = 0;  // smallest motor max speed of the online drives, 0 = unknown (no cap)

// ---------------- CAN / CANopen helpers ----------------

// Sends the Teensy's heartbeat when due. Called from every wait loop too, so the drive watchdog never
// trips while the Teensy is busy (SDO waits, enabling, ...).
static void heartbeatTick() {
  static uint32_t last = 0;
  if (!DRIVE_WATCHDOG || millis() - last < HEARTBEAT_MS) return;
  last = millis();
  bool sent[3] = {false, false, false};
  for (Drive &d : drives) {
    if (!d.configured || sent[d.port]) continue;  // only on buses with a drive, an unacked frame would jam the bus
    CAN_message_t m;
    m.id = 0x700u + MASTER_NODE_ID;
    m.len = 1;
    m.buf[0] = 0x05;  // NMT state "operational"
    canWrite(d.port, m);
    sent[d.port] = true;
  }
}

static Drive *findDrive(uint8_t port, uint8_t id) {
  for (Drive &d : drives)
    if (d.port == port && d.id == id) return &d;
  return nullptr;
}

static void handleAsyncFrame(uint8_t port, const CAN_message_t &m) {
  if (m.id > 0x700 && m.id <= 0x77F && m.len >= 1 && m.buf[0] == 0x00) {
    if (Drive *d = findDrive(port, m.id - 0x700)) {
      d->bootupSeen = true;
      if (d->configured) d->needsSetup = true;  // drive rebooted while running: reconfigure it
    }
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
  heartbeatTick();
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
                        uint32_t timeoutMs = 100, bool quiet = false) {
  CAN_message_t tx;
  tx.id = 0x600u + node;
  tx.len = 8;
  memcpy(tx.buf, req, 8);
  canWrite(port, tx);

  uint32_t start = millis();
  CAN_message_t rx;
  while (millis() - start < timeoutMs) {
    heartbeatTick();
    if (!canRead(port, rx)) continue;
    if (rx.id != 0x580u + node || rx.len < 8 ||
        rx.buf[1] != req[1] || rx.buf[2] != req[2] || rx.buf[3] != req[3]) {
      handleAsyncFrame(port, rx);
      continue;
    }
    memcpy(reply, rx.buf, 8);
    if (rx.buf[0] == 0x80) {
      uint32_t abortCode = rx.buf[4] | (rx.buf[5] << 8) | (rx.buf[6] << 16) | ((uint32_t)rx.buf[7] << 24);
      if (!quiet)
        Serial.printf("CAN%u node %u: SDO abort 0x%04X:%02X code 0x%08lX\n",
                      port, node, req[1] | (req[2] << 8), req[3], (unsigned long)abortCode);
      return false;
    }
    return true;
  }
  if (!quiet)
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
static bool sdoRead(const Drive &d, uint16_t index, uint8_t sub, int32_t &value, bool quiet = false) {
  uint8_t req[8] = {0x40, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0};
  uint8_t reply[8];
  return sdoTransfer(d.port, d.id, req, reply, 100, quiet) && decodeSdoRead(reply, value);
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

static bool readState(const Drive &d, Ds402State &state) {
  int32_t sw;
  if (!sdoRead(d, OD_STATUSWORD, 0, sw)) return false;
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

static uint32_t rampToCounts(const Drive &d, uint32_t rpmPerS) {
  return (uint32_t)((uint64_t)rpmPerS * d.countsPerRev / 60);
}

static bool anyOnline() {
  for (Drive &d : drives)
    if (d.online) return true;
  return false;
}

// ---------------- Speeds and ramps ----------------

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

// Applies rpm[i] to drives[i] at the same instant (RPDOs first, then one SYNC per bus).
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

// Writes 6083h / 6084h only when they change. Both are set to the same value: depending on the drive,
// slowing down while reversing can count as "acceleration", so a stop must not depend on 6084h alone.
static void setRamp(Drive &d, uint32_t rpmPerS) {
  uint32_t counts = rampToCounts(d, rpmPerS);
  if (counts == 0) counts = 1;
  if (counts != d.accelCounts && sdoWrite(d, OD_PROFILE_ACCEL, 0, counts, 4)) d.accelCounts = counts;
  if (counts != d.decelCounts && sdoWrite(d, OD_PROFILE_DECEL, 0, counts, 4)) d.decelCounts = counts;
}

// Ramps for moving from `from` to `to`: the wheel with the bigger change gets RAMP_RPM_PER_S, the other one
// is scaled down so both arrive together and the robot follows the new curve the whole way.
static void setSyncedRamps(const int32_t from[NUM_DRIVES], const int32_t to[NUM_DRIVES]) {
  uint32_t delta[NUM_DRIVES];
  uint32_t maxDelta = 0;
  for (uint8_t i = 0; i < NUM_DRIVES; i++) {
    delta[i] = (uint32_t)abs(to[i] - from[i]);
    maxDelta = max(maxDelta, delta[i]);
  }
  if (maxDelta == 0) return;
  for (uint8_t i = 0; i < NUM_DRIVES; i++) {
    if (!drives[i].online || delta[i] == 0) continue;
    uint32_t rate = (uint32_t)((uint64_t)RAMP_RPM_PER_S * delta[i] / maxDelta);
    setRamp(drives[i], max(rate, MIN_RAMP_RPM_PER_S));
  }
}

static void setHardStopRamps() {
  for (Drive &d : drives)
    if (d.online) setRamp(d, HARD_STOP_RPM_PER_S);
}

// ---------------- Enable / disable ----------------

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

// Enables the servos first if a non-zero speed is requested. Returns false if they would not enable.
static bool setSpeeds(const int32_t rpm[NUM_DRIVES]) {
  bool anyMotion = false;
  for (uint8_t i = 0; i < NUM_DRIVES; i++) anyMotion |= rpm[i] != 0;
  if (anyMotion && !allEnabled() && !servoEnableAll()) return false;
  applySpeeds(rpm);
  return true;
}

// ---------------- Drive setup ----------------

static void readCountsPerRev(Drive &d) {
  int32_t v;
  if (sdoRead(d, OD_PR0_08, 0, v) && v > 0) {
    d.countsPerRev = (uint32_t)v;
  } else if (sdoRead(d, OD_FEED_CONSTANT, 1, v) && v > 0) {
    d.countsPerRev = (uint32_t)v;
  } else {
    d.countsPerRev = FALLBACK_PPR;
    Serial.printf("CAN%u node %u: could not read counts/rev, assuming %lu\n", d.port, d.id, (unsigned long)d.countsPerRev);
  }
}

static void updateMotorMaxRpm() {
  motorMaxRpm = 0;
  for (Drive &d : drives)
    if (d.online && d.maxRpm > 0) motorMaxRpm = motorMaxRpm ? min(motorMaxRpm, d.maxRpm) : d.maxRpm;
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
  if (!sdoWrite(d, OD_RPDO1_COMM, 1, cobId, 4)) d.rpdoType = 0;
}

// Brings one drive from power-up (or reboot) to ready: everything that is lost when a drive restarts.
static bool setupDrive(Drive &d) {
  d.configured = false;
  d.needsSetup = false;
  sendNmt(d.port, NMT_PRE_OP, d.id);
  int32_t dummy;
  d.online = sdoRead(d, OD_STATUSWORD, 0, dummy, d.reportedOffline);
  if (!d.online) {
    if (!d.reportedOffline)
      Serial.printf("CAN%u node %u: not answering (%s) - check power, node ID, baud rate, wiring. Retrying every %lu s\n",
                    d.port, d.id, canPins(d.port), (unsigned long)(OFFLINE_RETRY_MS / 1000));
    d.reportedOffline = true;
    updateMotorMaxRpm();
    return false;
  }
  d.reportedOffline = false;

  readCountsPerRev(d);
  int32_t ratedRpm = 0, maxRpm = 0;
  sdoRead(d, OD_MOTOR_RATED_RPM, 0, ratedRpm);
  sdoRead(d, OD_MOTOR_MAX_RPM, 0, maxRpm);
  d.maxRpm = (maxRpm >= 100 && maxRpm <= 20000) ? maxRpm : 0;  // ignore values that don't look like rpm
  Serial.printf("CAN%u node %u: online, %lu counts/rev, motor rated %ld rpm, max %ld rpm\n", d.port, d.id,
                (unsigned long)d.countsPerRev, (long)ratedRpm, (long)maxRpm);

  d.accelCounts = d.decelCounts = 0;
  setRamp(d, RAMP_RPM_PER_S);
  sdoWrite(d, OD_MODE_OF_OP, 0, MODE_PROFILE_VELOCITY, 1);
  configureRpdo1(d);
  if (d.rpdoType == 0) Serial.printf("CAN%u node %u: RPDO setup failed, using SDO (not simultaneous)\n", d.port, d.id);

  if (DRIVE_WATCHDOG) {
    if (sdoWrite(d, OD_HEARTBEAT_CONSUMER, 1, ((uint32_t)MASTER_NODE_ID << 16) | DRIVE_WATCHDOG_MS, 4))
      Serial.printf("CAN%u node %u: watchdog on - stops itself if the Teensy is silent for %u ms\n",
                    d.port, d.id, DRIVE_WATCHDOG_MS);
    else
      Serial.printf("CAN%u node %u: watchdog not accepted (1016h) - drive will NOT stop on its own\n", d.port, d.id);
  }

  sendNmt(d.port, NMT_START, d.id);  // PDOs only work in NMT operational
  d.targetRpm = 0;
  d.configured = true;
  updateMotorMaxRpm();
  return true;
}

static void initDrives() {
  // Reset every drive on its own bus and wait for the boot-up messages
  for (Drive &d : drives) {
    d.configured = false;
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
  for (Drive &d : drives) setupDrive(d);
}

// Tries every baud rate the ELD2-CAN supports, adopts the first one where a drive boots up.
static bool scanBus() {
  static const uint32_t bauds[] = {1000000, 800000, 500000, 250000, 125000};
  Serial.println("No drive answered - scanning baud rates on CAN1 and CAN2...");
  for (uint32_t baud : bauds) {
    canSetBaud(baud);
    delay(20);
    pollCan();
    sendNmt(1, NMT_RESET, 0x00);
    sendNmt(2, NMT_RESET, 0x00);
    uint32_t start = millis();
    CAN_message_t m;
    while (millis() - start < 2500) {
      for (uint8_t port = 1; port <= 2; port++) {
        if (canRead(port, m) && m.id > 0x700 && m.id <= 0x77F) {
          canBaud = baud;
          Serial.printf("  node %lu answered on CAN%u at %lu bit/s\n", (unsigned long)(m.id - 0x700), port,
                        (unsigned long)baud);
          delay(500);  // let the other drive boot too
          return true;
        }
      }
    }
  }
  canSetBaud(canBaud);
  Serial.println("  no drive found at any baud rate - check transceivers and wiring");
  return false;
}

// Reconfigures drives that rebooted and retries drives that are missing.
static void maintainDrives() {
  static uint32_t lastRetry = 0;
  for (Drive &d : drives) {
    if (!d.needsSetup) continue;
    Serial.printf("CAN%u node %u: drive rebooted - reconfiguring\n", d.port, d.id);
    delay(100);
    setupDrive(d);
  }
  if (millis() - lastRetry < OFFLINE_RETRY_MS) return;
  lastRetry = millis();
  for (Drive &d : drives)
    if (!d.online && setupDrive(d)) Serial.printf("CAN%u node %u: drive found\n", d.port, d.id);
}

// ---------------- Motion commands -> wheel speeds ----------------

static bool turnBoost = false;      // 66 toggles (turbo turn)
static bool straightBoost = false;  // 65 toggles (straight + arcs)

// The old module's PWM figures, signed (+ = forward), plus the scale that turns them into rpm:
// rpm = pwm * scaleRpm / scalePwm. Keeping the old figures keeps every code's left/right proportions.
struct Wheels { int pwmL; int pwmR; int32_t scaleRpm; int32_t scalePwm; };

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
static const int TURN_PWM[3][2]  = {{80, 60}, {120, 80}, {180, 120}};                                // {turbo, normal}
static const int FOLLOW_TURN_PWM[3] = {90, 130, 190};                                                // 31-50

// Straight, slow straight and arcs: boosted by 65. Old straight PWM 200 = NORMAL_RPM, PWM 255 = BOOST_RPM.
static Wheels straightGroup(int pwmL, int pwmR) {
  if (straightBoost) return {pwmL, pwmR, BOOST_RPM, 255};
  return {pwmL, pwmR, NORMAL_RPM, 200};
}

static Wheels arc(const int t[3][4], int cmd, int lo, int dir) {
  const int *row = t[tierOf(cmd, lo)];
  return straightGroup(dir * pick(row[0], row[1], straightBoost), dir * pick(row[2], row[3], straightBoost));
}

// dir +1: L forward / R backward. Turbo: old PWM 180 = TURBO_TURN_RPM; normal uses the straight scale.
static Wheels turnInPlace(int cmd, int lo, int dir) {
  int t = tierOf(cmd, lo);
  if (turnBoost) return {dir * TURN_PWM[t][0], -dir * TURN_PWM[t][0], TURBO_TURN_RPM, 180};
  return {dir * TURN_PWM[t][1], -dir * TURN_PWM[t][1], NORMAL_RPM, 200};
}

static Wheels followTurn(int cmd, int lo, int dir) {
  int p = FOLLOW_TURN_PWM[tierOf(cmd, lo)];
  return {dir * p, -dir * p, NORMAL_RPM, 200};
}

// Fills `w` for a motion code. Returns false for 0 and anything that is not a motion (= stop).
static bool motionToWheels(int cmd, Wheels &w) {
  if (cmd == 172) cmd = 115;  // d-pad aliases, same as the old module
  if (cmd == 174) cmd = 125;
  if (cmd == 272) cmd = 215;
  if (cmd == 274) cmd = 225;

  if (cmd >= 1 && cmd <= 4) {
    int p = cmd <= 2 ? pick(255, 200, straightBoost) : pick(150, 120, straightBoost);
    int s = (cmd == 1 || cmd == 3) ? 1 : -1;
    w = straightGroup(s * p, s * p);
    return true;
  }
  if (cmd >= 11  && cmd <= 20)  { w = turnInPlace(cmd, 11, -1); return true; }  // right: L back, R fwd
  if (cmd >= 21  && cmd <= 30)  { w = turnInPlace(cmd, 21, +1); return true; }  // left:  L fwd,  R back
  if (cmd >= 31  && cmd <= 40)  { w = followTurn(cmd, 31, +1);  return true; }
  if (cmd >= 41  && cmd <= 50)  { w = followTurn(cmd, 41, -1);  return true; }
  if (cmd >= 111 && cmd <= 120) { w = arc(ARC_FWD_A, cmd, 111, +1); return true; }
  if (cmd >= 121 && cmd <= 130) { w = arc(ARC_FWD_B, cmd, 121, +1); return true; }
  if (cmd >= 211 && cmd <= 220) { w = arc(ARC_BWD_A, cmd, 211, -1); return true; }
  if (cmd >= 221 && cmd <= 230) { w = arc(ARC_BWD_B, cmd, 221, -1); return true; }
  return false;
}

static int32_t wheelRpm(int pwm, const Wheels &w, int8_t sign) {
  int32_t rpm = (int32_t)sign * pwm * w.scaleRpm / w.scalePwm;
  return motorMaxRpm > 0 ? constrain(rpm, -motorMaxRpm, motorMaxRpm) : rpm;
}

static int motionCmd = 0;  // current command, 0 = stop
static uint32_t lastCmdMs = 0;

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
      Serial.printf("Boost %s, turbo turn %s\n", straightBoost ? "ON" : "OFF", turnBoost ? "ON" : "OFF");
    }
    lastCmdMs = millis();  // keep the current motion alive, like the old module
    return;
  }
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
  if (motionCmd != 0 && millis() - lastCmdMs > COMMAND_TIMEOUT_MS) {
    Serial.printf("No command for %lu ms - hard stop\n", (unsigned long)COMMAND_TIMEOUT_MS);
    motionCmd = 0;
  }

  Wheels w = {0, 0, NORMAL_RPM, 200};
  if (!motionToWheels(motionCmd, w)) w = {0, 0, NORMAL_RPM, 200};
  int32_t want[NUM_DRIVES] = {wheelRpm(w.pwmL, w, DRIVE_CFG[0].sign), wheelRpm(w.pwmR, w, DRIVE_CFG[1].sign)};

  bool changed = firstRun;
  for (uint8_t i = 0; i < NUM_DRIVES; i++) changed |= want[i] != last[i];
  bool moving = want[0] != 0 || want[1] != 0;
  if (!changed && !(moving && millis() - lastSend >= REFRESH_MS)) return;

  if (changed) {
    if (moving) setSyncedRamps(last, want);
    else        setHardStopRamps();
    Serial.printf("cmd %d -> L %ld rpm, R %ld rpm%s\n", motionCmd, (long)want[0], (long)want[1],
                  moving ? "" : " (hard stop)");
  }

  firstRun = false;
  lastSend = millis();
  if (setSpeeds(want)) {
    for (uint8_t i = 0; i < NUM_DRIVES; i++) last[i] = want[i];
  } else {
    Serial.println("Servos would not enable - holding stop, retrying");
    retryAt = millis() + 1000;  // retry in 1 s instead of every loop
  }
}

// ---------------- I2C commands ----------------

// Runs in interrupt context: only stash the number, setCommand() runs from loop().
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
  char *p = buf;
  while (*p == ' ' || *p == '\r' || *p == '\n') p++;
  if ((*p >= '0' && *p <= '9') || *p == '-') {
    i2cValue = atoi(p);
    i2cNew = true;
  }
}

static void handleI2c() {
  if (!i2cNew) return;
  noInterrupts();
  int v = i2cValue;
  i2cNew = false;
  interrupts();
  setCommand(v);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1000) {}
  Serial.printf("Motion Module ROBOT build (I2C 0x%02X), %u drives, normal %ld / boost %ld / turbo turn %ld rpm\n",
                I2C_ADDRESS, NUM_DRIVES, (long)NORMAL_RPM, (long)BOOST_RPM, (long)TURBO_TURN_RPM);

  for (uint8_t i = 0; i < NUM_DRIVES; i++) {
    drives[i] = {};
    drives[i].id = DRIVE_CFG[i].id;
    drives[i].port = DRIVE_CFG[i].port;
    drives[i].countsPerRev = FALLBACK_PPR;
  }

  can1.begin();
  can2.begin();
  canSetBaud(canBaud);

  initDrives();
  if (!anyOnline() && scanBus()) initDrives();
  if (anyOnline()) servoEnableAll();  // hold position at zero speed

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Serial.println("Ready for I2C commands");
}

void loop() {
  pollCan();         // also sends the heartbeat
  handleI2c();       // before updateMotion(), so a command that arrived during a long wait isn't timed out
  maintainDrives();
  updateMotion();
}
