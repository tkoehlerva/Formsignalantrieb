/*
  Uno_LNCV_Servo_Signals_V1_1.ino

  Version 1.2: adjustable endpoints and per-slot movement-speed setting.
  Hardware:
    - Arduino Uno
    - LocoNet library by Alex Shepherd (1.1.13)
    - Adafruit 16-channel PCA9685 Servo Shield / Adafruit_PWMServoDriver
    - PCA9685 at default I2C address, 50 Hz
    - LocoNet TX pin 7

  LNCV map:
    LNCV 0: module address
    LNCV 1: stress-test command (write 1 to start; reads as 0)
    LNCV 2: stress-test servo slot (1..16)
    LNCV 3: proceed/straight duration in seconds
    LNCV 4: stop/thrown duration in seconds
    LNCV 5..9: reserved (reads as 0; writes rejected)

    Each servo slot has 10 LNCVs:
      base + 0: slot type: 0=disabled, 1=enabled
      base + 1: LocoNet accessory address
      base + 2: PCA9685 channel (0..15)
      base + 3: bounce profile ID: 0=current bounce, 1=no bounce
      base + 4: thrown/STOP endpoint (PCA9685 count)
      base + 5: straight/PROCEED endpoint (PCA9685 count)
      base + 6: movement speed in percent (100 = original speed; larger = faster)
      base + 7..9: reserved
    where base = 10 + (slotNumber - 1) * 10.

  Important:
    - Speed percentage: 25 = quarter speed, 50 = half speed, 100 = original, 150 = 1.5x, 200 = 2x. Larger values move faster.
    - Each slot is an independent physical servo.
    - Thrown (Direction 0) uses that slot's RED/STOP endpoint.
    - Straight (Direction 1) uses that slot's GREEN/PROCEED endpoint.
    - A multi-servo signal uses multiple independently configured slots/addresses.
    - The three-aspect signal logic is external: Rocrail/IB1 sends the needed
      commands to the relevant accessory addresses.
    - Endpoint defaults preserve the current calibration: slots 1..5 use
      255/225, slot 6 uses 270/390, slot 7 uses 250/405, and slots 8..16
      use 250/405. These can be adjusted independently via LNCV.
    - This file has not been compiled against the user's installed libraries
      or tested on hardware yet.
*/

#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LocoNet.h>

// ---------------- Hardware ----------------
const uint8_t LOCONET_TX_PIN = 7;
const uint16_t ARTICLE_NUMBER = 634;
const uint16_t EEPROM_MAGIC = 0x4C56; // schema changed in V1.2
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_MODULE_ADDR = 2;
const int EEPROM_GLOBAL_ADDR = 4;
const int EEPROM_SLOT_ADDR = 16;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---------------- LNCV definitions ----------------
const uint8_t SLOT_COUNT = 16;
const uint16_t DEFAULT_MODULE_ADDRESS = 1;
const uint16_t DEFAULT_FIRST_ACCESSORY_ADDRESS = 200;

const uint16_t SLOT_DISABLED = 0;
const uint16_t SLOT_ENABLED = 1;
const uint16_t BOUNCE_DEFAULT = 0;
const uint16_t BOUNCE_NONE = 1;

struct GlobalConfig {
  uint16_t testSlot;          // 1..16
  uint16_t proceedSeconds;    // stress-test straight duration
  uint16_t stopSeconds;       // stress-test thrown duration
};

struct ServoSlotConfig {
  uint16_t type;              // 0 disabled, 1 enabled
  uint16_t accessoryAddress;  // LocoNet accessory address
  uint16_t channel;           // PCA9685 channel
  uint16_t bounceProfile;     // 0 current bounce, 1 no bounce
  uint16_t thrownEndpoint;    // STOP endpoint, PCA9685 count
  uint16_t straightEndpoint;  // PROCEED endpoint, PCA9685 count
  uint16_t speedPercent;      // 100 = original speed; higher = faster
};

GlobalConfig globals;
ServoSlotConfig slots[SLOT_COUNT];
uint16_t moduleAddress = DEFAULT_MODULE_ADDRESS;
bool programmingActive = false;
LocoNetCVClass LocoNetCV;

// ---------------- Calibration ----------------
// Original calibrated endpoints, retained for the first nine physical channels.
// Each slot independently moves between its thrown and straight endpoints.
const int16_t defaultThrown[SLOT_COUNT] = {
  255, 255, 255, 255, 255, 270, 250, 250,
  250, 250, 250, 250, 250, 250, 250, 250
};
const int16_t defaultStraight[SLOT_COUNT] = {
  225, 225, 225, 225, 225, 390, 405, 405,
  405, 405, 405, 405, 405, 405, 405, 405
};

int currentPos[16];

// Current/default bounce profile 0.
const int16_t defaultBounceOffsets[4] = {5, -3, 2, 0};
const uint16_t defaultBounceDelays[4] = {20, 30, 40, 55};

// ---------------- Stress test ----------------
bool testRunning = false;
bool testStraightPhase = false;
unsigned long testNextAction = 0;

// ---------------- Forward declarations ----------------
void setDefaults();
void loadConfig();
void saveConfig();
void setupServoDriver();
void initServoPositions();
void handleSerialCommand(String cmd);
void handleSlotCommand(uint8_t slotIndex, bool straight);
void allStop();
void moveServoSlot(uint8_t slotIndex, bool straight);
void moveWithBounce(uint8_t channel, int from, int to, uint16_t profile, uint16_t speedPercent);
void slowMove(uint8_t channel, int from, int to, uint16_t delayMs);
void setServo(uint8_t channel, int pulse);
void startStressTest();
void stopStressTest();
bool validAccessoryAddress(uint16_t value);
uint16_t slotBase(uint8_t slotIndex);

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  Wire.begin();
  setupServoDriver();

  loadConfig();
  initServoPositions();
  LocoNet.init(LOCONET_TX_PIN);

  Serial.println(F("\nUno LNCV Servo Controller - Version 1"));
  Serial.print(F("Article number: ")); Serial.println(ARTICLE_NUMBER);
  Serial.print(F("LNCV module address: ")); Serial.println(moduleAddress);
  Serial.println(F("Serial: 1r..16r, 1g..16g, a0, t1, t0, ?"));
  Serial.println(F("LNCV blocks: 10-19, 20-29, ... 160-169"));
}

void loop() {
  lnMsg* packet;
  while ((packet = LocoNet.receive()) != nullptr) {
    LocoNet.processSwitchSensorMessage(packet);
    LocoNetCV.processLNCVMessage(packet);
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleSerialCommand(cmd);
  }

  if (testRunning && (long)(millis() - testNextAction) >= 0) {
    uint8_t idx = (uint8_t)(globals.testSlot - 1);
    if (!testStraightPhase) {
      moveServoSlot(idx, true);
      testStraightPhase = true;
      testNextAction = millis() + (unsigned long)globals.proceedSeconds * 1000UL;
      Serial.print(F("Stress test: slot "));
      Serial.print(globals.testSlot);
      Serial.println(F(" straight"));
    } else {
      moveServoSlot(idx, false);
      testStraightPhase = false;
      testNextAction = millis() + (unsigned long)globals.stopSeconds * 1000UL;
      Serial.print(F("Stress test: slot "));
      Serial.print(globals.testSlot);
      Serial.println(F(" thrown"));
    }
  }
}

// ---------------- Defaults / EEPROM ----------------
void setDefaults() {
  globals.testSlot = 1;
  globals.proceedSeconds = 5;
  globals.stopSeconds = 10;

  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    slots[i].type = SLOT_ENABLED;
    slots[i].accessoryAddress = DEFAULT_FIRST_ACCESSORY_ADDRESS + i;
    slots[i].channel = i;
    slots[i].bounceProfile = BOUNCE_DEFAULT;
    slots[i].thrownEndpoint = defaultThrown[i];
    slots[i].straightEndpoint = defaultStraight[i];
    slots[i].speedPercent = 100;
  }
}

void saveConfig() {
  EEPROM.put(EEPROM_MODULE_ADDR, moduleAddress);
  EEPROM.put(EEPROM_GLOBAL_ADDR, globals);
  EEPROM.put(EEPROM_SLOT_ADDR, slots);
}

void loadConfig() {
  uint16_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC) {
    moduleAddress = DEFAULT_MODULE_ADDRESS;
    setDefaults();
    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    saveConfig();
    return;
  }

  EEPROM.get(EEPROM_MODULE_ADDR, moduleAddress);
  EEPROM.get(EEPROM_GLOBAL_ADDR, globals);
  EEPROM.get(EEPROM_SLOT_ADDR, slots);

  bool repair = false;
  if (moduleAddress == 0 || moduleAddress == 65535) {
    moduleAddress = DEFAULT_MODULE_ADDRESS;
    repair = true;
  }
  if (globals.testSlot < 1 || globals.testSlot > SLOT_COUNT) {
    globals.testSlot = 1;
    repair = true;
  }
  if (globals.proceedSeconds < 1 || globals.proceedSeconds > 600) {
    globals.proceedSeconds = 5;
    repair = true;
  }
  if (globals.stopSeconds < 1 || globals.stopSeconds > 600) {
    globals.stopSeconds = 10;
    repair = true;
  }

  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    if (slots[i].type > SLOT_ENABLED) {
      slots[i].type = SLOT_ENABLED;
      repair = true;
    }
    if (!validAccessoryAddress(slots[i].accessoryAddress)) {
      slots[i].accessoryAddress = DEFAULT_FIRST_ACCESSORY_ADDRESS + i;
      repair = true;
    }
    if (slots[i].channel > 15) {
      slots[i].channel = i;
      repair = true;
    }
    if (slots[i].bounceProfile > BOUNCE_NONE) {
      slots[i].bounceProfile = BOUNCE_DEFAULT;
      repair = true;
    }
    if (slots[i].thrownEndpoint < 120 || slots[i].thrownEndpoint > 560) {
      slots[i].thrownEndpoint = defaultThrown[i];
      repair = true;
    }
    if (slots[i].straightEndpoint < 120 || slots[i].straightEndpoint > 560) {
      slots[i].straightEndpoint = defaultStraight[i];
    slots[i].speedPercent = 100;
      repair = true;
    }
  }
  if (repair) saveConfig();
}

void setupServoDriver() {
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);
}

void initServoPositions() {
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    currentPos[slots[i].channel] = slots[i].thrownEndpoint;
    if (slots[i].type == SLOT_ENABLED) {
      pwm.setPWM(slots[i].channel, 0, slots[i].thrownEndpoint);
    }
  }
}

// ---------------- LocoNet callbacks ----------------
void notifySwitchRequest(uint16_t Address, uint8_t Output, uint8_t Direction) {
  if (Output == 0) return;

  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    if (slots[i].type == SLOT_ENABLED &&
        Address == slots[i].accessoryAddress) {
      stopStressTest();
      moveServoSlot(i, Direction != 0);
      Serial.print(F("LocoNet address "));
      Serial.print(Address);
      Serial.print(F(" -> slot "));
      Serial.print(i + 1);
      Serial.println(Direction == 0 ? F(" thrown") : F(" straight"));
      return;
    }
  }
}

int8_t notifyLNCVprogrammingStart(uint16_t& artNr, uint16_t& requestedModuleAddress) {
  Serial.println(F("LNCV programming start received"));
  Serial.print(F("Article requested: ")); Serial.println(artNr);
  Serial.print(F("Module requested: ")); Serial.println(requestedModuleAddress);

  if (artNr != ARTICLE_NUMBER && artNr != 65535) return -1;
  if (requestedModuleAddress != moduleAddress &&
      requestedModuleAddress != 65535) return -1;

  artNr = ARTICLE_NUMBER;
  requestedModuleAddress = moduleAddress;
  programmingActive = true;
  stopStressTest();
  return LNCV_LACK_OK;
}

int8_t notifyLNCVread(uint16_t artNr, uint16_t lncvAddress, uint16_t& value) {
  if (!programmingActive || artNr != ARTICLE_NUMBER) return -1;

  if (lncvAddress == 0) {
    value = moduleAddress;
    return LNCV_LACK_OK;
  }

  if (lncvAddress == 1) { value = 0; return LNCV_LACK_OK; }
  if (lncvAddress == 2) { value = globals.testSlot; return LNCV_LACK_OK; }
  if (lncvAddress == 3) { value = globals.proceedSeconds; return LNCV_LACK_OK; }
  if (lncvAddress == 4) { value = globals.stopSeconds; return LNCV_LACK_OK; }
  if (lncvAddress >= 5 && lncvAddress <= 9) {
    value = 0;
    return LNCV_LACK_OK;
  }

  if (lncvAddress >= 10 && lncvAddress <= 169) {
    uint8_t slotIndex = (lncvAddress - 10) / 10;
    uint8_t offset = (lncvAddress - 10) % 10;
    switch (offset) {
      case 0: value = slots[slotIndex].type; break;
      case 1: value = slots[slotIndex].accessoryAddress; break;
      case 2: value = slots[slotIndex].channel; break;
      case 3: value = slots[slotIndex].bounceProfile; break;
      case 4: value = slots[slotIndex].thrownEndpoint; break;
      case 5: value = slots[slotIndex].straightEndpoint; break;
      case 6: value = slots[slotIndex].speedPercent; break;
      default: value = 0; break;
    }
    return LNCV_LACK_OK;
  }

  return LNCV_LACK_ERROR_UNSUPPORTED;
}

int8_t notifyLNCVwrite(uint16_t artNr, uint16_t lncvAddress, uint16_t value) {
  if (!programmingActive || artNr != ARTICLE_NUMBER) return -1;

  if (lncvAddress == 0) {
    if (value == 0 || value == 65535) return LNCV_LACK_ERROR_OUTOFRANGE;
    moduleAddress = value;
    saveConfig();
    return LNCV_LACK_OK;
  }

  if (lncvAddress == 1) {
    if (value == 1) startStressTest();
    else if (value == 0) stopStressTest();
    else return LNCV_LACK_ERROR_OUTOFRANGE;
    return LNCV_LACK_OK;
  }

  if (lncvAddress == 2) {
    if (value < 1 || value > SLOT_COUNT) return LNCV_LACK_ERROR_OUTOFRANGE;
    globals.testSlot = value;
    saveConfig();
    return LNCV_LACK_OK;
  }
  if (lncvAddress == 3 || lncvAddress == 4) {
    if (value < 1 || value > 600) return LNCV_LACK_ERROR_OUTOFRANGE;
    if (lncvAddress == 3) globals.proceedSeconds = value;
    else globals.stopSeconds = value;
    saveConfig();
    return LNCV_LACK_OK;
  }
  if (lncvAddress >= 5 && lncvAddress <= 9) {
    return LNCV_LACK_ERROR_UNSUPPORTED;
  }

  if (lncvAddress >= 10 && lncvAddress <= 169) {
    uint8_t slotIndex = (lncvAddress - 10) / 10;
    uint8_t offset = (lncvAddress - 10) % 10;
    switch (offset) {
      case 0:
        if (value > SLOT_ENABLED) return LNCV_LACK_ERROR_OUTOFRANGE;
        slots[slotIndex].type = value;
        break;
      case 1:
        if (!validAccessoryAddress(value)) return LNCV_LACK_ERROR_OUTOFRANGE;
        slots[slotIndex].accessoryAddress = value;
        break;
      case 2:
        if (value > 15) return LNCV_LACK_ERROR_OUTOFRANGE;
        slots[slotIndex].channel = value;
        break;
      case 3:
        if (value > BOUNCE_NONE) return LNCV_LACK_ERROR_OUTOFRANGE;
        slots[slotIndex].bounceProfile = value;
        break;
      case 4:
        if (value < 120 || value > 560) return LNCV_LACK_ERROR_OUTOFRANGE;
        slots[slotIndex].thrownEndpoint = value;
        currentPos[slots[slotIndex].channel] = value;
        if (slots[slotIndex].type == SLOT_ENABLED) pwm.setPWM(slots[slotIndex].channel, 0, value);
        break;
      case 5:
        if (value < 120 || value > 560) return LNCV_LACK_ERROR_OUTOFRANGE;
        slots[slotIndex].straightEndpoint = value;
        break;
      case 6:
        if (value < 25 || value > 200) return LNCV_LACK_ERROR_OUTOFRANGE;
        slots[slotIndex].speedPercent = value;
        break;
      default:
        return LNCV_LACK_ERROR_UNSUPPORTED;
    }
    saveConfig();
    Serial.print(F("Saved LNCV "));
    Serial.print(lncvAddress);
    Serial.print(F(" = "));
    Serial.println(value);
    return LNCV_LACK_OK;
  }

  return LNCV_LACK_ERROR_UNSUPPORTED;
}

void notifyLNCVprogrammingStop(uint16_t artNr, uint16_t requestedModuleAddress) {
  (void)requestedModuleAddress;
  if (artNr == ARTICLE_NUMBER || artNr == 65535) {
    programmingActive = false;
    stopStressTest();
    Serial.println(F("LNCV programming stopped."));
  }
}

// ---------------- Servo movement ----------------
bool validAccessoryAddress(uint16_t value) {
  // LocoNet accessory addresses are represented here as positive 1-based values.
  return value >= 1 && value <= 2048;
}

uint16_t slotBase(uint8_t slotIndex) {
  return 10 + (uint16_t)slotIndex * 10;
}

void moveServoSlot(uint8_t slotIndex, bool straight) {
  if (slotIndex >= SLOT_COUNT || slots[slotIndex].type != SLOT_ENABLED) return;

  uint8_t channel = (uint8_t)slots[slotIndex].channel;
  int target = straight ? slots[slotIndex].straightEndpoint : slots[slotIndex].thrownEndpoint;
  int from = currentPos[channel];
  if (from == target) return;

  moveWithBounce(channel, from, target, slots[slotIndex].bounceProfile, slots[slotIndex].speedPercent);
  currentPos[channel] = target;
}

void moveWithBounce(uint8_t channel, int from, int to, uint16_t profile, uint16_t speedPercent) {
  // 100% preserves the original timing. Higher percentages shorten delays
  // and therefore move faster; lower percentages lengthen delays.
  speedPercent = constrain(speedPercent, 25, 200);
  if (profile == BOUNCE_NONE) {
    uint16_t delayMs = (uint32_t)20 * 100 / speedPercent;
    slowMove(channel, from, to, max((uint16_t)1, delayMs));
    return;
  }

  int dir = (to > from) ? 1 : -1;
  int current = from;
  for (uint8_t i = 0; i < 4; i++) {
    int next = to + dir * defaultBounceOffsets[i];
    next = constrain(next, 120, 560);
    uint16_t delayMs = (uint32_t)defaultBounceDelays[i] * 100 / speedPercent;
    slowMove(channel, current, next, max((uint16_t)1, delayMs));
    current = next;
  }
}

void slowMove(uint8_t channel, int from, int to, uint16_t delayMs) {
  if (from == to) {
    setServo(channel, to);
    return;
  }
  int step = (to > from) ? 1 : -1;
  for (int p = from; p != to; p += step) {
    setServo(channel, p);
    delay(delayMs);
  }
  setServo(channel, to);
}

void setServo(uint8_t channel, int pulse) {
  pwm.setPWM(channel, 0, pulse);
}

void allStop() {
  stopStressTest();
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    if (slots[i].type == SLOT_ENABLED) moveServoSlot(i, false);
  }
  Serial.println(F("All enabled servo slots moved to their thrown endpoints."));
}

// ---------------- Stress test / serial ----------------
void startStressTest() {
  if (globals.testSlot < 1 || globals.testSlot > SLOT_COUNT) return;
  if (slots[globals.testSlot - 1].type != SLOT_ENABLED) {
    Serial.println(F("Stress test not started: selected slot is disabled."));
    return;
  }
  testRunning = true;
  testStraightPhase = false;
  testNextAction = millis();
  Serial.println(F("Stress test STARTED."));
}

void stopStressTest() {
  testRunning = false;
}

void handleSlotCommand(uint8_t slotIndex, bool straight) {
  if (slotIndex >= SLOT_COUNT) return;
  stopStressTest();
  moveServoSlot(slotIndex, straight);
  Serial.print(F("OK slot "));
  Serial.print(slotIndex + 1);
  Serial.println(straight ? F(" straight") : F(" thrown"));
}

void handleSerialCommand(String cmd) {
  cmd.trim();
  if (cmd == "a0") { allStop(); return; }
  if (cmd == "t1") { startStressTest(); return; }
  if (cmd == "t0") { stopStressTest(); Serial.println(F("Stress test STOPPED.")); return; }
  if (cmd == "?") {
    Serial.println(F("Commands: 1r..16r, 1g..16g, a0, t1, t0"));
    Serial.println(F("LNCV: 0 module; 1 test command; 2 test slot; 3 proceed sec; 4 stop sec."));
    Serial.println(F("Per slot offsets: 0 enable, 1 accessory, 2 channel, 3 bounce, 4 STOP endpoint, 5 PROCEED endpoint."));
    return;
  }

  if (cmd.length() < 2) return;
  char state = cmd.charAt(cmd.length() - 1);
  if (state != 'r' && state != 'g') return;

  String numberText = cmd.substring(0, cmd.length() - 1);
  int slotNumber = numberText.toInt();
  if (slotNumber < 1 || slotNumber > SLOT_COUNT) {
    Serial.println(F("Invalid slot number."));
    return;
  }
  handleSlotCommand((uint8_t)(slotNumber - 1), state == 'g');
}
