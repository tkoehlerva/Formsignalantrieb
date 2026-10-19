
#include <EEPROM.h>
#include <LocoNet.h>

// Temporary test identity.
// Enter article number 9999 and module address 1 in the IB1.
const uint16_t ARTICLE_NUMBER = 634;
#define ARTNR 634

// EEPROM layout
const int EEPROM_MAGIC_ADDR  = 0;
const int EEPROM_MODULE_ADDR = 2;
const int EEPROM_TEST_ADDR   = 4;
const uint16_t EEPROM_MAGIC  = 0x4C4E;

uint16_t moduleAddress = 1;
uint16_t testValue = 1234;
bool programmingActive = false;

// LNCV protocol handler supplied by MRRwA LocoNet 1.1.13
LocoNetCVClass LocoNetCV;

void loadConfig() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC) {
    // First run: create defaults.
    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.put(EEPROM_MODULE_ADDR, moduleAddress);
    EEPROM.put(EEPROM_TEST_ADDR, testValue);
  } else {
    EEPROM.get(EEPROM_MODULE_ADDR, moduleAddress);
    EEPROM.get(EEPROM_TEST_ADDR, testValue);

    // 65535 is reserved as a broadcast value.
    if (moduleAddress == 65535) {
      moduleAddress = 1;
      EEPROM.put(EEPROM_MODULE_ADDR, moduleAddress);
    }
  }
}

void setup() {
  Serial.begin(115200);

  loadConfig();

  // Initialize the MRRwA LocoNet interface.
  LocoNet.init(9);

  Serial.println(F("LNCV test ready"));
  Serial.print(F("Article number: "));
  Serial.println(ARTICLE_NUMBER);
  Serial.print(F("Module address: "));
  Serial.println(moduleAddress);
  Serial.print(F("Test LNCV 1: "));
  Serial.println(testValue);
}

void loop() {
  lnMsg* packet;

  // Process all packets currently waiting in the receive buffer.
  while ((packet = LocoNet.receive()) != NULL) {
    LocoNetCV.processLNCVMessage(packet);
  }
}

// Called when the IB1 starts an LNCV programming session.
int8_t notifyLNCVprogrammingStart(
    uint16_t& artNr, uint16_t& requestedModuleAddress) {

  Serial.println(F("LNCV programming start received"));
  Serial.print(F("Article number requested: "));
  Serial.println(artNr);
  Serial.print(F("Module address requested: "));
  Serial.println(requestedModuleAddress);

  if (artNr != ARTICLE_NUMBER && artNr != 65535) {
    return -1;  // Not this module.
  }
Serial.print(F("Flags are handled by library; module field = "));
Serial.println(requestedModuleAddress);
  
if (requestedModuleAddress != moduleAddress &&
    requestedModuleAddress != 65535) {
  Serial.println(F("Rejected: unexpected module address"));
  return -1;
}

  // Resolve broadcast values to this module's actual identity.
  artNr = ARTICLE_NUMBER;
  requestedModuleAddress = moduleAddress;
  programmingActive = true;

Serial.println(F("Returning LNCV_LACK_OK"));
return LNCV_LACK_OK;
}

// LNCV 0 = module address; LNCV 1 = test value.
int8_t notifyLNCVread(
    uint16_t artNr, uint16_t lncvAddress, uint16_t& value) {

  if (!programmingActive || artNr != ARTICLE_NUMBER) {
    return -1;
  }

  if (lncvAddress == 0) {
    value = moduleAddress;
  } else if (lncvAddress == 1) {
    value = testValue;
  } else {
    return LNCV_LACK_ERROR_UNSUPPORTED;
  }

  return LNCV_LACK_OK;
}

// Write LNCV 0 or 1.
int8_t notifyLNCVwrite(
    uint16_t artNr, uint16_t lncvAddress, uint16_t value) {

  if (!programmingActive || artNr != ARTICLE_NUMBER) {
    return -1;
  }

  if (lncvAddress == 0) {
    if (value == 65535) {
      return LNCV_LACK_ERROR_OUTOFRANGE;
    }

    moduleAddress = value;

    // Update immediately, as required for LNCV 0.
    EEPROM.put(EEPROM_MODULE_ADDR, moduleAddress);

  } else if (lncvAddress == 1) {
    testValue = value;
    EEPROM.put(EEPROM_TEST_ADDR, testValue);

  } else {
    return LNCV_LACK_ERROR_UNSUPPORTED;
  }

  return LNCV_LACK_OK;
}

void notifyLNCVprogrammingStop(
    uint16_t artNr, uint16_t requestedModuleAddress) {

  if (artNr == ARTICLE_NUMBER || artNr == 65535) {
    programmingActive = false;
  }
}
