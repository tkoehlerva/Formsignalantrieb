#include <LocoNet.h>

#include <LocoNet.h>

#include <LocoNet.h>

const uint8_t LOCONET_TX_PIN = 9;
const uint16_t ADDR_HS1 = 200;
const uint16_t ADDR_TEST = 300;

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define HS1    0
#define HS2    1
#define HS3    2
#define HS4    3
#define HS5    4
#define SPERR  5
#define VS2    6
#define VS3_A  7
#define VS3_B  8

int posHp0 = 255;
int posHp1 = 225;

int posSh0 = 270;
int posSh1 = 390;

int posVr0  = 250;
int posVr1  = 405;
int posVr2a = 330;
int posVr2b = 405;

int currentPos[16];


bool testRunning = false;
byte testPhase = 0;
unsigned long testNextAction = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);

  LocoNet.init(LOCONET_TX_PIN);

  initPositions();

  Serial.println("Servo-Signaltester bereit.");
  Serial.println("Befehle: 1r, 1g ... 8r, 8g, 8y, a0");
}

/*
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }
}
*/


void loop() {
  lnMsg* LnPacket;

  while ((LnPacket = LocoNet.receive()) != nullptr) {
    LocoNet.processSwitchSensorMessage(LnPacket);
  }
  
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  if (testRunning &&
      (long)(millis() - testNextAction) >= 0) {

    if (testPhase == 0) {
      // Move signal 1 to green
      moveSignal(HS1, posHp1);
      Serial.println("Stress test: 1g");

      testPhase = 1;
      testNextAction = millis() + 5000UL;

    } else {
      // Move signal 1 to red
      moveSignal(HS1, posHp0);
      Serial.println("Stress test: 1r");

      testPhase = 0;
      testNextAction = millis() + 10000UL;
    }
  }
}

void notifySwitchRequest(uint16_t Address,
                         uint8_t Output,
                         uint8_t Direction) {
  // Ignore switch-off packets
  if (Output == 0) return;

  // Address 200: normal control of HS1
  if (Address == ADDR_HS1) {
    testRunning = false;

    // IB1 button behavior was reversed in your test
    if (Direction == 0) {
      handleCommand("1r");
    } else {
      handleCommand("1g");
    }
    return;
  }

  // Address 300: stress-test control
  if (Address == ADDR_TEST) {
    if (Direction == 0) {
      // Red: stop the test
      testRunning = false;
      Serial.println("Stress test STOPPED by address 300.");
    } else {
      // Green: start the test
      testRunning = true;
      testPhase = 0;
      testNextAction = millis();
      Serial.println("Stress test STARTED by address 300.");
    }
  }
}


void initPositions() {
  for (int i = 0; i < 16; i++) currentPos[i] = 300;

  setAndRemember(HS1, posHp0);
  setAndRemember(HS2, posHp0);
  setAndRemember(HS3, posHp0);
  setAndRemember(HS4, posHp0);
  setAndRemember(HS5, posHp0);
  setAndRemember(SPERR, posSh0);
  setAndRemember(VS2, posVr0);
  setAndRemember(VS3_A, posVr0);
  setAndRemember(VS3_B, posVr0);
}

void handleCommand(String cmd) {
  if (cmd.length() < 2) return;

  char id = cmd.charAt(0);
  char state = cmd.charAt(1);

  if (id == 'a' && state == '0') {
    allStop();
    return;
  }

  
  if (cmd == "t1") {
    testRunning = true;
    testPhase = 0;
    testNextAction = millis();

    Serial.println("Stress test STARTED.");
    return;
  }

  if (cmd == "t0") {
    testRunning = false;

    Serial.println("Stress test STOPPED.");
    return;
  }


  switch (id) {
    case '1': moveSignal(HS1, state == 'g' ? posHp1 : posHp0); break;
    case '2': moveSignal(HS2, state == 'g' ? posHp1 : posHp0); break;
    case '3': moveSignal(HS3, state == 'g' ? posHp1 : posHp0); break;
    case '4': moveSignal(HS4, state == 'g' ? posHp1 : posHp0); break;
    case '5': moveSignal(HS5, state == 'g' ? posHp1 : posHp0); break;

    case '6': moveSignal(SPERR, state == 'g' ? posSh1 : posSh0); break;

    case '7': moveSignal(VS2, state == 'g' ? posVr1 : posVr0); break;

    case '8':
      if (state == 'r') {
        moveSignal(VS3_A, posVr0);
        moveSignal(VS3_B, posVr0);
      } else if (state == 'g') {
        moveSignal(VS3_A, posVr1);
        moveSignal(VS3_B, posVr0);
      } else if (state == 'y') {
        moveSignal(VS3_A, posVr2a);
        moveSignal(VS3_B, posVr2b);
      }
      break;
  }

  Serial.print("OK: ");
  Serial.println(cmd);
}

void allStop() {
  moveSignal(HS1, posHp0);
  moveSignal(HS2, posHp0);
  moveSignal(HS3, posHp0);
  moveSignal(HS4, posHp0);
  moveSignal(HS5, posHp0);
  moveSignal(SPERR, posSh0);
  moveSignal(VS2, posVr0);
  moveSignal(VS3_A, posVr0);
  moveSignal(VS3_B, posVr0);

  Serial.println("Alle Signale auf Halt/Vr0.");
}

void moveSignal(uint8_t channel, int target) {
  int from = currentPos[channel];
  if (from == target) return;

  moveWithBounce(channel, from, target);
  currentPos[channel] = target;
}


void moveWithBounce(uint8_t channel, int from, int to) {
  int bounce[] = {5, -3, 2, 0};
  int delays[] = {20, 30, 40, 55};

  int dir = (to > from) ? 1 : -1;
  int current = from;

  for (int i = 0; i < 4; i++) {
    int next = to + dir * bounce[i];
    next = constrain(next, 120, 560);
    slowMove(channel, current, next, delays[i]);
    current = next;
  }
}






void moveWithBounceX(uint8_t channel, int from, int to) {
  int dir = (to > from) ? 1 : -1;

  if (to < from) {
    // Upper movement: overshoot, then drop back
    slowMove(channel, from, to + dir * 5, 10);
    delay(100);

    slowMove(channel, to + dir * 5, to, 10);
    delay(50);

  } else {
    // Lower movement: fall past target, rebound, settle
    slowMove(channel, from, to - dir * 5, 10);
    delay(80);

    slowMove(channel, to - dir * 5, to + dir * 3, 10);
    delay(70);

    slowMove(channel, to + dir * 3, to - dir * 1, 10);
    delay(50);

    slowMove(channel, to - dir * 1, to, 10);
  }
}





void slowMove(uint8_t channel, int from, int to, int delayMs) {
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

void setAndRemember(uint8_t channel, int pulse) {
  setServo(channel, pulse);
  currentPos[channel] = pulse;
}
