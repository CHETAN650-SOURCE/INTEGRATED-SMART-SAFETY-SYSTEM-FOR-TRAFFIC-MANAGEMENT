/* ========================================================
   INTEGRATED SMART TRAFFIC & BLIND CURVE SYSTEM
   ========================================================
   Density-based adaptive traffic signal control + blind-curve
   collision warning, driven by an Arduino UNO and 6x cascaded
   74HC595 shift registers.
   ======================================================== */

const int dataPin = 4;
const int latchPin = 5;
const int clockPin = 6;

const int laneSensors[4] = {A0, A1, A2, A3};
const int curveSensA = 2;
const int curveSensB = 3;

const int ledA_Red = 7;
const int ledA_Grn = 8;
const int ledB_Red = 9;
const int ledB_Grn = 10;

byte digits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

int pendingVehicles[4] = {0, 0, 0, 0};
bool lastSensorState[4] = {HIGH, HIGH, HIGH, HIGH};

unsigned long curveTimer = 0;
int curveState = 0;

void updateShiftRegisters(byte d1, byte d2, byte d3, byte d4, byte ic5, byte ic6);
void checkBlindCurve();
void countVehiclesDelay(unsigned long ms);
void setCurveLights(int mode);
byte getBits(int s);
void runTrafficSequence(int current, int nextActive, int duration);

void setup() {
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);

  for (int i = 0; i < 4; i++) pinMode(laneSensors[i], INPUT);
  pinMode(curveSensA, INPUT);
  pinMode(curveSensB, INPUT);

  pinMode(ledA_Red, OUTPUT); pinMode(ledA_Grn, OUTPUT);
  pinMode(ledB_Red, OUTPUT); pinMode(ledB_Grn, OUTPUT);

  updateShiftRegisters(0, 0, 0, 0, 0x09, 0x09);
  setCurveLights(0);
  delay(1000);
}

void loop() {
  int totalPending = pendingVehicles[0] + pendingVehicles[1] +
                      pendingVehicles[2] + pendingVehicles[3];

  if (totalPending == 0) {
    updateShiftRegisters(0, 0, 0, 0, 0x09, 0x09);
    countVehiclesDelay(10000);
  }

  for (int i = 0; i < 4; i++) {
    int currentLane = i;
    int runTime = 0;

    if (pendingVehicles[currentLane] == 0) continue;
    else if (pendingVehicles[currentLane] > 9) {
      runTime = 9;
      pendingVehicles[currentLane] = pendingVehicles[currentLane] - 9;
    } else {
      runTime = pendingVehicles[currentLane];
      pendingVehicles[currentLane] = 0;
    }

    int nextActiveLane = -1;
    for (int j = 1; j < 4; j++) {
      int checkLane = (currentLane + j) % 4;
      if (pendingVehicles[checkLane] > 0) {
        nextActiveLane = checkLane;
        break;
      }
    }

    runTrafficSequence(currentLane, nextActiveLane, runTime);
  }
}

void runTrafficSequence(int current, int nextActive, int duration) {
  for (int t = duration; t >= 0; t--) {
    byte d[4] = {0, 0, 0, 0};
    d[current] = digits[t];

    int status[4] = {0, 0, 0, 0};
    if (t > 0) status[current] = 2;
    else status[current] = 1;

    if (t <= 2 && t > 0 && nextActive != -1) status[nextActive] = 1;

    byte ic5 = (getBits(status[1]) << 3) | getBits(status[0]);
    byte ic6 = (getBits(status[3]) << 3) | getBits(status[2]);

    updateShiftRegisters(d[0], d[1], d[2], d[3], ic5, ic6);
    countVehiclesDelay(1000);
  }
}

void countVehiclesDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    checkBlindCurve();

    for (int i = 0; i < 4; i++) {
      bool currentState = digitalRead(laneSensors[i]);
      if (currentState == LOW && lastSensorState[i] == HIGH) {
        pendingVehicles[i]++;
        delay(20);
      }
      lastSensorState[i] = currentState;
    }
  }
}

void checkBlindCurve() {
  if (curveState == 1 || curveState == 2) {
    if (millis() > curveTimer) {
      setCurveLights(3);
      curveTimer = millis() + 2000;
      curveState = 3;
    }
    return;
  } else if (curveState == 3) {
    if (millis() > curveTimer) {
      setCurveLights(0);
      curveState = 0;
    }
    return;
  }

  if (digitalRead(curveSensA) == LOW) {
    setCurveLights(1);
    curveTimer = millis() + 6000;
    curveState = 1;
  } else if (digitalRead(curveSensB) == LOW) {
    setCurveLights(2);
    curveTimer = millis() + 6000;
    curveState = 2;
  }
}

void setCurveLights(int mode) {
  if (mode == 0) {
    digitalWrite(ledA_Red, LOW); digitalWrite(ledA_Grn, HIGH);
    digitalWrite(ledB_Red, LOW); digitalWrite(ledB_Grn, HIGH);
  } else if (mode == 1) {
    digitalWrite(ledA_Red, LOW); digitalWrite(ledA_Grn, HIGH);
    digitalWrite(ledB_Red, HIGH); digitalWrite(ledB_Grn, LOW);
  } else if (mode == 2) {
    digitalWrite(ledA_Red, HIGH); digitalWrite(ledA_Grn, LOW);
    digitalWrite(ledB_Red, LOW); digitalWrite(ledB_Grn, HIGH);
  } else if (mode == 3) {
    digitalWrite(ledA_Red, HIGH); digitalWrite(ledA_Grn, LOW);
    digitalWrite(ledB_Red, HIGH); digitalWrite(ledB_Grn, LOW);
  }
}

byte getBits(int s) {
  if (s == 2) return 0b100;
  if (s == 1) return 0b010;
  return 0b001;
}

void updateShiftRegisters(byte d1, byte d2, byte d3, byte d4, byte ic5, byte ic6) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, ic6);
  shiftOut(dataPin, clockPin, MSBFIRST, ic5);
  shiftOut(dataPin, clockPin, MSBFIRST, d4);
  shiftOut(dataPin, clockPin, MSBFIRST, d3);
  shiftOut(dataPin, clockPin, MSBFIRST, d2);
  shiftOut(dataPin, clockPin, MSBFIRST, d1);
  digitalWrite(latchPin, HIGH);
}
