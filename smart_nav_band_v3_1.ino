/*
 * ============================================================
 *  SMART NAVIGATION BAND FOR BLIND PEOPLE
 *  Main Firmware — smart_nav_band.ino (OPTIMIZED v3.1)
 * ============================================================
 *  Hardware : Arduino Nano (ATmega328P)
 *             3x HC-SR04 Ultrasonic Sensors
 *             3x Coin Vibration Motors (via BC547 + 1N4007)
 *             18650 Li-ion + TP4056 + MT3608 (5V)
 *
 *  Author   : Nishat Pramanick
 *  Branch   : B.E. Electronics & Telecommunication — Year 2
 *  College  : Viva Institute of Technology
 *
 *  Fixes v3.1:
 *    - LUT build corrected (0–40cm = 255, not ramp)
 *    - dist[] changed to int (byte truncated values > 255)
 *    - LUT array expanded to 400, constrain() guard added
 * ============================================================
 */

#define DEBUG

// ─────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────
const byte TRIG_PINS[3]  = {8, 4, 6};
const byte ECHO_PINS[3]  = {9, 5, 7};
const byte MOTOR_PINS[3] = {10, 11, 3};

#define IDX_L 0
#define IDX_C 1
#define IDX_R 2

// ─────────────────────────────────────────────────────────────
//  THRESHOLDS
// ─────────────────────────────────────────────────────────────
#define DIST_MAX          150
#define DIST_STRONG        40
#define DIST_MIN_PWM       30
#define DIST_INVALID      350
#define DIST_CENTER_BLEND  80

// ─────────────────────────────────────────────────────────────
//  TIMING + SMOOTHING
// ─────────────────────────────────────────────────────────────
#define SCAN_INTERVAL  80
#define SMOOTH_N       4
#define SMOOTH_MASK    3

// ─────────────────────────────────────────────────────────────
//  RAMP + BLEND
// ─────────────────────────────────────────────────────────────
#define RAMP_UP    18
#define RAMP_DOWN  35
#define BLEND_PCT  70

// ─────────────────────────────────────────────────────────────
//  LUT + STATE
// ─────────────────────────────────────────────────────────────
byte pwmLUT[400];

int  buffer[3][SMOOTH_N];
byte bufIdx[3]   = {0, 0, 0};
byte target[3]   = {0, 0, 0};
byte current[3]  = {0, 0, 0};

unsigned long lastScanTime = 0;


// ═════════════════════════════════════════════════════════════
//  buildPWMLUT  (fixed v3.1)
// ═════════════════════════════════════════════════════════════
void buildPWMLUT() {
  // 0–1cm: physically impossible / noise → 0
  pwmLUT[0] = 0;
  pwmLUT[1] = 0;

  // 2–40cm: closer than DIST_STRONG → full buzz
  for (int i = 2; i <= DIST_STRONG; i++) {
    pwmLUT[i] = 255;
  }

  // 41–150cm: linear ramp down from 254 to DIST_MIN_PWM
  for (int i = DIST_STRONG + 1; i <= DIST_MAX; i++) {
    pwmLUT[i] = (byte)map(i, DIST_STRONG + 1, DIST_MAX, 254, DIST_MIN_PWM);
  }

  // 151–399: beyond detection range → 0
  for (int i = DIST_MAX + 1; i < 400; i++) {
    pwmLUT[i] = 0;
  }
}


// ═════════════════════════════════════════════════════════════
//  readOnce
// ═════════════════════════════════════════════════════════════
int readOnce(byte trigPin, byte echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return DIST_INVALID;

  int dist = (int)(duration * 0.0343 / 2.0);
  if (dist <= 1 || dist > DIST_INVALID) return DIST_INVALID;

  return dist;
}


// ═════════════════════════════════════════════════════════════
//  medianRead  (early exit on 3+ invalids)
// ═════════════════════════════════════════════════════════════
int medianRead(byte idx) {
  int r[5];
  int invalidCount = 0;

  for (int i = 0; i < 5; i++) {
    r[i] = readOnce(TRIG_PINS[idx], ECHO_PINS[idx]);
    if (r[i] == DIST_INVALID) invalidCount++;
    if (invalidCount >= 3) return DIST_INVALID;
    delayMicroseconds(600);
  }

  for (int i = 1; i < 5; i++) {
    int key = r[i], j = i - 1;
    while (j >= 0 && r[j] > key) { r[j + 1] = r[j]; j--; }
    r[j + 1] = key;
  }

  return r[2];
}


// ═════════════════════════════════════════════════════════════
//  updateMovingAvg  (returns int — fixes byte truncation)
// ═════════════════════════════════════════════════════════════
int updateMovingAvg(byte sensorIdx, int newVal) {
  if (newVal != DIST_INVALID) {
    buffer[sensorIdx][bufIdx[sensorIdx] & SMOOTH_MASK] = newVal;
    bufIdx[sensorIdx]++;
  }
  long sum = 0;
  for (int i = 0; i < SMOOTH_N; i++) sum += buffer[sensorIdx][i];
  return (int)(sum >> 2);
}


// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWrite(MOTOR_PINS[i], 0);
    for (int j = 0; j < SMOOTH_N; j++) buffer[i][j] = DIST_MAX;
  }

  buildPWMLUT();

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Smart Navigation Band v3.1");
  Serial.println("Dist_L | Dist_C | Dist_R | PWM_L | PWM_C | PWM_R");
#endif
}


// ═════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  if (now - lastScanTime >= SCAN_INTERVAL) {
    lastScanTime = now;

    int rawDist[3];
    for (int i = 0; i < 3; i++) rawDist[i] = medianRead(i);

    int dist[3];  // int — not byte (fixes truncation above 255)
    for (int i = 0; i < 3; i++) dist[i] = updateMovingAvg(i, rawDist[i]);

    // LUT lookup with bounds guard
    for (int i = 0; i < 3; i++) {
      int idx = constrain(dist[i], 0, 399);
      target[i] = pwmLUT[idx];
    }

    // Center bleed
    if (dist[IDX_C] < DIST_CENTER_BLEND && dist[IDX_C] != DIST_INVALID) {
      int lutIdx = constrain(dist[IDX_C], 0, 399);
      byte blended = (byte)((int)pwmLUT[lutIdx] * BLEND_PCT / 100);
      if (blended > target[IDX_L]) target[IDX_L] = blended;
      if (blended > target[IDX_R]) target[IDX_R] = blended;
    }

#ifdef DEBUG
    Serial.print(dist[IDX_L]);   Serial.print(" cm\t| ");
    Serial.print(dist[IDX_C]);   Serial.print(" cm\t| ");
    Serial.print(dist[IDX_R]);   Serial.print(" cm\t|| ");
    Serial.print(target[IDX_L]); Serial.print("\t| ");
    Serial.print(target[IDX_C]); Serial.print("\t| ");
    Serial.println(target[IDX_R]);
#endif
  }

  // Motor ramp — every iteration
  for (int i = 0; i < 3; i++) {
    if (current[i] < target[i]) {
      current[i] = min(current[i] + RAMP_UP, target[i]);
    } else if (current[i] > target[i]) {
      current[i] = max(current[i] - RAMP_DOWN, target[i]);
    }
    if (target[i] == 0 && current[i] < DIST_MIN_PWM) current[i] = 0;
    analogWrite(MOTOR_PINS[i], current[i]);
  }
}
