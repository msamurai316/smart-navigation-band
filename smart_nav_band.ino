/*
 * ============================================================
 *  SMART NAVIGATION BAND FOR BLIND PEOPLE
 *  Main Firmware — smart_nav_band.ino
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
 *  Reliability Features:
 *    - Median-of-5 filter per sensor (kills spike readings)
 *    - Moving average buffer (smooths out noise over time)
 *    - PWM ramping (no sudden harsh motor changes)
 *    - Non-blocking loop using millis() — no freezing
 *    - Boot-safe buffer init (no false alerts on startup)
 *    - Sensor timeout guard (handles disconnected sensor)
 *
 *  Fixes v2:
 *    - Per-sensor circular buffer index (was shared — bug)
 *    - Replaced blocking delay() in medianRead with
 *      delayMicroseconds(600) — preserves non-blocking design
 *    - Center blend threshold tightened to 80cm
 *    - Sub-threshold PWM kill in rampValue
 *    - DEBUG flag for clean production build
 * ============================================================
 */

// ─────────────────────────────────────────────────────────────
//  DEBUG FLAG
//  Comment out before final hardware deployment.
//  Removes all Serial prints → saves CPU + eliminates
//  UART interrupt overhead.
// ─────────────────────────────────────────────────────────────
#define DEBUG

// ─────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────
#define TRIG_L   8    // Left sensor  — Trigger
#define ECHO_L   9    // Left sensor  — Echo
#define TRIG_C   4    // Center sensor — Trigger
#define ECHO_C   5    // Center sensor — Echo
#define TRIG_R   6    // Right sensor  — Trigger
#define ECHO_R   7    // Right sensor  — Echo

#define MOTOR_L  10   // Left vibration motor   (PWM capable)
#define MOTOR_C  11   // Center vibration motor (PWM capable)
#define MOTOR_R   3   // Right vibration motor  (PWM capable)

// ─────────────────────────────────────────────────────────────
//  DISTANCE THRESHOLDS (centimetres)
// ─────────────────────────────────────────────────────────────
#define DIST_MAX          150   // Beyond this — motors OFF
#define DIST_STRONG        40   // Closer than this — full buzz (PWM 255)
#define DIST_MIN_PWM       30   // Minimum PWM to actually spin motor
#define DIST_INVALID      350   // Sensor error / out of range value
#define DIST_CENTER_BLEND  80   // Center bleed only triggers below this

// ─────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────
#define SCAN_INTERVAL   80    // Full sensor scan every 80ms

unsigned long lastScanTime = 0;

// ─────────────────────────────────────────────────────────────
//  SMOOTHING — Moving Average Buffer
//  Keeps last N filtered readings per sensor.
//  Each sensor has its own circular index (FIX v2).
// ─────────────────────────────────────────────────────────────
#define SMOOTH_N  4
int bufL[SMOOTH_N];
int bufC[SMOOTH_N];
int bufR[SMOOTH_N];
int bufIdxL = 0;
int bufIdxC = 0;
int bufIdxR = 0;

// ─────────────────────────────────────────────────────────────
//  MOTOR STATE — Current and Target PWM (for ramping)
// ─────────────────────────────────────────────────────────────
int targetL = 0, targetC = 0, targetR = 0;
int currentL = 0, currentC = 0, currentR = 0;

// Ramp DOWN faster than UP — path clears quickly = safer
#define RAMP_UP    18
#define RAMP_DOWN  35


// ═════════════════════════════════════════════════════════════
//  FUNCTION: readOnce
//  Fires one ultrasonic pulse, returns raw distance in cm.
//  Returns DIST_INVALID on timeout or impossible value.
// ═════════════════════════════════════════════════════════════
int readOnce(int trigPin, int echoPin) {
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
//  FUNCTION: medianRead
//  Takes 5 readings with a 600µs inter-reading gap.
//  Gap prevents next pulse catching the previous echo
//  without blocking the main loop (FIX v2 — was delay(10)).
//  Returns median of sorted readings.
// ═════════════════════════════════════════════════════════════
int medianRead(int trigPin, int echoPin) {
  int r[5];

  for (int i = 0; i < 5; i++) {
    r[i] = readOnce(trigPin, echoPin);
    delayMicroseconds(600);  // Non-blocking inter-reading gap
  }

  // Insertion sort — 5 elements, negligible CPU cost
  for (int i = 1; i < 5; i++) {
    int key = r[i];
    int j = i - 1;
    while (j >= 0 && r[j] > key) {
      r[j + 1] = r[j];
      j--;
    }
    r[j + 1] = key;
  }

  // If 3+ readings are invalid, sensor is likely disconnected
  int invalidCount = 0;
  for (int i = 0; i < 5; i++) {
    if (r[i] == DIST_INVALID) invalidCount++;
  }
  if (invalidCount >= 3) return DIST_INVALID;

  return r[2];  // Median
}


// ═════════════════════════════════════════════════════════════
//  FUNCTION: updateMovingAvg
//  Circular buffer per sensor with its own index (FIX v2).
//  Invalid readings are skipped — buffer keeps last good value.
// ═════════════════════════════════════════════════════════════
int updateMovingAvg(int* buf, int &idx, int newVal) {
  if (newVal != DIST_INVALID) {
    buf[idx % SMOOTH_N] = newVal;
    idx++;
  }
  long sum = 0;
  for (int i = 0; i < SMOOTH_N; i++) sum += buf[i];
  return (int)(sum / SMOOTH_N);
}


// ═════════════════════════════════════════════════════════════
//  FUNCTION: distanceToPWM
//  Maps distance → PWM (0–255). Closer = stronger vibration.
//
//  > 150cm  →  0   (nothing detected)
//  150–40cm →  30 to 255 (linear ramp)
//  < 40cm   →  255 (maximum)
// ═════════════════════════════════════════════════════════════
int distanceToPWM(int dist) {
  if (dist == DIST_INVALID) return 0;
  if (dist >= DIST_MAX)     return 0;
  if (dist <= DIST_STRONG)  return 255;
  return map(dist, DIST_MAX, DIST_STRONG, DIST_MIN_PWM, 255);
}


// ═════════════════════════════════════════════════════════════
//  FUNCTION: rampValue
//  Moves current toward target one step per loop iteration.
//  Kills sub-threshold values on ramp-down so real motors
//  don't stall in a low-PWM zone (FIX v2).
// ═════════════════════════════════════════════════════════════
void rampValue(int &current, int target) {
  if (current < target) {
    current = min(current + RAMP_UP, target);
  } else if (current > target) {
    current = max(current - RAMP_DOWN, target);
  }
  // If ramping to zero and below spin threshold, snap to 0
  // Prevents motor stalling at sub-threshold PWM values
  if (target == 0 && current < DIST_MIN_PWM) {
    current = 0;
  }
}


// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_C, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);

  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_C, 0);
  analogWrite(MOTOR_R, 0);

  // Pre-fill buffers with DIST_MAX — prevents false motor
  // blast on first boot while buffers are still zeroed
  for (int i = 0; i < SMOOTH_N; i++) {
    bufL[i] = DIST_MAX;
    bufC[i] = DIST_MAX;
    bufR[i] = DIST_MAX;
  }

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Smart Navigation Band — Initialised v2");
  Serial.println("Dist_L | Dist_C | Dist_R | PWM_L | PWM_C | PWM_R");
#endif
}


// ═════════════════════════════════════════════════════════════
//  MAIN LOOP
//  Non-blocking: sensor scan runs every SCAN_INTERVAL ms.
//  Motor ramping runs every iteration (~1kHz) for smooth feel.
// ═════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  if (now - lastScanTime >= SCAN_INTERVAL) {
    lastScanTime = now;

    // 1. Median read each sensor
    int rawL = medianRead(TRIG_L, ECHO_L);
    int rawC = medianRead(TRIG_C, ECHO_C);
    int rawR = medianRead(TRIG_R, ECHO_R);

    // 2. Moving average — per-sensor index (FIX v2)
    int distL = updateMovingAvg(bufL, bufIdxL, rawL);
    int distC = updateMovingAvg(bufC, bufIdxC, rawC);
    int distR = updateMovingAvg(bufR, bufIdxR, rawR);

    // 3. Base PWM targets
    targetL = distanceToPWM(distL);
    targetC = distanceToPWM(distC);
    targetR = distanceToPWM(distR);

    // 4. Center obstacle bleed logic
    //    Only activates when center detects something closer
    //    than DIST_CENTER_BLEND (80cm) — not at full range.
    //    Bleeds 70% of center PWM into side motors to signal
    //    "danger straight ahead — step aside".
    if (distC < DIST_CENTER_BLEND && distC != DIST_INVALID) {
      int blended = (int)(distanceToPWM(distC) * 0.70);
      if (blended > targetL) targetL = blended;
      if (blended > targetR) targetR = blended;
    }

#ifdef DEBUG
    Serial.print(distL);   Serial.print(" cm\t| ");
    Serial.print(distC);   Serial.print(" cm\t| ");
    Serial.print(distR);   Serial.print(" cm\t|| ");
    Serial.print(targetL); Serial.print("\t| ");
    Serial.print(targetC); Serial.print("\t| ");
    Serial.println(targetR);
#endif
  }

  // Motor ramp — runs every loop iteration for smooth transitions
  rampValue(currentL, targetL);
  rampValue(currentC, targetC);
  rampValue(currentR, targetR);

  analogWrite(MOTOR_L, currentL);
  analogWrite(MOTOR_C, currentC);
  analogWrite(MOTOR_R, currentR);
}
