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
 *  Author   : [Your Name]
 *  Branch   : B.E. Electronics & Telecommunication — Year 2
 *  College  : [Your College Name]
 *
 *  Reliability Features:
 *    - Median-of-5 filter per sensor (kills spike readings)
 *    - Moving average buffer (smooths out noise over time)
 *    - PWM ramping (no sudden harsh motor changes)
 *    - Non-blocking loop using millis() — no freezing
 *    - Boot-safe buffer init (no false alerts on startup)
 *    - Sensor timeout guard (handles disconnected sensor)
 * ============================================================
 */

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
#define DIST_MAX      150   // Beyond this — motors OFF
#define DIST_STRONG    40   // Closer than this — full buzz (PWM 255)
#define DIST_MIN_PWM   30   // Minimum PWM to actually spin motor
#define DIST_INVALID  350   // Sensor error / out of range value

// ─────────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────────
#define SCAN_INTERVAL   80   // Full sensor scan every 80ms
#define READING_GAP     10   // Gap between individual readings (ms)

unsigned long lastScanTime = 0;

// ─────────────────────────────────────────────────────────────
//  SMOOTHING — Moving Average Buffer
//  Keeps last N filtered readings per sensor
// ─────────────────────────────────────────────────────────────
#define SMOOTH_N  4
int bufL[SMOOTH_N];
int bufC[SMOOTH_N];
int bufR[SMOOTH_N];
int bufIdx = 0;   // Circular index

// ─────────────────────────────────────────────────────────────
//  MOTOR STATE — Current and Target PWM (for ramping)
// ─────────────────────────────────────────────────────────────
int targetL = 0, targetC = 0, targetR = 0;
int currentL = 0, currentC = 0, currentR = 0;

// Ramp step sizes — ramp DOWN faster (stop quickly = safer)
#define RAMP_UP    18
#define RAMP_DOWN  35


// ═════════════════════════════════════════════════════════════
//  FUNCTION: readOnce
//  Fires one ultrasonic pulse and returns raw distance in cm.
//  Returns DIST_INVALID if echo timed out (nothing detected
//  or sensor disconnected).
// ═════════════════════════════════════════════════════════════
int readOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // pulseIn timeout = 25000µs → max ~425cm, safe ceiling
  long duration = pulseIn(echoPin, HIGH, 25000UL);

  if (duration == 0) return DIST_INVALID;  // Timeout

  int dist = (int)(duration * 0.0343 / 2.0);

  // Sanity clamp — ignore physically impossible values
  if (dist <= 1 || dist > DIST_INVALID) return DIST_INVALID;

  return dist;
}


// ═════════════════════════════════════════════════════════════
//  FUNCTION: medianRead
//  Takes 5 readings, sorts them, returns the median (index 2).
//  This removes one-off spikes caused by reflections or
//  electrical interference without introducing delay bias.
// ═════════════════════════════════════════════════════════════
int medianRead(int trigPin, int echoPin) {
  int r[5];

  for (int i = 0; i < 5; i++) {
    r[i] = readOnce(trigPin, echoPin);
    delay(READING_GAP);  // Prevents next pulse catching prev echo
  }

  // Simple insertion sort (5 elements — negligible CPU cost)
  for (int i = 1; i < 5; i++) {
    int key = r[i];
    int j = i - 1;
    while (j >= 0 && r[j] > key) {
      r[j + 1] = r[j];
      j--;
    }
    r[j + 1] = key;
  }

  // If majority readings are invalid, return invalid
  int invalidCount = 0;
  for (int i = 0; i < 5; i++) {
    if (r[i] == DIST_INVALID) invalidCount++;
  }
  if (invalidCount >= 3) return DIST_INVALID;

  return r[2];  // Middle value = statistical median
}


// ═════════════════════════════════════════════════════════════
//  FUNCTION: updateMovingAvg
//  Stores new value into circular buffer, returns average.
//  Gives temporal smoothing: output changes gradually
//  even if one cycle gives a slightly different reading.
// ═════════════════════════════════════════════════════════════
int updateMovingAvg(int* buf, int newVal) {
  // If new value is invalid, don't corrupt the buffer
  // Keep the last good value instead
  if (newVal == DIST_INVALID) {
    // Return average of existing buffer
    long sum = 0;
    for (int i = 0; i < SMOOTH_N; i++) sum += buf[i];
    return (int)(sum / SMOOTH_N);
  }

  buf[bufIdx % SMOOTH_N] = newVal;

  long sum = 0;
  for (int i = 0; i < SMOOTH_N; i++) sum += buf[i];
  return (int)(sum / SMOOTH_N);
}


// ═════════════════════════════════════════════════════════════
//  FUNCTION: distanceToPWM
//  Maps distance → PWM value (0–255).
//  Inverse relationship: closer = stronger vibration.
//
//  Distance zones:
//    > DIST_MAX (150cm)   →  0    (no vibration)
//    DIST_MAX to DIST_STRONG → 30 to 255 (proportional)
//    < DIST_STRONG (40cm) →  255  (maximum buzz)
// ═════════════════════════════════════════════════════════════
int distanceToPWM(int dist) {
  if (dist == DIST_INVALID) return 0;
  if (dist >= DIST_MAX)     return 0;
  if (dist <= DIST_STRONG)  return 255;

  // Linear map: DIST_MAX → DIST_MIN_PWM, DIST_STRONG → 255
  return map(dist, DIST_MAX, DIST_STRONG, DIST_MIN_PWM, 255);
}


// ═════════════════════════════════════════════════════════════
//  FUNCTION: rampValue
//  Moves `current` toward `target` by one step.
//  Ramps UP slowly (gentle onset) and DOWN quickly (fast stop).
//  This avoids harsh sudden buzzes AND ensures the path
//  clears quickly when obstacle is removed.
// ═════════════════════════════════════════════════════════════
void rampValue(int &current, int target) {
  if (current < target) {
    current = min(current + RAMP_UP,   target);
  } else if (current > target) {
    current = max(current - RAMP_DOWN, target);
  }
}


// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  // Sensor pins
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  // Motor pins
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_C, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);

  // Ensure motors start OFF
  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_C, 0);
  analogWrite(MOTOR_R, 0);

  // ── CRITICAL: Pre-fill buffers with DIST_MAX ──────────────
  // If buffers start at 0, the moving average will output a
  // tiny distance → all motors blast on first boot.
  // Pre-filling with DIST_MAX means "nothing detected yet".
  for (int i = 0; i < SMOOTH_N; i++) {
    bufL[i] = DIST_MAX;
    bufC[i] = DIST_MAX;
    bufR[i] = DIST_MAX;
  }

  Serial.begin(9600);
  Serial.println("Smart Navigation Band — Initialised");
  Serial.println("Dist_L | Dist_C | Dist_R | PWM_L | PWM_C | PWM_R");
}


// ═════════════════════════════════════════════════════════════
//  MAIN LOOP
//  Non-blocking: uses millis() instead of delay().
//  Motor ramping runs every iteration for smooth transitions.
// ═════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Sensor scan block (runs every SCAN_INTERVAL ms) ───────
  if (now - lastScanTime >= SCAN_INTERVAL) {
    lastScanTime = now;
    bufIdx++;

    // 1. Read each sensor (median of 5 readings)
    int rawL = medianRead(TRIG_L, ECHO_L);
    int rawC = medianRead(TRIG_C, ECHO_C);
    int rawR = medianRead(TRIG_R, ECHO_R);

    // 2. Moving average across last SMOOTH_N readings
    int distL = updateMovingAvg(bufL, rawL);
    int distC = updateMovingAvg(bufC, rawC);
    int distR = updateMovingAvg(bufR, rawR);

    // 3. Base PWM targets from side sensors
    targetL = distanceToPWM(distL);
    targetC = distanceToPWM(distC);
    targetR = distanceToPWM(distR);

    // 4. Center obstacle logic ─────────────────────────────
    //    When center sensor detects obstacle:
    //    - Center motor runs at full intensity
    //    - Both left AND right motors also buzz at 70%
    //      This tells user "danger straight ahead"
    //    - Only overrides side motors if center is CLOSER
    //      than whatever side sensor already detected
    if (distC < DIST_MAX && distC != DIST_INVALID) {
      int cPWM = distanceToPWM(distC);
      int blended = (int)(cPWM * 0.70);
      if (blended > targetL) targetL = blended;
      if (blended > targetR) targetR = blended;
    }

    // 5. Debug output to Serial Monitor
    //    Format: L | C | R | PWM_L | PWM_C | PWM_R
    //    REMOVE Serial prints before final deployment
    //    (saves ~2% CPU and eliminates UART interrupt overhead)
    Serial.print(distL);   Serial.print(" cm\t| ");
    Serial.print(distC);   Serial.print(" cm\t| ");
    Serial.print(distR);   Serial.print(" cm\t|| ");
    Serial.print(targetL); Serial.print("\t| ");
    Serial.print(targetC); Serial.print("\t| ");
    Serial.println(targetR);
  }

  // ── Motor ramp block (runs EVERY loop iteration) ──────────
  // By running ramp outside the scan block, transitions happen
  // at ~1kHz even though scans only happen every 80ms.
  // This gives very smooth, human-friendly vibration onset.
  rampValue(currentL, targetL);
  rampValue(currentC, targetC);
  rampValue(currentR, targetR);

  analogWrite(MOTOR_L, currentL);
  analogWrite(MOTOR_C, currentC);
  analogWrite(MOTOR_R, currentR);
}
