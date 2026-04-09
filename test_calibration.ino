/*
 * ============================================================
 *  TEST 3 — Calibration & Threshold Tuning Helper
 *  test_calibration.ino
 * ============================================================
 *  PURPOSE:
 *    Helps you fine-tune DIST_MAX and DIST_STRONG thresholds
 *    for your specific environment and use case.
 *    Also shows real-time PWM output so you can verify the
 *    distance-to-intensity mapping makes sense.
 *
 *  HOW TO USE:
 *    1. Upload this sketch
 *    2. Open Serial Monitor at 9600 baud
 *    3. Walk toward a wall slowly from 200cm away
 *    4. Note the distance at which you WANT vibration to START
 *       → Set this as DIST_MAX in main firmware
 *    5. Note the distance at which you WANT maximum buzz
 *       → Set this as DIST_STRONG in main firmware
 *    6. Typical values:
 *       Indoor (tight spaces) : DIST_MAX=120, DIST_STRONG=30
 *       Outdoor (walking path): DIST_MAX=200, DIST_STRONG=50
 *       Default (general use) : DIST_MAX=150, DIST_STRONG=40
 *
 *  ALSO USEFUL FOR:
 *    - Checking if sensors interfere with each other
 *    - Measuring sensor accuracy vs a measuring tape
 *    - Verifying median filter effect (compare raw vs filtered)
 * ============================================================
 */

// ── ADJUST THESE and copy final values to smart_nav_band.ino
#define CAL_DIST_MAX    150   // Try changing this
#define CAL_DIST_STRONG  40   // Try changing this

// Pin definitions
#define TRIG_L   8
#define ECHO_L   9
#define TRIG_C   4
#define ECHO_C   5
#define TRIG_R   6
#define ECHO_R   7

#define MOTOR_L  10
#define MOTOR_C  11
#define MOTOR_R   3

// ── Single raw read
int readRaw(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 25000UL);
  if (dur == 0) return -1;
  return (int)(dur * 0.0343 / 2.0);
}

// ── Median of 5 (same as main firmware)
int readMedian(int trigPin, int echoPin) {
  int r[5];
  for (int i = 0; i < 5; i++) { r[i] = readRaw(trigPin, echoPin); delay(10); }
  for (int i = 1; i < 5; i++) {
    int k = r[i]; int j = i - 1;
    while (j >= 0 && r[j] > k) { r[j+1] = r[j]; j--; }
    r[j+1] = k;
  }
  return r[2];
}

// ── Map distance to PWM using calibration thresholds
int calDistToPWM(int dist) {
  if (dist < 0 || dist >= CAL_DIST_MAX) return 0;
  if (dist <= CAL_DIST_STRONG)          return 255;
  return map(dist, CAL_DIST_MAX, CAL_DIST_STRONG, 30, 255);
}

// ── Visual PWM bar for serial monitor
void printPWMBar(int pwm) {
  int bars = pwm / 10;  // 0–25 bars
  Serial.print("[");
  for (int i = 0; i < 25; i++) Serial.print(i < bars ? "█" : "·");
  Serial.print("] ");
  if (pwm < 100) Serial.print(" ");
  if (pwm < 10)  Serial.print(" ");
  Serial.print(pwm);
}

void setup() {
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_C, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);

  Serial.begin(9600);
  while (!Serial);

  Serial.println("=============================================");
  Serial.println("  CALIBRATION TOOL — Smart Navigation Band");
  Serial.println("=============================================");
  Serial.print("DIST_MAX (vibration starts) : ");
  Serial.print(CAL_DIST_MAX); Serial.println(" cm");
  Serial.print("DIST_STRONG (max buzz at)   : ");
  Serial.print(CAL_DIST_STRONG); Serial.println(" cm");
  Serial.println("---------------------------------------------");
  Serial.println("Dist(raw) | Dist(med) | Sensor | PWM | Bar");
  Serial.println("---------------------------------------------");
  delay(1500);
}

void loop() {
  // ── Read raw and filtered for CENTER sensor (most important)
  int raw_C  = readRaw(TRIG_C, ECHO_C);
  int med_C  = readMedian(TRIG_C, ECHO_C);
  int med_L  = readMedian(TRIG_L, ECHO_L);
  int med_R  = readMedian(TRIG_R, ECHO_R);

  int pwmL = calDistToPWM(med_L);
  int pwmC = calDistToPWM(med_C);
  int pwmR = calDistToPWM(med_R);

  // Drive motors so you can feel the calibration live
  analogWrite(MOTOR_L, pwmL);
  analogWrite(MOTOR_C, pwmC);
  analogWrite(MOTOR_R, pwmR);

  // ── Print calibration table
  Serial.print(raw_C < 0 ? -1 : raw_C);
  Serial.print(" cm\t  | ");
  Serial.print(med_C < 0 ? -1 : med_C);
  Serial.print(" cm\t  | ");

  // Print all three readings inline
  Serial.print("L:");
  Serial.print(med_L < 0 ? 0 : med_L);
  Serial.print(" C:");
  Serial.print(med_C < 0 ? 0 : med_C);
  Serial.print(" R:");
  Serial.print(med_R < 0 ? 0 : med_R);
  Serial.print(" | C-PWM: ");

  printPWMBar(pwmC);
  Serial.println();

  delay(300);
}
