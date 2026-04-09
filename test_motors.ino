/*
 * ============================================================
 *  TEST 2 — Motor Driver Test
 *  test_motors.ino
 * ============================================================
 *  PURPOSE:
 *    Tests all three vibration motors independently.
 *    Verifies BC547 transistor circuit is correctly wired.
 *    Tests PWM intensity variation (soft → strong buzz).
 *
 *  HOW TO USE:
 *    1. Upload this sketch
 *    2. Open Serial Monitor at 9600 baud
 *    3. Watch which motor should buzz and feel it
 *    4. Test runs automatically in a sequence
 *
 *  PASS CRITERIA:
 *    - Each motor buzzes when Serial Monitor says it should
 *    - Intensity increases from soft to strong during sweep
 *    - No motor stays on when it shouldn't
 *
 *  FAIL DIAGNOSIS:
 *    - Motor doesn't buzz at all:
 *        → Check transistor orientation (BC547: flat face front,
 *          pins left-to-right = Emitter, Base, Collector)
 *        → Check 1kΩ resistor on base
 *        → Measure collector-emitter with multimeter (should
 *          be near 0V when base is HIGH)
 *    - Motor buzzes weakly:
 *        → Check 5V supply is actually 5V (not 3.3V)
 *        → Motor may be low quality — try at PWM 255 directly
 *    - All motors buzz simultaneously:
 *        → Check motor GND wiring — common GND with Arduino?
 * ============================================================
 */

#define MOTOR_L  10
#define MOTOR_C  11
#define MOTOR_R   3

void setup() {
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_C, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);

  // Start with all motors off
  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_C, 0);
  analogWrite(MOTOR_R, 0);

  Serial.begin(9600);
  while (!Serial);

  Serial.println("=========================================");
  Serial.println("  MOTOR DRIVER TEST — Navigation Band");
  Serial.println("=========================================");
  delay(2000);
}

// ── Helper: Buzz one motor at given intensity, print status
void buzzMotor(int pin, const char* name, int pwm) {
  Serial.print(">>> Buzzing ");
  Serial.print(name);
  Serial.print(" motor — PWM: ");
  Serial.println(pwm);

  analogWrite(pin, pwm);
  delay(800);
  analogWrite(pin, 0);
  delay(400);
}

// ── Helper: Sweep motor from 0→255→0 (intensity test)
void sweepMotor(int pin, const char* name) {
  Serial.print(">>> Sweeping ");
  Serial.print(name);
  Serial.println(" motor (soft → strong → off)");

  // Ramp up
  for (int i = 30; i <= 255; i += 5) {
    analogWrite(pin, i);
    delay(20);
  }
  delay(300);
  // Ramp down
  for (int i = 255; i >= 0; i -= 5) {
    analogWrite(pin, i);
    delay(15);
  }
  delay(400);
}

void loop() {
  // ── PHASE 1: Individual motor test at full power ──────────
  Serial.println("\n--- PHASE 1: Individual motor test ---");

  buzzMotor(MOTOR_L, "LEFT",   255);
  delay(300);
  buzzMotor(MOTOR_C, "CENTER", 255);
  delay(300);
  buzzMotor(MOTOR_R, "RIGHT",  255);
  delay(600);

  // ── PHASE 2: PWM intensity levels ─────────────────────────
  Serial.println("\n--- PHASE 2: Intensity levels (LEFT motor) ---");
  Serial.println("(Feeling softest → strongest buzz)");

  buzzMotor(MOTOR_L, "LEFT (30% intensity)",  76);
  buzzMotor(MOTOR_L, "LEFT (60% intensity)",  153);
  buzzMotor(MOTOR_L, "LEFT (100% intensity)", 255);
  delay(600);

  // ── PHASE 3: Sweep test ────────────────────────────────────
  Serial.println("\n--- PHASE 3: Intensity sweep ---");
  sweepMotor(MOTOR_L, "LEFT");
  sweepMotor(MOTOR_C, "CENTER");
  sweepMotor(MOTOR_R, "RIGHT");

  // ── PHASE 4: All motors together (center-obstacle sim) ────
  Serial.println("\n--- PHASE 4: All motors ON (center obstacle sim) ---");
  Serial.println(">>> All motors buzzing simultaneously");
  analogWrite(MOTOR_L, 178);  // 70%
  analogWrite(MOTOR_C, 255);  // 100%
  analogWrite(MOTOR_R, 178);  // 70%
  delay(1500);
  analogWrite(MOTOR_L, 0);
  analogWrite(MOTOR_C, 0);
  analogWrite(MOTOR_R, 0);

  Serial.println("\n--- MOTOR TEST COMPLETE ---");
  Serial.println("If all phases felt correct, motors are working.");
  Serial.println("Restarting in 5 seconds...\n");
  delay(5000);
}
