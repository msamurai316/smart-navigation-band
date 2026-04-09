/*
 * ============================================================
 *  TEST 5 — Power & Startup Test
 *  test_power.ino
 * ============================================================
 *  PURPOSE:
 *    Run this sketch to verify the power supply is stable
 *    before connecting sensors and motors.
 *    Also performs a boot-up LED blink sequence so you know
 *    the Arduino is alive even without Serial Monitor.
 *
 *  HOW TO USE:
 *    1. Connect ONLY the Arduino + power circuit (no sensors/motors)
 *    2. Upload this sketch
 *    3. Power via MT3608 boost converter (not USB)
 *    4. Watch the built-in LED on pin 13:
 *       - 3 rapid blinks = Arduino is alive
 *       - 1 long blink every 2s = normal heartbeat
 *    5. Open Serial Monitor — check VCC reading
 *
 *  PASS CRITERIA:
 *    - LED blinks correctly
 *    - VCC reads between 4.8V and 5.2V
 *    - No resets during 2-minute run
 *
 *  NOTE:
 *    Arduino Nano reads internal 1.1V reference to estimate VCC.
 *    Accuracy is ±5%, but good enough to detect brownout (VCC<4.5V)
 *    which would cause erratic sensor behavior.
 * ============================================================
 */

#define LED_PIN  13  // Built-in LED on Arduino Nano

// ── Read VCC by comparing against internal 1.1V reference
// Credit: Nick Gammon's technique (gammon.com.au/adc)
long readVCC() {
  // Set reference to internal 1.1V, measure VCC rail
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Start conversion
  while (bit_is_set(ADCSRA, ADSC));  // Wait for result
  long result = ADCL;
  result |= ADCH << 8;
  // 1.1V × 1024 × 1000 / result = VCC in millivolts
  result = 1125300L / result;
  return result;  // in mV
}

// ── Blink LED n times quickly
void blinkN(int n, int onMs = 100, int offMs = 100) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH); delay(onMs);
    digitalWrite(LED_PIN, LOW);  delay(offMs);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Boot signature: 3 rapid blinks
  blinkN(3, 80, 80);
  delay(500);

  Serial.begin(9600);
  while (!Serial);

  Serial.println("=========================================");
  Serial.println("  POWER TEST — Smart Navigation Band");
  Serial.println("=========================================");
  Serial.println("Monitoring VCC and system stability.");
  Serial.println("Expected VCC range: 4800mV – 5200mV");
  Serial.println("-----------------------------------------");
}

void loop() {
  long vcc = readVCC();

  Serial.print("VCC: ");
  Serial.print(vcc);
  Serial.print(" mV  (");
  Serial.print(vcc / 1000.0, 2);
  Serial.print(" V)  ");

  if (vcc >= 4800 && vcc <= 5200) {
    Serial.println("✓ STABLE");
    // Heartbeat blink: 1 short blink
    blinkN(1, 50, 0);
  } else if (vcc < 4800) {
    Serial.println("⚠ LOW VOLTAGE — Check boost converter setting!");
    // Rapid warning blinks
    blinkN(5, 50, 50);
  } else {
    Serial.println("⚠ HIGH VOLTAGE — Reduce boost converter output!");
    blinkN(3, 200, 100);
  }

  delay(2000);  // Check every 2 seconds
}
