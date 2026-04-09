/*
 * ============================================================
 *  TEST 1 — Sensor Test
 *  test_sensors.ino
 * ============================================================
 *  PURPOSE:
 *    Upload this sketch FIRST before the main firmware.
 *    Verifies all three HC-SR04 sensors work correctly
 *    and read accurate distances.
 *
 *  HOW TO USE:
 *    1. Upload this sketch
 *    2. Open Serial Monitor (Tools → Serial Monitor)
 *    3. Set baud rate to 9600
 *    4. Hold your hand at 10cm → should read ~10
 *    5. Move hand to 50cm → should read ~50
 *    6. Move hand to 100cm → should read ~100
 *    7. All three sensors should respond independently
 *
 *  PASS CRITERIA:
 *    - Readings stable within ±3cm on flat surface
 *    - No "TIMEOUT" messages on connected sensors
 *    - Values drop when you bring hand closer
 *
 *  FAIL DIAGNOSIS:
 *    - "TIMEOUT" on one sensor → check wiring / power
 *    - Jumpy readings (±20cm) → add 100µF cap across VCC-GND
 *    - All readings same wrong value → check TRIG/ECHO swap
 * ============================================================
 */

// Pin definitions — must match main firmware
#define TRIG_L   8
#define ECHO_L   9
#define TRIG_C   4
#define ECHO_C   5
#define TRIG_R   6
#define ECHO_R   7

// Read single sensor, return distance in cm
int readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long dur = pulseIn(echoPin, HIGH, 25000UL);  // 25ms timeout
  if (dur == 0) return -1;  // -1 = timeout / error

  return (int)(dur * 0.0343 / 2.0);
}

// Print bar chart of distance (visual in Serial Monitor)
void printBar(int dist) {
  if (dist < 0) { Serial.print("[TIMEOUT - CHECK WIRING]"); return; }
  if (dist > 200) { Serial.print("[OUT OF RANGE >200cm]"); return; }

  int bars = map(dist, 0, 200, 40, 1);  // Invert: close = more bars
  for (int i = 0; i < bars; i++) Serial.print("|");
  Serial.print(" ");
  Serial.print(dist);
  Serial.print("cm");
}

void setup() {
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  Serial.begin(9600);
  while (!Serial);  // Wait for Serial Monitor to open

  Serial.println("=========================================");
  Serial.println("  SENSOR TEST — Smart Navigation Band");
  Serial.println("=========================================");
  Serial.println("Hold hand at different distances.");
  Serial.println("Bar length = proximity (more bars = closer)");
  Serial.println("-----------------------------------------");
  delay(1000);
}

void loop() {
  int dL = readSensor(TRIG_L, ECHO_L);
  delay(15);
  int dC = readSensor(TRIG_C, ECHO_C);
  delay(15);
  int dR = readSensor(TRIG_R, ECHO_R);

  Serial.print("LEFT   ");
  printBar(dL);
  Serial.println();

  Serial.print("CENTER ");
  printBar(dC);
  Serial.println();

  Serial.print("RIGHT  ");
  printBar(dR);
  Serial.println();

  Serial.println("-----------------------------------------");
  delay(500);  // Refresh every 500ms
}
