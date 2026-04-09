/*
 * ============================================================
 *  TEST 4 — Full Integration Test
 *  test_integration.ino
 * ============================================================
 *  PURPOSE:
 *    Final pre-deployment test. Simulates exactly what the
 *    main firmware does, but with verbose logging so you can
 *    verify every stage of the pipeline is working:
 *
 *      Raw reading → Median filter → Moving average
 *      → PWM map → Ramp → Motor output
 *
 *  HOW TO USE:
 *    1. Upload this sketch
 *    2. Open Serial Monitor at 9600 baud
 *    3. Walk toward a wall with the band on your wrist
 *    4. Observe each stage of output
 *    5. If all stages look correct, you're ready for main code
 *
 *  PASS CRITERIA:
 *    - Raw and Median values are close (±5cm typically)
 *    - Moving average changes smoothly, not in jumps
 *    - PWM increases as distance decreases
 *    - Ramped PWM follows target without sudden spikes
 *    - Motors respond correctly to what you're detecting
 * ============================================================
 */

#define TRIG_L   8
#define ECHO_L   9
#define TRIG_C   4
#define ECHO_C   5
#define TRIG_R   6
#define ECHO_R   7
#define MOTOR_L  10
#define MOTOR_C  11
#define MOTOR_R   3

#define DIST_MAX      150
#define DIST_STRONG    40
#define DIST_INVALID  350
#define SMOOTH_N        4
#define SCAN_INTERVAL  80
#define RAMP_UP        18
#define RAMP_DOWN      35

int bufL[SMOOTH_N], bufC[SMOOTH_N], bufR[SMOOTH_N];
int bufIdx = 0;
int targetL=0, targetC=0, targetR=0;
int currentL=0, currentC=0, currentR=0;
unsigned long lastScan = 0;

int readOnce(int tp, int ep) {
  digitalWrite(tp, LOW); delayMicroseconds(2);
  digitalWrite(tp, HIGH); delayMicroseconds(10);
  digitalWrite(tp, LOW);
  long d = pulseIn(ep, HIGH, 25000UL);
  if (!d) return DIST_INVALID;
  int dist = (int)(d*0.0343/2.0);
  return (dist<1||dist>DIST_INVALID) ? DIST_INVALID : dist;
}

int medianRead(int tp, int ep) {
  int r[5];
  for(int i=0;i<5;i++){r[i]=readOnce(tp,ep);delay(10);}
  for(int i=1;i<5;i++){int k=r[i],j=i-1;while(j>=0&&r[j]>k){r[j+1]=r[j];j--;}r[j+1]=k;}
  int inv=0; for(int i=0;i<5;i++) if(r[i]==DIST_INVALID) inv++;
  return inv>=3 ? DIST_INVALID : r[2];
}

int movAvg(int* buf, int val) {
  if(val!=DIST_INVALID) buf[bufIdx%SMOOTH_N]=val;
  long s=0; for(int i=0;i<SMOOTH_N;i++) s+=buf[i];
  return s/SMOOTH_N;
}

int toPWM(int d) {
  if(d==DIST_INVALID||d>=DIST_MAX) return 0;
  if(d<=DIST_STRONG) return 255;
  return map(d,DIST_MAX,DIST_STRONG,30,255);
}

void ramp(int &cur, int tgt) {
  if(cur<tgt) cur=min(cur+RAMP_UP,tgt);
  else if(cur>tgt) cur=max(cur-RAMP_DOWN,tgt);
}

void setup() {
  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_C,OUTPUT); pinMode(ECHO_C,INPUT);
  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);
  pinMode(MOTOR_L,OUTPUT); pinMode(MOTOR_C,OUTPUT); pinMode(MOTOR_R,OUTPUT);
  analogWrite(MOTOR_L,0); analogWrite(MOTOR_C,0); analogWrite(MOTOR_R,0);
  for(int i=0;i<SMOOTH_N;i++) bufL[i]=bufC[i]=bufR[i]=DIST_MAX;

  Serial.begin(9600);
  while(!Serial);

  Serial.println("==============================================");
  Serial.println("  INTEGRATION TEST — Smart Navigation Band");
  Serial.println("==============================================");
  Serial.println();
  Serial.println("Pipeline: Raw → Median → MovAvg → PWM → Ramp");
  Serial.println("----------------------------------------------");
  delay(1000);
}

void loop() {
  unsigned long now = millis();

  if(now - lastScan >= SCAN_INTERVAL) {
    lastScan = now;
    bufIdx++;

    // Stage 1: Raw single reads
    int rawL = readOnce(TRIG_L, ECHO_L);
    int rawC = readOnce(TRIG_C, ECHO_C);
    int rawR = readOnce(TRIG_R, ECHO_R);

    // Stage 2: Median filtered reads
    int medL = medianRead(TRIG_L, ECHO_L);
    int medC = medianRead(TRIG_C, ECHO_C);
    int medR = medianRead(TRIG_R, ECHO_R);

    // Stage 3: Moving average
    int avgL = movAvg(bufL, medL);
    int avgC = movAvg(bufC, medC);
    int avgR = movAvg(bufR, medR);

    // Stage 4: PWM targets
    targetL = toPWM(avgL);
    targetC = toPWM(avgC);
    targetR = toPWM(avgR);

    // Center blending
    if(avgC < DIST_MAX && avgC != DIST_INVALID) {
      int cp = (int)(toPWM(avgC)*0.70);
      if(cp > targetL) targetL = cp;
      if(cp > targetR) targetR = cp;
    }

    // ── Verbose pipeline log ──────────────────────────────
    Serial.println("──────────────────────────────────────────────");
    Serial.print("RAW      L:"); Serial.print(rawL==DIST_INVALID?-1:rawL);
    Serial.print("  C:"); Serial.print(rawC==DIST_INVALID?-1:rawC);
    Serial.print("  R:"); Serial.println(rawR==DIST_INVALID?-1:rawR);

    Serial.print("MEDIAN   L:"); Serial.print(medL==DIST_INVALID?-1:medL);
    Serial.print("  C:"); Serial.print(medC==DIST_INVALID?-1:medC);
    Serial.print("  R:"); Serial.println(medR==DIST_INVALID?-1:medR);

    Serial.print("MOV_AVG  L:"); Serial.print(avgL);
    Serial.print("  C:"); Serial.print(avgC);
    Serial.print("  R:"); Serial.println(avgR);

    Serial.print("TARGET   L:"); Serial.print(targetL);
    Serial.print("  C:"); Serial.print(targetC);
    Serial.print("  R:"); Serial.println(targetR);

    Serial.print("RAMPED   L:"); Serial.print(currentL);
    Serial.print("  C:"); Serial.print(currentC);
    Serial.print("  R:"); Serial.println(currentR);

    // Feedback summary
    Serial.print("STATUS   ");
    if(avgL < DIST_MAX)   { Serial.print("[LEFT OBS "); Serial.print(avgL); Serial.print("cm] "); }
    if(avgC < DIST_MAX)   { Serial.print("[CENTER OBS "); Serial.print(avgC); Serial.print("cm] "); }
    if(avgR < DIST_MAX)   { Serial.print("[RIGHT OBS "); Serial.print(avgR); Serial.print("cm] "); }
    if(avgL >= DIST_MAX && avgC >= DIST_MAX && avgR >= DIST_MAX) Serial.print("[PATH CLEAR]");
    Serial.println();
  }

  // Ramp runs every loop
  ramp(currentL, targetL);
  ramp(currentC, targetC);
  ramp(currentR, targetR);
  analogWrite(MOTOR_L, currentL);
  analogWrite(MOTOR_C, currentC);
  analogWrite(MOTOR_R, currentR);
}
