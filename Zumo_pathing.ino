#include <Wire.h>
#include <Zumo32U4.h>
#include <PID_v1.h>

Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

const bool DEBUG = false;            // true = motors stay off, roll the wheels by hand to calibrate


// ---------- Line sensors: 0 = far left, 1 = middle, 2 = far right ----------
#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];
const uint16_t LINE_THRESHOLD = 800; // above this = dark (black tape)

// ---------- Speeds ----------
const int BASE_SPEED = 150;          // 0-400
const int TURN_SPEED = 100;
const int MAX_CORRECTION = 100;

// ---------- Pattern distances, in encoder counts (tune these) ----------
const long ARM_COUNTS     = 300;     // after a turn, ignore the line for this long
const long ADVANCE_COUNTS = 300;     // roll forward after seeing the line so the wheels are over it
const long FOLLOW_COUNTS  = 745;     // how far to follow the line before turning back

// ---------- Turning ----------
const float TURN_LEAD = 5.0;         // stop turning this many degrees early (coasting)

// ---------- Line following (small nudges of the target heading) ----------
const float NUDGE_RATE = 30.0;       // deg/s the target heading rotates while a side sensor sees the tape
const float MAX_NUDGE  = 6.0;        // never aim more than this far from the lane direction
uint32_t lastFollowMicros = 0;

// ---------- Gyro / PID ----------
const float HEADING_LIMIT = 3.0;     // degrees before forced correction
const int FORCED_CORRECTION = 60;

const float GYRO_SCALE = 0.07;       // deg/s per raw unit at 2000 dps
float gyroOffset = 0;
float heading = 0;                   // positive = turned left
uint32_t lastMicros = 0;

double Setpoint = 0, Input = 0, Output = 0;
double Kp = 15, Ki = 0.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ---------- Encoders ----------
long distLeft = 0, distRight = 0;    // running totals, counts
long lastDistLeft = 0, lastDistRight = 0;
float trim = 0;                      // learned left/right motor mismatch
const float KTRIM = 0.2;             // how fast the trim learns
const float TRIM_MAX = 30;

// ---------- Squaring up against the tape border (backing up) ----------
const int  ALIGN_SPEED      = 80;    // reverse speed, keep it slow
const long ALIGN_MAX_COUNTS = 600;   // give up and carry on if no border is found by now

bool alignInit = false;
bool alignStart[3];                  // sensor state (dark?) when the backing began
bool atEdge[3];                      // has this sensor crossed the border yet

// ---------- The pattern ----------
enum Action { DRIVE_TO_LINE, TURN_RIGHT, TURN_LEFT, FOLLOW_LINE, ALIGN_BACK };
const Action SEQUENCE[] = {
  DRIVE_TO_LINE, TURN_RIGHT, FOLLOW_LINE, TURN_RIGHT, ALIGN_BACK,
  DRIVE_TO_LINE, TURN_LEFT,  FOLLOW_LINE, TURN_LEFT,  ALIGN_BACK
};
const uint8_t SEQ_LEN = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);
uint8_t seqIndex = 0;
float targetHeading = 0;             // ideal direction of the current lane (absolute, degrees)
long segStart = 0;                   // encoder average at the start of the current action
bool lineSeen = false;

// ---------------------------------------------------------------------------

void calibrateGyro() {
  ledYellow(1);
  long total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  gyroOffset = total / 1024.0;
  ledYellow(0);
}

void updateHeading() {
  if (!imu.gyroDataReady()) return;
  imu.readGyro();

  uint32_t now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  float rate = (imu.g.z - gyroOffset) * GYRO_SCALE;
  heading += rate * dt;
}

void updateEncoders() {
  distLeft  += encoders.getCountsAndResetLeft();
  distRight += encoders.getCountsAndResetRight();
}

long avgDist() {
  return (distLeft + distRight) / 2;
}

void resyncEncoders() {
  lastDistLeft = distLeft;
  lastDistRight = distRight;
}

// Gyro PID holds Setpoint. The encoders learn the motors' built-in left/right
// mismatch (trim) while the robot is going straight, so the PID has less to fix.
void driveStraight() {
  Input = heading;
  if (myPID.Compute()) {
    double error = heading - Setpoint;
    double correction = Output;

    long dL = distLeft - lastDistLeft;
    long dR = distRight - lastDistRight;
    resyncEncoders();
    if (fabs(Output) < 15) {         // only learn while actually going straight
      trim += KTRIM * (dL - dR);     // left wheel moved more -> slow the left one down
      trim = constrain(trim, -TRIM_MAX, TRIM_MAX);
    }
    correction += trim;

    if (fabs(error) > HEADING_LIMIT) {
      if (error > 0) correction = min(correction, (double)-FORCED_CORRECTION);
      else           correction = max(correction, (double)FORCED_CORRECTION);
    }

    int left  = constrain(BASE_SPEED - (int)correction, -400, 400);
    int right = constrain(BASE_SPEED + (int)correction, -400, 400);
    motors.setSpeeds(left, right);
  }
}

// Tape under the middle sensor. A side sensor seeing tape means the tape is
// on that side, so aim the target heading a little toward it.
void followLine(bool leftDark, bool rightDark) {
  uint32_t now = micros();
  float dt = (now - lastFollowMicros) / 1000000.0;
  lastFollowMicros = now;
  if (dt > 0.05) dt = 0.05;

  int dir = 0;
  if (leftDark && !rightDark)      dir = +1;   // steer left
  else if (rightDark && !leftDark) dir = -1;   // steer right

  if (dir != 0) {
    Setpoint += dir * NUDGE_RATE * dt;
    Setpoint = constrain(Setpoint, targetHeading - MAX_NUDGE, targetHeading + MAX_NUDGE);
  }
  driveStraight();
}

void startAction() {
  Action a = SEQUENCE[seqIndex];
  if (a == TURN_RIGHT) targetHeading -= 90;    // absolute heading, so turn errors never add up
  if (a == TURN_LEFT)  targetHeading += 90;
  Setpoint = targetHeading;
  segStart = avgDist();
  lineSeen = false;
  resyncEncoders();
  lastFollowMicros = micros();
  alignInit = false;
}

void nextAction() {
  seqIndex = (seqIndex + 1) % SEQ_LEN;
  startAction();
}

void resetPID() {
  Setpoint = targetHeading;
  Output = 0;
  myPID.SetMode(MANUAL);
  myPID.SetMode(AUTOMATIC);
}

void finishTurn() {
  resetPID();
  nextAction();
}

// DEBUG mode: the motors never run. Roll the robot by hand and read the encoder
// counts (and line sensor values) in the Serial Monitor at 9600 baud.
// Press button A to zero the counts, e.g. before pushing the robot exactly 10 cm.
void calibrationMode() {
  motors.setSpeeds(0, 0);
  Serial.println(F("CALIBRATION MODE: motors off. Roll the robot by hand, press A to zero."));

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  distLeft = 0;
  distRight = 0;
  uint32_t lastPrint = 0;

  while (true) {
    updateEncoders();
    if (buttonA.getSingleDebouncedPress()) {
      distLeft = 0;
      distRight = 0;
    }

    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);

      Serial.print(F("left: "));   Serial.print(distLeft);
      Serial.print(F("  right: ")); Serial.print(distRight);
      Serial.print(F("  avg: "));   Serial.print(avgDist());
      Serial.print(F("  | line: "));
      Serial.print(lineSensorValues[0]); Serial.print(' ');
      Serial.print(lineSensorValues[1]); Serial.print(' ');
      Serial.println(lineSensorValues[2]);
    }
  }
}

// Back up with each side wheel controlled by its own sensor. A sensor "reaches the
// border" when its dark/light state flips from what it was when backing began
// (works whether the sensors start on the tape or on the floor). A wheel stops as
// soon as its sensor hits the border, so the robot pivots square to the tape edge.
void alignBack(bool leftDark, bool middleDark, bool rightDark, long travelled) {
  bool dark[3] = { leftDark, middleDark, rightDark };

  if (!alignInit) {
    for (uint8_t i = 0; i < 3; i++) { alignStart[i] = dark[i]; atEdge[i] = false; }
    alignInit = true;
  }
  for (uint8_t i = 0; i < 3; i++) {
    if (dark[i] != alignStart[i]) atEdge[i] = true;
  }

  bool allDone = atEdge[0] && atEdge[1] && atEdge[2];
  if (allDone || travelled <= -ALIGN_MAX_COUNTS) {
    motors.setSpeeds(0, 0);
    heading = targetHeading;       // the tape edge is the truth, so this also cancels gyro drift
    resetPID();
    nextAction();
    return;
  }

  int left  = atEdge[0] ? 0 : -ALIGN_SPEED;
  int right = atEdge[2] ? 0 : -ALIGN_SPEED;
  if (atEdge[0] && atEdge[2]) {    // both sides square, creep until the middle one is on it too
    left = right = -ALIGN_SPEED;
  }
  motors.setSpeeds(left, right);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init()) {
    ledRed(1);
    while (true) {}
  }
  imu.enableDefault();
  imu.configureForTurnSensing();

  uint8_t linePins[] = { SENSOR_DOWN1, SENSOR_DOWN3, SENSOR_DOWN5 };
  lineSensors.init(linePins, NUM_SENSORS);

  if (DEBUG) calibrationMode();   // never returns, the motors stay off

  myPID.SetOutputLimits(-MAX_CORRECTION, MAX_CORRECTION);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);

  calibrateGyro();             // keep the robot still
  buttonA.waitForButton();
  delay(1000);

  heading = 0;
  targetHeading = 0;
  lastMicros = micros();
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  seqIndex = 0;
  startAction();
}

void loop() {
  updateHeading();             // no delay() anywhere in the loop
  updateEncoders();

  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  bool leftDark   = lineSensorValues[0] > LINE_THRESHOLD;
  bool middleDark = lineSensorValues[1] > LINE_THRESHOLD;
  bool rightDark  = lineSensorValues[2] > LINE_THRESHOLD;
  long travelled  = avgDist() - segStart;

  switch (SEQUENCE[seqIndex]) {
    case DRIVE_TO_LINE:
      driveStraight();
      if (!lineSeen) {
        if (travelled > ARM_COUNTS && (leftDark || middleDark || rightDark)) {
          lineSeen = true;
          segStart = avgDist();            // measure the advance from here
        }
      } else if (travelled >= ADVANCE_COUNTS) {
        nextAction();
      }
      break;

    case FOLLOW_LINE:
      followLine(leftDark, rightDark);
      if (travelled >= FOLLOW_COUNTS) nextAction();
      break;

    case TURN_RIGHT:
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      if (heading <= targetHeading + TURN_LEAD) finishTurn();
      break;

    case TURN_LEFT:
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      if (heading >= targetHeading - TURN_LEAD) finishTurn();
      break;
    
    case ALIGN_BACK:
      alignBack(leftDark, middleDark, rightDark, travelled);
      break;
  }
}