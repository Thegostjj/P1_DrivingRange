#include <Wire.h>
#include <Zumo32U4.h>
#include <PID_v1.h>

Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

const float NUDGE_DEG = 3.0;   // how far to aim off the current heading for a small correction
int nudgeDir = 0;              // -1 = steering right, 0 = holding, +1 = steering left

const int BASE_SPEED = 150;          // 0-400
const int TURN_SPEED = 100;
const int MAX_CORRECTION = 100;
const uint16_t LINE_THRESHOLD = 800; // above this = dark (black line)

const float HEADING_LIMIT = 3.0;     // degrees before forced correction
const int FORCED_CORRECTION = 60;

const float GYRO_SCALE = 0.07;       // deg/s per raw unit at 2000 dps
float gyroOffset = 0;
float heading = 0;                   // positive = turned left
uint32_t lastMicros = 0;

double Setpoint = 0, Input = 0, Output = 0;
double Kp = 15, Ki = 0.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

enum State { DRIVING, ADVANCING, TURNING_LEFT, TURNING_RIGHT };
State state = DRIVING;
State nextTurn;
uint32_t stateStart = 0;
float turnStartHeading = 0;

const uint16_t ADVANCE_MS = 150;  // roll forward so the wheels reach the corner; tune this
const float TURN_ANGLE = 85;      // slightly under 90 to allow for overshoot

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

void driveStraight() {
  Input = heading;
  if (myPID.Compute()) {
    double error = heading - Setpoint;       // was: heading
    double correction = Output;

    if (fabs(error) > HEADING_LIMIT) {
      if (error > 0) correction = min(correction, (double)-FORCED_CORRECTION);
      else           correction = max(correction, (double)FORCED_CORRECTION);
    }

    int left  = constrain(BASE_SPEED - (int)correction, -400, 400);
    int right = constrain(BASE_SPEED + (int)correction, -400, 400);
    motors.setSpeeds(left, right);
  }
}

void finishTurn() {
  Setpoint = heading;
  Output = 0;                 // otherwise the old output gets baked into the PID
  myPID.SetMode(MANUAL);
  myPID.SetMode(AUTOMATIC);
  state = DRIVING;
}

void followLine(bool innerLeftDark, bool innerRightDark) {
  int dir = 0;
  if (!innerLeftDark && innerRightDark)      dir = -1;   // left sensor lost the tape -> tape is to the right -> steer right
  else if (!innerRightDark && innerLeftDark) dir = +1;   // right sensor lost the tape -> tape is to the left -> steer left
  // both on the tape = centred, both off = gap or lost line: just hold heading

  if (dir != 0) {
    Setpoint = heading + dir * NUDGE_DEG;   // aim a few degrees toward the tape
  } else if (nudgeDir != 0) {
    Setpoint = heading;                     // back on the tape: hold this heading
  }
  nudgeDir = dir;

  driveStraight();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.init()) {
    ledRed(1);
    while (true) {}
  }
  imu.enableDefault();
  imu.configureForTurnSensing();

  lineSensors.initFiveSensors();

  myPID.SetOutputLimits(-MAX_CORRECTION, MAX_CORRECTION);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);

  calibrateGyro();             // keep the robot still
  buttonA.waitForButton();
  delay(1000);

  heading = 0;
  lastMicros = micros();
}

void loop() {
  updateHeading();

  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  bool cornerLeft  = lineSensorValues[0] > LINE_THRESHOLD;
  bool innerLeft   = lineSensorValues[1] > LINE_THRESHOLD;
  bool innerRight  = lineSensorValues[3] > LINE_THRESHOLD;
  bool cornerRight = lineSensorValues[4] > LINE_THRESHOLD;

  Serial.print("InnerLeft reads: ");
  Serial.println(lineSensorValues[1]);
  Serial.print("InnerRight reads: ");
  Serial.println(lineSensorValues[3]);

  switch (state) {
    case DRIVING:
      if (cornerRight || cornerLeft) {           // outer sensors win over inner ones
        nextTurn = cornerRight ? TURNING_RIGHT : TURNING_LEFT;
        Setpoint = heading;                      // cancel any nudge, advance straight
        nudgeDir = 0;
        stateStart = millis();
        state = ADVANCING;
      } else {
        followLine(innerLeft, innerRight);
      }
      break;

    case ADVANCING:
      driveStraight();
      if (millis() - stateStart >= ADVANCE_MS) {
        turnStartHeading = heading;
        state = nextTurn;
      }
      break;

    case TURNING_RIGHT:
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      if (turnStartHeading - heading >= TURN_ANGLE) finishTurn();
      break;

    case TURNING_LEFT:
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      if (heading - turnStartHeading >= TURN_ANGLE) finishTurn();
      break;
  }
}
