#include <Wire.h>
#include <Zumo32U4.h>
#include "Gyro.h"
#include "StateClasses.h"
#include "defines.h"
#include "proxSensor.h"

////////////////////////////////////////////////////////////
// Structs and Enums
////////////////////////////////////////////////////////////
enum Direction
{
  DirectionLeft,
  DirectionRight,
};


////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////
Zumo32U4Buzzer soundPlayer;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4LineSensors s;
L3G gyro;
Zumo32U4ProximitySensors newProxSensors;

unsigned int lineSensorValues[3];
const char beep1[] PROGMEM = "!>32";


uint16_t borderAnalyzeEncoderCounts;

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;
Direction turnCenterDir;
uint32_t turnCenterAngle;

// The time, in milliseconds, that we entered the current top-level state.
uint16_t beginStateTime;

// The time, in milliseconds, that the LCD was last updated.
uint16_t displayTime;

// This gets set to true whenever we change to a new state.
// A state can read and write this variable this in order to
// perform actions just once at the beginning of the state.
bool recentStateChange;

// This gets set whenever we clear the display.
bool displayCleared;
bool lastStopAtEdge;

// The state we are in
State * currState;



////////////////////////////////////////////////////////////
// Helper Functions
////////////////////////////////////////////////////////////

// Gets the amount of time we have been in this state, in
// milliseconds.  After 65535 milliseconds (65 seconds), this
// overflows to 0.
uint16_t timeInThisState()
{
  return (uint16_t)(millis() - beginStateTime);
}

// Returns true if the display has been cleared or the contents
// on it have not been updated in a while.  The time limit used
// to decide if the contents are staled is specified in
// milliseconds by the staleTime parameter.
bool displayStaled(uint16_t staleTime)
{
    bool stale = displayCleared || (millis() - displayTime) > staleTime;
    if (stale) {
        displayTime = millis();
        displayCleared = false;
    }
    return stale;
}


uint32_t calculateTurnCenterAngle(uint16_t counts)
{
  return ((uint32_t)angle45 * 4 * ANGLE_FUDGE) -
    (uint32_t)((RAD_TO_INTERNAL_ANGLE * ANGLE_FUDGE) * atan((double)counts / sensorDistance));
}


// Changes to a new state.  It also clears the LCD and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(State & state)
{
  recentStateChange = true;
  displayCleared = true;
  beginStateTime = millis();
  ledRed(0); ledYellow(0); ledGreen(0);
  lcd.clear();
  currState = &state;
}

void changeStateToPausing();
void changeStateToWaiting();
void changeStateToTurningToCenter();
void changeStateToDriving();
void changeStateToBacking();
void changeStateToPushing();
void changeStateToScanning();
void changeStateToAnalyzingBorder();
void changeStateToDriveAlmostCenter();
void changeStateToCircling();

bool isStatePushing();
bool isStateScanning();


bool borderCheck()
{
    // Check for the white border.
    s.read(lineSensorValues);
    if (lineSensorValues[0] < borderThreshold || lineSensorValues[2] < borderThreshold)
    {
      turnCenterDir = (lineSensorValues[0] < borderThreshold) ? DirectionRight : DirectionLeft;
      changeStateToAnalyzingBorder();
      return true;
    }
    return false;
}

bool sideCheck() 
{
    newProxSensors.read();
    bool onSides = false;
    if (newProxSensors.countsLeftWithLeftLeds() >= 2)
    {
        scanDir = DirectionLeft;
        onSides = true;
        motors.setSpeeds(rammingSpeedLow, rammingSpeed);
    }
    else if (newProxSensors.countsRightWithLeftLeds() >= 2)
    {
        scanDir = DirectionRight;
        onSides = true;
        motors.setSpeeds(rammingSpeedLow, rammingSpeed);
    }

    if (onSides && (!isStatePushing())) {
        changeStateToPushing();
    }
}


////////////////////////////////////////////////////////////
// States
////////////////////////////////////////////////////////////

// In this state, we just wait for the user to press butt
// A, while displaying the battery voltage every 100 ms.
class StatePausing : public State
{
public:
  void setup()
  {
    motors.setSpeeds(0, 0);
    lcd.print(F("Press A"));
    lastStopAtEdge = true;
  }

  void loop()
  {
    if (displayStaled(100))
    {
      lcd.gotoXY(0, 1);
      lcd.print(readBatteryMillivolts());
      lcd.print(F("     "));
    }

    if (buttonA.getSingleDebouncedPress())
    {
      soundPlayer.playFromProgramSpace(beep1);
      changeStateToWaiting();
    }
  }
} statePausing;
void changeStateToPausing() { changeState(statePausing); }


class StateWaiting : public State
{
  void setup()
  {
    motors.setSpeeds(0, 0);
  }

  void loop()
  {
    // In this state, we wait for a while and then move on to the
    // scanning state.

    uint16_t time = timeInThisState();

    if (time < waitTime)
    {
      // Display the remaining time we have to wait.
      uint16_t timeLeft = waitTime - time;
      lcd.gotoXY(0, 0);
      lcd.print(timeLeft / 1000 % 10);
      lcd.print('.');
      lcd.print(timeLeft / 100 % 10);
    }
    else
    {
      // We have waited long enough.  Start moving.
      changeStateToDisplaying();
      //changeStateToDriving();
    }
  }
} stateWaiting;
void changeStateToWaiting() { changeState(stateWaiting); }

class StateDisplaying : public State
{
    void setup() {}
    void loop() {
        lcd.gotoXY(0, 0);
        newProxSensors.read();
        lcd.print(newProxSensors.countsFrontWithRightLeds());
        lcd.print('.');
        lcd.print(newProxSensors.countsFrontWithLeftLeds());
        lcd.print('.');
        lcd.print(newProxSensors.countsLeftWithLeftLeds());
        lcd.print('.');
        lcd.print(newProxSensors.countsRightWithRightLeds());
        lcd.print('.');
    }
}

class StateTurningToCenter : public State
{
  void setup()
  {
    gyroReset();
    lcd.print(F("TurnCent"));
  }

  void loop()
  {
    (turnCenterDir == DirectionLeft) ?
      motors.setSpeeds(-turnCenterSpeed, turnCenterSpeed) :
      motors.setSpeeds(turnCenterSpeed, -turnCenterSpeed);

    gyroUpdate();

    uint32_t angle = (turnCenterDir == DirectionRight) ? -turnAngle: turnAngle;
    if (angle > turnCenterAngle && angle < (angle45 * 7))
    {
      changeStateToDriving();
    }
  }
} stateTurningToCenter;
void changeStateToTurningToCenter() { changeState(stateTurningToCenter); }



class StateDriving : public State
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(forwardSpeed, forwardSpeed);
    lcd.print(F("Driving"));
  }

  void loop()
  {
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();

    if (lastStopAtEdge && counts > (int16_t)edgeToCenterTicks * 2)
    {
      changeStateToScanning();
    }

    if (borderCheck()) { return; }

    // Read the proximity sensors to sense the opponent.
    sense();
    if (objectSeen)
    {
      changeStateToPushing();
    }

  }
} stateDriving;
void changeStateToDriving() { changeState(stateDriving); }


class StatePushing : public State
{
  void setup()
  {
    lcd.print(F("PUSHING"));
  }

  void loop()
  {
    ledRed(1);

    sense();
    ledYellow(objectSeen);

    // Within the first second, we try to steer towards the enemy.
    // After that, we are probably locked in a stalemate and we should
    // ensure our motors are running at full power.
    if (objectSeen && timeInThisState() < stalemateTime)
    {
      if (brightnessLeft > brightnessRight)
      {
        // Swerve to the right.
        motors.setSpeeds(rammingSpeed, rammingSpeedLow);
      }
      else
      {
        motors.setSpeeds(rammingSpeedLow, rammingSpeed);
      }
    }
    else
    {
      motors.setSpeeds(rammingSpeed, rammingSpeed);
    }

    if (borderCheck()) { return; }
  }
} statePushing;
void changeStateToPushing() { changeState(statePushing); }
bool isStatePushing() { (&currState == &statePushing); }


// In this state, the robot drives in reverse.
class StateBacking : public State
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    lcd.print(F("Backing"));
  }

  void loop()
  {
    // After backing up for a specific distance, start scanning.
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
    if (-counts > (int16_t)reverseTicks * 2)
    {
      changeStateToTurningToCenter();
    }
  }
} stateBacking;
void changeStateToBacking() { changeState(stateBacking); }


// In this state the robot rotates in place and tries to find
// its opponent.
class StateScanning : public State
{
  uint16_t degreesTurned;
  uint32_t angleBase;

  void setup()
  {
    lastStopAtEdge = false;
    degreesTurned = 0;
    angleBase = 0;
    gyroReset();
    senseReset();

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(turnSpeedHigh, -turnSpeedLow);
    }
    else
    {
      motors.setSpeeds(-turnSpeedLow, turnSpeedHigh);
    }

    lcd.print(F("Scanning"));
  }

  void loop()
  {
    // Use the gyro to figure out how far we have turned while in this
    // state.
    gyroUpdate();
    uint32_t angle1;
    if (scanDir == DirectionRight)
    {
      angle1 = -turnAngle;
    }
    else
    {
      angle1 = turnAngle;
    }
    if ((int32_t)(angle1 - angleBase) > angle45)
    {
      angleBase += angle45;
      degreesTurned += 45;
    }

    sense();

    uint16_t time = timeInThisState();

    if (degreesTurned >= scanDegreesMax)
    {
      // We have not seen anything for a while, so start driving.
      changeStateToDriving();
    }
    else if (time > scanTimeMin)
    {
        if (objectSeen)
        {
            changeStateToPushing();
        }
    }
  }
} stateScanning;
void changeStateToScanning() { changeState(stateScanning); }
bool isStateScanning() { (&currState == &stateScanning); }


class StateAnalyzingBorder : public State
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(analyzeSpeed, analyzeSpeed);
    lcd.print(F("Anlyzing"));
    lastStopAtEdge = true;
  }

  void loop()
  {
    if (timeInThisState() < 1000)
    {
      // For the first second of this state, drive slowly.
      motors.setSpeeds(analyzeSpeed, analyzeSpeed);
    }
    else
    {
      // The state lasted too long, so we are probably pushing against
      // a robot and should drive at ramming speed.  Changing to the
      // Pushing state wouldn't help because it would detect the
      // border and immediate go back here.
      motors.setSpeeds(rammingSpeed, rammingSpeed);
    }

    // Check the encoders.
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();

    // Check the middle line sensor.
    s.read(lineSensorValues);
    if (lineSensorValues[1] < borderThreshold)
    {
      if (counts < 0) { counts = 0; }

      borderAnalyzeEncoderCounts = counts;
      turnCenterAngle = calculateTurnCenterAngle(counts);

      changeStateToBacking();
    }

    // Make sure we don't travel too far.
    if (counts > 1600)
    {
      turnCenterAngle = angle45 * 3;
      changeStateToBacking();
    }
  }
} stateAnalyzingBorder;
void changeStateToAnalyzingBorder() { changeState(stateAnalyzingBorder); }

class StateDriveAlmostCenter : public State
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(testingSpeed, testingSpeed);
    lcd.print(F("Drive Almost Center"));
  }
  void loop()
  {
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
    if (counts > 2 * fractionAlmostCenter * edgeToCenterTicks)
    {
      changeStateToCircling();
    }
  }
} stateDriveAlmostCenter;
void changeStateToDriveAlmostCenter() { changeState(stateDriveAlmostCenter); }

// In this state the robot rotates in place and tries to find
// its opponent.
class StateCircling : public State
{
  uint16_t degreesTurned;
  uint32_t angleBase;
  bool turned90;

  void setup()
  {
    lastStopAtEdge = false;
    degreesTurned = 0;
    angleBase = 0;
    gyroReset();

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(turnSpeedHigh, -turnSpeedLow);
    }
    else
    {
      motors.setSpeeds(-turnSpeedLow, turnSpeedHigh);
    }

    lcd.print(F("turn90"));
    turned90 = false;
  }

  void loop()
  {
    if (!turned90)
    {
      // Use the gyro to figure out how far we have turned while in this
      // state.
      gyroUpdate();
      uint32_t angle1;
      if (scanDir == DirectionRight)
      {
        angle1 = -turnAngle;
      }
      else
      {
        angle1 = turnAngle;
      }
      if ((int32_t)(angle1 - angleBase) > angle45)
      {
        angleBase += angle45;
        degreesTurned += 45;
      }

      uint16_t time = timeInThisState();

      if (degreesTurned >= 2 * angle45)
      {
        // We have not seen anything for a while, so start driving.
        turned90 = true;
      }
    }
    else
    {
      motors.setSpeeds(circleLeftSpeed,circleRightSpeed);
      lcd.print(F("Circling"));
    }

  }
} stateCircling;
void changeStateToCircling() { changeState(stateCircling); }

void setup()
{
  gyroInit();
  s.initThreeSensors();
  senseInit();
  newProxSensors.initThreeSensors();
  changeStateToPausing();
}


void loop()
{
  if (recentStateChange)
  {
    recentStateChange = false;
    currState->setup();
  }

  currState->loop();
}
