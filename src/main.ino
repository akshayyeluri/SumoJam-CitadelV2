#include <Wire.h>
#include <Zumo32U4.h>
#include "Gyro.h"
#include "StateClasses.h"
#include "defines.h"

Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
L3G gyro;

const char beep1[] PROGMEM = "!>c32";

enum Direction
{
  DirectionLeft,
  DirectionRight,
};

bool motorsEnabled = false;
unsigned int lineSensorValues[3];

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

Direction turnCenterDir;
uint32_t turnCenterAngle;

uint16_t borderAnalyzeEncoderCounts;

// The time, in milliseconds, that we entered the current top-level state.
uint16_t stateStartTime;

// The time, in milliseconds, that the LCD was last updated.
uint16_t displayTime;

// This gets set to true whenever we change to a new state.
// A state can read and write this variable this in order to
// perform actions just once at the beginning of the state.
bool justChangedState;

// This gets set whenever we clear the display.
bool displayCleared;

bool lastStopAtEdge;

// Gets the amount of time we have been in this state, in
// milliseconds.  After 65535 milliseconds (65 seconds), this
// overflows to 0.
uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}

// Returns true if the display has been cleared or the contents
// on it have not been updated in a while.  The time limit used
// to decide if the contents are staled is specified in
// milliseconds by the staleTime parameter.
bool displayIsStale(uint16_t staleTime)
{
  return displayCleared || (millis() - displayTime) > staleTime;
}

// Any part of the code that uses displayIsStale to decide when
// to update the LCD should call this function when it updates the
// LCD.
void displayUpdated()
{
  displayTime = millis();
  displayCleared = false;
}

uint32_t calculateTurnCenterAngle(uint16_t counts)
{
  return ((uint32_t)turnAngle45 * 4 * ANGLE_FUDGE) -
    (uint32_t)((RAD_TO_INTERNAL_ANGLE * ANGLE_FUDGE) * atan((double)counts / sensorDistance));
}

extern State * robotState;

// Changes to a new state.  It also clears the LCD and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(State & state)
{
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  lcd.clear();
  displayCleared = true;
  robotState = &state;
}

void changeStateToPausing();
void changeStateToWaiting();
void changeStateToTurningToCenter();
void changeStateToDriving();
void changeStateToBacking();
void changeStateToPushing();
void changeStateToScanning();
void changeStateToAnalyzingBorder();

// In this state, we just wait for the user to press button
// A, while displaying the battery voltage every 100 ms.
class StatePausing : public State
{
public:
  void setup()
  {
    lastStopAtEdge = true;
    motors.setSpeeds(0, 0);
    lcd.print(F("Press A"));
  }

  void loop()
  {
    if (displayIsStale(100))
    {
      displayUpdated();
      lcd.gotoXY(0, 1);
      lcd.print(readBatteryMillivolts());
      lcd.print(F("     "));
    }

    if (buttonA.getSingleDebouncedPress())
    {
      buzzer.playFromProgramSpace(beep1);
      // The user pressed button A, so go to the waiting state.
      changeStateToWaiting();
    }
  }
} statePausing;
void changeStateToPausing() { changeState(statePausing); }
State * robotState = &statePausing;

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
      changeStateToDriving();
    }
  }
} stateWaiting;
void changeStateToWaiting() { changeState(stateWaiting); }

class StateTurningToCenter : public State
{
  void setup()
  {
    gyroReset();
    lcd.print(F("turncent"));
  }

  void loop()
  {
    if (turnCenterDir == DirectionLeft)
    {
      motors.setSpeeds(-turnCenterSpeed, turnCenterSpeed);
    }
    else
    {
      motors.setSpeeds(turnCenterSpeed, -turnCenterSpeed);
    }

    gyroUpdate();

    uint32_t angle = turnAngle;
    if (turnCenterDir == DirectionRight) { angle = -angle; }
    if (angle > turnCenterAngle && angle < (turnAngle45 * 7))
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
    lcd.print(F("drive"));
  }

  void loop()
  {
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();

    if (lastStopAtEdge && counts > (int16_t)edgeToCenterTicks * 2)
    {
      changeStateToScanning();
    }

    // Check for the white border.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      turnCenterDir = DirectionRight;
      changeStateToAnalyzingBorder();
      return;
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      turnCenterDir = DirectionLeft;
      changeStateToAnalyzingBorder();
      return;
    }
  }
} stateDriving;
void changeStateToDriving() { changeState(stateDriving); }

class StatePushing : public State
{
  void setup()
  {
    lcd.print(F("PUSH"));
  }

  void loop()
  {
    ledRed(1);

    motors.setSpeeds(rammingSpeed, rammingSpeed);

    // Check for the white border.
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      turnCenterDir = DirectionRight;
      changeStateToAnalyzingBorder();
      return;
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      turnCenterDir = DirectionLeft;
      changeStateToAnalyzingBorder();
      return;
    }
  }
} statePushing;
void changeStateToPushing() { changeState(statePushing); }

// In this state, the robot drives in reverse.
class StateBacking : public State
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    lcd.print(F("back"));
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

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(turnSpeedHigh, -turnSpeedLow);
    }
    else
    {
      motors.setSpeeds(-turnSpeedLow, turnSpeedHigh);
    }

    lcd.print(F("scan"));
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
    if ((int32_t)(angle1 - angleBase) > turnAngle45)
    {
      angleBase += turnAngle45;
      degreesTurned += 45;
    }

    uint16_t time = timeInThisState();

    if (degreesTurned >= scanDegreesMax)
    {
      // We have not seen anything for a while, so start driving.
      changeStateToDriving();
    }
    else if (time > scanTimeMin)
    {
        changeStateToPushing();
    }
  }
} stateScanning;
void changeStateToScanning() { changeState(stateScanning); }

class StateAnalyzingBorder : public State
{
  void setup()
  {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    motors.setSpeeds(analyzeSpeed, analyzeSpeed);
    lcd.print(F("analyze"));
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
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[1] < lineSensorThreshold)
    {
      if (counts < 0) { counts = 0; }

      borderAnalyzeEncoderCounts = counts;
      turnCenterAngle = calculateTurnCenterAngle(counts);

      changeStateToBacking();
    }

    // Make sure we don't travel too far.
    if (counts > 1600)
    {
      turnCenterAngle = turnAngle45 * 3;
      changeStateToBacking();
    }
  }
} stateAnalyzingBorder;
void changeStateToAnalyzingBorder() { changeState(stateAnalyzingBorder); }

void setup()
{
  gyroInit();
  lineSensors.initThreeSensors();
  changeStateToPausing();

  //senseTest();
}

void loop()
{
  if (justChangedState)
  {
    justChangedState = false;
    robotState->setup();
  }

  robotState->loop();
}
