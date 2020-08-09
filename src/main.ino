#include <Wire.h>
#include <Zumo32U4.h>
#include "Gyro.h"
#include "StateClasses.h"
#include "defines.h"
#include "proxSensor.h"
#include "CollisionDetect.h"

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
Accelerometer lsm303;

unsigned int lineSensorValues[3];
const char beep1[] PROGMEM = "!>32";
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody


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
void changeStateToDisplaying();
void changeStateToStaled();

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
    uint8_t l = newProxSensors.countsFrontWithLeftLeds();
    uint8_t r = newProxSensors.countsFrontWithRightLeds();
    if (l + r > 4) {
<<<<<<< HEAD
        if (l - r >= 2) {
          scanDir = DirectionLeft;
          motors.setSpeeds(-rammingSpeedLow, rammingSpeedLow);
        } else if (r - l >= 2) {
          scanDir = DirectionRight;
          motors.setSpeeds(rammingSpeedLow, -rammingSpeedLow);
        }
=======
        if (l - r >= 2) { motors.setSpeeds(-rammingSpeedLow, rammingSpeedLow); }
        if (r - l >= 2) { motors.setSpeeds(rammingSpeedLow, -rammingSpeedLow); }
>>>>>>> f177136b6b83ecb8b018d9d3b291cbdf18e62666
        return onSides;
    }
    if (newProxSensors.countsLeftWithLeftLeds() >= 1)
    {
        scanDir = DirectionLeft;
        onSides = true;
        motors.setSpeeds(-rammingSpeed, rammingSpeed);
    }
    else if (newProxSensors.countsRightWithRightLeds() >= 1)
    {
        scanDir = DirectionRight;
        onSides = true;
        motors.setSpeeds(rammingSpeed, -rammingSpeed);
    }

    if (onSides && (!isStatePushing())) {
        changeStateToPushing();
    }
    return onSides;
}

// sound horn and accelerate on contact -- fight or flight
bool contactCheck()
{
  bool inContact = false;
  if (check_for_contact()) {
    ledGreen(1);
    soundPlayer.playFromProgramSpace(sound_effect);
    motors.setSpeeds(-rammingSpeed, rammingSpeed);
    inContact = true;
  }
  if (inContact && (!isStatePushing())) {
        changeStateToPushing();
  }
  return inContact;
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
      //changeStateToDisplaying();
      changeStateToDriving();
    }
  }
} stateWaiting;
void changeStateToWaiting() { changeState(stateWaiting); }

class StateDisplaying : public State
{
    int16_t lcount;
    void setup() { lcount = 0;}
    void loop() {
        motors.setSpeeds(400, 400);
        lsm303.readAcceleration(millis());
        if (lcount % 100 == 0) {
            lcd.clear();
            lcd.gotoXY(0,0);
            lcd.print(F("Y"));
            lcd.print(lsm303.y_avg());
        }
        lcount++;
    }
} stateDisplaying;
void changeStateToDisplaying() { changeState(stateDisplaying); }

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
    if (borderCheck()) { return; }
    // if (contactCheck()) { return; }
    
    int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();

    if (lastStopAtEdge && counts > (int16_t)edgeToCenterTicks * 2)
    {
      changeStateToScanning();
    }

    if (sideCheck()) { return; }

    // Read the proximity sensors to sense the opponent.
    sense();
    if (objectSeen)
    {
      changeStateToPushing();
    }

  }
} stateDriving;
void changeStateToDriving() { changeState(stateDriving); }

class StateStaled : public State
{
  uint16_t staleSpeed;
  uint32_t loopcount;
  const uint8_t inc = 5;
  const uint16_t staleSpeedCap = 250;
  void setup() {
    loopcount = 0;
    staleSpeed = rammingSpeed;
    lcd.clear();
    lcd.gotoXY(0,0);
    lcd.print(F("STALED"));
  }

  void loop() {
    if (borderCheck()) { return; }
    
    sense();
    ledYellow(objectSeen);

    //if (!objectSeen) {
    //  changeStateToDriving();
    //}
    loopcount++;
    if (loopcount % 100 == 0) {
      lcd.clear();
      lcd.gotoXY(0,0);
      lcd.print(staleSpeed);
      if (staleSpeed > staleSpeedCap) {
        staleSpeed -= inc;
        motors.setSpeeds(staleSpeed, staleSpeed);
      }
      else {
        changeStateToPushing();
        return;
      }
    }
  }
} stateStaled;
void changeStateToStaled() { changeState(stateStaled); }



class StatePushing : public State
{
  void setup()
  {
    lcd.print(F("PUSHING"));
  }

  void loop()
  {
    ledRed(1);
    
    if (borderCheck()) { return; }
    if (sideCheck()) { return; }

    sense();
    newProxSensors.read();
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
    else if (objectSeen || (newProxSensors.countsFrontWithLeftLeds() + newProxSensors.countsFrontWithRightLeds() >= 4)) {
      changeStateToStaled();
    }
    else
    {
      motors.setSpeeds(rammingSpeed, rammingSpeed);
    }


  }
} statePushing;
void changeStateToPushing() { changeState(statePushing); }
bool isStatePushing() { return (currState == &statePushing); }


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
    if (sideCheck()) { return; }
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
            //changeStateToDriving();
        }
    }
  }
} stateScanning;
void changeStateToScanning() { changeState(stateScanning); }
bool isStateScanning() { return (currState == &stateScanning); }


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

// class StateDriveAlmostCenter : public State
// {
//   void setup()
//   {
//     encoders.getCountsAndResetLeft();
//     encoders.getCountsAndResetRight();
//     motors.setSpeeds(testingSpeed, testingSpeed);
//     lcd.print(F("Drive Almost Center"));
//   }
//   void loop()
//   {
//     int16_t counts = encoders.getCountsLeft() + encoders.getCountsRight();
//     if (counts > 2 * fractionAlmostCenter * edgeToCenterTicks)
//     {
//       changeStateToCircling();
//     }
//   }
// } stateDriveAlmostCenter;
// void changeStateToDriveAlmostCenter() { changeState(stateDriveAlmostCenter); }

// // In this state the robot rotates in place and tries to find
// // its opponent.
// class StateCircling : public State
// {
//   uint16_t degreesTurned;
//   uint32_t angleBase;
//   bool turned90;

//   void setup()
//   {
//     lastStopAtEdge = false;
//     degreesTurned = 0;
//     angleBase = 0;
//     gyroReset();

//     if (scanDir == DirectionRight)
//     {
//       motors.setSpeeds(turnSpeedHigh, -turnSpeedLow);
//     }
//     else
//     {
//       motors.setSpeeds(-turnSpeedLow, turnSpeedHigh);
//     }

//     lcd.print(F("turn90"));
//     turned90 = false;
//   }

//   void loop()
//   {
//     if (!turned90)
//     {
//       // Use the gyro to figure out how far we have turned while in this
//       // state.
//       gyroUpdate();
//       uint32_t angle1;
//       if (scanDir == DirectionRight)
//       {
//         angle1 = -turnAngle;
//       }
//       else
//       {
//         angle1 = turnAngle;
//       }
//       if ((int32_t)(angle1 - angleBase) > angle45)
//       {
//         angleBase += angle45;
//         degreesTurned += 45;
//       }

//       uint16_t time = timeInThisState();

//       if (degreesTurned >= 2 * angle45)
//       {
//         // We have not seen anything for a while, so start driving.
//         turned90 = true;
//       }
//     }
//     else
//     {
//       motors.setSpeeds(circleLeftSpeed,circleRightSpeed);
//       lcd.print(F("Circling"));
//     }

//   }
// } stateCircling;
// void changeStateToCircling() { changeState(stateCircling); }

void setup()
{
  gyroInit();
  s.initThreeSensors();
  senseInit();
  newProxSensors.initThreeSensors();
  changeStateToPausing();
  lsm303.init();
  lsm303.enable();
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
