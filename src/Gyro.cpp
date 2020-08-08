/* Functions for using the gyro to get to the center */

#include <Wire.h>
#include "Gyro.h"
#include "defines.h"

// Time offset from last read
uint16_t lastClockGyro = 0;

// Average height (see calibration)
int16_t avVelGyro;

/* turnAngle is a 32-bit uint for amount of robot turn 
0x20000000 represents a 45 degree counter-clockwise rotation. */
uint32_t turnAngle = 0;

// turnRate is 0.07 degrees per second.
int16_t turnRate;

void gyroInit()
{
  lcd.clear();
  lcd.print(F("Gyro cal"));

  Wire.begin();
  gyro.init();

  gyro.writeReg(L3G::CTRL1, LOW_PASS);
  gyro.writeReg(L3G::CTRL4, MED_PASS);
  gyro.writeReg(L3G::CTRL5, HIGH_PASS);
  
  ledYellow(1);

  delay(400);

  // gyro calib
  int32_t i = 0; 
  int total=0;
  while (i++ < GYRO_CALIB_COUNT) {
      while(!gyro.readReg(L3G::STATUS_REG) & 0x08);
      gyro.read();

      // Add z axis to total;
      total += gyro.g.z;
  }
  ledYellow(0);
  avVelGyro = total / GYRO_CALIB_COUNT;

  gyroReset();
  lcd.clear();
  while (1)
  {
    gyroUpdate();
    lcd.gotoXY(0, 0);
    lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    lcd.print(F("   "));
    if (buttonA.getSingleDebouncedRelease()) {
        break;
    }
  }
  lcd.clear();
}

void gyroReset()
{
  turnAngle = 0;
  lastClockGyro = micros();
}

void updateTurnAngle(uint16_t dt) {
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * DPS_TO_INTERNAL_ANGLE;
}

void gyroUpdate()
{
  gyro.read();
  turnRate = gyro.g.z - avVelGyro;

  uint16_t m = micros();
  uint16_t dt = m - lastClockGyro;
  lastClockGyro = m;

  updateTurnAngle(dt);
}

