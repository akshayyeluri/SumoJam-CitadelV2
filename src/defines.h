
////////////////////////////////////////////////////////////
/* Gyro.cpp */
#define GYRO_CALIB_COUNT 512
// Filters for gyro
#define LOW_PASS 0b11111111
#define MED_PASS 0b00100000
#define HIGH_PASS 0b00000000
// (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
// = 14680063/17578125 unit/(digit*us)
#define DPS_TO_INTERNAL_ANGLE 14680063 / 17578125


////////////////////////////////////////////////////////////
/* Main.ino */
// Cuz our turning isn't perfect
#define ANGLE_FUDGE 0.9
// Convert from radians to internal angle (0x80000000 / pi)
#define RAD_TO_INTERNAL_ANGLE 0x28BE60DB


////////////////////////////////////////////////////////////
/* Speeds and Dists */
// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring.  This value might need to be tuned for
// different lighting conditions, surfaces, etc.
#define borderThreshold 1000

// The speed that the robot uses when backing up.
#define reverseSpeed 400

// The speed that the robot uses when turning.
#define turnSpeedHigh 400
#define turnSpeedLow 400

// The speed that the robot usually uses when moving forward.
#define forwardSpeed 400

// The speed that the robot drives when it thinks it is pushing or
// about to push an opponent.
#define rammingSpeed 400

// The speed used non-dominant wheel to turn while ramming.
#define rammingSpeedLow 200

// The minimum amount of time to spend scanning for nearby
// opponents, in milliseconds.
#define scanTimeMin 200

// The maximum amount of time to spend scanning for nearby
// opponents, in milliseconds.
// #define scanTimeMax 2100

// The maximum number of degrees to turn while scanning for the opponent.
#define scanDegreesMax 360 * 2 + 90

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
#define waitTime 5000

// If the robot has been driving forward for this amount of time,
// in milliseconds, without reaching a border, the robot decides
// that it must be pushing on another robot and this is a
// stalemate, so it increases its motor speed.
#define stalemateTime 1500

// The speed for border analysis.
#define analyzeSpeed 100

// The speed for center turning.
#define turnCenterSpeed 80

// The number of encoder ticks from edge to center
#define edgeToCenterTicks 2100 // 2180

// The number of encoder ticks to travel when backing away from the
// edge.
#define reverseTicks 400

// The number of encoder ticks of distance separating the middle and
// side line sensors.
#define sensorDistance 440


